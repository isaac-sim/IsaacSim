# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Set to True to update golden images with test output, then set back to False
"""Verifies the AttachHydraTexture node connects render products to Hydra texture output. Covers render var handling, disabled execution, invalid or missing prim inputs, non-render-product prims, and golden image output."""

UPDATE_GOLDEN_IMAGES = False

import os

import carb
import cv2
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.test.utils.image_comparison import compare_arrays_within_tolerances
from pxr import Gf, UsdGeom, UsdRender


class TestAttachHydraTexture(ogts.OmniGraphTestCase):
    """Test suite for the IsaacAttachHydraTexture OmniGraph node."""

    GOLDEN_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "golden", "attach_hydra_texture")
    RGB_MEAN_DIFF_TOLERANCE = 5.0

    async def setUp(self) -> None:
        """Set up test environment, to be torn down when done."""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        self._stage = stage_utils.get_current_stage()

        # Create a camera for testing
        self._camera_path = "/World/TestCamera"
        UsdGeom.Camera.Define(self._stage, self._camera_path)

    async def tearDown(self) -> None:
        """Get rid of temporary data used by the test."""

    async def _create_render_product(self, camera_path: str, resolution: tuple = (1280, 720)) -> str:
        """Create a render product prim for testing.

        Args:
            camera_path: Path to the camera prim.
            resolution: Resolution tuple (width, height).

        Returns:
            Path to the created render product prim.
        """
        render_product_path = "/Render/TestRenderProduct"

        # Create the render product prim in the root layer (not session layer)
        # so that the hydra plugin can find it
        render_prod_prim = UsdRender.Product.Define(self._stage, render_product_path)
        render_prod_prim.GetResolutionAttr().Set(Gf.Vec2i(resolution[0], resolution[1]))
        render_prod_prim.GetCameraRel().SetTargets([camera_path])

        await omni.kit.app.get_app().next_update_async()
        return render_product_path

    async def test_attach_hydra_texture_basic(self) -> None:
        """Test basic hydra texture attachment to existing render product."""
        # Create a render product
        render_product_path = await self._create_render_product(self._camera_path)

        # Create the action graph with our node
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("attachHydraTexture", "isaacsim.core.nodes.IsaacAttachHydraTexture"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "attachHydraTexture.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("attachHydraTexture.inputs:renderProductPrim", render_product_path),
                ],
            },
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify the node executed and produced output
        output_path = og.Controller.attribute("outputs:renderProductPath", new_nodes[1]).get()
        self.assertIsNotNone(output_path)
        self.assertTrue(len(output_path) > 0)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_attach_hydra_texture_with_render_vars(self) -> None:
        """Test hydra texture attachment with render vars."""
        # Create a render product
        render_product_path = await self._create_render_product(self._camera_path)

        # Create the action graph with render vars
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("attachHydraTexture", "isaacsim.core.nodes.IsaacAttachHydraTexture"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "attachHydraTexture.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("attachHydraTexture.inputs:renderProductPrim", render_product_path),
                    ("attachHydraTexture.inputs:renderVars", ["LdrColor", "Depth"]),
                ],
            },
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify render vars were added to the render product
        render_prod_prim = UsdRender.Product(self._stage.GetPrimAtPath(render_product_path))
        self.assertTrue(render_prod_prim)

        ordered_vars = render_prod_prim.GetOrderedVarsRel().GetTargets()
        render_var_names = [str(path).split("/")[-1] for path in ordered_vars]
        self.assertIn("LdrColor", render_var_names)
        self.assertIn("Depth", render_var_names)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_attach_hydra_texture_disabled(self) -> None:
        """Test that disabled node doesn't execute."""
        # Create a render product
        render_product_path = await self._create_render_product(self._camera_path)

        # Create the action graph with node disabled
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("attachHydraTexture", "isaacsim.core.nodes.IsaacAttachHydraTexture"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "attachHydraTexture.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("attachHydraTexture.inputs:renderProductPrim", render_product_path),
                    ("attachHydraTexture.inputs:enabled", False),
                ],
            },
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify the node didn't produce output when disabled
        output_path = og.Controller.attribute("outputs:renderProductPath", new_nodes[1]).get()
        self.assertEqual(output_path, "")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_attach_hydra_texture_invalid_prim(self) -> None:
        """Test error handling for invalid render product prim."""
        # Create the action graph with invalid prim path
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("attachHydraTexture", "isaacsim.core.nodes.IsaacAttachHydraTexture"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "attachHydraTexture.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("attachHydraTexture.inputs:renderProductPrim", "/NonExistent/Prim"),
                ],
            },
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify the node didn't produce output for invalid prim
        output_path = og.Controller.attribute("outputs:renderProductPath", new_nodes[1]).get()
        self.assertEqual(output_path, "")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_attach_hydra_texture_non_render_product_prim(self) -> None:
        """Test error handling when prim exists but is not a RenderProduct."""
        # Create a non-RenderProduct prim
        non_rp_path = "/World/SomeXform"
        UsdGeom.Xform.Define(self._stage, non_rp_path)
        await omni.kit.app.get_app().next_update_async()

        # Create the action graph with non-RenderProduct prim
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("attachHydraTexture", "isaacsim.core.nodes.IsaacAttachHydraTexture"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "attachHydraTexture.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("attachHydraTexture.inputs:renderProductPrim", non_rp_path),
                ],
            },
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify the node didn't produce output for non-RenderProduct prim
        output_path = og.Controller.attribute("outputs:renderProductPath", new_nodes[1]).get()
        self.assertEqual(output_path, "")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_attach_hydra_texture_no_prim_specified(self) -> None:
        """Test error handling when no render product prim is specified."""
        # Create the action graph without specifying the prim
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("attachHydraTexture", "isaacsim.core.nodes.IsaacAttachHydraTexture"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "attachHydraTexture.inputs:execIn"),
                ],
            },
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify the node didn't produce output when no prim specified
        output_path = og.Controller.attribute("outputs:renderProductPath", new_nodes[1]).get()
        self.assertEqual(output_path, "")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_attach_hydra_texture_golden_image(self) -> None:
        """Test IsaacAttachHydraTexture with golden image comparison.

        This test verifies the node's visual output by:
        - Spawning a FlatGrid environment
        - Creating Cube, Cone, and Sphere primitives at specified positions
        - Creating a Camera and RenderProduct
        - Using the IsaacAttachHydraTexture node to attach a hydra texture
        - Capturing RGB output and comparing against a golden image
        """
        # Create a new stage with a simple environment (no Nucleus dependency)
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        stage = omni.usd.get_context().get_stage()

        # Create the World xform and a ground plane with dome light
        rep.functional.create.xform(name="World")
        rep.functional.create.plane(position=(0, 0, 0), scale=(10, 10, 1), parent="/World", name="GroundPlane")
        rep.functional.create.dome_light(intensity=1000, parent="/World", name="DomeLight")

        # Create Cube at (-0.05, -1.5, 1.5)
        cube = rep.functional.create.cube(
            position=(-0.05, -1.5, 1.5),
            scale=(0.5, 0.5, 0.5),
            parent="/World",
            name="TestCube",
        )
        rep.functional.modify.semantics(cube, {"class": "cube"}, mode="add")

        # Create Cone at (-1.1, 0.15, 1)
        cone = rep.functional.create.cone(
            position=(-1.1, 0.15, 1),
            scale=(0.5, 0.5, 0.5),
            parent="/World",
            name="TestCone",
        )
        rep.functional.modify.semantics(cone, {"class": "cone"}, mode="add")

        # Create Sphere at (-1.53, 1.133, 0.3949)
        sphere = rep.functional.create.sphere(
            position=(-1.53, 1.133, 0.3949),
            scale=(0.5, 0.5, 0.5),
            parent="/World",
            name="TestSphere",
        )
        rep.functional.modify.semantics(sphere, {"class": "sphere"}, mode="add")

        # Create Camera at (4, -0.5, 1.25) looking at the scene
        camera = rep.functional.create.camera(
            position=(4, -0.5, 1.25),
            look_at=(-0.5, 0, 1),
            parent="/World",
            name="TestCamera",
        )
        camera_path = str(camera.GetPath())

        await omni.kit.app.get_app().next_update_async()

        # Create RenderProduct at /RenderProduct_Camera with resolution 1280x720
        render_product_path = "/RenderProduct_Camera"
        render_prod_prim = UsdRender.Product.Define(stage, render_product_path)
        render_prod_prim.GetResolutionAttr().Set(Gf.Vec2i(1280, 720))
        render_prod_prim.GetCameraRel().SetTargets([camera_path])

        await omni.kit.app.get_app().next_update_async()

        # Create ActionGraph with OnPlaybackTick -> IsaacAttachHydraTexture
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("AttachHydraTexture", "isaacsim.core.nodes.IsaacAttachHydraTexture"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "AttachHydraTexture.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("AttachHydraTexture.inputs:renderProductPrim", render_product_path),
                    ("AttachHydraTexture.inputs:renderVars", ["LdrColor"]),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        # Attach RGB annotator to the render product
        rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb_annotator.attach([render_product_path])

        # Start playback and wait for rendering to settle
        self._timeline.play()

        # Wait for multiple frames to ensure hydra texture is attached and rendering
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        # Get the RGB data from the annotator (returns RGBA, strip alpha for comparison)
        image_data = rgb_annotator.get_data()
        self.assertIsNotNone(image_data, "RGB annotator returned None")
        if image_data.shape[2] == 4:
            image_data = image_data[:, :, :3]

        # Golden image path
        golden_dir = os.path.join(self.GOLDEN_DIR, "rgb")
        golden_path = os.path.join(golden_dir, "attach_hydra_texture_rgb.png")

        # If UPDATE_GOLDEN_IMAGES is True, save to golden directory
        if UPDATE_GOLDEN_IMAGES:
            os.makedirs(golden_dir, exist_ok=True)
            # cv2.imwrite expects BGR, so convert from RGB
            image_bgr = cv2.cvtColor(image_data, cv2.COLOR_RGB2BGR)
            cv2.imwrite(golden_path, image_bgr)
            carb.log_warn(f"Updated golden image at: {golden_path}")
            return

        # Fail immediately if golden image doesn't exist
        self.assertTrue(
            os.path.exists(golden_path),
            f"Golden image not found at {golden_path}. Set UPDATE_GOLDEN_IMAGES = True to generate it.",
        )

        # Load golden image (cv2.imread returns BGR, convert to RGB for comparison)
        golden_bgr = cv2.imread(golden_path)
        self.assertIsNotNone(golden_bgr, f"Failed to load golden image from {golden_path}")
        golden_rgb = cv2.cvtColor(golden_bgr, cv2.COLOR_BGR2RGB)

        # Compare images
        result = compare_arrays_within_tolerances(
            golden_array=golden_rgb,
            test_array=image_data,
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=self.RGB_MEAN_DIFF_TOLERANCE,
            print_all_stats=True,
        )

        self.assertTrue(
            result["passed"],
            f"Golden image comparison failed. Mean diff: {result['metrics']['mean_abs']:.3f}, "
            f"tolerance: {self.RGB_MEAN_DIFF_TOLERANCE}.",
        )
