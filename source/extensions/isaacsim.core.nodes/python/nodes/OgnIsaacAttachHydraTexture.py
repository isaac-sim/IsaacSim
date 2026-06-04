# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import uuid

import carb
import carb.eventdispatcher
import carb.settings
import omni
import omni.kit.hydra_texture
import omni.timeline
from isaacsim.core.nodes import BaseResetNode
from isaacsim.core.nodes.ogn.OgnIsaacAttachHydraTextureDatabase import OgnIsaacAttachHydraTextureDatabase
from pxr import Sdf, Usd, UsdRender


class OgnIsaacAttachHydraTextureInternalState(BaseResetNode):
    """Internal state for the OgnIsaacAttachHydraTexture node."""

    def __init__(self):
        self.hydra_texture = None
        self.applied_render_vars = set()
        self.rp_sub_stop = None
        self.rp_sub_play = None
        self.drawable_changed_sub = None
        # Cache async rendering setting on initialization
        settings = carb.settings.get_settings()
        self.is_async = settings.get("/app/asyncRendering") or False
        super().__init__(initialize=False)

    def on_timeline_stop(self, event: carb.eventdispatcher.Event):
        """Timeline stop event callback - disable hydra texture updates."""
        if self.hydra_texture:
            self.hydra_texture.set_updates_enabled(False)
        self.initialized = False

    def on_timeline_play(self, event: carb.eventdispatcher.Event):
        """Timeline play event callback - enable hydra texture updates."""
        if self.hydra_texture:
            self.hydra_texture.set_updates_enabled(True)


class OgnIsaacAttachHydraTexture:
    """Isaac Sim node that attaches render vars and a hydra texture to an existing render product."""

    @staticmethod
    def internal_state():
        """Create internal state for the node instance."""
        return OgnIsaacAttachHydraTextureInternalState()

    @staticmethod
    def _add_render_var(stage: Usd.Stage, render_product_path: str, render_var_name: str) -> tuple[bool, str]:
        """Add a render var (AOV) to the render product.

        Args:
            stage: The USD stage.
            render_product_path: Path to the render product prim.
            render_var_name: Name of the render var to add.

        Returns:
            Tuple of (success, error_message). If success is True, error_message is empty.
        """
        render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(render_product_path))
        if not render_prod_prim:
            return False, f'Invalid renderProduct "{render_product_path}"'

        render_var_prim_path = Sdf.Path(f"/Render/Vars/{render_var_name}")
        render_var_prim = stage.GetPrimAtPath(render_var_prim_path)
        if not render_var_prim:
            render_var_prim = stage.DefinePrim(render_var_prim_path)
        if not render_var_prim:
            return False, f'Cannot create renderVar "{render_var_prim_path}"'

        render_var_prim.CreateAttribute("sourceName", Sdf.ValueTypeNames.String).Set(render_var_name)

        render_prod_var_rel = render_prod_prim.GetOrderedVarsRel()
        if not render_prod_var_rel:
            render_prod_prim.CreateOrderedVarsRel()
            render_prod_var_rel = render_prod_prim.GetOrderedVarsRel()
        if not render_prod_var_rel:
            return False, f'Cannot set orderedVars relationship for renderProduct "{render_product_path}"'

        # Check if render var is already added
        existing_targets = render_prod_var_rel.GetTargets()
        if render_var_prim_path not in existing_targets:
            render_prod_var_rel.AddTarget(render_var_prim_path)

        return True, ""

    @staticmethod
    def compute(db) -> bool:
        """Compute method for the node.

        Args:
            db: The OmniGraph database.

        Returns:
            True if computation succeeded, False otherwise.
        """
        state = db.per_instance_state

        # Handle disabled state
        if db.inputs.enabled is False:
            if state.hydra_texture is not None:
                state.hydra_texture.set_updates_enabled(False)
            return False
        else:
            if state.hydra_texture is not None:
                state.hydra_texture.set_updates_enabled(True)

        # Validate render product prim input
        if len(db.inputs.renderProductPrim) == 0:
            db.log_error("Render product prim must be specified")
            return False

        render_product_path = db.inputs.renderProductPrim[0].GetString()

        # Get stage and validate render product
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(render_product_path)

        if not prim.IsValid():
            db.log_error(f'Render product prim does not exist: "{render_product_path}"')
            return False

        if not prim.IsA(UsdRender.Product):
            db.log_error(f'Prim is not a RenderProduct: "{render_product_path}"')
            return False

        render_prod_prim = UsdRender.Product(prim)
        if not render_prod_prim:
            db.log_error(f'Invalid RenderProduct prim: "{render_product_path}"')
            return False

        with Usd.EditContext(stage, stage.GetSessionLayer()):
            # Apply render vars
            render_vars = db.inputs.renderVars
            for render_var in render_vars:
                render_var_name = str(render_var)
                if render_var_name and render_var_name not in state.applied_render_vars:
                    success, error_msg = OgnIsaacAttachHydraTexture._add_render_var(
                        stage, render_product_path, render_var_name
                    )
                    if not success:
                        db.log_error(error_msg)
                        return False
                    state.applied_render_vars.add(render_var_name)
                    carb.log_info(f'Added render var "{render_var_name}" to render product "{render_product_path}"')

            # Create/attach hydra texture if not already done
            if state.hydra_texture is None:
                # Get camera path from render product
                camera_rel = render_prod_prim.GetCameraRel()
                camera_targets = camera_rel.GetTargets()
                if not camera_targets:
                    db.log_error(f'RenderProduct has no camera assigned: "{render_product_path}"')
                    return False
                camera_path = str(camera_targets[0])

                # Get resolution from render product
                resolution_attr = render_prod_prim.GetResolutionAttr()
                resolution = resolution_attr.Get()
                if resolution is None:
                    resolution = (1280, 720)  # Default resolution
                else:
                    resolution = (int(resolution[0]), int(resolution[1]))

                # Ensure the hydra engine is attached to the USD context
                engine_name = "rtx"
                usd_context_name = ""
                usd_context = omni.usd.get_context(usd_context_name)
                if engine_name not in usd_context.get_attached_hydra_engine_names():
                    omni.usd.add_hydra_engine(engine_name, usd_context)

                # Generate a unique name incorporating the render product name for debugging
                rp_name = render_product_path.replace("/", "_").strip("_")
                texture_name = f"IsaacHydraTexture_{rp_name}_{uuid.uuid4().hex[:8]}"

                # Create a hydra texture using the module-level function (non-deprecated)
                state.hydra_texture = omni.kit.hydra_texture.create_hydra_texture(
                    name=texture_name,
                    width=resolution[0],
                    height=resolution[1],
                    usd_context_name=usd_context_name,
                    usd_camera_path=camera_path,
                    hydra_engine_name=engine_name,
                    is_async=state.is_async,
                )

                # Attach the hydra texture to the existing render product
                state.hydra_texture.set_render_product_path(render_product_path)

                # Subscribe to timeline events for hydra texture management
                state.rp_sub_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
                    event_name=omni.timeline.GLOBAL_EVENT_STOP,
                    on_event=state.on_timeline_stop,
                    observer_name="isaacsim.core.nodes.OgnIsaacAttachHydraTexture.on_timeline_stop",
                )
                state.rp_sub_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
                    event_name=omni.timeline.GLOBAL_EVENT_PLAY,
                    on_event=state.on_timeline_play,
                    observer_name="isaacsim.core.nodes.OgnIsaacAttachHydraTexture.on_timeline_play",
                )

                carb.log_info(f'Attached hydra texture "{texture_name}" to render product "{render_product_path}"')

        # Output the render product path
        db.outputs.renderProductPath = render_product_path
        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        """Release resources when the node instance is destroyed."""
        try:
            state = OgnIsaacAttachHydraTextureDatabase.per_instance_internal_state(node)
        except Exception:
            state = None

        if state is not None:
            # Clean up the hydra texture
            state.hydra_texture = None
            state.rp_sub_stop = None
            state.rp_sub_play = None
            state.drawable_changed_sub = None
            state.applied_render_vars.clear()
