# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Golden image validation tests for Isaac Sim environment assets."""

from pathlib import Path

import carb
import omni.kit.app
import omni.kit.commands
import omni.kit.material.library
import omni.kit.test
import omni.usd
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils.app import get_extension_path
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path_async
from isaacsim.test.utils import (
    capture_viewport_annotator_data_async,
    compare_arrays_within_tolerances,
    read_image_as_array,
    save_rgb_image,
)
from omni.kit.viewport.utility import get_active_viewport


class TestEnvironmentGolden(omni.kit.test.AsyncTestCase):
    """Validate environment USD assets against golden reference images."""

    async def setUp(self):
        """Set up test fixtures."""
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        ext_path = Path(get_extension_path("isaacsim.test.collection"))
        self._golden_img_dir = ext_path / "data" / "tests" / "golden_img"

        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        await self._wait_for_stage_loading()

        self._viewport_api = get_active_viewport()
        self._usd_selection = omni.usd.get_context().get_selection()

    async def tearDown(self):
        """Tear down test fixtures."""
        await self._wait_for_stage_loading()

    async def _wait_for_stage_loading(self):
        while stage_utils.is_stage_loading():
            await omni.kit.app.get_app().next_update_async()
        await omni.kit.material.library.get_mdl_list_async()
        await omni.kit.app.get_app().next_update_async()

    async def _wait_n_frames(self, n: int = 10):
        for _ in range(n):
            await omni.kit.app.get_app().next_update_async()

    async def _validate_environment(
        self,
        usd_path: str,
        stage_path: str,
        golden_img_name: str,
        eye: list[float] | None = None,
        target: list[float] | None = None,
    ):
        """Add an environment asset as a reference and compare the viewport to a golden image.

        Args:
            usd_path: Relative USD path under the assets root.
            stage_path: Prim path to create the reference at.
            golden_img_name: Filename of the golden reference image.
            eye: Camera eye position.
            target: Camera look-at target.
        """
        if eye is None:
            eye = [3, -3, 3]
        if target is None:
            target = [0, 0, 0]

        asset_path = self._assets_root_path + usd_path
        omni.kit.commands.execute(
            "CreateReferenceCommand",
            usd_context=omni.usd.get_context(),
            path_to=stage_path,
            asset_path=asset_path,
            instanceable=False,
        )

        await self._wait_n_frames(10)
        await self._wait_for_stage_loading()
        await self._wait_n_frames(1)

        self._viewport_api.resolution = (1280, 720)
        self._usd_selection.clear_selected_prim_paths()
        await self._wait_n_frames(1)

        set_camera_view(eye=eye, target=target)

        golden_img_path = self._golden_img_dir / golden_img_name

        retries = 3
        results = None
        while retries > 0:
            await self._wait_n_frames(10)

            rgb_data = await capture_viewport_annotator_data_async(self._viewport_api)

            if not golden_img_path.exists():
                save_rgb_image(rgb_data, str(self._golden_img_dir), golden_img_name)
                self.fail(
                    f"Golden image not found at {golden_img_path}. "
                    f"Captured image saved to {self._golden_img_dir / golden_img_name} for reference. "
                    f"Please review and copy to golden directory if correct."
                )

            golden_img_data = read_image_as_array(golden_img_path)
            results = compare_arrays_within_tolerances(
                golden_img_data,
                rgb_data,
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=10,
                print_all_stats=True,
            )
            if results["passed"]:
                break
            retries -= 1
            await self._wait_n_frames(10)

        if not results["passed"]:
            stem = Path(golden_img_name).stem
            captured_name = f"{stem}_captured.png"
            save_rgb_image(rgb_data, str(self._golden_img_dir), captured_name)
            self.fail(
                f"Golden image mismatch for {usd_path}: {results}. "
                f"Captured image saved to {self._golden_img_dir / captured_name}"
            )

    async def test_black_grid(self):
        """Validate the Black Grid environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Grid/gridroom_black.usd",
            "/BlackGrid",
            "Black Grid.png",
        )

    async def test_curved_grid(self):
        """Validate the Curved Grid environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Grid/gridroom_curved.usd",
            "/CurvedGrid",
            "Curved Grid.png",
        )

    async def test_flat_grid(self):
        """Validate the Flat Grid environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Grid/default_environment.usd",
            "/FlatGrid",
            "Flat Grid.png",
        )

    async def test_full_warehouse(self):
        """Validate the Full Warehouse environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
            "/FullWarehouse",
            "Full Warehouse.png",
            eye=[-4, 4, 2],
            target=[0, 0, 1],
        )

    async def test_hospital(self):
        """Validate the Hospital environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Hospital/hospital.usd",
            "/Hospital",
            "Hospital.png",
            eye=[-4, 4, 2],
            target=[0, 0, 1],
        )

    async def test_jetracer_track(self):
        """Validate the Jetracer Track environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Jetracer/jetracer_track_solid.usd",
            "/JetracerTrack",
            "Jetracer Track.png",
        )

    async def test_office(self):
        """Validate the Office environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Office/office.usd",
            "/Office",
            "Office.png",
            eye=[-4, 4, 2],
            target=[0, 0, 1],
        )

    async def test_simple_room(self):
        """Validate the Simple Room environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Simple_Room/simple_room.usd",
            "/SimpleRoom",
            "Simple Room.png",
        )

    async def test_small_warehouse(self):
        """Validate the Small Warehouse environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
            "/SmallWarehouse",
            "Small Warehouse.png",
            eye=[-4, 4, 2],
            target=[0, 0, 1],
        )

    async def test_small_warehouse_digital_twin(self):
        """Validate the Small Warehouse Digital Twin environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Digital_Twin_Warehouse/small_warehouse_digital_twin.usd",
            "/SmallWarehouseDigitalTwin",
            "Small Warehouse Digital Twin.png",
            eye=[-4, 4, 2],
            target=[0, 0, 1],
        )

    async def test_small_warehouse_with_forklifts(self):
        """Validate the Small Warehouse With Forklifts environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd",
            "/SmallWarehouseWithForklifts",
            "Small Warehouse With Forklifts.png",
            eye=[-4, 4, 2],
            target=[0, 0, 1],
        )

    async def test_small_warehouse_with_multiple_shelves(self):
        """Validate the Small Warehouse With Multiple Shelves environment against its golden image."""
        await self._validate_environment(
            "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd",
            "/SmallWarehouseWithMultipleShelves",
            "Small Warehouse With Multiple Shelves.png",
            eye=[-4, 4, 2],
            target=[0, 0, 1],
        )
