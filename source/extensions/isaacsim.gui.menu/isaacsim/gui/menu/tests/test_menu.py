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

"""Menu-driven asset creation tests for Isaac Sim."""

import omni.kit.app
import omni.kit.commands
import omni.kit.test
import omni.usd
from isaacsim.core.experimental.utils import prim as prim_utils
from isaacsim.test.utils import (
    MenuUITestCase,
    get_all_menu_paths,
)
from omni.kit.mainwindow import get_main_window
from omni.kit.ui_test import get_context_menu
from pxr import Usd, UsdPhysics

# =============================================================================
# Robot Menu Tests
# =============================================================================

ROBOT_ROOT_PATH = "Create/Robots"
ROBOT_SKIP_LIST = ["Create/Robots/Asset Browser"]


class TestRobotMenuAssets(MenuUITestCase):
    """Test class for verifying robot menu asset loading functionality."""

    async def _test_robot_menu_option(self, test_path: str) -> None:
        """Test a specific robot menu option.

        Args:
            test_path: Menu path to trigger.
        """
        await self.menu_click_with_retry(test_path)
        await self.wait_n_frames(20)
        await self.wait_for_stage_loading()

        has_robot = False
        for prim in self._stage.Traverse():
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                has_robot = True
                prim_path = prim_utils.get_prim_path(prim)
                print(f"articulation root found at {prim_path}")
                break

        self.assertTrue(has_robot, f"Failed to find articulation root for {test_path}")

    async def test_robot_menu_items(self) -> None:
        """Test all robot menu items."""
        # Get menu dict at runtime instead of module load time
        window = get_main_window()
        menu_dict = await get_context_menu(window._ui_main_window.main_menu_bar, get_all=False)

        robot_menu_dict = menu_dict.get("Create", {}).get("Robots", {})
        robot_menu_list = get_all_menu_paths(robot_menu_dict, root_path=ROBOT_ROOT_PATH)

        self.assertGreater(len(robot_menu_list), 0, f"No menu items found in {ROBOT_ROOT_PATH}")

        for test_path in robot_menu_list:
            if test_path not in ROBOT_SKIP_LIST:
                with self.subTest(menu_path=test_path):
                    await self._test_robot_menu_option(test_path)
                    # Reset stage for next iteration
                    await self.new_stage()


# =============================================================================
# Environment Menu Tests
# =============================================================================

ENVIRONMENT_ROOT_PATH = "Create/Environments"
ENVIRONMENT_SKIP_LIST = ["Create/Environments/Asset Browser"]


class TestEnvironmentMenuAssets(MenuUITestCase):
    """Test class for verifying environment menu asset loading functionality."""

    async def setUp(self) -> None:
        """Set up test environment before each test method."""
        await super().setUp()

    async def _test_environment_menu_option(self, test_path: str) -> None:
        """Test a specific environment menu option.

        Args:
            test_path: Menu path to trigger.
        """
        await self.menu_click_with_retry(test_path, delays=[100, 150, 200])
        await self.wait_n_frames(10)
        await self.wait_for_stage_loading()
        await self.wait_n_frames(1)

        prim_roots = {
            "Create/Environments/Black Grid": "/BlackGrid",
            "Create/Environments/Flat Grid": "/FlatGrid",
            "Create/Environments/Simple Room": "/SimpleRoom",
        }

        # verify stage is loaded
        prim_list = self._get_prims(omni.usd.get_context().get_stage())
        self.assertTrue(prim_roots[test_path] in prim_list, f"{prim_roots[test_path]} not found in {prim_list}")

    async def test_environment_menu_items(self) -> None:
        """Test all environment menu items."""
        # Get menu dict at runtime instead of module load time
        window = get_main_window()
        menu_dict = await get_context_menu(window._ui_main_window.main_menu_bar, get_all=False)

        environment_menu_dict = menu_dict.get("Create", {}).get("Environments", {})
        environment_menu_list = get_all_menu_paths(environment_menu_dict, root_path=ENVIRONMENT_ROOT_PATH)

        self.assertGreater(len(environment_menu_list), 0, f"No menu items found in {ENVIRONMENT_ROOT_PATH}")

        for test_path in environment_menu_list:
            if test_path not in ENVIRONMENT_SKIP_LIST:
                with self.subTest(menu_path=test_path):
                    await self._test_environment_menu_option(test_path)
                    # Reset stage for next iteration
                    await self.new_stage()

    def _get_prims(self, stage: object, exclude_list: list | None = None) -> list[str]:
        """Retrieve prims by traversing the stage and excluding specified prims.

        Args:
            stage: Stage to traverse for prims.
            exclude_list: List of prims to exclude.

        Returns:
            A list of prims found during traversal.
        """
        if exclude_list is None:
            exclude_list = []
        prims = []
        for p in stage.Traverse(
            Usd.TraverseInstanceProxies(Usd.PrimIsActive and Usd.PrimIsDefined and Usd.PrimIsLoaded)
        ):
            if p not in exclude_list:
                prims.append(p.GetPath().pathString)
        return prims


# =============================================================================
# April Tag Menu Test
# =============================================================================


class TestAprilTagMenu(MenuUITestCase):
    """Test class for verifying April Tag menu functionality."""

    async def test_apriltag_menu(self) -> None:
        """Test that April Tags menu creates material that can be bound to a mesh."""
        apriltag_path = "Create/April Tags"

        await self.wait_n_frames(1)
        await self.menu_click_with_retry(apriltag_path)
        await self.wait_n_frames(1)

        omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Cube", above_ground=True)
        await self.wait_n_frames(1)

        omni.kit.commands.execute(
            "BindMaterial", material_path="/Looks/AprilTag", prim_path=["/Cube"], strength=["weakerThanDescendants"]
        )
        await self.wait_n_frames(1)
