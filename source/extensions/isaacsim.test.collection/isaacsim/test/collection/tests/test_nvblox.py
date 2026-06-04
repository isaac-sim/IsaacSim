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

"""Tests for NVBlox functionality in Isaac Sim."""

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html

# Disabled test due to issues with sensor paths not working correctly in test setup.
# class TestNvBloxScenes(omni.kit.test.AsyncTestCase):
#     # Before running each test
#     async def setUp(self):
#         self._assets_root_path = await get_assets_root_path_async()
#         if self._assets_root_path is None:
#             carb.log_error("Could not find Isaac Sim assets folder")
#             return
#         self._timeline = omni.timeline.get_timeline_interface()
#         pass

#     # After running each test
#     async def tearDown(self):
#         await omni.kit.stage_templates.new_stage_async()
#         self._timeline = None
#         pass

#     async def test_nvBlox_sample_scene(self):
#         # open scene
#         self.usd_path = self._assets_root_path + "/Isaac/Samples/NvBlox/nvblox_sample_scene.usd"
#         (result, error) = await open_stage_async(self.usd_path)

#         # Make sure the stage loaded
#         self.assertTrue(result)
#         await omni.kit.app.get_app().next_update_async()

#         self._timeline.play()

#         for i in range(10):
#             await omni.kit.app.get_app().next_update_async()

#         self._timeline.stop()
#         return True
