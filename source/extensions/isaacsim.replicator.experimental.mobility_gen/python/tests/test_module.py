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

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from isaacsim.replicator.experimental.mobility_gen.impl.common import Buffer, Module


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestPathPlanner(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        pass

    # After running each test
    async def tearDown(self):
        pass

    # test to make sure this runs
    async def test_module_named_buffers(self):

        class ModuleA(Module):
            def __init__(self):
                super().__init__()
                self.buffer_a = Buffer(value=0)

        class ModuleB(Module):
            def __init__(self):
                super().__init__()
                self.module_a = ModuleA()
                self.buffer_b = Buffer(value=1)

        module = ModuleB()

        self.assertTrue("buffer_b" in module.named_buffers())
        self.assertTrue("module_a.buffer_a" in module.named_buffers())

    async def test_module_state_dict(self):

        class ModuleA(Module):
            def __init__(self):
                super().__init__()
                self.buffer_a = Buffer(value=0)

        class ModuleB(Module):
            def __init__(self):
                super().__init__()
                self.module_a = ModuleA()
                self.buffer_b = Buffer(value=1)

        module = ModuleB()

        state_dict = module.state_dict()

        self.assertTrue("buffer_b" in state_dict)
        self.assertEqual(state_dict["buffer_b"], 1)
        self.assertTrue("module_a.buffer_a" in state_dict)
        self.assertEqual(state_dict["module_a.buffer_a"], 0)

    async def test_module_named_buffer_with_tags(self):

        class ModuleA(Module):
            def __init__(self):
                super().__init__()
                self.buffer_a = Buffer(value=0, tags=["foo", "bar"])

        class ModuleB(Module):
            def __init__(self):
                super().__init__()
                self.module_a = ModuleA()
                self.buffer_b = Buffer(value=1, tags=["foo"])

        module = ModuleB()

        foo_buffers = module.named_buffers(include_tags=["foo"])
        bar_buffers = module.named_buffers(include_tags=["bar"])
        nonbar_buffers = module.named_buffers(exclude_tags=["bar"])

        self.assertTrue("buffer_b" in foo_buffers)
        self.assertTrue("buffer_b" not in bar_buffers)
        self.assertTrue("buffer_b" in nonbar_buffers)

        self.assertTrue("module_a.buffer_a" in foo_buffers)
        self.assertTrue("module_a.buffer_a" in bar_buffers)
        self.assertTrue("module_a.buffer_a" not in nonbar_buffers)
