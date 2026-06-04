# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verify torch is imported from the expected path and CUDA is available."""

from isaacsim import SimulationApp

# instantiate SimulationApp first as omni.isaac.ml_archive has been removed from PYTHONPATH
simulation_app = SimulationApp({"headless": True})

import torch

print(torch.__path__[0])
assert "omni.isaac.ml_archive" in torch.__path__[0]
print(f"Cuda available: {torch.cuda.is_available()}")
assert torch.cuda.is_available()


@torch.jit.script
def add(a, b):
    """Add two tensors element-wise using torch.jit.script."""
    return a + b


a = torch.ones((10, 2), device="cuda:0")
b = torch.ones((10, 2), device="cuda:0")
c = add(a, b)
d = a + b
assert torch.allclose(c, d)

simulation_app.close()
