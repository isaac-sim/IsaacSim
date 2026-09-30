# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provide shared physics test helpers."""

from __future__ import annotations

import isaacsim.physics.registration as physics_registration

# Import OvPhysX through its public registration contract.
import isaacsim.physics_engines.ovphysx as _ovphysx

OVPHYSX_SIM_NAME = "ovphysx"


def set_suppress_readback(enable: bool) -> None:
    """Select whether OvPhysX keeps simulation data on the GPU.

    Args:
        enable: Whether to suppress CPU readback.

    """
    _ovphysx.set_suppress_readback(enable)


def set_tensor_device_ordinal(ordinal: int) -> None:
    """Declare the device holding OvPhysX tensors.

    OvPhysX exposes no query for the device its scene selected, so the caller that configured the tensor
    device states it and every entity view reports it.

    Args:
        ordinal: CUDA device ordinal, or -1 for host memory.

    """
    _ovphysx.set_tensor_device_ordinal(ordinal)


def get_tensor_device_ordinal() -> int:
    """Get the declared OvPhysX tensor-device ordinal.

    Returns:
        The ordinal last declared, or -1 when none was.

    """
    return _ovphysx.get_tensor_device_ordinal()


def find_ovphysx_sim_id() -> physics_registration.SimulationId | None:
    """Find the registered OvPhysX simulation identifier.

    Returns:
        The OvPhysX identifier, or None when the backend is not registered.

    """
    physics = physics_registration
    for sim_id in physics.get_simulation_ids():
        if physics.get_simulation_name(sim_id) == OVPHYSX_SIM_NAME:
            return sim_id
    return None
