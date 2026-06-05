# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Module providing PhysX-specific scene management with GPU configuration and solver settings."""

from __future__ import annotations

import dataclasses
from typing import Literal

import isaacsim.core.experimental.utils.prim as prim_utils
from pxr import PhysxSchema, Usd

from .physics_scene import PhysicsScene

_MSG_PHYSX_SCENE_API_NOT_VALID = "The Physics Scene has no 'PhysxSceneAPI' applied"


@dataclasses.dataclass(kw_only=True)
class PhysxGpuCfg:
    """Configuration dataclass for PhysX GPU settings.

    All fields are optional. When set to None, the corresponding setting is not modified.
    """

    gpu_collision_stack_size: int | None = None
    """Size of the GPU collision stack in bytes."""
    gpu_found_lost_aggregate_pairs_capacity: int | None = None
    """Capacity for GPU found/lost aggregate pairs."""
    gpu_found_lost_pairs_capacity: int | None = None
    """Capacity for GPU found/lost pairs."""
    gpu_heap_capacity: int | None = None
    """Size of the GPU heap in bytes."""
    gpu_max_deformable_surface_contacts: int | None = None
    """Maximum number of deformable surface contacts on GPU."""
    gpu_max_num_partitions: int | None = None
    """Maximum number of GPU partitions."""
    gpu_max_particle_contacts: int | None = None
    """Maximum number of particle contacts on GPU."""
    gpu_max_rigid_contact_count: int | None = None
    """Maximum number of rigid body contacts on GPU."""
    gpu_max_rigid_patch_count: int | None = None
    """Maximum number of rigid patches on GPU."""
    gpu_max_soft_body_contacts: int | None = None
    """Maximum number of soft body contacts on GPU."""
    gpu_temp_buffer_capacity: int | None = None
    """Size of the GPU temporary buffer in bytes."""
    gpu_total_aggregate_pairs_capacity: int | None = None
    """Total capacity for GPU aggregate pairs."""


class PhysxScene(PhysicsScene):
    """PhysX-specific wrapper for manipulating a USD Physics Scene prim with PhysX attributes.

    This class extends PhysicsScene to provide PhysX-specific functionality including
    solver configuration, GPU dynamics, CCD, and other PhysX-specific settings.

    Args:
        prim: USD Physics Scene prim path or prim instance.
            If the input is a path, a new USD Physics Scene prim is created if it does not exist.

    Raises:
        ValueError: If the input prim exists and is not a USD Physics Scene prim.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.simulation_manager import PhysxScene
        >>>
        >>> physx_scene = PhysxScene("/World/physicsScene")
    """

    def __init__(self, prim: str | Usd.Prim) -> None:
        super().__init__(prim)
        prim_utils.ensure_api(self._prim, PhysxSchema.PhysxSceneAPI)

    def get_dt(self) -> float:
        """Get the PhysX Scene's delta time (DT).

        This method is a convenience method to get the delta time (DT) from the steps per second
        returned by the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.get_steps_per_second` method.

        .. warning::

            Due to truncation of values in the *steps-per-second* (int) -- *delta-time* (float) conversion process,
            the delta time returned by this method might not be exactly the same as that established
            via the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.set_dt` method.

        Returns:
            PhysX Scene's delta time (DT).

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.get_dt()
            0.01666666...
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        steps_per_second = physx_scene_api.GetTimeStepsPerSecondAttr().Get()
        return 1.0 / steps_per_second if steps_per_second else 0.0

    def set_dt(self, dt: float) -> None:
        """Set the PhysX Scene's delta time (DT).

        This method is a convenience method to set the delta time (DT) by computing the steps per second and setting it
        via the :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.set_steps_per_second` method.

        .. warning::

            Due to truncation of values in the *steps-per-second* (int) -- *delta-time* (float) conversion process,
            the delta time set by this method might not be exactly the same as that returned by the
            :py:meth:`~isaacsim.core.simulation_manager.PhysxScene.get_dt` method.

        Args:
            dt: PhysX Scene's delta time (DT).

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.
            ValueError: If the delta time (DT) is less than 0 or greater than 1.0.

        Example:

        .. code-block:: python

            >>> physx_scene.set_dt(0.00833333)
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        if dt < 0.0 or dt > 1.0:
            raise ValueError(f"The delta time (DT) must be in the range [0.0, 1.0], got {dt}")
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        steps_per_second = int(1.0 / dt) if dt else 0
        physx_scene_api.GetTimeStepsPerSecondAttr().Set(steps_per_second)

    def get_steps_per_second(self) -> int:
        """Get the PhysX Scene's steps per second.

        Returns:
            PhysX Scene's steps per second.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.get_steps_per_second()
            60
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        return physx_scene_api.GetTimeStepsPerSecondAttr().Get()

    def set_steps_per_second(self, steps_per_second: int) -> None:
        """Set the PhysX Scene's steps per second.

        Args:
            steps_per_second: PhysX Scene's steps per second.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.
            ValueError: If the steps per second is less than 0.

        Example:

        .. code-block:: python

            >>> physx_scene.set_steps_per_second(120)
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        if steps_per_second < 0:
            raise ValueError(f"The steps per second must be greater than or equal to 0, got {steps_per_second}")
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        physx_scene_api.GetTimeStepsPerSecondAttr().Set(int(steps_per_second))

    def get_solver_type(self) -> Literal["TGS", "PGS"]:
        """Get the PhysX Scene's solver type.

        Returns:
            PhysX Scene's solver type.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.get_solver_type()
            'TGS'
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        return physx_scene_api.GetSolverTypeAttr().Get()

    def set_solver_type(self, solver_type: Literal["TGS", "PGS"]) -> None:
        """Set the PhysX Scene's solver type.

        Args:
            solver_type: PhysX Scene's solver type.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.
            ValueError: If the solver type is not a supported value.

        Example:

        .. code-block:: python

            >>> physx_scene.set_solver_type('PGS')
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        if solver_type not in ["TGS", "PGS"]:
            raise ValueError(f"Solver type must be 'TGS' or 'PGS', got '{solver_type}'")
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        physx_scene_api.GetSolverTypeAttr().Set(solver_type)

    def get_enabled_gpu_dynamics(self) -> bool:
        """Get the enabled state of the PhysX Scene's GPU dynamics.

        Returns:
            Boolean flag indicating if the PhysX Scene's GPU dynamics is enabled.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.get_enabled_gpu_dynamics()
            True
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        return physx_scene_api.GetEnableGPUDynamicsAttr().Get()

    def set_enabled_gpu_dynamics(self, enabled: bool) -> None:
        """Enable or disable the PhysX Scene's GPU dynamics.

        .. note::

            If GPU dynamics is enabled, the Continuous Collision Detection (CCD) will be automatically
            disabled as it is not supported.

        Args:
            enabled: Boolean flag to enable/disable the PhysX Scene's GPU dynamics.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.set_enabled_gpu_dynamics(False)
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        physx_scene_api.GetEnableGPUDynamicsAttr().Set(enabled)
        if enabled:
            self.set_enabled_ccd(False)

    def get_enabled_ccd(self) -> bool:
        """Get the enabled state of the PhysX Scene's Continuous Collision Detection (CCD).

        Returns:
            Boolean flag indicating if the PhysX Scene's CCD is enabled.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.get_enabled_ccd()
            False
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        return physx_scene_api.GetEnableCCDAttr().Get()

    def set_enabled_ccd(self, enabled: bool) -> None:
        """Enable or disable the PhysX Scene's Continuous Collision Detection (CCD).

        Args:
            enabled: Boolean flag to enable/disable the PhysX Scene's CCD.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.set_enabled_ccd(True)
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        physx_scene_api.GetEnableCCDAttr().Set(enabled)

    def get_broadphase_type(self) -> Literal["MBP", "GPU", "SAP"]:
        """Get the PhysX Scene's broadphase type.

        Returns:
            PhysX Scene's broadphase type.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.get_broadphase_type()
            'GPU'
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        return physx_scene_api.GetBroadphaseTypeAttr().Get()

    def set_broadphase_type(self, broadphase_type: Literal["MBP", "GPU", "SAP"]) -> None:
        """Set the PhysX Scene's broadphase type.

        Args:
            broadphase_type: PhysX Scene's broadphase type.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.
            ValueError: If the broadphase type is not a supported value.

        Example:

        .. code-block:: python

            >>> physx_scene.set_broadphase_type('MBP')
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        if broadphase_type not in ["MBP", "GPU", "SAP"]:
            raise ValueError(f"Broadphase type must be 'MBP', 'GPU' or 'SAP', got '{broadphase_type}'")
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        physx_scene_api.GetBroadphaseTypeAttr().Set(broadphase_type)

    def get_enabled_stabilization(self) -> bool:
        """Get the enabled state of the PhysX Scene's stabilization.

        Returns:
            Boolean flag indicating if the PhysX Scene's stabilization is enabled.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.get_enabled_stabilization()
            False
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        return physx_scene_api.GetEnableStabilizationAttr().Get()

    def set_enabled_stabilization(self, enabled: bool) -> None:
        """Enable or disable the PhysX Scene's stabilization.

        Args:
            enabled: Boolean flag to enable/disable the PhysX Scene's stabilization.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.set_enabled_stabilization(True)
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        physx_scene_api.GetEnableStabilizationAttr().Set(enabled)

    def get_gpu_configuration(self) -> PhysxGpuCfg:
        """Get the PhysX Scene's GPU configuration.

        Returns:
            PhysX Scene's GPU configuration.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> physx_scene.get_gpu_configuration()
            PhysxGpuCfg(gpu_collision_stack_size=67108864,
                gpu_found_lost_aggregate_pairs_capacity=1024, gpu_found_lost_pairs_capacity=262144,
                gpu_heap_capacity=67108864, gpu_max_deformable_surface_contacts=1048576,
                gpu_max_num_partitions=8, gpu_max_particle_contacts=1048576,
                gpu_max_rigid_contact_count=524288, gpu_max_rigid_patch_count=81920,
                gpu_max_soft_body_contacts=1048576, gpu_temp_buffer_capacity=16777216,
                gpu_total_aggregate_pairs_capacity=1024)
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        return PhysxGpuCfg(
            gpu_collision_stack_size=physx_scene_api.GetGpuCollisionStackSizeAttr().Get(),
            gpu_found_lost_aggregate_pairs_capacity=physx_scene_api.GetGpuFoundLostAggregatePairsCapacityAttr().Get(),
            gpu_found_lost_pairs_capacity=physx_scene_api.GetGpuFoundLostPairsCapacityAttr().Get(),
            gpu_heap_capacity=physx_scene_api.GetGpuHeapCapacityAttr().Get(),
            gpu_max_deformable_surface_contacts=physx_scene_api.GetGpuMaxDeformableSurfaceContactsAttr().Get(),
            gpu_max_num_partitions=physx_scene_api.GetGpuMaxNumPartitionsAttr().Get(),
            gpu_max_particle_contacts=physx_scene_api.GetGpuMaxParticleContactsAttr().Get(),
            gpu_max_rigid_contact_count=physx_scene_api.GetGpuMaxRigidContactCountAttr().Get(),
            gpu_max_rigid_patch_count=physx_scene_api.GetGpuMaxRigidPatchCountAttr().Get(),
            gpu_max_soft_body_contacts=physx_scene_api.GetGpuMaxDeformableVolumeContactsAttr().Get(),
            gpu_temp_buffer_capacity=physx_scene_api.GetGpuTempBufferCapacityAttr().Get(),
            gpu_total_aggregate_pairs_capacity=physx_scene_api.GetGpuTotalAggregatePairsCapacityAttr().Get(),
        )

    def set_gpu_configuration(self, cfg: PhysxGpuCfg | dict) -> None:
        """Set the PhysX Scene's GPU configuration.

        Args:
            cfg: PhysX Scene's GPU configuration.

        Raises:
            RuntimeError: If the Physics Scene has no 'PhysxSceneAPI' applied.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.simulation_manager import PhysxGpuCfg
            >>>
            >>> physx_scene.set_gpu_configuration(PhysxGpuCfg(gpu_max_num_partitions=16))
        """
        if not self.prim.HasAPI(PhysxSchema.PhysxSceneAPI):
            raise RuntimeError(_MSG_PHYSX_SCENE_API_NOT_VALID)
        cfg = PhysxGpuCfg(**cfg) if isinstance(cfg, dict) else cfg
        physx_scene_api = prim_utils.ensure_api(self.prim, PhysxSchema.PhysxSceneAPI)
        if cfg.gpu_collision_stack_size is not None:
            physx_scene_api.GetGpuCollisionStackSizeAttr().Set(cfg.gpu_collision_stack_size)
        if cfg.gpu_found_lost_aggregate_pairs_capacity is not None:
            physx_scene_api.GetGpuFoundLostAggregatePairsCapacityAttr().Set(cfg.gpu_found_lost_aggregate_pairs_capacity)
        if cfg.gpu_found_lost_pairs_capacity is not None:
            physx_scene_api.GetGpuFoundLostPairsCapacityAttr().Set(cfg.gpu_found_lost_pairs_capacity)
        if cfg.gpu_heap_capacity is not None:
            physx_scene_api.GetGpuHeapCapacityAttr().Set(cfg.gpu_heap_capacity)
        if cfg.gpu_max_deformable_surface_contacts is not None:
            physx_scene_api.GetGpuMaxDeformableSurfaceContactsAttr().Set(cfg.gpu_max_deformable_surface_contacts)
        if cfg.gpu_max_num_partitions is not None:
            physx_scene_api.GetGpuMaxNumPartitionsAttr().Set(cfg.gpu_max_num_partitions)
        if cfg.gpu_max_particle_contacts is not None:
            physx_scene_api.GetGpuMaxParticleContactsAttr().Set(cfg.gpu_max_particle_contacts)
        if cfg.gpu_max_rigid_contact_count is not None:
            physx_scene_api.GetGpuMaxRigidContactCountAttr().Set(cfg.gpu_max_rigid_contact_count)
        if cfg.gpu_max_rigid_patch_count is not None:
            physx_scene_api.GetGpuMaxRigidPatchCountAttr().Set(cfg.gpu_max_rigid_patch_count)
        if cfg.gpu_max_soft_body_contacts is not None:
            physx_scene_api.GetGpuMaxDeformableVolumeContactsAttr().Set(cfg.gpu_max_soft_body_contacts)
        if cfg.gpu_temp_buffer_capacity is not None:
            physx_scene_api.GetGpuTempBufferCapacityAttr().Set(cfg.gpu_temp_buffer_capacity)
        if cfg.gpu_total_aggregate_pairs_capacity is not None:
            physx_scene_api.GetGpuTotalAggregatePairsCapacityAttr().Set(cfg.gpu_total_aggregate_pairs_capacity)
