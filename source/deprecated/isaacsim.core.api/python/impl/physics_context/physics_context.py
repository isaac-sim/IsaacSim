# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""High level wrapper for creating and configuring PhysicsScene prims and managing physics simulation settings."""

from __future__ import annotations

import carb
import omni
import omni.kit.app
import omni.physics.core
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.carb import get_carb_setting, set_carb_setting
from isaacsim.core.utils.constants import AXES_INDICES
from isaacsim.core.utils.prims import get_prim_at_path, get_prim_path, is_prim_path_valid
from isaacsim.core.utils.stage import get_current_stage, get_stage_units, traverse_stage
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


class PhysicsContext(object):
    """Provide high-level functions for managing physics scene and simulation settings.

    Create a PhysicsScene prim at the specified prim path when no PhysicsScene is present in the current stage.
    If a PhysicsScene already exists, use the existing scene and apply default settings regardless of the
    specified prim_path.

    Args:
        physics_dt: Specifies the physics_dt of the simulation.
        prim_path: Specifies the prim path to create a PhysicsScene at,
            only in the case where no PhysicsScene already defined.
        sim_params: Dictionary of simulation parameters to configure physics settings.
        set_defaults: Set to True to use the defaults physics parameters
            [physics_dt = 1.0/ 60.0,
            gravity = -9.81 m / s
            ccd_enabled,
            stabilization_enabled,
            gpu dynamics turned off,
            broadphase type is MBP,
            solver type is TGS].

    Raises:
        Exception: If prim_path is not absolute.
        Exception: If prim_path already exists and its type is not a PhysicsScene.

    """

    def __init__(
        self,
        physics_dt: float | None = None,
        prim_path: str = "/physicsScene",
        sim_params: dict = None,
        set_defaults: bool = True,
    ) -> None:
        self._prim_path = prim_path
        if not Sdf.Path(self._prim_path).IsAbsolutePath():
            raise Exception(f"Input prim path is not absolute: {self._prim_path}")
        # check if there is a current physics scene defined already in the scene
        current_physics_prim = self.get_current_physics_scene_prim()
        self._physx_scene_api = None
        self._carb_settings = carb.settings.get_settings()
        if current_physics_prim is None:
            # creating a new physics scene
            if is_prim_path_valid(prim_path):
                raise Exception(f"A non physics scene prim already exists at: {self._prim_path}")
            self._physics_scene = self._create_new_physics_scene(prim_path=prim_path)
        else:
            # already exists a physics scene
            self._prim_path = get_prim_path(current_physics_prim)
            carb.log_info(f"Physics Scene at path `{self._prim_path}` is already defined - reusing it")
            self._physics_scene = UsdPhysics.Scene(current_physics_prim)
            # get or apply PhysxScene API
            if current_physics_prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                self._physx_scene_api = PhysxSchema.PhysxSceneAPI(current_physics_prim)
            else:
                self._physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(current_physics_prim)
        # set the default physics scene
        SimulationManager.set_default_physics_scene(self._prim_path)
        self._physics_sim_interface = omni.physics.core.get_physics_simulation_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        self._device = SimulationManager.get_physics_sim_device()
        if "cuda" in self._device:
            self._use_gpu_pipeline = True
            self._use_gpu = True
        else:
            self._use_gpu_pipeline = False
            self._use_gpu = False

        if self._use_gpu:
            self.set_broadphase_type("GPU")
            self.enable_gpu_dynamics(flag=True)
            self.enable_fabric(True)
            self.enable_ccd(flag=False)  # Disable CCD for GPU dynamics as its not supported
        else:
            self.set_broadphase_type("MBP")
            self.enable_gpu_dynamics(flag=False)

        if sim_params is None and set_defaults:
            meters_per_unit = get_stage_units()
            self.set_gravity(value=-9.81 / meters_per_unit)
            self.enable_stablization(flag=False)
            if self._use_gpu_pipeline:
                self.enable_ccd(flag=False)
                self._carb_settings.set_bool("/physics/suppressReadback", True)
            else:
                self.enable_ccd(flag=True)
                self._carb_settings.set_bool("/physics/suppressReadback", False)
            self.set_solver_type(solver_type="TGS")
            self.set_physics_dt(dt=1.0 / 60.0)

        if sim_params is not None:
            if "gravity" in sim_params:
                up_axis = UsdGeom.GetStageUpAxis(get_current_stage())
                self.set_gravity(sim_params["gravity"][AXES_INDICES[up_axis]])

            if "substeps" in sim_params:
                substeps = sim_params["substeps"]
            else:
                substeps = None
            if "dt" in sim_params:
                self.set_physics_dt(dt=sim_params["dt"], substeps=substeps)
                stage = get_current_stage()
                with Usd.EditContext(stage, Usd.EditTarget(stage.GetRootLayer())):
                    stage.SetTimeCodesPerSecond(1 / sim_params["dt"])

            if "use_gpu_pipeline" in sim_params:
                self._carb_settings.set_bool("/physics/suppressReadback", sim_params["use_gpu_pipeline"])
                if sim_params["use_gpu_pipeline"]:
                    self._use_gpu_pipeline = True
            else:
                self._carb_settings.set_bool("/physics/suppressReadback", self._use_gpu_pipeline)

            if "worker_thread_count" in sim_params:
                self._carb_settings.set_int("/persistent/physics/numThreads", sim_params["worker_thread_count"])

            if "use_fabric" in sim_params and sim_params["use_fabric"]:
                self.enable_fabric(True)

            if "enable_scene_query_support" in sim_params:
                self.set_enable_scene_query_support(sim_params["enable_scene_query_support"])

            # GPU buffers
            if "gpu_max_rigid_contact_count" in sim_params:
                self.set_gpu_max_rigid_contact_count(sim_params["gpu_max_rigid_contact_count"])
            if "gpu_max_rigid_patch_count" in sim_params:
                self.set_gpu_max_rigid_patch_count(sim_params["gpu_max_rigid_patch_count"])
            if "gpu_found_lost_pairs_capacity" in sim_params:
                self.set_gpu_found_lost_pairs_capacity(sim_params["gpu_found_lost_pairs_capacity"])
            if "gpu_found_lost_aggregate_pairs_capacity" in sim_params:
                self.set_gpu_found_lost_aggregate_pairs_capacity(sim_params["gpu_found_lost_aggregate_pairs_capacity"])
            if "gpu_total_aggregate_pairs_capacity" in sim_params:
                self.set_gpu_total_aggregate_pairs_capacity(sim_params["gpu_total_aggregate_pairs_capacity"])
            if "gpu_max_soft_body_contacts" in sim_params:
                self.set_gpu_max_soft_body_contacts(sim_params["gpu_max_soft_body_contacts"])
            if "gpu_max_particle_contacts" in sim_params:
                self.set_gpu_max_particle_contacts(sim_params["gpu_max_particle_contacts"])
            if "gpu_heap_capacity" in sim_params:
                self.set_gpu_heap_capacity(sim_params["gpu_heap_capacity"])
            if "gpu_temp_buffer_capacity" in sim_params:
                self.set_gpu_temp_buffer_capacity(sim_params["gpu_temp_buffer_capacity"])
            if "gpu_max_num_partitions" in sim_params:
                self.set_gpu_max_num_partitions(sim_params["gpu_max_num_partitions"])
            if "gpu_collision_stack_size" in sim_params:
                self.set_gpu_collision_stack_size(sim_params["gpu_collision_stack_size"])
            if "solver_type" in sim_params:
                solver_val = sim_params["solver_type"]
                if isinstance(solver_val, str):
                    self.set_solver_type(solver_val)
                elif solver_val == 0:
                    self.set_solver_type("PGS")
                else:
                    self.set_solver_type("TGS")
            if "enable_stabilization" in sim_params:
                self.enable_stablization(sim_params["enable_stabilization"])
            if "bounce_threshold_velocity" in sim_params:
                self.set_bounce_threshold(sim_params["bounce_threshold_velocity"])
            if "friction_offset_threshold" in sim_params:
                self.set_friction_offset_threshold(sim_params["friction_offset_threshold"])
            if "friction_correlation_distance" in sim_params:
                self.set_friction_correlation_distance(sim_params["friction_correlation_distance"])

            # create default physics material
            if "default_physics_material" in sim_params:
                default_material_path = self._prim_path + "/defaultMaterial"
                default_material = UsdShade.Material.Define(get_current_stage(), default_material_path)
                mat = UsdPhysics.MaterialAPI.Apply(default_material.GetPrim())
                mat.CreateStaticFrictionAttr().Set(sim_params["default_physics_material"]["static_friction"])
                mat.CreateDynamicFrictionAttr().Set(sim_params["default_physics_material"]["dynamic_friction"])
                mat.CreateRestitutionAttr().Set(sim_params["default_physics_material"]["restitution"])
                # bind default physics material to scene
                material_api = UsdShade.MaterialBindingAPI.Apply(self._physics_scene.GetPrim())
                material_api.Bind(default_material, UsdShade.Tokens.weakerThanDescendants, "physics")

        if physics_dt is not None:
            self.set_physics_dt(dt=physics_dt)

        self._physx_fabric_interface = None

    @property
    def prim_path(self) -> str:
        """Path to the PhysicsScene prim in the USD stage.

        Returns:
            The absolute prim path of the PhysicsScene.

        """
        return self._prim_path

    @property
    def device(self) -> str:
        """Physics simulation device being used.

        Returns:
            The device name (e.g., 'cpu' or 'cuda').

        """
        return SimulationManager.get_physics_sim_device()

    @property
    def use_gpu_sim(self) -> bool:
        """Whether GPU simulation is enabled.

        Returns:
            True if using CUDA device for physics simulation, False otherwise.

        """
        return True if "cuda" in SimulationManager.get_physics_sim_device() else False

    @property
    def use_gpu_pipeline(self) -> bool:
        """Whether GPU pipeline is enabled for physics simulation.

        Returns:
            True if using CUDA device for physics simulation, False otherwise.

        """
        return True if "cuda" in SimulationManager.get_physics_sim_device() else False

    @property
    def use_fabric(self) -> bool:
        """Whether Fabric is enabled for physics simulation.

        Returns:
            True if Fabric is enabled, False otherwise.

        """
        return SimulationManager.is_fabric_enabled()

    def __del__(self) -> None:
        """Cleanup method called when the PhysicsContext instance is destroyed."""
        return

    def warm_start(self) -> None:
        """Deprecated method for physics simulation warm start.

        Note:
            This method is deprecated and no longer performs any operations.

        """
        carb.log_info("PhysicsContext.warm_start is deprecated.")
        return

    def get_current_physics_scene_prim(self) -> Usd.Prim | None:
        """Used to return the PhysicsScene prim in stage by traversing the stage.

        Returns:
            A PhysicsScene prim if found in current stage. Otherwise, None.

        """
        for prim in traverse_stage():
            if prim.HasAPI(PhysxSchema.PhysxSceneAPI) or prim.GetTypeName() == "PhysicsScene":
                return prim
        return None

    def _create_new_physics_scene(self, prim_path: str) -> UsdPhysics.Scene:
        """Create a new PhysicsScene prim at the specified path.

        Args:
            prim_path: The absolute prim path where the PhysicsScene will be created.

        Returns:
            The newly created UsdPhysics.Scene object.

        """
        carb.log_info(f"Defining a new Physics Scene at path `{prim_path}`")
        stage = get_current_stage()
        scene = UsdPhysics.Scene.Define(stage, prim_path)
        self._physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(get_prim_at_path(prim_path))
        return scene

    def set_physics_dt(self, dt: float = 1.0 / 60.0, substeps: int = 1) -> None:
        """Set the physics dt on the PhysicsScene.

        Args:
            dt: Physics dt.
            substeps: Number of physics steps to run for before rendering a frame.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
            ValueError: Physics dt must be a >= 0.
            ValueError: Physics dt must be a <= 1.0.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if dt < 0:
            raise ValueError("physics dt cannot be <0")
        # if no stage or no change in physics timestep, exit.
        if get_current_stage() is None:
            return
        # if physics substeps is not valid, make default = 1.
        if substeps is None or substeps <= 1:
            substeps = 1
        if dt == 0:
            self._physx_scene_api.GetTimeStepsPerSecondAttr().Set(0)
            min_steps = 0
        elif dt > 1.0:
            raise ValueError("physics dt must be <= 1.0")
        else:
            steps_per_second = int(1.0 / dt)
            min_steps = int(steps_per_second / substeps)
            self._physx_scene_api.GetTimeStepsPerSecondAttr().Set(steps_per_second)

        set_carb_setting(carb.settings.get_settings(), "persistent/simulation/minFrameRate", min_steps)
        return

    def get_physics_dt(self) -> float:
        """Current physics dt.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            Physics dt.

        """
        return SimulationManager.get_physics_dt()

    def enable_fabric(self, enable: bool) -> None:
        """Enable or disable fabric for physics simulation.

        Args:
            enable: Whether to enable fabric.

        """
        SimulationManager.enable_fabric(enable=enable)

    def enable_ccd(self, flag: bool) -> None:
        """Enable a second broad phase after integration that makes it possible to prevent objects from tunneling.

               through each other. If GPU is enabled, CCD is not supported and the request will be ignored. If CCD is enabled and then the GPU pipeline is requested, CCD will be disabled automatically.

        Args:
            flag: Enables or disables ccd on the PhysicsScene. CCD is not supported on GPU, so the request will be ignored if GPU is enabled.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        """
        SimulationManager.enable_ccd(flag=flag)

    def is_ccd_enabled(self) -> bool:
        """Check if ccd is enabled.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            True if ccd is enabled, otherwise False.

        """
        return SimulationManager.is_ccd_enabled()

    def enable_stabilization(self, flag: bool) -> None:
        """Enable additional stabilization pass in the solver.

        Args:
            flag: Enables or disables stabilization on the PhysicsScene

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetEnableStabilizationAttr().Get() is None:
            self._physx_scene_api.CreateEnableStabilizationAttr(flag)
        else:
            self._physx_scene_api.GetEnableStabilizationAttr().Set(flag)
        return

    def enable_stablization(self, flag: bool) -> None:
        """Enable additional stabilization pass in the solver.

        .. deprecated::
            Use :meth:`enable_stabilization` instead.

        Args:
            flag: Enables or disables stabilization on the PhysicsScene

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        """
        return self.enable_stabilization(flag)

    def is_stablization_enabled(self) -> bool:
        """Check if stabilization is enabled.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            True if stabilization is enabled, otherwise False.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetEnableStabilizationAttr().Get()

    def enable_gpu_dynamics(self, flag: bool) -> None:
        """Enable gpu dynamics pipeline, required for deformables for instance.

        Args:
            flag: Enables or disables gpu dynamics on the PhysicsScene

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        """
        SimulationManager.enable_gpu_dynamics(flag=flag)

    def is_gpu_dynamics_enabled(self) -> bool:
        """Check if Gpu Dynamics is enabled.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            True if Gpu Dynamics is enabled, otherwise False.

        """
        return SimulationManager.is_gpu_dynamics_enabled()

    def set_broadphase_type(self, broadcast_type: str) -> None:
        """Broadphase algorithm used in simulation.

        Args:
            broadcast_type: Broadphase algorithm type (e.g. "MBP", "GPU", "SAP").

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        """
        SimulationManager.set_broadphase_type(val=broadcast_type)
        return

    def get_broadphase_type(self) -> str:
        """Current broadphase algorithm type.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            Broadphase algorithm used.

        """
        return SimulationManager.get_broadphase_type()

    def set_solver_type(self, solver_type: str) -> None:
        """Solver used for simulation.

        Args:
            solver_type: can be "TGS" or "PGS".

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
            ValueError: If solver_type is not "TGS" or "PGS".

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if solver_type not in ("TGS", "PGS"):
            raise ValueError(f"solver_type must be 'TGS' or 'PGS', got '{solver_type}'")
        if self._physx_scene_api.GetSolverTypeAttr().Get() is None:
            self._physx_scene_api.CreateSolverTypeAttr(solver_type)
        else:
            self._physx_scene_api.GetSolverTypeAttr().Set(solver_type)
        return

    def get_solver_type(self) -> str:
        """Get current solver type.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            solver used for simulation.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetSolverTypeAttr().Get()

    def set_gravity(self, value: float) -> None:
        """Set the gravity direction and magnitude.

        Args:
            value: gravity value to be used in simulation.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if value <= 0:
            z_dir = -1
            magnitude = abs(value)
        else:
            z_dir = 1
            magnitude = value
        up_axis = UsdGeom.GetStageUpAxis(get_current_stage())
        gravity_dir = Gf.Vec3f(0.0)
        gravity_dir[AXES_INDICES[up_axis]] = z_dir
        if self._physics_scene.GetGravityDirectionAttr().Get() is None:
            self._physics_scene.CreateGravityDirectionAttr(gravity_dir)
        else:
            self._physics_scene.GetGravityDirectionAttr().Set(gravity_dir)

        if self._physics_scene.GetGravityMagnitudeAttr().Get() is None:
            self._physics_scene.CreateGravityMagnitudeAttr(magnitude)
        else:
            self._physics_scene.GetGravityMagnitudeAttr().Set(magnitude)
        return

    def get_gravity(self) -> tuple[list, float]:
        """Get current gravity.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            A tuple, first element corresponds to the gravity direction vector and second element is the magnitude.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        direction = self._physics_scene.GetGravityDirectionAttr().Get()
        magnitude = self._physics_scene.GetGravityMagnitudeAttr().Get()
        if direction is None:
            direction = Gf.Vec3f(0.0, 0.0, -1.0)
        if magnitude is None:
            magnitude = 0.0
        return (list(direction), magnitude)

    def set_physx_update_transformations_settings(
        self,
        update_to_usd: bool | None = None,
        update_velocities_to_usd: bool | None = None,
        output_velocities_local_space: bool | None = None,
    ) -> None:
        """Set how physx syncs with the usd when transformations are updated.

        Args:
            update_to_usd: Updates to USD the transformations.
            update_velocities_to_usd: Updates Velocities to USD.
            output_velocities_local_space: Output the velocities in the local frame and not the world frame.

        """
        if update_to_usd is not None:
            set_carb_setting(self._carb_settings, "/physics/updateToUsd", update_to_usd)
        if update_velocities_to_usd is not None:
            set_carb_setting(self._carb_settings, "/physics/updateVelocitiesToUsd", update_velocities_to_usd)
        if output_velocities_local_space is not None:
            set_carb_setting(self._carb_settings, "/physics/outputVelocitiesLocalSpace", output_velocities_local_space)
        return

    def get_physx_update_transformations_settings(self) -> tuple[bool, bool, bool]:
        """Get how physx syncs with the usd when transformations are updated.

        Returns:
            [update_to_usd, update_velocities_to_usd, output_velocities_local_space]

        """
        return (
            get_carb_setting(self._carb_settings, "/physics/updateToUsd"),
            get_carb_setting(self._carb_settings, "/physics/updateVelocitiesToUsd"),
            get_carb_setting(self._carb_settings, "/physics/outputVelocitiesLocalSpace"),
        )

    def _step(self, current_time: float, update_fabric: bool = False) -> None:
        """Executes a single physics simulation step.

        Args:
            current_time: The current simulation time.
            update_fabric: Whether to update the fabric interface after simulation.

        """
        self._physics_sim_interface.simulate(self.get_physics_dt(), current_time)
        if update_fabric:
            if self._physx_fabric_interface is None:
                if omni.kit.app.get_app().get_extension_manager().is_extension_enabled("omni.physx.fabric"):
                    from omni.physxfabric import get_physx_fabric_interface

                    self._physx_fabric_interface = get_physx_fabric_interface()
            if self._physx_fabric_interface is not None:
                self._physx_fabric_interface.update(current_time, self.get_physics_dt())

    def set_invert_collision_group_filter(self, invert_collision_group_filter: bool) -> None:
        """Set whether to invert the collision group filter.

        Args:
            invert_collision_group_filter: Whether to invert collision group filtering.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetInvertCollisionGroupFilterAttr().Get() is None:
            self._physx_scene_api.CreateInvertCollisionGroupFilterAttr(invert_collision_group_filter)
        else:
            self._physx_scene_api.GetInvertCollisionGroupFilterAttr().Set(invert_collision_group_filter)
        return

    def get_invert_collision_group_filter(self) -> int:
        """Get whether collision group filter is inverted.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            Whether collision group filtering is inverted.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetInvertCollisionGroupFilterAttr().Get()

    def set_bounce_threshold(self, value: float) -> None:
        """Set the bounce threshold for contact resolution.

        Args:
            value: The bounce threshold value.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetBounceThresholdAttr().Get() is None:
            self._physx_scene_api.CreateBounceThresholdAttr(value)
        else:
            self._physx_scene_api.GetBounceThresholdAttr().Set(value)
        return

    def get_bounce_threshold(self) -> float:
        """Bounce threshold for contact resolution.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The current bounce threshold value.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetBounceThresholdAttr().Get()

    def set_friction_offset_threshold(self, value: float) -> None:
        """Set the friction offset threshold.

        Args:
            value: The friction offset threshold value.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetFrictionOffsetThresholdAttr().Get() is None:
            self._physx_scene_api.CreateFrictionOffsetThresholdAttr(value)
        else:
            self._physx_scene_api.GetFrictionOffsetThresholdAttr().Set(value)
        return

    def get_friction_offset_threshold(self) -> float:
        """Get the friction offset threshold.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The current friction offset threshold value.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetFrictionOffsetThresholdAttr().Get()

    def set_friction_correlation_distance(self, value: float) -> None:
        """Set the friction correlation distance.

        Args:
            value: The friction correlation distance value.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetFrictionCorrelationDistanceAttr().Get() is None:
            self._physx_scene_api.CreateFrictionCorrelationDistanceAttr(value)
        else:
            self._physx_scene_api.GetFrictionCorrelationDistanceAttr().Set(value)
        return

    def get_friction_correlation_distance(self) -> float:
        """Get the friction correlation distance.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The current friction correlation distance value.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetFrictionCorrelationDistanceAttr().Get()

    def set_enable_scene_query_support(self, enable_scene_query_support: bool) -> None:
        """Set the Enable Scene Query Support attribute in Physx Scene.

        Args:
            enable_scene_query_support: Whether to enable scene query support

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetEnableSceneQuerySupportAttr().Get() is None:
            self._physx_scene_api.CreateEnableSceneQuerySupportAttr(enable_scene_query_support)
        else:
            self._physx_scene_api.GetEnableSceneQuerySupportAttr().Set(enable_scene_query_support)
        return

    def get_enable_scene_query_support(self) -> bool:
        """Enable Scene Query Support attribute in Physx Scene.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            Enable scene query support attribute.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetEnableSceneQuerySupportAttr().Get()

    def set_gpu_max_rigid_contact_count(self, value: int) -> None:
        """Set the maximum number of rigid body contacts on GPU.

        Args:
            value: The maximum rigid contact count.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuMaxRigidContactCountAttr().Get() is None:
            self._physx_scene_api.CreateGpuMaxRigidContactCountAttr(value)
        else:
            self._physx_scene_api.GetGpuMaxRigidContactCountAttr().Set(value)
        return

    def get_gpu_max_rigid_contact_count(self) -> int:
        """Get the maximum number of rigid body contacts on GPU.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The maximum rigid contact count.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuMaxRigidContactCountAttr().Get()

    def set_gpu_max_rigid_patch_count(self, value: int) -> None:
        """Set the maximum number of rigid body contact patches on GPU.

        Args:
            value: The maximum rigid patch count.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuMaxRigidPatchCountAttr().Get() is None:
            self._physx_scene_api.CreateGpuMaxRigidPatchCountAttr(value)
        else:
            self._physx_scene_api.GetGpuMaxRigidPatchCountAttr().Set(value)
        return

    def get_gpu_max_rigid_patch_count(self) -> int:
        """Get the maximum number of rigid body contact patches on GPU.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The maximum rigid patch count.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuMaxRigidPatchCountAttr().Get()

    def set_gpu_found_lost_pairs_capacity(self, value: int) -> None:
        """Set the GPU capacity for found/lost contact pairs.

        Args:
            value: The found/lost pairs capacity.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuFoundLostPairsCapacityAttr().Get() is None:
            self._physx_scene_api.CreateGpuFoundLostPairsCapacityAttr(value)
        else:
            self._physx_scene_api.GetGpuFoundLostPairsCapacityAttr().Set(value)
        return

    def get_gpu_found_lost_pairs_capacity(self) -> int:
        """Get the GPU capacity for found/lost contact pairs.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The found/lost pairs capacity.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuFoundLostPairsCapacityAttr().Get()

    def set_gpu_found_lost_aggregate_pairs_capacity(self, value: int) -> None:
        """Set the GPU capacity for found/lost aggregate contact pairs.

        Args:
            value: The found/lost aggregate pairs capacity.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuFoundLostAggregatePairsCapacityAttr().Get() is None:
            self._physx_scene_api.CreateGpuFoundLostAggregatePairsCapacityAttr(value)
        else:
            self._physx_scene_api.GetGpuFoundLostAggregatePairsCapacityAttr().Set(value)
        return

    def get_gpu_found_lost_aggregate_pairs_capacity(self) -> int:
        """Get the GPU capacity for found/lost aggregate contact pairs.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The found/lost aggregate pairs capacity.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuFoundLostAggregatePairsCapacityAttr().Get()

    def set_gpu_total_aggregate_pairs_capacity(self, value: int) -> None:
        """Set the GPU capacity for total aggregate contact pairs.

        Args:
            value: The total aggregate pairs capacity.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuTotalAggregatePairsCapacityAttr().Get() is None:
            self._physx_scene_api.CreateGpuTotalAggregatePairsCapacityAttr(value)
        else:
            self._physx_scene_api.GetGpuTotalAggregatePairsCapacityAttr().Set(value)
        return

    def get_gpu_total_aggregate_pairs_capacity(self) -> int:
        """Get the GPU capacity for total aggregate contact pairs.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The total aggregate pairs capacity.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuTotalAggregatePairsCapacityAttr().Get()

    def set_gpu_max_soft_body_contacts(self, value: int) -> None:
        """Set the maximum number of soft body contacts on GPU.

        Args:
            value: The maximum soft body contacts count.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuMaxDeformableVolumeContactsAttr().Get() is None:
            self._physx_scene_api.CreateGpuMaxDeformableVolumeContactsAttr(value)
        else:
            self._physx_scene_api.GetGpuMaxDeformableVolumeContactsAttr().Set(value)
        return

    def get_gpu_max_soft_body_contacts(self) -> int:
        """Get the maximum number of soft body contacts on GPU.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The maximum soft body contacts count.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuMaxDeformableVolumeContactsAttr().Get()

    def set_gpu_max_particle_contacts(self, value: int) -> None:
        """Set the maximum number of particle contacts on GPU.

        Args:
            value: The maximum particle contacts count.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuMaxParticleContactsAttr().Get() is None:
            self._physx_scene_api.CreateGpuMaxParticleContactsAttr(value)
        else:
            self._physx_scene_api.GetGpuMaxParticleContactsAttr().Set(value)
        return

    def get_gpu_max_particle_contacts(self) -> int:
        """Get the maximum number of particle contacts on GPU.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The maximum particle contacts count.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuMaxParticleContactsAttr().Get()

    def set_gpu_heap_capacity(self, value: int) -> None:
        """Set the GPU heap capacity for physics simulation.

        Args:
            value: The GPU heap capacity in bytes.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuHeapCapacityAttr().Get() is None:
            self._physx_scene_api.CreateGpuHeapCapacityAttr(value)
        else:
            self._physx_scene_api.GetGpuHeapCapacityAttr().Set(value)
        return

    def get_gpu_heap_capacity(self) -> int:
        """Get the GPU heap capacity for physics simulation.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The GPU heap capacity in bytes.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuHeapCapacityAttr().Get()

    def set_gpu_temp_buffer_capacity(self, value: int) -> None:
        """Set the GPU temporary buffer capacity.

        Args:
            value: The GPU temp buffer capacity in bytes.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuTempBufferCapacityAttr().Get() is None:
            self._physx_scene_api.CreateGpuTempBufferCapacityAttr(value)
        else:
            self._physx_scene_api.GetGpuTempBufferCapacityAttr().Set(value)
        return

    def get_gpu_temp_buffer_capacity(self) -> int:
        """Get the GPU temporary buffer capacity.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The GPU temp buffer capacity in bytes.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuTempBufferCapacityAttr().Get()

    def set_gpu_max_num_partitions(self, value: int) -> None:
        """Set the maximum number of GPU partitions for simulation.

        Args:
            value: The maximum number of partitions.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuMaxNumPartitionsAttr().Get() is None:
            self._physx_scene_api.CreateGpuMaxNumPartitionsAttr(value)
        else:
            self._physx_scene_api.GetGpuMaxNumPartitionsAttr().Set(value)
        return

    def get_gpu_max_num_partitions(self) -> int:
        """Get the maximum number of GPU partitions for simulation.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The maximum number of partitions.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuMaxNumPartitionsAttr().Get()

    def set_gpu_collision_stack_size(self, value: int) -> None:
        """Set the GPU collision stack size.

        Args:
            value: The collision stack size.

        Raises:
            Exception: If the physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        if self._physx_scene_api.GetGpuCollisionStackSizeAttr().Get() is None:
            self._physx_scene_api.CreateGpuCollisionStackSizeAttr(value)
        else:
            self._physx_scene_api.GetGpuCollisionStackSizeAttr().Set(value)
        return

    def get_gpu_collision_stack_size(self) -> int:
        """Get the GPU collision stack size.

        Raises:
            Exception: If the physics scene path is invalid.

        Returns:
            The collision stack size.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        return self._physx_scene_api.GetGpuCollisionStackSizeAttr().Get()

    def set_solve_articulation_contact_last(self, solve_articulation_contact_last: bool) -> None:
        """Set the ``solveArticulationContactLast`` state in PhysX scene.

        When enabled, the solver orders the articulation contact constraints and the articulation joint maximum
        velocity constraints to be solved after all the other constraints.

        Args:
            solve_articulation_contact_last: Whether to reorder the constraints to be solved last.

        Raises:
            Exception: The physics scene path is invalid.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        attribute_name = "physxScene:solveArticulationContactLast"
        attribute = self._physx_scene_api.GetPrim().GetAttribute(attribute_name)
        if attribute.Get() is None:
            attribute = self._physx_scene_api.GetPrim().CreateAttribute(attribute_name, Sdf.ValueTypeNames.Bool, False)
        attribute.Set(solve_articulation_contact_last)

    def get_solve_articulation_contact_last(self) -> bool:
        """Retrieves the ``solveArticulationContactLast`` state in PhysX scene.

        Raises:
            Exception: The physics scene path is invalid.

        Returns:
            Whether the articulation contact constraints and the articulation joint maximum velocity constraints
            are ordered to be solved last.

        """
        if not is_prim_path_valid(self._prim_path):
            raise Exception("The Physics Context's physics scene path is invalid, you need to reinit Physics Context")
        attribute_name = "physxScene:solveArticulationContactLast"
        attribute = self._physx_scene_api.GetPrim().GetAttribute(attribute_name)
        if attribute.Get() is None:
            attribute = self._physx_scene_api.GetPrim().CreateAttribute(attribute_name, Sdf.ValueTypeNames.Bool, False)
        return attribute.Get()
