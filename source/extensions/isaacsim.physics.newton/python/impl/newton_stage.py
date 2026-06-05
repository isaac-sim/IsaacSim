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

"""Newton simulation stage management."""

from __future__ import annotations

import carb
import newton
import omni.timeline
import omni.usd
import usdrt
import warp as wp
from pxr import Usd

from .fabric import FabricManager
from .newton_config import NewtonConfig


class NewtonStage:
    """Newton simulation stage that manages physics simulation state and stepping.

    Args:
        cfg: Configuration for Newton simulation.
        device: Device to run simulation on.
    """

    def __init__(self, cfg: NewtonConfig | None = None, device: str | None = None) -> None:
        # Default to a generic NewtonConfig unless provided
        self.cfg = cfg or NewtonConfig()

        # Sync config to carb settings
        carb.settings.get_settings().set(
            "/exts/isaacsim.physics.newton/capture_graph_physics_step", self.cfg.use_cuda_graph
        )

        self.init()
        timeline_events = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self.timeline_event_sub = timeline_events.create_subscription_to_pop(self.on_timeline_event)

        # Use provided device or fall back to warp's current device
        # Store both device string and device object
        if device is not None:
            self.device_str = device if isinstance(device, str) else str(device)
            self.device = wp.get_device(self.device_str)
        else:
            self.device = wp.get_device()
            self.device_str = str(self.device)

        self.playing = False
        self.sim_time = 0
        self.sim_dt = 1.0 / self.cfg.physics_frequency
        self.time_code_per_second = 24
        self.physics_callbacks: list = []

        # Simulation tracking for unified physics interface
        self.simulation_timestamp = 0
        self.simulation_step_count = 0
        self.stage_id = None

    def init(self) -> None:
        """Reset simulation state to initial values."""
        self.initialized = False
        self._initializing = False  # Reentrant guard
        self._init_failed = False
        self.builder = None
        self.model = None
        self.state_0 = None
        self.state_1 = None
        self.state_temp = None
        self.graph = None
        self._kernels_compiled = False
        self.q_ik = None
        self.qd_ik = None
        self.joint_torques = None
        self.stage = None
        self.usd_stage = None
        self.simulation_timestamp = 0
        self.simulation_step_count = 0

    def on_timeline_event(self, e: omni.timeline.TimelineEventType) -> None:
        """Handle timeline events (play, stop, pause).

        Args:
            e: Timeline event.
        """
        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            self.time_code_per_second = omni.timeline.get_timeline_interface().get_time_codes_per_seconds()
            self.playing = True
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self.playing = False
            self.graph = None
            self._init_failed = False
            self._restore_fabric_transforms()
        if e.type == int(omni.timeline.TimelineEventType.PAUSE):
            self.playing = False
            self.graph = None
        if e.type == int(omni.timeline.TimelineEventType.CURRENT_TIME_CHANGED):
            pass

    @classmethod
    def _get_solver(
        cls, model: newton.Model, solver_cfg: "NewtonConfig.Solver"  # type: ignore[name-defined]
    ) -> newton.solvers.SolverXPBD | newton.solvers.SolverMuJoCo:
        """Get solver instance from configuration.

        Args:
            model: Newton model instance.
            solver_cfg: Solver configuration object.

        Returns:
            Configured solver instance.

        Raises:
            ValueError: If an invalid solver type is specified.
        """
        # Convert dataclass to dict for **kwargs expansion
        if hasattr(solver_cfg, "__dict__"):
            solver_dict = solver_cfg.__dict__.copy()
        else:
            solver_dict = dict(solver_cfg)

        solver_type = solver_dict.pop("solver_type")
        # Remove simulation-level parameters that are not solver parameters
        solver_dict.pop("num_substeps", None)
        if solver_type == "xpbd":
            return newton.solvers.SolverXPBD(model, **solver_dict)
        elif solver_type == "mujoco":
            return newton.solvers.SolverMuJoCo(model, **solver_dict)
        else:
            raise ValueError(f"Invalid solver type: {solver_type}")

    @carb.profiler.profile
    def on_update(self, event: "carb.events.IEvent", dt: float) -> None:
        """Update callback for stage update events.

        Args:
            event: Unused parameter (event).
            dt: Time delta since last update.
        """
        # If timeline is playing but our flag isn't set yet, sync it here
        try:
            if not self.playing and omni.timeline.get_timeline_interface().is_playing():
                self.playing = True
        except Exception:
            pass

        if not self.playing:
            return

        final_time = self.sim_time + dt
        if self.cfg.time_step_app:
            # Use half of sim_dt as tolerance to avoid floating point precision issues
            eps = self.sim_dt * 0.5
            substep_count = 0
            while self.sim_time + eps < final_time:
                if hasattr(self, "simulation_functions") and self.simulation_functions:
                    self.simulation_functions.simulate(self.sim_dt, self.sim_time)
                else:
                    self.step_sim(self.sim_dt)
                substep_count += 1

        self.update_fabric()

    @carb.profiler.profile
    def step_sim(self, dt: float) -> None:
        """Step the simulation by the given time delta.

        Args:
            dt: Time delta for this step.
        """
        if not self.initialized:
            if not self.playing and not omni.timeline.get_timeline_interface().is_playing():
                return
            self.initialize_newton(self.device)  # type: ignore[arg-type]

        self.sim_time += dt  # type: ignore[assignment]
        self.simulation_timestamp += 1  # type: ignore[assignment]
        self.simulation_step_count += 1
        if self.playing:
            use_cuda_graph = (
                self.cfg.use_cuda_graph and self.device.is_cuda
                if hasattr(self.device, "is_cuda")
                else "cuda" in str(self.device)
            )

            if use_cuda_graph and self.initialized:
                if not self._kernels_compiled:
                    # Run the first step without graph capture so that
                    # dynamically-generated solver kernels (tile_matmul,
                    # tile_cholesky with model-specific tile sizes) are
                    # compiled with full LTO retry support.  Capturing
                    # this step would record a graph with failed kernels.
                    self._kernels_compiled = True
                    self.simulate(dt=dt)
                elif self.graph is None:
                    wp.capture_begin()
                    try:
                        self.simulate(dt=dt)
                    finally:
                        self.graph = wp.capture_end()  # type: ignore[assignment]
                    wp.capture_launch(self.graph)  # type: ignore[arg-type]
                else:
                    wp.capture_launch(self.graph)
            else:
                self.simulate(dt=dt)

            for callback in self.physics_callbacks:
                if callback is not None:
                    callback(dt)

    def update_fabric(self) -> None:
        """Update Fabric attributes with current simulation state."""
        if self.playing and self.initialized:
            if self.cfg.disable_physx_fabric_tracker:
                try:
                    import importlib

                    omni_physx = importlib.import_module("omni.physx")
                    omni_physx.get_physx_simulation_interface().pause_change_tracking(True)
                except Exception:
                    pass
            self.update_fabric_attrs()

    def on_detach(self) -> None:
        """Handle stage detach event."""
        self._in_stage_transition = True
        self.init()

    def on_attach(self, stage_id: int, meters_per_unit: float) -> None:
        """Handle stage attach event.

        Args:
            stage_id: USD stage identifier.
            meters_per_unit: Scene scale in meters per unit.
        """
        if not hasattr(self, "device_str"):
            self.device = wp.get_device()
            self.device_str = str(self.device)
        else:
            try:
                self.device = wp.get_device(self.device_str)
            except Exception:
                self.device = wp.get_device()
                self.device_str = str(self.device)
        self.initialized = False
        self._in_stage_transition = False
        self.simulation_timestamp = 0
        self.simulation_step_count = 0
        self.sim_time = 0.0  # type: ignore[assignment]

    def on_resume(self, currentTime: float) -> None:
        """Handle simulation resume event.

        Args:
            currentTime: Current simulation time.
        """

    def on_change(self, path: str) -> None:
        """Handle USD prim change event.

        Args:
            path: Path of the changed prim.
        """
        self.initialized = False

    def update_fabric_attrs(self) -> None:
        """Update Fabric attributes from Newton state."""
        if self.cfg.update_fabric and self.model:
            self.fabric_manager.update_fabric(  # type: ignore[has-type]
                self.model,
                self.state_0,
                self.scene_scale,  # type: ignore[has-type]
                self.device,
            )

    def _restore_fabric_transforms(self) -> None:
        """Restore Fabric body transforms to the initial USD state.

        Called on timeline STOP to ensure nested rigid body hierarchies return
        to their initial poses. Without this, Fabric hierarchy propagation
        recomputes world matrices from stale local matrices, producing wrong
        transforms for nested bodies.
        """
        if not self.initialized or not self.cfg.update_fabric or self.model is None:
            return

        if not hasattr(self, "stage") or self.stage is None:
            return

        for path in self.model.body_label:
            prim = self.stage.GetPrimAtPath(usdrt.Sdf.Path(path))
            if not prim:
                continue
            xformable = usdrt.Rt.Xformable(prim)
            xformable.SetWorldXformFromUsd()

    def initialize_newton(self, device: str | None) -> None:
        """Initialize Newton simulation from the current USD stage.

        Args:
            device: Device to run simulation on.
        """
        if getattr(self, "_in_stage_transition", False):
            return

        if getattr(self, "_initializing", False):
            return

        if getattr(self, "_init_failed", False):
            return

        device_changed = False
        if device is not None:
            new_device_str = device if isinstance(device, str) else str(device)
            if hasattr(self, "device_str") and self.device_str != new_device_str and self.initialized:
                carb.log_warn(f"[Newton] Device changed from {self.device_str} to {new_device_str}, reinitializing...")
                self.init()
                device_changed = True
            self.device_str = new_device_str
            self.device = wp.get_device(self.device_str)
        else:
            self.device_str = str(self.device)

        if self.initialized and not device_changed:
            return

        try:
            self._initialize_newton_impl()
        except Exception as e:
            carb.log_error(f"[Newton] Initialization failed: {e}")
            self.init()
            self._init_failed = True

    def _initialize_newton_impl(self) -> None:
        """Internal implementation of Newton initialization."""
        self._initializing = True
        wp.set_device(self.device)

        current_stage: Usd.Stage = omni.usd.get_context().get_stage()
        if current_stage is None:
            carb.log_warn("[Newton] No USD stage available, skipping initialization")
            self._initializing = False
            return
        self.usd_stage = current_stage
        self.stage_id = omni.usd.get_context().get_stage_id()
        if self.stage_id is None or self.stage_id < 0:
            carb.log_warn(f"[Newton] Invalid stage ID ({self.stage_id}), skipping initialization")
            self._initializing = False
            return
        usdrt_stage = usdrt.Usd.Stage.Attach(self.stage_id)
        self.fabric_manager = FabricManager(usdrt_stage)

        if self.cfg.disable_physx_fabric_tracker:
            try:
                import importlib

                omni_physx = importlib.import_module("omni.physx")
                omni_physx.get_physx_simulation_interface().pause_change_tracking(True)
            except Exception:
                pass

        use_warp_cloner = False
        cloned_env_prim = current_stage.GetPrimAtPath("/World/envs/env_0")
        if cloned_env_prim:
            use_warp_cloner = True

        self.builder = newton.ModelBuilder()
        self.builder.validate_inertia_detailed = True

        # Set default contact and joint properties from config
        self.builder.default_shape_cfg.ke = self.cfg.contact_ke
        self.builder.default_shape_cfg.kd = self.cfg.contact_kd
        self.builder.default_shape_cfg.kf = self.cfg.contact_kf
        self.builder.default_shape_cfg.ka = self.cfg.contact_ka
        self.builder.default_shape_cfg.mu = self.cfg.contact_mu
        self.builder.default_shape_cfg.restitution = self.cfg.restitution

        self.builder.default_joint_cfg.limit_ke = self.cfg.joint_limit_ke
        self.builder.default_joint_cfg.limit_kd = self.cfg.joint_limit_kd
        self.builder.default_joint_cfg.armature = self.cfg.armature
        from newton._src.usd.schemas import (
            SchemaResolverMjc,
            SchemaResolverNewton,
            SchemaResolverPhysx,
        )

        solver_type = self.cfg.solver_cfg.solver_type
        if solver_type == "mujoco":
            newton.solvers.SolverMuJoCo.register_custom_attributes(self.builder)
        elif solver_type == "xpbd":
            newton.solvers.SolverXPBD.register_custom_attributes(self.builder)

        # Parse USD using Newton API
        add_usd_kwargs = {
            "verbose": False,
            "collapse_fixed_joints": self.cfg.collapse_fixed_joints,
            "joint_drive_gains_scaling": self.cfg.pd_scale,
            "only_load_enabled_rigid_bodies": True,
            "schema_resolvers": [SchemaResolverNewton(), SchemaResolverMjc(), SchemaResolverPhysx()],
            "force_position_velocity_actuation": True,
        }
        try:
            self.parsing_results = self.builder.add_usd(source=current_stage, **add_usd_kwargs)
        except RuntimeError as e:
            if "Cycle detected" in str(e):
                carb.log_warn("[Newton] USD stage has composition cycles; retrying with flattened stage")
                flat_stage = Usd.Stage.CreateInMemory()
                flat_stage.GetRootLayer().TransferContent(current_stage.Flatten())
                self.parsing_results = self.builder.add_usd(source=flat_stage, **add_usd_kwargs)
            else:
                raise

        self.scene_scale = 1.0 / self.parsing_results["linear_unit"]

        # Get physics timestep from parser results, fall back to config
        physics_dt = self.parsing_results.get("physics_dt")
        if physics_dt is not None and physics_dt > 0:
            self.sim_dt = physics_dt
            self.physics_frequency = 1.0 / physics_dt
            carb.log_info(f"[Newton] Using physics timestep from USD: {self.physics_frequency} Hz (dt={self.sim_dt})")
        else:
            self.physics_frequency = self.cfg.physics_frequency
            self.sim_dt = 1.0 / self.physics_frequency
            carb.log_info(
                f"[Newton] Using physics timestep from config: {self.physics_frequency} Hz (dt={self.sim_dt})"
            )

        if self.builder.body_count == 0:
            self.init()
            self.initialized = True
            self._initializing = False
            return

        self.model = self.builder.finalize(self.device_str)

        self.control = self.model.control()
        self.model.ground = True
        self.model.request_contact_attributes("force")
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        if self.cfg.use_cuda_graph:
            self.state_temp = self.model.state()

        self.contacts = self.model.collide(self.state_0)

        newton.eval_fk(self.model, self.state_0.joint_q, self.state_0.joint_qd, self.state_0, None)

        self.solver = self._get_solver(self.model, self.cfg.solver_cfg)

        self.initial_body_q = self.state_0.body_q.numpy().copy()
        self.initial_body_qd = self.state_0.body_qd.numpy().copy()

        self.q_ik = self.model.joint_q
        self.qd_ik = self.model.joint_qd
        self.joint_torques = wp.zeros(self.model.joint_dof_count, dtype=wp.float32)

        valid_body_paths = set(self.model.body_label)
        self.fabric_manager.cleanup_stale_newton_index(valid_body_paths, self.device)

        for i, path in enumerate(self.model.body_label):
            prim = usdrt_stage.GetPrimAtPath(usdrt.Sdf.Path(path))
            if not prim:
                continue
            prim.CreateAttribute(self.fabric_manager.newton_index_attr, usdrt.Sdf.ValueTypeNames.UInt, True)
            prim.GetAttribute(self.fabric_manager.newton_index_attr).Set(i)
            xformable_prim = usdrt.Rt.Xformable(prim)
            if not xformable_prim.HasWorldXform():
                xformable_prim.SetWorldXformFromUsd()

        self.fabric_manager.update_fabric(
            self.model,
            self.state_0,
            self.scene_scale,
            self.device,
        )

        self.stage = usdrt_stage
        self.sim_time = 0.0
        self.graph = None
        self.initialized = True
        self._initializing = False

    def simulate(self, num_substeps: int | None = None, dt: float | None = None) -> None:
        """Simulate the world with the given number of substeps.

        Args:
            num_substeps: Number of substeps.
            dt: Physics timestep.
        """
        if num_substeps is None:
            num_substeps = getattr(self.cfg, "num_substeps", 1)

        step_dt = dt if dt is not None else self.sim_dt

        if self.model is None or self.state_0 is None or self.state_1 is None:
            return

        if not hasattr(self, "solver") or self.solver is None:  # type: ignore[has-type]
            return

        state_0_dict = None
        state_1_dict = None
        state_temp_dict = None
        if self.cfg.use_cuda_graph:
            if self.state_temp is None:
                return
            state_0_dict = self.state_0.__dict__
            state_1_dict = self.state_1.__dict__
            state_temp_dict = self.state_temp.__dict__

        solver_type = self.cfg.solver_cfg.solver_type

        if solver_type == "mujoco":
            use_mujoco_native_contacts = getattr(self.cfg.solver_cfg, "use_mujoco_contacts", False)
            if not use_mujoco_native_contacts:
                self.contacts = self.model.collide(self.state_0, self.contacts)  # type: ignore[has-type]
            contacts = self.contacts  # type: ignore[has-type]
        else:
            contacts = self.model.collide(self.state_0)
            self.contacts = contacts  # type: ignore[has-type]

        for i in range(num_substeps):
            self.solver.step(self.state_0, self.state_1, self.control, contacts, step_dt / float(num_substeps))  # type: ignore[has-type]

            if i == num_substeps - 1 and self.cfg.use_cuda_graph and state_0_dict is not None:
                for key, value in state_0_dict.items():
                    if isinstance(value, wp.array):
                        state_temp_dict[key].assign(value)  # type: ignore[index]
                        state_0_dict[key].assign(state_1_dict[key])  # type: ignore[index]
                        state_1_dict[key].assign(state_temp_dict[key])  # type: ignore[index]
            else:
                self.state_0, self.state_1 = self.state_1, self.state_0

            self.state_0.clear_forces()

        if solver_type == "mujoco" and self.contacts is not None and not getattr(self.solver, "use_mujoco_cpu", False):  # type: ignore[has-type]
            self.solver.update_contacts(self.contacts, self.state_0)  # type: ignore[has-type]

        self.q_ik = self.model.joint_q
        self.qd_ik = self.model.joint_qd
