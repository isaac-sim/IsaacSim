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

import math

import carb
import newton
import omni.timeline
import omni.usd
import usdrt
import warp as wp
from pxr import Usd, UsdGeom, UsdPhysics

from . import carb_config
from ._warning_logging import _log_python_warnings
from .fabric import FabricManager
from .newton_config import NewtonConfig
from .solver_config import MuJoCoSolverConfig, NewtonSolverConfig, XPBDSolverConfig
from .usd import UsdManager, _get_world_scale
from .utils import newton_solver_to_api_schema, newton_solver_to_solver_config, newton_solver_to_solver_object


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
        self.physics_scene_prim = None

    def init(self) -> None:
        """Reset simulation state to initial values."""
        self.initialized = False
        self._initializing = False  # Reentrant guard
        self._init_failed = False
        self.builder = None
        self.model = None
        self.collision_pipeline = None
        self.contacts = None
        self.state_0 = None
        self.state_1 = None
        self.state_temp = None
        self.graph = None
        self._graph_capture_dt = None
        self._graph_capture_substeps = None
        self._kernels_compiled = False
        self.q_ik = None
        self.qd_ik = None
        self.joint_torques = None
        self.stage = None
        self.usd_stage = None
        # A manager is valid only for the currently initialized USD stage.
        self.usd_manager = None
        self.simulation_timestamp = 0
        self.simulation_step_count = 0
        self.physics_scene_prim = None

    def _get_physics_scene(self) -> None:
        """Select the first physics scene declaring one Newton solver API."""
        self.physics_scene_prim = None
        if self.usd_stage is None:
            return

        for physics_scene in newton.usd.get_physics_scenes(self.usd_stage):
            prim = physics_scene.GetPrim()
            if not prim or not prim.IsValid():
                continue
            solver_api_count = sum(prim.HasAPI(api) for api in newton_solver_to_api_schema.values())
            if solver_api_count == 1:
                self.physics_scene_prim = prim
                return

    def _get_solver_type(self) -> str | None:
        if not self.physics_scene_prim:
            return None
        counter = 0
        for newton_solver in newton_solver_to_api_schema:
            if self.physics_scene_prim.HasAPI(newton_solver_to_api_schema[newton_solver]):
                solver = newton_solver
                counter = counter + 1
        if counter == 1:
            return solver
        if counter > 1:
            carb.log_error(f"[Newton] Multiple solver APIs detected on physics scene.")
        return None

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
            if carb_config.reset_on_stop():
                self._restore_fabric_transforms()
            else:
                self._write_usd_state_on_stop()
        if e.type == int(omni.timeline.TimelineEventType.PAUSE):
            self.playing = False
            self.graph = None
        if e.type == int(omni.timeline.TimelineEventType.CURRENT_TIME_CHANGED):
            pass

    def _write_usd_state_on_stop(self) -> None:
        """Preserve the final MuJoCo pose when reset-on-stop is disabled."""
        try:
            solver_type = str(self.cfg.solver_cfg.solver_type)
            if solver_type != "mujoco":
                carb.log_warn(
                    "[Newton] Preserving the simulated pose when Reset Simulation on Stop is disabled is supported "
                    f"only by the MuJoCo solver. The USD stage was not updated for the {solver_type.upper()} solver."
                )
                return

            current_stage = omni.usd.get_context().get_stage()
            current_stage_id = omni.usd.get_context().get_stage_id()
            if (
                not self.initialized
                or self.usd_manager is None
                or self.usd_manager.stage is not current_stage
                or self.stage_id != current_stage_id
                or self.model is None
                or self.state_0 is None
            ):
                carb.log_warn(
                    "[Newton] Final simulation pose was not written because the initialized USD stage is stale."
                )
                return

            self.usd_manager.write_newton_state(self.model, self.state_0, self.scene_scale)
        except Exception as exception:
            # Timeline callbacks must not propagate USD authoring failures into Kit's event dispatcher.
            carb.log_warn(f"[Newton] Final simulation pose was not written to USD: {exception}")

    @classmethod
    def _create_solver(
        cls, model: newton.Model, solver_cfg: "NewtonConfig.Solver"  # type: ignore[name-defined]
    ) -> newton.solvers.SolverXPBD | newton.solvers.SolverVBD | newton.solvers.SolverMuJoCo:
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
        if solver_type in newton_solver_to_solver_object:
            return newton_solver_to_solver_object[solver_type](model, **solver_dict)
        else:
            raise ValueError(f"Invalid solver type: {solver_type}")

    def _build_collision_pipeline(self, model: newton.Model) -> newton.CollisionPipeline:
        """Build the collision pipeline from runtime configuration.

        Args:
            model: Finalized Newton model.

        Returns:
            Configured collision pipeline instance.
        """
        from newton.geometry import HydroelasticSDF

        cc = self.cfg.collision_cfg
        flags = model.shape_flags.numpy()
        has_hydro = bool((flags & int(newton.ShapeFlags.HYDROELASTIC)).any())

        sdf_cfg = None
        if has_hydro and cc.hydroelastic.enabled:
            h = cc.hydroelastic
            sdf_cfg = HydroelasticSDF.Config(
                reduce_contacts=h.reduce_contacts,
                buffer_fraction=h.buffer_fraction,
                buffer_mult_broad=h.buffer_mult_broad,
                buffer_mult_iso=h.buffer_mult_iso,
                buffer_mult_contact=h.buffer_mult_contact,
                normal_matching=h.normal_matching,
                anchor_contact=h.anchor_contact,
                margin_contact_area=h.margin_contact_area,
                output_contact_surface=h.output_contact_surface,
                mc_edge_clamp_min=h.mc_edge_clamp_min,
            )

        # CollisionPipeline builds the hydroelastic narrow phase whenever any
        # shape carries the HYDROELASTIC flag, independent of
        # sdf_hydroelastic_config. Honor a runtime opt-out by masking the flag
        # for construction only, then restoring it so the parsed model is unchanged.
        saved_flags = None
        if has_hydro and not cc.hydroelastic.enabled:
            saved_flags = model.shape_flags
            hydro_bit = flags.dtype.type(int(newton.ShapeFlags.HYDROELASTIC))
            model.shape_flags = wp.array(
                flags & ~hydro_bit,
                dtype=saved_flags.dtype,
                device=saved_flags.device,
            )

        try:
            return newton.CollisionPipeline(
                model,
                rigid_contact_max=cc.rigid_contact_max,
                broad_phase=cc.broad_phase,
                sdf_hydroelastic_config=sdf_cfg,
            )
        finally:
            if saved_flags is not None:
                model.shape_flags = saved_flags

    @carb.profiler.profile
    def on_update(self, event: "carb.events.IEvent", dt: float) -> None:
        """Update callback for stage update events for simulation.

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
        # Sync playing flag when physics is stepped directly (e.g. SimulationManager.step)
        # without an app update to dispatch timeline PLAY events first.
        try:
            if not self.playing and omni.timeline.get_timeline_interface().is_playing():
                self.playing = True
        except Exception:
            pass

        if not self.initialized:
            self.initialize_newton(self.device)  # type: ignore[arg-type]

        self.sim_time += dt  # type: ignore[assignment]
        self.simulation_timestamp += 1  # type: ignore[assignment]
        self.simulation_step_count += 1

        use_cuda_graph = (
            self.cfg.use_cuda_graph and self.device.is_cuda
            if hasattr(self.device, "is_cuda")
            else "cuda" in str(self.device)
        )

        if use_cuda_graph and self.initialized:
            # The timestep and the substep count are baked into the captured graph, so the
            # graph must be recaptured whenever either changes; otherwise dynamics advance
            # with the captured values while bookkeeping advances with the current ones.
            num_substeps = getattr(self.cfg, "num_substeps", 1)
            graph_stale = self.graph is not None and (
                abs(dt - self._graph_capture_dt) > 1e-9 or num_substeps != self._graph_capture_substeps
            )
            if not self._kernels_compiled:
                # Run the first step without graph capture so that
                # dynamically-generated solver kernels (tile_matmul,
                # tile_cholesky with model-specific tile sizes) are
                # compiled with full LTO retry support.  Capturing
                # this step would record a graph with failed kernels.
                self._kernels_compiled = True
                self.simulate(num_substeps=num_substeps, dt=dt)
            elif self.graph is None or graph_stale:
                self._graph_capture_dt = dt
                self._graph_capture_substeps = num_substeps
                wp.capture_begin()
                try:
                    self.simulate(num_substeps=num_substeps, dt=dt)
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

    def _initialize_joint_state_from_body_poses(self) -> None:
        """Derive the initial joint state from the parsed body poses."""
        if self.model is None or self.state_0 is None or self.state_0.joint_q is None or self.state_0.joint_qd is None:
            return

        reference_joint_q = self.state_0.joint_q.numpy().copy()
        newton.eval_ik(self.model, self.state_0, self.state_0.joint_q, self.state_0.joint_qd, None)

        # IK may return an equivalent revolute angle on a different 2-pi branch.
        joint_q = self.state_0.joint_q.numpy()
        for joint_type, q_start in zip(self.model.joint_type.numpy(), self.model.joint_q_start.numpy()):
            if joint_type == newton.JointType.REVOLUTE:
                joint_q[q_start] += round((reference_joint_q[q_start] - joint_q[q_start]) / math.tau) * math.tau
        self.state_0.joint_q.assign(joint_q)
        newton.eval_fk(self.model, self.state_0.joint_q, self.state_0.joint_qd, self.state_0, None)

        # Keep future states and solver initialization consistent with the pose selected from USD.
        self.model.body_q.assign(self.state_0.body_q)
        self.model.body_qd.assign(self.state_0.body_qd)
        self.model.joint_q.assign(self.state_0.joint_q)
        self.model.joint_qd.assign(self.state_0.joint_qd)

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

        solver_changed = False
        # If initialized, we must have a valid usd_stage already
        if self.initialized:
            solver_type = self._get_solver_type()
            if not solver_type:
                carb.log_warn(f"[Newton] No valid physics scene with newton solver found, skipping initialization")
                self.init()
                return
            solver_changed = self.cfg.solver_cfg.solver_type != solver_type

        if self.initialized and not device_changed and not solver_changed:
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
        # Keep USD authoring separate from timeline and Newton-model lifecycle handling in this class.
        self.usd_manager = UsdManager(current_stage)
        self.stage_id = omni.usd.get_context().get_stage_id()

        # Solver type is stored in the scene prim, so we need to get it and set the solver_type
        self._get_physics_scene()
        solver_type = self._get_solver_type()
        if solver_type is None:
            carb.log_warn("[Newton] No valid physics scene with newton solver available, skipping initialization")
            self._initializing = False
            return
        if solver_type in newton_solver_to_solver_config:
            config_type = newton_solver_to_solver_config[solver_type]
            if not isinstance(self.cfg.solver_cfg, config_type):
                self.cfg.solver_cfg = config_type()
        self.cfg.solver_cfg.solver_type = solver_type

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
        # Universal attributes
        self.builder.default_shape_cfg.ke = self.cfg.contact_ke
        self.builder.default_shape_cfg.kd = self.cfg.contact_kd
        self.builder.default_shape_cfg.kf = self.cfg.contact_kf
        self.builder.default_shape_cfg.ka = self.cfg.contact_ka
        self.builder.default_shape_cfg.mu = self.cfg.contact_mu
        self.builder.default_shape_cfg.restitution = self.cfg.restitution

        self.builder.default_joint_cfg.limit_ke = self.cfg.joint_limit_ke
        self.builder.default_joint_cfg.limit_kd = self.cfg.joint_limit_kd
        self.builder.default_joint_cfg.armature = self.cfg.armature

        # register solver specific attributes from USD (parsed in builder)
        newton_solver_to_solver_object[solver_type].register_custom_attributes(self.builder)

        # Parse USD using Newton API
        add_usd_kwargs = {
            "verbose": False,
            "collapse_fixed_joints": self.cfg.collapse_fixed_joints,
            "joint_drive_gains_scaling": self.cfg.pd_scale,
            "only_load_enabled_rigid_bodies": True,
            "force_position_velocity_actuation": True,
        }
        from newton.usd import (
            SchemaResolverMjc,
            SchemaResolverNewton,
            SchemaResolverPhysx,
        )

        if solver_type == "mujoco":
            add_usd_kwargs["schema_resolvers"] = [SchemaResolverNewton(), SchemaResolverMjc(), SchemaResolverPhysx()]
        elif solver_type == "xpbd" or solver_type == "vbd":
            # having Mjc resolver will cause issues if it is not mujoco solver
            add_usd_kwargs["schema_resolvers"] = [SchemaResolverNewton(), SchemaResolverPhysx()]
        try:
            with _log_python_warnings():
                self.parsing_results = self.builder.add_usd(source=current_stage, **add_usd_kwargs)
        except RuntimeError as e:
            if "Cycle detected" in str(e):
                carb.log_warn("[Newton] USD stage has composition cycles; retrying with flattened stage")
                flat_stage = Usd.Stage.CreateInMemory()
                flat_stage.GetRootLayer().TransferContent(current_stage.Flatten())
                with _log_python_warnings():
                    self.parsing_results = self.builder.add_usd(source=flat_stage, **add_usd_kwargs)
            else:
                raise

        linear_unit = self.parsing_results["linear_unit"]
        self.scene_scale = 1.0 / linear_unit
        if not math.isclose(linear_unit, 1.0):
            carb.log_error(
                f"[Newton] USD stage metersPerUnit is {linear_unit} (not 1.0). "
                "Input parameters may be inconsistent and simulation results may be incorrect."
            )

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

        if self.builder.body_count == 0 and self.builder.shape_count == 0 and self.builder.particle_count == 0:
            self.init()
            self.initialized = True
            self._initializing = False
            return

        # For VBD
        if solver_type == "vbd":
            self.builder.color()

        with _log_python_warnings():
            self.model = self.builder.finalize(self.device_str)

        # MuJoCo conversion requires at least one joint. Floating dynamic bodies normally
        # receive free joints, but zero-mass bodies are skipped, which leaves joint_count
        # at 0 when RigidBodyAPI is applied without MassAPI or collision geometry/density.
        if self.cfg.solver_cfg.solver_type == "mujoco" and self.model.joint_count == 0:
            raise RuntimeError(
                "Newton model has rigid bodies but no joints. Dynamic bodies need non-zero "
                "mass (UsdPhysics MassAPI, or CollisionAPI with density) so Newton can attach "
                "free joints for the MuJoCo solver. RigidBodyAPI alone is not sufficient."
            )

        self.control = self.model.control()
        self.model.ground = True
        if self.cfg.solver_outputs.contact_forces:
            self.model.request_contact_attributes("force")

        if self.cfg.solver_outputs.link_incoming_joint_force:
            if solver_type == "vbd":
                carb.log_warn(
                    "[Newton] Link incoming joint force reporting is not supported by the VBD solver; "
                    "the output will remain unavailable."
                )
            elif solver_type == "mujoco" and getattr(self.cfg.solver_cfg, "use_mujoco_cpu", False):
                carb.log_warn(
                    "[Newton] Link incoming joint force reporting is not supported by native MuJoCo CPU; "
                    "use MuJoCo Warp or XPBD."
                )
            else:
                self.model.request_state_attributes("body_parent_f")

        # `state_0` is retained only when reset-on-stop is disabled.
        if not carb_config.reset_on_stop() and self.state_0:
            restart_state = self.state_0
            self.state_0 = self.model.state()
            self.state_0.assign(restart_state)
        else:
            self.state_0 = self.model.state()
        self._initialize_joint_state_from_body_poses()

        self.state_1 = self.model.state()
        self.state_1.assign(self.state_0)

        valid_body_paths = set(self.model.body_label)
        self.fabric_manager.cleanup_stale_newton_index(valid_body_paths, self.device)

        xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        body_scales: list[tuple[float, float, float]] = []
        for i, path in enumerate(self.model.body_label):
            prim = usdrt_stage.GetPrimAtPath(usdrt.Sdf.Path(path))
            if not prim:
                body_scales.append((1.0, 1.0, 1.0))
                continue
            prim.CreateAttribute(self.fabric_manager.newton_index_attr, usdrt.Sdf.ValueTypeNames.UInt, True)
            prim.GetAttribute(self.fabric_manager.newton_index_attr).Set(i)
            world_scale = (1.0, 1.0, 1.0)
            usd_prim = current_stage.GetPrimAtPath(path)
            if usd_prim.IsValid():
                world_scale = _get_world_scale(usd_prim, xform_cache)
            body_scales.append(world_scale)
            xformable_prim = usdrt.Rt.Xformable(prim)
            if not xformable_prim.HasWorldXform():
                xformable_prim.SetWorldXformFromUsd()
        self.fabric_manager.set_body_scales(body_scales, self.device_str)

        # use simulation state to set fabirc data
        self.fabric_manager.update_fabric(
            self.model,
            self.state_0,
            self.scene_scale,
            self.device,
        )
        # Copy over initial states after body_q has been synced with the usd stage
        self.collision_pipeline = self._build_collision_pipeline(self.model)
        self.contacts = self.collision_pipeline.contacts()
        self.collision_pipeline.collide(self.state_0, self.contacts)

        if self.state_0.body_q:
            self.initial_body_q = self.state_0.body_q.numpy().copy()
        if self.state_0.body_qd:
            self.initial_body_qd = self.state_0.body_qd.numpy().copy()

        self.q_ik = self.model.joint_q
        self.qd_ik = self.model.joint_qd
        self.joint_torques = wp.zeros(self.model.joint_dof_count, dtype=wp.float32)
        self.solver = self._create_solver(self.model, self.cfg.solver_cfg)

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

        if self.collision_pipeline is None:
            return

        solver_type = self.cfg.solver_cfg.solver_type

        if solver_type == "mujoco":
            use_mujoco_native_contacts = getattr(self.cfg.solver_cfg, "use_mujoco_contacts", False)
            if not use_mujoco_native_contacts:
                self.collision_pipeline.collide(self.state_0, self.contacts)  # type: ignore[arg-type]
            contacts = self.contacts  # type: ignore[has-type]
        else:
            self.collision_pipeline.collide(self.state_0, self.contacts)  # type: ignore[arg-type]
            contacts = self.contacts  # type: ignore[has-type]

        # ``state_0`` must name the same buffer at the end of every step, for any substep count and
        # with or without graph capture: a captured CUDA graph replays on the buffers that were live
        # at capture time, and tensor views cache ``state_0`` pointers when they are created. The
        # substeps ping-pong the two buffers; after an odd number of swaps the newest state is copied
        # back into the pinned buffer (the ``need_state_copy`` pattern of the Newton examples).
        pinned_state = self.state_0
        for _ in range(num_substeps):
            self.solver.step(self.state_0, self.state_1, self.control, contacts, step_dt / float(num_substeps))  # type: ignore[has-type]
            self.state_0, self.state_1 = self.state_1, self.state_0
            self.state_0.clear_forces()
        if self.state_0 is not pinned_state:
            pinned_state.assign(self.state_0)
            self.state_0, self.state_1 = pinned_state, self.state_0

        if solver_type == "mujoco" and self.contacts is not None and not getattr(self.solver, "use_mujoco_cpu", False):  # type: ignore[has-type]
            self.solver.update_contacts(self.contacts, self.state_0)  # type: ignore[has-type]

        self.q_ik = self.model.joint_q
        self.qd_ik = self.model.joint_qd
