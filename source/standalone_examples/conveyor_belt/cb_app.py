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

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import math

import cb_kernels as conveyor_belt_kernels
import numpy as np
import warp as wp
from cb_actuators import (
    VelocityFieldActuator,
)
from cb_body_manager import BodyManager
from cb_conveyor_belt_manager import ConveyorBeltManager
from cb_material_pair_manager import MaterialPairManager
from cb_scene import (
    create_scene,
)
from cb_visualizers import VelocityFieldVisualizer
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from pxr import UsdLux

# not needed for the purpose of this sample
wp.config.enable_backward = False


#
# Number of simulation steps to re-capture a CUDA graph after
# a reset. All parameters to kernels in a CUDA graph need to
# remain fixed to re-launch the same CUDA graph.
#
WARM_START_COUNTER = 2

#
# The computation of the tangential force to apply at a contact point
# will happen in groups of N contacts per thread. For larger setups
# this can be a tuning factor to balance between the level of parallelism
# and the workload per thread.
#
CONTACT_PROCESSING_BATCH_SIZE = 5

#
# Time (in seconds) until the conveyor belts run at full speed. An acceleration
# phase seems more realistic and can improve simulation behavior by reducing the
# magnitude of forces involved.
#
CONVEYOR_BELT_SPEED_STARTUP_DURATION = 1.0

#
# Enable visualization of conveyor belt velocity field speeds.
#
ENABLE_VELOCITY_FIELD_VISUALIZER = True


class ConveyorBeltExample:

    def __init__(
        self,
    ) -> None:

        self._sim_dt = 1.0 / 120.0

        self._world = World(
            physics_dt=self._sim_dt,
            rendering_dt=1.0 / 60.0,
            stage_units_in_meters=1.0,
            physics_prim_path="/World/physics_scene",
            set_defaults=False,
            backend="warp",
            device="cuda",
        )

        self._warm_start_counter = WARM_START_COUNTER
        self._cuda_graph = None
        self._use_cuda_graph = wp.get_device(self._world.device).is_cuda

        self._stage = simulation_app.context.get_stage()

        self._velocity_field_actuator = VelocityFieldActuator()

        self._conveyor_belt_manager = ConveyorBeltManager()

        self._body_manager = BodyManager()

        self._material_pair_manager = MaterialPairManager()

        if ENABLE_VELOCITY_FIELD_VISUALIZER:
            #
            # Create visual markers to observe conveyor belt speeds.
            #

            self._velocity_field_visualizer = VelocityFieldVisualizer(CONVEYOR_BELT_SPEED_STARTUP_DURATION)
        else:
            self._velocity_field_visualizer = None

        self._physics_post_step_callback_id = None

    def create_restart_buffers(
        self,
        device: str | None = None,
    ) -> None:
        """Allocate (or re-allocate) a set of warp buffers needed for contact processing and
        force computation.

        The buffers created here are recreated when the sample is stopped and restarted.

        Args:
            device: Warp device string (e.g. ``"cuda:0"`` or ``"cpu"``). Uses the default
                device when ``None``.
        """

        # Single element array to store the total elapsed simulation time.
        self._total_elapsed_time = wp.zeros(shape=1, dtype=wp.float32, device=device)

        # Single element array to store a global scale parameter to apply to the velocity
        # fields of the conveyor belts. Used to slowly speed up the conveyor belts at the
        # start.
        self._global_conveyor_belt_speed_scale = wp.zeros(shape=1, dtype=wp.float32, device=device)

    def create_buffers(
        self,
        max_contact_count: int,
        device: str | None = None,
    ) -> None:
        """Allocate all Warp buffers needed for contact processing and force computation.

        Args:
            max_contact_count: Upper bound on the number of contact points that can be
                processed in a single simulation step.
            device: Warp device string (e.g. ``"cuda:0"`` or ``"cpu"``). Uses the default
                device when ``None``.
        """

        self._velocity_field_actuator.create_buffers(device)

        self._conveyor_belt_manager.create_buffers(device)

        self._body_manager.create_buffers(device)

        self._material_pair_manager.create_buffers(device)

        self.create_restart_buffers()

        # Single element array to store the total contact point count.
        self._total_contact_count = wp.zeros(shape=1, dtype=wp.uint32, device=device)

        # (N, 3) array that holds for each of the N contact points the following indices:
        # - index of the body the contact belongs to
        # - type of the velocity field that will be used to compute the force to apply
        #   at the contact point
        # - ID of the velocity field instance that will be used to compute the force to
        #   apply at the contact point
        self._point_to_indices_map = wp.empty(shape=(max_contact_count, 3), dtype=wp.uint32, device=device)

        # Array to store the mass splitting scale to use at each contact point.
        # When computing the tangential force at a contact point, the rigid body
        # mass will be multiplied by this scale to have each point represent a
        # fraction of the total mass only.
        self._mass_splitting_scale_buffer = wp.empty(shape=max_contact_count, dtype=wp.float32, device=device)

        # Array to store the friction coefficient to use at each contact point.
        self._friction_coefficient_buffer = wp.empty(shape=max_contact_count, dtype=wp.float32, device=device)

        # Array to correlate contact points with similar normals to the same contact patch
        # (within a rigid body)
        self._contact_patch_buffer = wp.empty(shape=max_contact_count, dtype=conveyor_belt_kernels.Patch, device=device)

        # Array that stores the potentially redistributed normal force at each contact point
        self._adjusted_contact_normal_force_buffer = wp.empty(
            shape=(max_contact_count, 1), dtype=wp.float32, device=device
        )

        # Array to store the force that each contact point applies to the rigid body.
        # The first 3 entries of the spatial vector hold the force, the last 3 entries hold
        # the torque. The force/torque will be in world space and should get applied at the
        # center of mass of the rigid body.
        self._per_point_force_torque_buffer = wp.empty(shape=max_contact_count, dtype=wp.spatial_vector, device=device)

    def make_env(
        self,
    ) -> None:
        """Build the simulation environment: lighting, camera, scene, data buffers, contact views, and callbacks."""

        # Add a dome light
        domeLight = UsdLux.DomeLight.Define(self._stage, "/World/DomeLight")
        domeLight.CreateIntensityAttr(800)

        set_camera_view(eye=np.array([0.0, -10.0, 10.0]), target=np.array([0.0, 0.0, 0.0]))

        self._world.get_physics_context().enable_ccd(False)
        self._world.get_physics_context().enable_stablization(False)
        self._world.get_physics_context().set_gravity(-9.81)

        #
        # Create the rigid bodies and the collision geometry of the scene. Register
        # velocity fields, conveyor belts, objects moving on conveyor belts and material
        # pairs.
        #

        self._world.scene.add_ground_plane()

        create_scene(
            self._stage,
            self._velocity_field_actuator,
            self._conveyor_belt_manager,
            self._body_manager,
            self._material_pair_manager,
            self._velocity_field_visualizer,
        )

        #
        # Data buffers
        #

        self._moving_body_count = len(self._body_manager.body_path_list)
        self._conveyor_belt_count = len(self._conveyor_belt_manager.conveyor_belt_path_list)

        # note: The chosen maximum average of contact points per body needs to be tuned based on
        #       the application. Higher numbers are usually needed with...
        #       - ...growing complexity/resolution of the involved collision geometry
        #       - ...growing contact offset on the involved collision geometry
        #       - ...growing number of bodies interacting with conveyor belts at any one time
        max_average_contact_count_per_body = 32

        self._max_contact_count = self._moving_body_count * max_average_contact_count_per_body

        self.create_buffers(
            self._max_contact_count,
            self._world.device,
        )

        #
        # Rigid prim view to...
        # - ...track contacts between moving bodies and conveyor belt objects
        # - ...extract rigid body simulation state (transforms, velocities etc.)
        # - ...apply forces
        #

        self._body_vs_conveyor_belt_contact_view = RigidPrim(
            prim_paths_expr=self._body_manager.body_path_list,
            name="body_vs_conveyor_belt_contact_view",
            contact_filter_prim_paths_expr=[self._conveyor_belt_manager.conveyor_belt_path_list]
            * self._moving_body_count,
            max_contact_count=self._max_contact_count,
        )

        #
        # Adding things to the scene and general initialization
        #

        self._world.scene.add(self._body_vs_conveyor_belt_contact_view)
        self._world.reset(soft=False)

        #
        # Registration of callbacks
        #

        # Callback after a physics simulation step ran
        self._physics_post_step_callback_id = SimulationManager.register_callback(
            self.post_physics_step, SimulationEvent.PHYSICS_POST_STEP
        )

    def step_conveyor_belts(
        self,
        dt: float,
        body_positions: wp.indexedarray2d(dtype=wp.float32),
        body_orientations: wp.array2d(dtype=wp.float32),
        body_linear_velocities: wp.indexedarray2d(dtype=wp.float32),
        body_angular_velocities: wp.indexedarray2d(dtype=wp.float32),
        body_com_positions: wp.indexedarray2d(dtype=wp.float32),
        body_com_orientations: wp.array2d(dtype=wp.float32),
        body_inverse_masses: wp.indexedarray2d(dtype=wp.float32),
        body_inverse_inertias: wp.indexedarray2d(dtype=wp.float32),
        contact_forces: wp.array2d(dtype=wp.float32),
        contact_points: wp.array2d(dtype=wp.float32),
        contact_normals: wp.array2d(dtype=wp.float32),
        pair_contacts_count: wp.indexedarray2d(dtype=wp.uint32),
        pair_contacts_start_indices: wp.indexedarray2d(dtype=wp.uint32),
    ) -> None:
        """Run the full conveyor-belt force pipeline for one simulation step.

        Based on contact information and the provided contact normal forces, the pipeline will
        compute the tangential forces that the conveyor belts shall apply to the rigid bodies
        sitting on the conveyor belts.

        Args:
            dt: Simulation time-step in seconds.
            body_positions: (N, 3) world-space positions for each tracked body.
            body_orientations: (N, 4) world-space orientations (wxyz) for each body.
            body_linear_velocities: (N, 3) world-space linear velocities for each body.
            body_angular_velocities: (N, 3) world-space angular velocities for each body.
            body_com_positions: (N, 3) center-of-mass positions (body-local) for each body.
            body_com_orientations: (N, 4) center-of-mass orientations (body-local) (wxyz) for each body.
            body_inverse_masses: (N,) inverse masses for each body.
            body_inverse_inertias: (N, 9) inverse inertia tensors (body-local) for each body.
            contact_forces: (C, 1) normal contact forces for each contact point.
            contact_points: (C, 3) world-space contact point positions.
            contact_normals: (C, 3) contact normals for each contact point.
            pair_contacts_count: (N, M) number of contacts per body-conveyor-belt pair.
            pair_contacts_start_indices: (N, M) start index into the contact forces/points/normals buffers per body-conveyor-belt pair.
        """

        max_thread_count = 4096

        #
        # Prepare buffers:
        # - Computing transforms and rotating inertias
        # - Mapping contacts to bodies, velocity fields, friction coefficients etc.
        # - Computing total contact count
        #

        # maximum number of conveyor belts to process in parallel per rigid body
        parallel_conveyor_belt_processing_count = 16
        max_body_thread_count = math.floor(max_thread_count / parallel_conveyor_belt_processing_count)
        parallel_body_processing_count = min(max_body_thread_count, self._moving_body_count)

        wp.launch(
            kernel=conveyor_belt_kernels.prepare_buffers,
            dim=(parallel_body_processing_count, parallel_conveyor_belt_processing_count),
            inputs=[
                parallel_body_processing_count,
                parallel_conveyor_belt_processing_count,
                self._moving_body_count,
                self._conveyor_belt_count,
                dt,
                CONVEYOR_BELT_SPEED_STARTUP_DURATION,
                body_positions,
                body_orientations,
                body_com_positions,
                body_com_orientations,
                body_inverse_inertias,
                self._body_manager.material_index_buffer,
                self._conveyor_belt_manager.conveyor_belt_to_indices_map,
                self._material_pair_manager.friction_table,
                pair_contacts_count,
                pair_contacts_start_indices,
            ],
            outputs=[
                self._body_manager.world_transform_buffer,
                self._body_manager.inverse_inertia_buffer,
                self._point_to_indices_map,
                self._friction_coefficient_buffer,
                self._total_contact_count,
                self._total_elapsed_time,
                self._global_conveyor_belt_speed_scale,
            ],
            device=self._world.device,
        )

        #
        # Correlate contact points to contact patches. Set mass splitting scale to 0
        # for filtered out contacts.
        #

        parallel_body_processing_count = min(max_thread_count, self._moving_body_count)

        wp.launch(
            kernel=conveyor_belt_kernels.correlate_and_filter_contact_points,
            dim=parallel_body_processing_count,
            inputs=[
                parallel_body_processing_count,
                self._moving_body_count,
                self._conveyor_belt_count,
                self._conveyor_belt_manager.surface_normal_buffer,
                self._conveyor_belt_manager.contact_processing_threshold_buffer,
                pair_contacts_count,
                pair_contacts_start_indices,
                contact_normals,
                contact_forces,
            ],
            outputs=[
                self._contact_patch_buffer,
                self._body_manager.body_to_patch_buffer,
                self._mass_splitting_scale_buffer,
            ],
            device=self._world.device,
        )

        #
        # Redistribute contact normal forces among contact points of a patch if
        # the patch spans multiple conveyor belts.
        #

        # maximum number of patches to process in parallel per rigid body. Since this sample
        # uses rather simple conveyor belt and rigid body geometries, it is rare to ever get
        # more than one contact patch per interaction. Hence, disabling parallelism on a
        # patch level here.
        parallel_patch_processing_count = 1
        max_body_thread_count = math.floor(max_thread_count / parallel_patch_processing_count)
        parallel_body_processing_count = min(max_body_thread_count, self._moving_body_count)

        wp.launch(
            kernel=conveyor_belt_kernels.redistribute_contact_force,
            dim=(parallel_body_processing_count, parallel_patch_processing_count),
            inputs=[
                parallel_body_processing_count,
                parallel_patch_processing_count,
                self._moving_body_count,
                self._body_manager.body_to_patch_buffer,
                self._contact_patch_buffer,
                contact_points,
                contact_forces,
                self._body_manager.world_transform_buffer,
            ],
            outputs=[
                self._adjusted_contact_normal_force_buffer,
                self._mass_splitting_scale_buffer,
            ],
            device=self._world.device,
        )

        #
        # At each contact point, compute the tangential force that the velocity field of
        # a conveyor belt shall apply to the rigid body.
        #

        self._velocity_field_actuator.step(
            self._sim_dt,
            self._max_contact_count,
            self._body_manager.world_transform_buffer,
            body_inverse_masses,
            self._body_manager.inverse_inertia_buffer,
            body_linear_velocities,
            body_angular_velocities,
            contact_points,
            contact_normals,
            self._adjusted_contact_normal_force_buffer,
            self._point_to_indices_map,
            self._mass_splitting_scale_buffer,
            self._friction_coefficient_buffer,
            self._total_contact_count,
            self._global_conveyor_belt_speed_scale,
            # output
            self._per_point_force_torque_buffer,
            # input
            max_thread_count=max_thread_count,
            batch_size=CONTACT_PROCESSING_BATCH_SIZE,
            device=self._world.device,
        )

        #
        # Sum up the per contact point forces/torques to a single force/torque
        # per rigid body.
        #

        parallel_body_processing_count = min(max_thread_count, self._moving_body_count)

        wp.launch(
            kernel=conveyor_belt_kernels.sum_up_force,
            dim=parallel_body_processing_count,
            inputs=[
                parallel_body_processing_count,
                self._moving_body_count,
                self._body_manager.body_to_patch_buffer,
                self._contact_patch_buffer,
                self._per_point_force_torque_buffer,
            ],
            outputs=[
                self._body_manager.force_buffer,
                self._body_manager.torque_buffer,
            ],
            device=self._world.device,
        )

    def post_physics_step(self, dt: float, _context) -> None:
        """Physics post-step callback: fetches contact data, computes conveyor forces, and applies them.

        Also updates the velocity-field visualizer markers if one is present.

        Args:
            dt: Elapsed simulation time for the current step in seconds.
            _context: Simulation event context (PhysicsStepContext) provided by the SimulationManager callback system.
        """

        #
        # Fetch the active simulation state and physical properties of the rigid bodies that
        # are to interact with the conveyor belts. Fetch the contact information between those
        # bodies and the conveyor belts.
        #

        (
            contact_forces,
            contact_points,
            contact_normals,
            _contact_distances,
            pair_contacts_count,
            pair_contacts_start_indices,
        ) = self._body_vs_conveyor_belt_contact_view.get_contact_force_data(dt=self._sim_dt)

        states = self._body_vs_conveyor_belt_contact_view.get_current_dynamic_state()

        com_positions, com_orientations = self._body_vs_conveyor_belt_contact_view.get_coms()

        inverse_masses = self._body_vs_conveyor_belt_contact_view.get_inv_masses()
        inverse_inertias = self._body_vs_conveyor_belt_contact_view.get_inv_inertias()

        #
        # Compute the forces that the conveyor belts shall apply to the rigid bodies placed
        # on the conveyor belts.
        #

        if self._use_cuda_graph and (self._warm_start_counter > 0):

            self._warm_start_counter -= 1

            # after the warm start the kernel parameters stay fixed in this example, thus a
            # CUDA graph can be captured and re-launched to improve performance. If any kernel
            # parameter changes, the graph will need to be re-captured.

            with wp.ScopedCapture(device=self._world.device) as capture:
                self.step_conveyor_belts(
                    dt,
                    states.positions,
                    states.orientations,
                    states.linear_velocities,
                    states.angular_velocities,
                    com_positions,
                    com_orientations,
                    inverse_masses,
                    inverse_inertias,
                    contact_forces,
                    contact_points,
                    contact_normals,
                    pair_contacts_count,
                    pair_contacts_start_indices,
                )

            self._cuda_graph = capture.graph

        if self._cuda_graph is not None:
            wp.capture_launch(self._cuda_graph)
        else:
            self.step_conveyor_belts(
                dt,
                states.positions,
                states.orientations,
                states.linear_velocities,
                states.angular_velocities,
                com_positions,
                com_orientations,
                inverse_masses,
                inverse_inertias,
                contact_forces,
                contact_points,
                contact_normals,
                pair_contacts_count,
                pair_contacts_start_indices,
            )

        #
        # Apply the computed forces to the rigid bodies and clear buffers.
        #

        self._body_vs_conveyor_belt_contact_view.apply_forces_and_torques_at_pos(
            self._body_manager.force_buffer,
            self._body_manager.torque_buffer,
        )

        wp.launch(
            kernel=conveyor_belt_kernels.clear_buffers,
            dim=1,
            outputs=[
                self._total_contact_count,
            ],
            device=self._world.device,
        )

        #
        # Update the markers that visualize the conveyor belt movement speed, i.e, the
        # velocity fields attached to the conveyor belts. This is just a help for visual
        # inspection during development and would be disabled/removed during general usage.
        #

        if self._velocity_field_visualizer is not None:
            self._velocity_field_visualizer.update(self._sim_dt)

    def play(
        self,
    ) -> None:
        """Set up the environment and run the simulation loop until the application is closed."""

        self.make_env()

        reset_needed = False

        while simulation_app.is_running():

            if self._world.is_playing():

                # deal with sim re-initialization after restarting sim
                if reset_needed:
                    # initialize simulation views
                    self._world.reset(soft=False)

                    self.create_restart_buffers()

                    if self._velocity_field_visualizer is not None:
                        self._velocity_field_visualizer.reset()

                    reset_needed = False

                self._world.step(render=True)

            else:

                if self._world.is_stopped():

                    if not reset_needed:
                        reset_needed = True

                        self._cuda_graph = None
                        self._warm_start_counter = WARM_START_COUNTER

                # this ensures the UI remains responsive when paused or stopped
                self._world.render()

        #
        # Clean up registered callbacks
        #

        if self._physics_post_step_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_post_step_callback_id)
            self._physics_post_step_callback_id = None

        simulation_app.close()


ConveyorBeltExample().play()
