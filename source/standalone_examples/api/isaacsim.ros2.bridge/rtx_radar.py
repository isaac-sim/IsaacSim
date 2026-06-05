# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
"""Demonstrate RTX radar sensor with ROS 2 PointCloud2 publishing including radial velocity.

The example creates an RTX Radar with ``aux_output_level="BASIC"`` (the level
that enables the GMO BASIC channel carrying per-point Doppler velocity) and
publishes the radar detections as a ``sensor_msgs/PointCloud2`` message with
the ``radial_velocity_ms`` field enabled. The default generic RTX Radar has a
90-degree horizontal field of view; the script rotates the radar +45 degrees
about Z and places two sheets on the +X and +Y axes so each sheet sits at
+/-45 degrees from boresight and both fall inside the field of view. The
sheets are scaled cubes - flat "broadside-on" slabs oriented to maximise the
radar's projected cross-section - run as gravity-free rigid bodies. Each sheet oscillates back and forth along its
radial line at its own amplitude, frequency, and phase, so the published
``radial_velocity_ms`` field sweeps through positive and negative values that
differ between sheets and can be inspected in RViz2 (color PointCloud2 by
``radial_velocity_ms``). Motion is driven by PhysX integrating per-frame
linear velocity setpoints, not by transform teleports.

RTX Radar requires Motion BVH to be enabled in order to create the sensor at
all (the ``Radar`` constructor raises ``RuntimeError`` otherwise), so the script
launches SimulationApp with ``enable_motion_bvh=True``.
"""

import argparse
import math

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

# enable_motion_bvh=True is REQUIRED to create an RTX Radar at all. It sets the
# /renderer/raytracingMotion/* settings that the Radar constructor validates;
# without it the constructor raises RuntimeError.
simulation_app = SimulationApp({"headless": False, "enable_motion_bvh": True})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.rtx import Radar, RadarSensor

# Enable the ROS 2 bridge (registers RtxRadar*ROS2Publish* writers) and the
# RTX sensor nodes extension (registers the "draw-point-cloud" short writer
# name backed by RtxSensorDebugDrawPointCloud).
app_utils.enable_extension("isaacsim.ros2.bridge")
app_utils.enable_extension("isaacsim.sensors.rtx.nodes")
simulation_app.update()

# Minimal scene: a /World Xform, a ground plane, and a distant light. The radar
# does not need a textured environment - the sheets alone are enough to
# populate the point cloud.
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.define_prim("/World", type_name="Xform")
GroundPlane("/World/GroundPlane")
DistantLight("/World/DistantLight").set_intensities(1000)

# Move the viewport's perspective camera to (13.5, 13.5, 13.5) so the +X/+Y
# quadrant - where the radar looks and the sheets oscillate - is in view.
# Passing eye without target keeps the default orientation. wait_for_viewport
# guarantees the active viewport is fully initialized so get_camera returns a
# valid USD Camera prim.
ViewportManager.wait_for_viewport()
ViewportManager.set_camera_view(ViewportManager.get_camera(), eye=[13.5, 13.5, 13.5])

# Create the RTX Radar at /World/radar.
#
# - aux_output_level="BASIC" turns on the GMO BASIC channel, which is what
#   carries per-point radial velocity. The publisher reads this channel and
#   exposes it as the ``radial_velocity_ms`` PointCloud2 field. Without BASIC,
#   the ``outputRadialVelocityMS=True`` flag below would silently produce a
#   zero-filled velocity field.
# - The default outputFrameOfReference is SENSOR, so the published points are
#   in the radar-local frame and ``frameId="radar"`` is the natural TF frame.
# - Rotated +45 degrees about Z (wxyz quaternion ``(cos(pi/8), 0, 0, sin(pi/8))``)
#   so the radar's forward axis points between the +X and +Y sheets, placing
#   both inside its forward field of view.
radar = Radar(
    "/World/radar",
    aux_output_level="BASIC",
    translations=np.array([0.0, 0.0, 1.0]),
    orientations=np.array([math.cos(math.pi / 8.0), 0.0, 0.0, math.sin(math.pi / 8.0)]),
)

# Two flat "sheets" are placed on the +X and +Y axes around the radar. Each
# one is a USD Cube scaled into a thin slab and rotated so its broad face is
# normal to the line from the radar to the sheet - that orientation maximises
# the projected cross-section the radar's rays can strike, so each return
# carries dense detections. The default generic RTX Radar has a 90-degree
# horizontal field of view, so yawing the radar by +45 degrees about Z and
# placing the sheets on the +X and +Y axes puts each sheet at +/-45 degrees
# from boresight - i.e. right at the edges of the field of view, with both
# sheets visible to the same scan. Sheets on -X/-Y would fall outside the
# field of view and are omitted.
#
# Each sheet oscillates back and forth along its radial line. The motion is
# implemented by updating the rigid body's linear velocity each frame to
# follow ``v_radial(t) = A * omega * cos(omega * t + phase)``, which is the
# time-derivative of the position trajectory
# ``r(t) = r_rest + A * sin(omega * t + phase)``. PhysX integrates this
# velocity to advance the sheet, so at every instant the sheet has a real
# rigid-body velocity (and a real Doppler return); the radial velocity simply
# reverses sign every half period rather than being a constant outward drift.
# Each sheet uses a different amplitude, frequency, and phase, so the
# ``radial_velocity_ms`` readings differ noticeably between returns.
NUM_SHEETS = 2
SHEET_AZIMUTHS = np.array([0.0, math.pi / 2.0])  # +X, +Y
SHEET_REST_RADII = np.full(NUM_SHEETS, 6.0)  # meters from radar
SHEET_OSC_AMPLITUDES = np.array([2.0, 2.5])  # meters of in/out swing
SHEET_OSC_OMEGAS = np.array([0.8, 1.2])  # rad/s; peak radial speed = A * omega
SHEET_OSC_PHASES = np.array([0.0, math.pi / 3.0])  # rad
SHEET_HEIGHT = 1.0
# Cube authoring size (per-axis extent before scaling) and the per-axis scales
# that turn the base cube into a 0.1 m x 2.0 m x 1.5 m slab in its local frame
# (local X = radial thickness, local Y = tangential width, local Z = vertical).
SHEET_BASE_SIZE = 0.5
SHEET_SCALES = np.array([0.2, 4.0, 3.0])


def _radial_velocities(t: float) -> np.ndarray:
    """Linear velocities (world frame) for each sheet at simulation time ``t``."""
    # Radial speed = d/dt [A * sin(omega * t + phase)] = A * omega * cos(...).
    radial_speeds = SHEET_OSC_AMPLITUDES * SHEET_OSC_OMEGAS * np.cos(SHEET_OSC_OMEGAS * t + SHEET_OSC_PHASES)
    return np.stack(
        [
            radial_speeds * np.cos(SHEET_AZIMUTHS),
            radial_speeds * np.sin(SHEET_AZIMUTHS),
            np.zeros(NUM_SHEETS),
        ],
        axis=-1,
    )


# Initial positions: rest radius offset by A * sin(phase) along the radial
# direction, so the per-frame velocity formula above is consistent with the
# starting pose.
initial_radii = SHEET_REST_RADII + SHEET_OSC_AMPLITUDES * np.sin(SHEET_OSC_PHASES)
initial_positions = np.stack(
    [
        initial_radii * np.cos(SHEET_AZIMUTHS),
        initial_radii * np.sin(SHEET_AZIMUTHS),
        np.full(NUM_SHEETS, SHEET_HEIGHT),
    ],
    axis=-1,
)
# Rotate each sheet by its azimuth around the Z axis so the local +X axis
# (the thin dimension) points radially outward. wxyz quaternion for rotation
# by angle theta about Z: (cos(theta/2), 0, 0, sin(theta/2)).
half_azimuths = SHEET_AZIMUTHS / 2.0
initial_orientations = np.stack(
    [
        np.cos(half_azimuths),
        np.zeros(NUM_SHEETS),
        np.zeros(NUM_SHEETS),
        np.sin(half_azimuths),
    ],
    axis=-1,
)

# Author the USD Cube prims (scaled and oriented as sheets), then wrap them
# with RigidPrim so the UsdPhysics.RigidBodyAPI is applied. Gravity is
# disabled so the sheets hold their height; their translational motion comes
# entirely from the per-frame velocity setpoints in the main loop below.
sheet_paths = [f"/World/sheet_{i}" for i in range(NUM_SHEETS)]
Cube(
    sheet_paths,
    sizes=SHEET_BASE_SIZE,
    positions=initial_positions,
    orientations=initial_orientations,
    scales=np.broadcast_to(SHEET_SCALES, (NUM_SHEETS, 3)),
)
sheet_bodies = RigidPrim(sheet_paths)
sheet_bodies.set_enabled_gravities([False])

SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
simulation_app.update()

# Wrap the radar with the RadarSensor runtime. The constructor creates the
# render product internally; subsequent ``attach_writer`` calls attach writers
# to that render product. ``annotators=[]`` is used here because neither writer
# needs a sensor-runtime annotator (each writer pulls its data directly from
# the radar's GenericModelOutput RenderVar).
sensor = RadarSensor(radar, annotators=[])

# PointCloud2 publisher with radial_velocity_ms enabled. The writer name is
# registered by isaacsim.ros2.bridge for any sensor whose render product is
# attached to an OmniRadar prim. ``attach_writer`` forwards the kwargs to
# ``writer.initialize()``.
sensor.attach_writer(
    "RtxRadarROS2PublishPointCloud",
    topicName="radar_point_cloud",
    frameId="radar",
    outputRadialVelocityMS=True,
)

# Visualize radar returns in the viewport. ``"draw-point-cloud"`` is the short
# name registered by isaacsim.sensors.rtx.nodes for RtxSensorDebugDrawPointCloud.
# The radar outputs in SENSOR frame (the default), so doTransform=True asks the
# debug writer to transform points into world coordinates before drawing.
sensor.attach_writer(
    "draw-point-cloud",
    doTransform=True,
    size=0.2,
    color=[1.0, 0.2, 0.3, 1.0],
)

simulation_app.update()

app_utils.play()
# One update tick lets PhysX initialize the rigid bodies so the tensor
# backend used by ``set_velocities`` is valid.
simulation_app.update()

# Main loop: at each frame, stamp the sinusoidal radial velocity setpoint
# into every sheet's rigid body, then advance the simulation. PhysX
# integrates the velocity to update the sheet's pose, so the sheets sweep
# back and forth around their rest radii.
sim_dt = 1.0 / 60.0
frame_count = 0
while simulation_app.is_running():
    sheet_bodies.set_velocities(linear_velocities=_radial_velocities(frame_count * sim_dt))
    simulation_app.update()
    frame_count += 1
    if args.test and frame_count >= 10:
        break

app_utils.stop()
simulation_app.close()
