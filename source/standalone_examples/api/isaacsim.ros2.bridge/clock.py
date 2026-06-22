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

"""Demonstrate ROS 2 clock publishing and subscribing with simulation time."""

import time

from isaacsim import SimulationApp

# Example ROS2 bridge sample showing rclpy and rosclock interaction
simulation_app = SimulationApp({"renderer": "RealTimePathTracing", "headless": True})
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
from isaacsim.core.simulation_manager import SimulationManager

# enable ROS2 bridge extension
app_utils.enable_extension("isaacsim.ros2.bridge")

simulation_app.update()
# Note that this is not the system level rclpy, but one compiled for omniverse
import rclpy
from rosgraph_msgs.msg import Clock

rclpy.init()
clock_topic = "sim_time"
manual_clock_topic = "manual_time"

# Creating a action graph with ROS component nodes
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("PublishManualClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                # Connecting execution of OnPlaybackTick node to PublishClock to automatically publish each frame
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                # Connecting execution of OnImpulseEvent node to PublishManualClock so it will only publish when an impulse event is triggered
                ("OnImpulseEvent.outputs:execOut", "PublishManualClock.inputs:execIn"),
                # Connecting simulationTime data of ReadSimTime to the clock publisher nodes
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishManualClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Assigning topic names to clock publishers
                ("PublishClock.inputs:topicName", clock_topic),
                ("PublishManualClock.inputs:topicName", manual_clock_topic),
            ],
        },
    )
except Exception as e:
    print(e)


simulation_app.update()
simulation_app.update()


# Define ROS2 callbacks
def sim_clock_callback(data: Clock) -> None:
    """Log the simulation clock time."""
    print("sim time:", data.clock)


def manual_clock_callback(data: Clock) -> None:
    """Log the manually stepped simulation clock time."""
    print("manual stepped sim time:", data.clock)


# Create rclpy ndoe
node = rclpy.create_node("isaac_sim_clock")

# create subscribers
sim_clock_sub = node.create_subscription(Clock, clock_topic, sim_clock_callback, 1)
manual_clock_sub = node.create_subscription(Clock, manual_clock_topic, manual_clock_callback, 1)

time.sleep(1.0)
stage_utils.set_stage_units(meters_per_unit=1.0)
SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")

app_utils.play()
simulation_app.update()

# perform a fixed number of steps with fixed step size
for frame in range(20):

    # publish manual clock every 10 frames
    if frame % 10 == 0:
        og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)
        simulation_app.update()  # This updates rendering/app loop which calls the sim clock

    simulation_app.update()  # runs with a non-realtime clock
    rclpy.spin_once(node, timeout_sec=0.0)  # Spin node once
    # This sleep is to make this sample run a bit more deterministically for the subscriber callback
    # In general this sleep is not needed
    time.sleep(0.1)

# perform a fixed number of steps with realtime clock
for frame in range(20):

    # publish manual clock every 10 frames
    if frame % 10 == 0:
        og.Controller.set(og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True)

    simulation_app.update()  # runs with a realtime clock
    rclpy.spin_once(node, timeout_sec=0.0)  # Spin node once
    # This sleep is to make this sample run a bit more deterministically for the subscriber callback
    # In general this sleep is not needed
    time.sleep(0.1)

# shutdown
rclpy.shutdown()
app_utils.stop()
simulation_app.close()
