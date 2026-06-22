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

"""Verifies that ROS 2 TF and camera image messages published from a zero-delay Isaac Sim graph carry synchronized simulation timestamps. Measures timestamp deltas after startup settling and fails on multi-frame ordering delays."""

import argparse
import logging
import os
import time
from collections import deque
from typing import Any

import numpy as np
from isaacsim import SimulationApp

logging.basicConfig(level=logging.DEBUG, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("tf_cam_sync_test")

# CLI args
parser = argparse.ArgumentParser()
parser.add_argument("--test-steps", type=int, default=30)
args, _ = parser.parse_known_args()

# Launch Isaac Sim in headless zero-delay mode
simulation_app = SimulationApp(
    {"headless": True}, experience=f'{os.environ["EXP_PATH"]}/isaacsim.exp.base.zero_delay.kit'
)

# Import post-launch modules
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.rendering_manager import RenderingManager, ViewportManager
from isaacsim.core.simulation_manager import SimulationManager

app_utils.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

ros_context = rclpy.Context()
rclpy.init(context=ros_context)
ros_executor = SingleThreadedExecutor(context=ros_context)

# ROS state
tf_msg = None
img_msg = None
tf_recv_time = None
img_recv_time = None
tf_recv_count = 0
img_recv_count = 0
tf_queue = deque()
img_queue = deque()


def tf_callback(msg: Any) -> None:
    """Handle incoming TF messages and record their timestamps."""
    global tf_msg, tf_recv_time, tf_recv_count
    tf_msg = msg
    tf_recv_time = time.perf_counter()
    tf_recv_count += 1
    stamp = msg.transforms[0].header.stamp
    tf_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    tf_queue.append((tf_ns, msg, tf_recv_time))
    log.debug(
        f"  TF  callback #{tf_recv_count}: stamp={stamp.sec}.{stamp.nanosec:09d}, "
        f"frame_id='{msg.transforms[0].header.frame_id}', "
        f"child_frame_id='{msg.transforms[0].child_frame_id}'"
    )


def img_callback(msg: Any) -> None:
    """Handle incoming image messages and record their timestamps."""
    global img_msg, img_recv_time, img_recv_count
    img_msg = msg
    img_recv_time = time.perf_counter()
    img_recv_count += 1
    stamp = msg.header.stamp
    img_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    img_queue.append((img_ns, msg, img_recv_time))
    log.debug(
        f"  IMG callback #{img_recv_count}: stamp={stamp.sec}.{stamp.nanosec:09d}, "
        f"encoding='{msg.encoding}', size={msg.width}x{msg.height}"
    )


def clear_message_state() -> None:
    """Reset all message state and clear queues."""
    global tf_msg, img_msg, tf_recv_time, img_recv_time
    tf_msg = None
    img_msg = None
    tf_recv_time = None
    img_recv_time = None
    tf_queue.clear()
    img_queue.clear()


def collect_latest_pair(
    max_wait_steps: int = 20,
) -> tuple[int, int, Any, Any, float, float, int, float, int, int] | None:
    """Collect the latest TF and image message pair by spinning the ROS node."""
    wait_count = 0
    spin_start = time.perf_counter()

    while wait_count < max_wait_steps:
        if tf_queue and img_queue:
            break

        rclpy.spin_once(node, executor=ros_executor, timeout_sec=0.05)
        wait_count += 1

    if tf_queue and img_queue:
        dropped_tf = max(0, len(tf_queue) - 1)
        dropped_img = max(0, len(img_queue) - 1)
        tf_ns, tf_pair_msg, tf_pair_recv_time = tf_queue[-1]
        img_ns, img_pair_msg, img_pair_recv_time = img_queue[-1]
        tf_queue.clear()
        img_queue.clear()
        spin_elapsed = time.perf_counter() - spin_start
        return (
            tf_ns,
            img_ns,
            tf_pair_msg,
            img_pair_msg,
            tf_pair_recv_time,
            img_pair_recv_time,
            wait_count,
            spin_elapsed,
            dropped_tf,
            dropped_img,
        )

    spin_elapsed = time.perf_counter() - spin_start
    return (None, None, None, None, None, None, wait_count, spin_elapsed, 0, 0)


# Create ROS2 node + subscribers
node = rclpy.create_node("sync_test_node", context=ros_context)
tf_sub = node.create_subscription(TFMessage, "/tf_test", tf_callback, 10)
img_sub = node.create_subscription(Image, "rgb", img_callback, 10)

# Setup physics and rendering rates
SimulationManager.set_physics_dt(1.0 / 60.0)
RenderingManager.set_dt(1.0 / 60.0)

# Create scene
stage_utils.create_new_stage()
CUBE_PRIM_PATH = "/Cube"
Cube(paths=CUBE_PRIM_PATH, scales=[0.5, 0.5, 0.5])
cube = RigidPrim(paths=CUBE_PRIM_PATH)
GeomPrim(paths=CUBE_PRIM_PATH, apply_collision_apis=True)
GroundPlane("/World/GroundPlane")

# Add camera
CAMERA_PRIM_PATH = "/Camera"
CAMERA_EYE = [0, -2, 1]
CAMERA_TARGET = [0, 0, 0.1]

stage = omni.usd.get_context().get_stage()
stage.DefinePrim(CAMERA_PRIM_PATH, "Camera")
ViewportManager.set_camera_view(CAMERA_PRIM_PATH, eye=CAMERA_EYE, target=CAMERA_TARGET)

app_utils.play()
simulation_app.update()

# Create Action Graph
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ComputeTF", "isaacsim.core.nodes.IsaacComputeTransformTree"),
            ("TfPublisher", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("CameraPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ComputeTF.inputs:targetPrims", [usdrt.Sdf.Path(CUBE_PRIM_PATH)]),
            ("TfPublisher.inputs:topicName", "/tf_test"),
            ("CameraPublisher.inputs:type", "rgb"),
            ("CameraPublisher.inputs:topicName", "rgb"),
            ("CreateRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_PRIM_PATH)]),
            ("CreateRenderProduct.inputs:width", 1920),
            ("CreateRenderProduct.inputs:height", 1080),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ComputeTF.inputs:execIn"),
            ("ComputeTF.outputs:execOut", "TfPublisher.inputs:execIn"),
            ("ComputeTF.outputs:parentFrames", "TfPublisher.inputs:parentFrames"),
            ("ComputeTF.outputs:childFrames", "TfPublisher.inputs:childFrames"),
            ("ComputeTF.outputs:translations", "TfPublisher.inputs:translations"),
            ("ComputeTF.outputs:orientations", "TfPublisher.inputs:orientations"),
            ("OnPlaybackTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "TfPublisher.inputs:timeStamp"),
            ("CreateRenderProduct.outputs:execOut", "CameraPublisher.inputs:execIn"),
            ("CreateRenderProduct.outputs:renderProductPath", "CameraPublisher.inputs:renderProductPath"),
        ],
    },
)

simulation_app.update()
log.info("Action graph created, starting warmup...")

# Warmup
for i in range(20):
    simulation_app.update()
    rclpy.spin_once(node, executor=ros_executor, timeout_sec=0.0)
    warmup_tf = tf_msg is not None
    warmup_img = img_msg is not None
    if i % 5 == 0 or warmup_tf or warmup_img:
        log.debug(f"  warmup step {i}: tf_received={warmup_tf}, img_received={warmup_img}")

log.info(f"Warmup complete. Total callbacks so far: tf={tf_recv_count}, img={img_recv_count}")

# Flush the render pipeline: the camera publisher has ~1 frame of latency, so after
# clearing warmup messages we need to step+spin once more to drain the stale render
# frame. Without this, the first test step receives an image stamped from the previous
# warmup frame while TF already carries the current frame's timestamp.
clear_message_state()
simulation_app.update()
for _ in range(20):
    rclpy.spin_once(node, executor=ros_executor, timeout_sec=0.05)
    if tf_queue and img_queue:
        break
log.info(f"Pipeline flush: tf_queue={'OK' if tf_queue else 'NONE'}, " f"img_queue={'OK' if img_queue else 'NONE'}")
clear_message_state()

# Settle phase: require several consecutive synced pairs before measuring.
# This removes startup transients from CI while keeping strict checks in measured steps.
# A small tolerance absorbs rational-time rounding noise (1/60 s is not exact in
# nanoseconds, so consecutive stamps can toggle between adjacent integers).
SETTLE_REQUIRED_CONSECUTIVE = 5
SETTLE_MAX_STEPS = 200
SYNC_TOLERANCE_NS = 1_000  # 1 us, above rational-time rounding floor
settle_consecutive_synced = 0
settled = False

log.info(
    "Starting settle phase: require "
    f"{SETTLE_REQUIRED_CONSECUTIVE} consecutive synced pairs "
    f"(max {SETTLE_MAX_STEPS} steps)"
)
for settle_step in range(SETTLE_MAX_STEPS):
    simulation_app.update()
    (
        tf_ns,
        img_ns,
        tf_pair_msg,
        img_pair_msg,
        _,
        _,
        settle_wait_count,
        settle_spin_elapsed,
        settle_dropped_tf,
        settle_dropped_img,
    ) = collect_latest_pair(max_wait_steps=20)

    if tf_pair_msg is None or img_pair_msg is None:
        settle_consecutive_synced = 0
        log.debug(
            f"  settle step {settle_step}: no pair, "
            f"spins={settle_wait_count}, spin_time={settle_spin_elapsed * 1000:.1f} ms, "
            f"dropped_tf={settle_dropped_tf}, dropped_img={settle_dropped_img}"
        )
        continue

    raw_offset_ns = img_ns - tf_ns
    settle_delta_ns = abs(raw_offset_ns)
    if settle_delta_ns <= SYNC_TOLERANCE_NS:
        settle_consecutive_synced += 1
    else:
        settle_consecutive_synced = 0

    tf_settle_stamp = tf_pair_msg.transforms[0].header.stamp
    img_settle_stamp = img_pair_msg.header.stamp
    log.debug(
        f"  settle step {settle_step}: "
        f"tf_stamp={tf_settle_stamp.sec}.{tf_settle_stamp.nanosec:09d}, "
        f"img_stamp={img_settle_stamp.sec}.{img_settle_stamp.nanosec:09d}, "
        f"raw_offset={raw_offset_ns / 1e6:+.3f} ms, "
        f"delta={settle_delta_ns / 1e6:.3f} ms, "
        f"consecutive_synced={settle_consecutive_synced}/{SETTLE_REQUIRED_CONSECUTIVE}, "
        f"spins={settle_wait_count}, spin_time={settle_spin_elapsed * 1000:.1f} ms, "
        f"dropped_tf={settle_dropped_tf}, dropped_img={settle_dropped_img}"
    )

    if settle_consecutive_synced >= SETTLE_REQUIRED_CONSECUTIVE:
        settled = True
        settle_steps_used = settle_step + 1
        log.info(f"Settle phase complete at step {settle_step}")
        break

settle_steps_used = SETTLE_MAX_STEPS if not settled else settle_steps_used

if not settled:
    log.error(
        "Settle phase failed: did not observe required consecutive synced pairs " f"within {SETTLE_MAX_STEPS} steps."
    )

clear_message_state()

# Run sync test
log.info(f"Starting sync test with {args.test_steps} steps...")
deltas = []
missed_steps = []
for step in range(args.test_steps):
    step_start = time.perf_counter()
    new_pos = np.random.uniform(-1, 1, 3)
    cube.set_world_poses(positions=[new_pos])
    simulation_app.update()
    step_elapsed = time.perf_counter() - step_start

    (
        tf_ns,
        img_ns,
        tf_pair_msg,
        img_pair_msg,
        tf_pair_recv_time,
        img_pair_recv_time,
        wait_count,
        spin_elapsed,
        dropped_tf,
        dropped_img,
    ) = collect_latest_pair(max_wait_steps=20)

    if tf_pair_msg and img_pair_msg:
        tf_stamp = tf_pair_msg.transforms[0].header.stamp
        img_stamp = img_pair_msg.header.stamp
        raw_offset_ns = img_ns - tf_ns
        delta = abs(raw_offset_ns)
        deltas.append(delta)

        # Determine which message has the later timestamp.
        behind = "TF" if tf_ns < img_ns else ("IMG" if img_ns < tf_ns else "EQUAL")

        log.info(
            f"Step {step:3d}: "
            f"tf_stamp={tf_stamp.sec}.{tf_stamp.nanosec:09d}  "
            f"img_stamp={img_stamp.sec}.{img_stamp.nanosec:09d}  "
            f"raw_offset={raw_offset_ns / 1e6:+8.3f} ms  "
            f"delta={delta / 1e6:8.3f} ms  "
            f"behind={behind}  "
            f"spins={wait_count}  "
            f"step_time={step_elapsed * 1000:.1f} ms  "
            f"spin_time={spin_elapsed * 1000:.1f} ms  "
            f"tf_recv_wall={tf_pair_recv_time:.6f}  "
            f"img_recv_wall={img_pair_recv_time:.6f}  "
            f"wall_recv_delta={(abs(tf_pair_recv_time - img_pair_recv_time)) * 1000:.3f} ms  "
            f"dropped_tf={dropped_tf}  "
            f"dropped_img={dropped_img}"
        )
    else:
        missed_steps.append(step)
        log.warning(
            f"Step {step:3d}: MISSED - tf_queue={'OK' if tf_queue else 'NONE'}, "
            f"img_queue={'OK' if img_queue else 'NONE'}, "
            f"spins={wait_count}, spin_time={spin_elapsed * 1000:.1f} ms, "
            f"dropped_tf={dropped_tf}, dropped_img={dropped_img}"
        )

# Print results
log.info("=" * 80)
log.info("SUMMARY")
log.info("=" * 80)
log.info(f"Total TF  callbacks received: {tf_recv_count}")
log.info(f"Total IMG callbacks received: {img_recv_count}")
log.info(f"Total test steps:             {args.test_steps}")
log.info(f"Steps with valid pair:        {len(deltas)}")
log.info(f"Missed steps:                 {len(missed_steps)} {missed_steps if missed_steps else ''}")
log.info(
    f"Settle steps used:            {settle_steps_used}/{SETTLE_MAX_STEPS} ({'converged' if settled else 'did NOT converge'})"
)

exit_code = 1
if deltas:
    avg_delay = np.mean(deltas) / 1e6
    max_delay = np.max(deltas) / 1e6
    min_delay = np.min(deltas) / 1e6
    median_delay = np.median(deltas) / 1e6
    std_delay = np.std(deltas) / 1e6

    # Threshold is set just above the rational-time rounding floor (1/60 s cannot
    # be represented exactly in nanoseconds, so consecutive stamps can toggle
    # between adjacent integers). Anything above this threshold indicates a
    # genuine multi-frame ordering issue; the next-smallest real signal would be
    # a full 1-frame slip (~16.667 ms).
    THRESHOLD_MS = 0.001  # 1 us

    # Classify deltas using the same rounding floor as the threshold so that
    # 1 ns rational-time noise does not get mis-bucketed as "Other".
    zero_count = sum(1 for d in deltas if d / 1e6 <= THRESHOLD_MS)
    one_frame_count = sum(1 for d in deltas if abs(d / 1e6 - 16.667) < 1.0)
    other_count = len(deltas) - zero_count - one_frame_count

    passed = settled and max_delay <= THRESHOLD_MS
    exit_code = 0 if passed else 1

    print("\nTF / CAMERA TIMESTAMP SYNC TEST")
    print(f"Steps:              {len(deltas)}")
    print(f"Average delay:      {avg_delay:.3f} ms")
    print(f"Median delay:       {median_delay:.3f} ms")
    print(f"Std dev:            {std_delay:.3f} ms")
    print(f"Min delay:          {min_delay:.3f} ms")
    print(f"Max delay:          {max_delay:.3f} ms  (threshold: {THRESHOLD_MS} ms)")
    print(f"Zero-delta steps:   {zero_count}/{len(deltas)}")
    print(f"One-frame (~16ms):  {one_frame_count}/{len(deltas)}")
    print(f"Other:              {other_count}/{len(deltas)}")
    print("Status:        ", "PASS ✅" if passed else "[fatal] ❌")

    if not passed:
        if not settled:
            log.error(
                "FAIL: pre-test settle phase did not converge to consecutive synced pairs. "
                "This indicates persistent startup/ordering instability before measurement."
            )
        else:
            log.error(
                f"FAIL: max_delay={max_delay:.3f} ms >= {THRESHOLD_MS} ms threshold. "
                f"At least one frame had TF and camera timestamps from different "
                f"simulation frames. A delay of ~16.667 ms = exactly 1 frame at 60 FPS, "
                f"suggesting TF is stamped from action-graph simulation time while "
                f"the camera image is stamped from the render product's reference time."
            )
else:
    print("\n[error] No messages received.")
    log.error(
        f"No message pairs received at all. "
        f"TF callbacks={tf_recv_count}, IMG callbacks={img_recv_count}. "
        f"Check that the action graph is wired correctly and ROS2 bridge is loaded."
    )

# Cleanup
node.destroy_node()
ros_executor.shutdown()
rclpy.shutdown(context=ros_context)
simulation_app.close(exit_code=exit_code)
raise SystemExit(exit_code)
