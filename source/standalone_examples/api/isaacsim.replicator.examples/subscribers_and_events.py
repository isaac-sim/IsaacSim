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

"""Demonstrate timeline, physics, render, and app update event subscriptions."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import time

import carb.eventdispatcher
import carb.settings
import omni.kit.app
import omni.physics.core
import omni.timeline
import omni.usd
from pxr import PhysxSchema, UsdPhysics

# TIMELINE / STAGE
USE_CUSTOM_TIMELINE_SETTINGS = True
USE_FIXED_TIME_STEPPING = True
PLAY_EVERY_FRAME = True
PLAY_DELAY_COMPENSATION = 0.0
SUBSAMPLE_RATE = 1
STAGE_FPS = 30.0

# PHYSX
USE_CUSTOM_PHYSX_FPS = False
PHYSX_FPS = 60.0
MIN_SIM_FPS = 30

# Simulations can also be enabled/disabled at runtime
DISABLE_SIMULATIONS = False

# APP / RENDER
LIMIT_APP_FPS = False
APP_FPS = 120

# Number of app updates to run while collecting events
NUM_APP_UPDATES = 100

# Print the captured events
VERBOSE = False


def on_timeline_event(event: carb.eventdispatcher.Event):
    """Handle timeline tick events."""
    global timeline_events
    timeline_events.append(event)
    if VERBOSE:
        print(f"  [timeline][{len(timeline_events)}] {event}")


def on_physics_step(dt, context):
    """Handle physics step events."""
    global physics_events
    physics_events.append(dt)
    if VERBOSE:
        print(f"  [physics][{len(physics_events)}] dt={dt}")


def on_stage_render_event(event: carb.eventdispatcher.Event):
    """Handle stage render new frame events."""
    global stage_render_events
    stage_render_events.append(event.event_name)
    if VERBOSE:
        print(f"  [stage render][{len(stage_render_events)}] {event.event_name}")


def on_app_update(event: carb.eventdispatcher.Event):
    """Handle application update events."""
    global app_update_events
    app_update_events.append(event.event_name)
    if VERBOSE:
        print(f"  [app update][{len(app_update_events)}] {event.event_name}")


stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()


if USE_CUSTOM_TIMELINE_SETTINGS:
    # When True, the timeline forces `dt = 1 / timeCodesPerSecond` per accepted tick
    # (ignoring the run loop's measured wall-clock dt). On Play it also overrides
    # `/app/runLoops/main/rateLimitFrequency` to a value computed from
    # `targetFramerate` and `timeCodesPerSecond`.
    # Default: True in editor, False in standalone.
    # NOTE:
    # - If the app cannot sustain that rate, animation playback may slow down (see 'CompensatePlayDelayInSecs').
    # - For performance benchmarks, turn this off so the timeline advances by the loop's measured dt.
    carb.settings.get_settings().set("/app/player/useFixedTimeStepping", USE_FIXED_TIME_STEPPING)

    # This compensates for frames that require more computation time than the frame's fixed delta time, by temporarily speeding up playback.
    # The parameter represents the length of these "faster" playback periods, which means that it must be larger than the fixed frame time to take effect.
    # Default: 0.0
    # NOTE:
    # - only effective if `useFixedTimeStepping` is set to True
    # - setting a large value results in long fast playback after a huge lag spike
    carb.settings.get_settings().set("/app/player/CompensatePlayDelayInSecs", PLAY_DELAY_COMPENSATION)

    # If set to True, no frames are skipped and in every frame time advances by `1 / TimeCodesPerSecond`.
    # Default: False
    # NOTE:
    # - only effective if `useFixedTimeStepping` is set to True
    # - simulation is usually faster than real-time and processing is only limited by the frame rate of the runloop
    # - useful for recording
    # - same as `carb.settings.get_settings().set("/app/player/useFastMode", PLAY_EVERY_FRAME)`
    timeline.set_play_every_frame(PLAY_EVERY_FRAME)

    # Timeline sub-stepping, i.e. how many times updates are called (update events are dispatched) each frame.
    # Default: 1
    # NOTE: same as `carb.settings.get_settings().set("/app/player/timelineSubsampleRate", SUBSAMPLE_RATE)`
    timeline.set_ticks_per_frame(SUBSAMPLE_RATE)

    # Time codes per second for the stage
    # NOTE: same as `stage.SetTimeCodesPerSecond(STAGE_FPS)` and `carb.settings.get_settings().set("/app/stage/timeCodesPerSecond", STAGE_FPS)`
    timeline.set_time_codes_per_second(STAGE_FPS)


# Create a PhysX scene to set the physics time step
if USE_CUSTOM_PHYSX_FPS:
    physx_scene = None
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            physx_scene = PhysxSchema.PhysxSceneAPI.Apply(prim)
            break
    if physx_scene is None:
        UsdPhysics.Scene.Define(stage, "/PhysicsScene")
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/PhysicsScene"))

    # Time step for the physics simulation
    # Default: 60.0
    physx_scene.GetTimeStepsPerSecondAttr().Set(PHYSX_FPS)

    # Minimum simulation frequency to prevent clamping; if the frame rate drops below this,
    # physics steps are discarded to avoid app slowdown if the overall frame rate is too low.
    # Default: 30.0
    # NOTE: Matching `minFrameRate` with `TimeStepsPerSecond` ensures a single physics step per update.
    carb.settings.get_settings().set("/persistent/simulation/minFrameRate", MIN_SIM_FPS)


# Throttle Render/UI/Main thread update rate
if LIMIT_APP_FPS:
    # Enable rate limiting of the main run loop (UI, rendering, etc.)
    # Default: False
    carb.settings.get_settings().set("/app/runLoops/main/rateLimitEnabled", LIMIT_APP_FPS)

    # FPS limit of the main run loop (UI, rendering, etc.)
    # Default: 120
    # NOTE: This caps the loop's tick rate (sleeps at end of frame); it does NOT set the
    # timeline's per-tick dt. On Play with `/app/player/useFixedTimeStepping=True`,
    # the timeline computes its own `rateLimitFrequency` from `targetFramerate` and
    # `timeCodesPerSecond` and overrides this value at that moment.
    carb.settings.get_settings().set("/app/runLoops/main/rateLimitFrequency", int(APP_FPS))


# Simulations can be selectively disabled (or toggled at specific times)
if DISABLE_SIMULATIONS:
    carb.settings.get_settings().set("/app/player/playSimulations", False)

print("Configuration:")
print(f"  Timeline:")
print(f"    - Stage FPS: {STAGE_FPS}  (/app/stage/timeCodesPerSecond)")
print(f"    - Fixed time stepping: {USE_FIXED_TIME_STEPPING}  (/app/player/useFixedTimeStepping)")
print(f"    - Play every frame: {PLAY_EVERY_FRAME}  (/app/player/useFastMode)")
print(f"    - Subsample rate: {SUBSAMPLE_RATE}  (/app/player/timelineSubsampleRate)")
print(f"    - Play delay compensation: {PLAY_DELAY_COMPENSATION}s  (/app/player/CompensatePlayDelayInSecs)")
print(f"  Physics:")
print(f"    - PhysX FPS: {PHYSX_FPS}  (physxScene.timeStepsPerSecond)")
print(f"    - PhysX min frame-rate clamp: {MIN_SIM_FPS}  (/persistent/simulation/minFrameRate)")
print(f"    - Simulations enabled: {not DISABLE_SIMULATIONS}  (/app/player/playSimulations)")
print(f"  Rendering:")
print(f"    - App FPS limit: {APP_FPS if LIMIT_APP_FPS else 'unlimited'}  (/app/runLoops/main/rateLimitFrequency)")


# Start the timeline
print(f"Starting the timeline...")
timeline.set_current_time(0)
timeline.set_end_time(10000)
timeline.set_looping(False)
timeline.play()
timeline.commit()
wall_start_time = time.time()

# Subscribe to events
print(f"Subscribing to events...")
timeline_events = []
timeline_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
    event_name=omni.timeline.GLOBAL_EVENT_CURRENT_TIME_TICKED,
    on_event=on_timeline_event,
    observer_name="subscribers_and_events.on_timeline_event",
)
physics_events = []
physics_sub = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
    pre_step=False, order=0, on_update=on_physics_step
)
stage_render_events = []
stage_render_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
    event_name=omni.usd.get_context().stage_rendering_event_name(omni.usd.StageRenderingEventType.NEW_FRAME, True),
    on_event=on_stage_render_event,
    observer_name="subscribers_and_events.on_stage_render_event",
)
app_update_events = []
app_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
    event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
    on_event=on_app_update,
    observer_name="subscribers_and_events.on_app_update",
)

# Run app updates and cache events
print(f"Starting running the application for {NUM_APP_UPDATES} updates.")
for i in range(NUM_APP_UPDATES):
    if VERBOSE:
        print(f"[app update loop][{i+1}/{NUM_APP_UPDATES}]")
    simulation_app.update()
elapsed_wall_time = time.time() - wall_start_time
print(f"Finished running the application for {NUM_APP_UPDATES} updates...")

# Stop timeline and unsubscribe from all events
timeline.stop()
app_sub = None
stage_render_sub = None
if physics_sub:
    physics_sub.unsubscribe()
    physics_sub = None
timeline_sub = None


# Print summary statistics
print("\nStats:")
print(f"- App updates: {NUM_APP_UPDATES}")
print(f"- Wall time: {elapsed_wall_time:.4f} seconds")
print(f"- Timeline events: {len(timeline_events)}")
print(f"- Physics events: {len(physics_events)}")
print(f"- Stage render events: {len(stage_render_events)}")
print(f"- App update events: {len(app_update_events)}")

# Calculate and display real-time performance factor
if len(physics_events) > 0:
    sim_time = sum(physics_events)
    realtime_factor = sim_time / elapsed_wall_time if elapsed_wall_time > 0 else 0
    print(f"- Simulation time: {sim_time:.4f}s")
    print(f"- Real-time factor: {realtime_factor:.2f}x")

simulation_app.close()
