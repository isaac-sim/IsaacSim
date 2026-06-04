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

"""Standalone script to validate async rendering with handshake mode.

This script runs a simple simulation with async rendering and handshake enabled,
then validates that the FrametimeStats contain both App_Update and Render stats
with reasonable values.

Usage:
    python benchmark_async_handshake_validation.py [--num-frames N]
"""

import argparse
import sys

# Parse arguments before SimulationApp initialization
parser = argparse.ArgumentParser(description="Validate async rendering with handshake mode")
parser.add_argument("--num-frames", type=int, default=100, help="Number of frames to simulate (default: 100)")
parser.add_argument("--non-headless", action="store_false", help="Run with GUI - nonheadless mode")
args, unknown = parser.parse_known_args()

headless = args.non_headless

from isaacsim import SimulationApp

# Launch SimulationApp with async rendering and handshake enabled
# Note: /renderer/asyncInit=false forces synchronous hydra engine initialization.
# This ensures the RTX engine attaches during startup (critical for framework test mode).
# This is separate from /app/asyncRendering which controls async render thread execution.
# Explicitly disable multi-tick rendering while async is enabled.
simulation_app = SimulationApp(
    {
        "headless": headless,
        "extra_args": [
            "--/app/asyncRendering=true",
            "--/app/omni.usd/asyncHandshake=true",
            "--/renderer/asyncInit=false",
            "--/rtx/hydra/supportMultiTickRate=false",
            "--/rtx/rendering/perSensorTickTlas=false",
        ],
    }
)

import carb

# Enable the benchmark services extension to access collectors
import omni.kit.app
import omni.usd

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.benchmark.services", True)

# Wait for extension to fully load
for _ in range(10):
    omni.kit.app.get_app().update()


from isaacsim.core.api import SimulationContext
from pxr import UsdGeom


def validate_async_handshake_behavior(num_frames: int = 100) -> bool:
    """Run a simulation with async rendering + handshake and validate behavior.

    Args:
        num_frames: Number of frames to simulate.

    Returns:
        True if validation passes, False otherwise.
    """
    # Number of boundary frames to discard from each end of the collection
    # window when computing the frametime ratio (see trimming logic below).
    TRIM_FRAMES = 2

    # Verify async settings are actually enabled
    settings = carb.settings.get_settings()
    async_rendering = settings.get("/app/asyncRendering")
    async_handshake = settings.get("/app/omni.usd/asyncHandshake")
    replicator_async = settings.get("/omni/replicator/asyncRendering")

    print("=" * 60)
    print("ASYNC RENDERING SETTINGS")
    print("=" * 60)
    print(f"  /app/asyncRendering: {async_rendering}")
    print(f"  /app/omni.usd/asyncHandshake: {async_handshake}")
    print(f"  /omni/replicator/asyncRendering: {replicator_async}")
    print("=" * 60)
    print()

    if not async_rendering:
        print("✗ [error] Async rendering is NOT enabled!")
        return False
    if not async_handshake:
        print("✗ [error] Async handshake is NOT enabled!")
        return False

    # Create a new stage with simple scene
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    # Add simple geometry to trigger rendering
    sphere = UsdGeom.Sphere.Define(stage, "/World/Sphere")
    sphere.GetRadiusAttr().Set(1.0)

    # Create simulation context and initialize physics
    simulation_context = SimulationContext()
    simulation_context.initialize_physics()

    print(f"Starting simulation with async rendering + handshake enabled")
    print(f"Will simulate {num_frames} frames")
    print()

    # Start recording app, render, GPU, and physics frametimes
    from isaacsim.benchmark.services.datarecorders.app_frametime import AppFrametimeRecorder
    from isaacsim.benchmark.services.datarecorders.gpu_frametime import GPUFrametimeRecorder
    from isaacsim.benchmark.services.datarecorders.physics_frametime import PhysicsFrametimeRecorder
    from isaacsim.benchmark.services.datarecorders.render_frametime import RenderFrametimeRecorder

    app_recorder = AppFrametimeRecorder()
    render_recorder = RenderFrametimeRecorder()
    gpu_recorder = GPUFrametimeRecorder()
    physics_recorder = PhysicsFrametimeRecorder()

    # Start simulation
    simulation_context.play()

    # Give render thread time to initialize
    for i in range(5):
        simulation_context.step()

    # Start collecting now that we've warmed up.
    app_recorder.start_collecting()
    render_recorder.start_collecting()
    gpu_recorder.start_collecting()
    physics_recorder.start_collecting()

    carb.log_info(
        f"After warmup - Render samples: {render_recorder.sample_count}, App samples: {app_recorder.sample_count}"
    )

    # Run extra frames to compensate for boundary trimming so the trimmed
    # dataset still contains num_frames of usable data.
    total_frames = num_frames + 2 * TRIM_FRAMES
    for frame in range(total_frames):
        simulation_context.step()
        if frame % 10 == 0:
            carb.log_info(
                f"Frame {frame}/{total_frames} - Render: {render_recorder.sample_count}, App: {app_recorder.sample_count}"
            )

    # Stop collecting
    app_recorder.stop_collecting()
    render_recorder.stop_collecting()
    gpu_recorder.stop_collecting()
    physics_recorder.stop_collecting()

    carb.log_info(
        f"Collection stopped - Final counts: Render={render_recorder.sample_count}, App={app_recorder.sample_count}"
    )

    # Analyze results
    print()
    print("=" * 60)
    print("VALIDATION RESULTS")
    print("=" * 60)

    validation_passed = True

    # Check App_Update stats
    app_frames = app_recorder.sample_count
    if app_frames > 0:
        app_mean = sum(app_recorder.samples) / len(app_recorder.samples)
        print(f"✓ App_Update frames collected: {app_frames}")
        print(f"  Mean App_Update frametime: {app_mean:.2f} ms")
    else:
        print("✗ [fail] No App_Update frames collected!")
        validation_passed = False

    # Check Render stats (key validation for async + handshake)
    render_frames = render_recorder.sample_count
    if render_frames > 0:
        render_mean = sum(render_recorder.samples) / len(render_recorder.samples)
        print(f"✓ Render frames collected: {render_frames}")
        print(f"  Mean Render frametime: {render_mean:.2f} ms")

        # Validate that render stats exist (they don't in sync mode)
        print("✓ Render stats present (confirms async rendering is active)")
    else:
        print("✗ [fail] No Render frames collected!")
        print("  This suggests async rendering may not be working correctly")
        validation_passed = False

    # Validate frame count similarity (key for handshake validation)
    if app_frames > 0 and render_frames > 0:
        frame_diff = abs(app_frames - render_frames)
        frame_tolerance = max(app_frames, render_frames) * 0.05  # 5% tolerance

        if frame_diff < frame_tolerance:
            print(f"✓ Frame counts similar: App={app_frames}, Render={render_frames}")
            print("  This confirms handshake is synchronizing threads")
        else:
            print(f"⚠ Frame count mismatch: App={app_frames}, Render={render_frames}")
            print(f"  Difference: {frame_diff}, tolerance: {frame_tolerance:.1f}")
            print("  Handshake may not be working as expected")
            validation_passed = False

        # Build paired frame data and apply trimming to remove collection
        # boundary artifacts. The first/last frames of a collection window often
        # have anomalous timing because stop_collecting() is not instantaneous
        # across the app and render threads (e.g. a final app sample with no
        # corresponding render sample, or transient spikes at collection start).
        # Use index alignment for the ratio: pair app[i] with render[i] over the
        # overlapping range. Display preserves holes (all frame indices 0..max-1)
        # with "-" where one stream has no sample; unpaired rows are marked [TRIMMED].
        app_count = len(app_recorder.samples)
        render_count = len(render_recorder.samples)
        paired_count = min(app_count, render_count)
        app_samples_raw = list(app_recorder.samples[:paired_count])
        render_samples_raw = list(render_recorder.samples[:paired_count])

        if paired_count > TRIM_FRAMES * 2 + 1:
            app_samples_trimmed = app_samples_raw[TRIM_FRAMES:-TRIM_FRAMES]
            render_samples_trimmed = render_samples_raw[TRIM_FRAMES:-TRIM_FRAMES]
        else:
            app_samples_trimmed = app_samples_raw
            render_samples_trimmed = render_samples_raw

        trimmed_app_mean = sum(app_samples_trimmed) / len(app_samples_trimmed)
        trimmed_render_mean = sum(render_samples_trimmed) / len(render_samples_trimmed)

        print(
            f"  Trimmed analysis: using {len(app_samples_trimmed)} of {paired_count} paired frames"
            f" (dropped first/last {TRIM_FRAMES})"
        )
        trimmed = paired_count > TRIM_FRAMES * 2 + 1
        if trimmed:
            print(
                f"  Trimmed analysis: using {len(app_samples_trimmed)} of {paired_count} paired frames"
                f" (dropped first/last {TRIM_FRAMES})"
            )
        else:
            print(f"  Trimmed analysis: using all {paired_count} paired frames" f" (too few to trim)")
        print(f"  Trimmed mean App: {trimmed_app_mean:.2f} ms, Render: {trimmed_render_mean:.2f} ms")

        # Validate mean frametime similarity on the trimmed data
        ratio = max(trimmed_app_mean, trimmed_render_mean) / min(trimmed_app_mean, trimmed_render_mean)
        if ratio < 1.10:
            print(f"✓ Frametime ratio reasonable: {ratio:.2f}x (trimmed)")
            print("  App and Render threads are in sync")
        else:
            print(f"⚠ Large frametime ratio: {ratio:.2f}x (trimmed)")
            print("  Threads may not be properly synchronized")

            # Debug output: show individual frame times, preserving holes (unpaired
            # frames shown with "-" and marked [TRIMMED]).
            print()
            print("  DEBUG: Individual frame times (first 20 and last 20):")
            print("-" * 60)
            trimming_active = paired_count > TRIM_FRAMES * 2 + 1
            total_display_frames = max(app_count, render_count)
            if trimming_active or total_display_frames > paired_count:
                print("  (Frames marked [TRIMMED] are excluded from the ratio or unpaired.)")
                print()

            def is_trimmed(idx):
                boundary = trimming_active and (idx < TRIM_FRAMES or idx >= paired_count - TRIM_FRAMES)
                unpaired = idx >= app_count or idx >= render_count
                return boundary or unpaired

            def format_row(i):
                app_val = app_recorder.samples[i] if i < app_count else None
                render_val = render_recorder.samples[i] if i < render_count else None
                app_str = f"{app_val:.2f}" if app_val is not None else "-"
                render_str = f"{render_val:.2f}" if render_val is not None else "-"
                if app_val is not None and render_val is not None and render_val > 0:
                    ratio_str = f"{app_val / render_val:.2f}"
                else:
                    ratio_str = "-"
                marker = " [TRIMMED]" if is_trimmed(i) else ""
                return f"  {i:<8} {app_str:<12} {render_str:<12} {ratio_str:<8}{marker}"

            num_to_show = min(20, total_display_frames)
            print(f"  First {num_to_show} frames:")
            print(f"  {'Frame':<8} {'App (ms)':<12} {'Render (ms)':<12} {'Ratio':<8}")
            for i in range(num_to_show):
                print(format_row(i))

            if total_display_frames > num_to_show:
                print()
                print(f"  Last {num_to_show} frames:")
                print(f"  {'Frame':<8} {'App (ms)':<12} {'Render (ms)':<12} {'Ratio':<8}")
                start_idx = max(0, total_display_frames - num_to_show)
                for i in range(start_idx, total_display_frames):
                    print(format_row(i))

            # Show statistics
            print()
            print("  Frame time statistics (raw):")
            app_min = min(app_samples_raw)
            app_max = max(app_samples_raw)
            render_min = min(render_samples_raw)
            render_max = max(render_samples_raw)
            print(f"  App range:    {app_min:.2f} - {app_max:.2f} ms (variance: {app_max - app_min:.2f} ms)")
            print(
                f"  Render range: {render_min:.2f} - {render_max:.2f} ms (variance: {render_max - render_min:.2f} ms)"
            )
            print()
            print("  Frame time statistics (trimmed):")
            tapp_min = min(app_samples_trimmed)
            tapp_max = max(app_samples_trimmed)
            trender_min = min(render_samples_trimmed)
            trender_max = max(render_samples_trimmed)
            print(f"  App range:    {tapp_min:.2f} - {tapp_max:.2f} ms (variance: {tapp_max - tapp_min:.2f} ms)")
            print(
                f"  Render range: {trender_min:.2f} - {trender_max:.2f} ms (variance: {trender_max - trender_min:.2f} ms)"
            )
            print("-" * 60)

            validation_passed = False

    # Check GPU stats if available
    if gpu_recorder.sample_count > 0:
        gpu_mean = sum(gpu_recorder.samples) / len(gpu_recorder.samples)
        print(f"  Mean GPU frametime: {gpu_mean:.2f} ms")

    # Check Physics stats if available
    if physics_recorder.sample_count > 0:
        physics_mean = sum(physics_recorder.samples) / len(physics_recorder.samples)
        print(f"  Mean Physics frametime: {physics_mean:.2f} ms")

    print("=" * 60)

    if validation_passed:
        print("✓ VALIDATION PASSED")
        print("  Async rendering with handshake is working correctly!")
    else:
        print("✗ VALIDATION [fail] - Async rendering handshake test failed")
        print("  Issues detected with async rendering handshake mode")

    print("=" * 60)
    print()

    return validation_passed


def main():
    """Main function."""
    try:
        validation_passed = validate_async_handshake_behavior(args.num_frames)

        # Clean shutdown
        simulation_app.close()

        # Return appropriate exit code
        sys.exit(0 if validation_passed else 1)

    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback

        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)


if __name__ == "__main__":
    main()
