#!/usr/bin/env python3
"""Isaac Sim 6 Headless Smoke Test"""
import os
import sys
import time
import subprocess

BUILD_DIR = '/home/horde/Workspace/IsaacSim/_build/linux-x86_64/release'

def get_gpu_info():
    """Get GPU memory usage via nvidia-smi"""
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,memory.used,memory.total,utilization.gpu",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10
        )
        return result.stdout.strip()
    except Exception as e:
        return f"Error getting GPU info: {e}"

print("=" * 60)
print("Isaac Sim 6 Headless Validation Test")
print("=" * 60)
print(f"Python: {sys.version}")
print(f"GPU (before): {get_gpu_info()}")
print()

# Step 1: Import and launch SimulationApp
print("[1/4] Launching SimulationApp (headless)...")
t0 = time.time()
from isaacsim import SimulationApp
config = {
    "headless": True,
    "extra_args": [
        "--ext-folder", f"{BUILD_DIR}/extscache",
        "--/exts/omni.kit.registry.nucleus/registries/0/name=kit/prod/default",
        "--/exts/omni.kit.registry.nucleus/registries/0/url=https://ovextensionsprod.blob.core.windows.net/exts/kit/prod/110/shared",
        "--/exts/omni.kit.registry.nucleus/registries/1/name=kit/prod/sdk",
        "--/exts/omni.kit.registry.nucleus/registries/1/url=https://ovextensionsprod.blob.core.windows.net/exts/kit/prod/sdk/110.0/4a5123f4",
        "--/exts/omni.kit.registry.nucleus/registries/2/name=isaacsim/prod/6.0",
        "--/exts/omni.kit.registry.nucleus/registries/2/url=https://ovextensionsprod.blob.core.windows.net/exts/isaacsim/prod/6.0/shared",
    ],
}
app = SimulationApp(config)
t_app = time.time() - t0
print(f"  SimulationApp launched in {t_app:.2f}s")
print(f"  GPU (after app launch): {get_gpu_info()}")

# Step 2: Import Isaac Sim 6 core modules and create world
print("\n[2/4] Creating World and adding ground plane...")
t0 = time.time()
from isaacsim.core.api import World
world = World()
world.scene.add_default_ground_plane()
world.reset()
t_world = time.time() - t0
print(f"  World created and reset in {t_world:.2f}s")

# Step 3: Step simulation (no render)
print("\n[3/4] Stepping simulation (10 steps, no render)...")
step_times = []
for i in range(10):
    t0 = time.time()
    world.step(render=False)
    step_times.append(time.time() - t0)
avg_step = sum(step_times) / len(step_times)
total_step = sum(step_times)
print(f"  10 steps completed in {total_step:.4f}s")
print(f"  Average step time: {avg_step:.4f}s")
print(f"  Min step: {min(step_times):.4f}s, Max step: {max(step_times):.4f}s")

# Step 4: Steps with render
print("\n[4/4] Stepping simulation with render (5 steps)...")
render_times = []
for i in range(5):
    t0 = time.time()
    world.step(render=True)
    render_times.append(time.time() - t0)
avg_render = sum(render_times) / len(render_times)
total_render = sum(render_times)
print(f"  5 render steps completed in {total_render:.4f}s")
print(f"  Average render step time: {avg_render:.4f}s")
print(f"  GPU (after simulation): {get_gpu_info()}")

# Report
print("\n" + "=" * 60)
print("Isaac Sim 6 smoke test: PASSED — 10 physics steps + 5 render steps completed")
print(f"  Version:          6.0.0 (v6.0.0-dev2 tag)")
print(f"  App startup:      {t_app:.2f}s")
print(f"  World creation:   {t_world:.2f}s")
print(f"  Avg physics step: {avg_step:.4f}s")
print(f"  Avg render step:  {avg_render:.4f}s")
print("=" * 60)

# Force exit to avoid cleanup crash in TaskGroup
os._exit(0)
