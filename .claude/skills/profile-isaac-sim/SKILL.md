---
name: profile-isaac-sim
description: Profile and optimize Isaac Sim performance using benchmark scripts and Tracy profiling. Use when the user asks to profile, benchmark, measure performance, compare frame times, or optimize Isaac Sim workloads.
---

# Profile Isaac Sim Performance

Iterative profiling workflow: run a benchmark with GPU frame-time recording, capture a Tracy profile, export to CSV, compare against a reference, make changes, and repeat.

## Prerequisites

All commands run from the release build directory:

```bash
cd _build/linux-x86_64/release
```

### Asset root / Nucleus authentication

Benchmarks load scenes from an asset server. If `OMNI_USER` and `OMNI_PASS` are set, Nucleus authentication is automatic. Otherwise, add the S3 fallback flag to every benchmark invocation (after `--`) to skip interactive auth:

```
--/persistent/isaac/asset_root/default=https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0
```

### ROS2 environment variables

Benchmarks that use ROS2 (e.g. `robots_nova_carter_ros2`) require these environment variables so the internal humble distro is found:

```bash
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/exts/isaacsim.ros2.core/humble/lib
```

These are printed in the startup log by `isaacsim.ros2.core` if missing. Set them before running `./python.sh`.

## Step 1 — Validate User Request

When the user asks to profile a benchmark, check:

1. **`--gpu-frametime`** — must always be present. Add it if the user omitted it.
2. **`--num-frames`** — should be >= 100 for stable measurements (default in most benchmarks is 600, which is fine). If the user explicitly passes a low value like `--num-frames 10`, warn them and suggest at least 100.
3. **Benchmark short name** — resolve from the available set (see list below).

### Available benchmarks (short names)

| Short name | Script |
|---|---|
| `camera` | `benchmark_camera.py` |
| `rtx_lidar` | `benchmark_rtx_lidar.py` |
| `rtx_radar` | `benchmark_rtx_radar.py` |
| `physx_lidar` | `benchmark_physx_lidar.py` |
| `robots_nova_carter` | `benchmark_robots_nova_carter.py` |
| `robots_nova_carter_ros2` | `benchmark_robots_nova_carter_ros2.py` |
| `robots_humanoid` | `benchmark_robots_humanoid.py` |
| `robots_ur10` | `benchmark_robots_ur10.py` |
| `robots_o3dyn` | `benchmark_robots_o3dyn.py` |
| `robots_evobot` | `benchmark_robots_evobot.py` |
| `sdg` | `benchmark_sdg.py` |
| `scene_loading` | `benchmark_scene_loading.py` |
| `core_world` | `benchmark_core_world.py` |
| `single_view_depth_sensor` | `benchmark_single_view_depth_sensor.py` |

Benchmark scripts live in `source/standalone_examples/benchmarks/`.

## Step 2 — Run Benchmark + Capture Tracy Profile + CSV

Use the Tracy capture wrapper to run the benchmark, record the Tracy profile, and export a CSV — all in one shot. The wrapper also saves the full benchmark stdout/stderr to a `.log` file, which contains the summary report metrics.

```bash
./python.sh tools/profiling/tracy_capture.py \
    --benchmark <short_name> \
    --output-dir /tmp/tracy_profiles \
    --csv \
    -- --gpu-frametime --num-frames <N> <extra-args>
```

Key flags:
- `--csv` — auto-export the tracy profile to CSV (tab-separated by default).
- `--csv-sep <char>` — column separator (default: `\t`). Tab avoids conflicts with commas in C++ zone names. When reading exported CSVs, **always parse with `sep='\t'`**.
- `--csv-self` — report self-times (useful for identifying hotspots).
- `--capture-loading` — include the scene-loading phase (omit to capture only the benchmark phase).
- `--enable-python-profiling` — adds Python function scopes (slower; use only when investigating Python-level bottlenecks).
- Everything after `--` is forwarded to the benchmark script verbatim.

The output directory will contain:
- `benchmark_<name>_<timestamp>.tracy` (or `.compressed.tracy`)
- `benchmark_<name>_<timestamp>.csv`
- `benchmark_<name>_<timestamp>.log` (full stdout/stderr — includes the summary report)

Extract key metrics from the summary report at the end of the `.log`:
- Average frame time (ms)
- Min / Max frame time
- FPS (frames per second)
- GPU frame time stats (when `--gpu-frametime` is enabled)

## Step 3 — Compare CSVs and Analyze

### Zone-level comparison

Run the comparison script bundled with this skill (resolved via `${CLAUDE_SKILL_DIR}`; falls back to the canonical repo path):

```bash
python3 "${CLAUDE_SKILL_DIR:-skills/profile-isaac-sim}/scripts/compare_tracy_csvs.py" \
    <reference.csv> <new.csv> --top 25
```

The script reads **tab-separated** Tracy CSV exports (the default from `tracy_capture.py`) and prints:
1. Top-N zones by total time in the reference, with the new run's mean and delta.
2. Zones that appear **only** in the new run (new hot paths).
3. Zones that **disappeared** from the new run.

Use `--sep ','` if comparing older comma-separated exports.

### Summary report comparison

Also extract key metrics from each run's `.log` file and present a side-by-side table:

| Metric | Reference | Current | Delta |
|---|---|---|---|
| Mean App Update Frametime | 16.5 ms | 15.2 ms | -7.9% |
| Mean GPU Frametime | 12.1 ms | 11.3 ms | -6.6% |
| Mean FPS | 60.6 FPS | 65.8 FPS | +8.6% |
| Runtime | 4,500 ms | 4,100 ms | -8.9% |
| Real Time Factor | 2.22 | 2.43 | +9.5% |

## Step 4 — Iterate

After sharing the analysis:

1. **Explain findings** — which zones dominate, what changed, hypotheses for why.
2. **Propose a change** — a specific code modification to improve performance.
3. **Ask for user feedback** — confirm the proposed change before implementing it.

After the user approves:

4. **Implement the change.**
5. **Re-run the benchmark + capture** (Step 2) with the exact same arguments.
6. **Compare to both the original reference AND the previous iteration** (Step 3) so progress is tracked across all iterations.
7. **Present the updated comparison table** with a new column for each iteration.

Repeat until the user is satisfied or no further gains are found.

### Multi-iteration table format

| Metric | Reference | v1 | v2 | v3 |
|---|---|---|---|---|
| Avg frame time (ms) | 16.5 | 15.2 | 14.8 | 14.1 |
| GPU frame time (ms) | 12.1 | 11.3 | 10.9 | 10.5 |
| FPS | 60.6 | 65.8 | 67.6 | 70.9 |

## Profiling Extension Unit Tests

When the user wants to profile a unit test (extension test) rather than a standalone benchmark, the test runner launches the actual `kit` process as a child, and Tracy must attach to that child process directly. Follow this workflow:

### Step A — Discover the child process command

Run the test normally to capture the child process command line:

```bash
cd _build/linux-x86_64/release
./tests/tests-<extension>.sh -n <test_name> -f "<TestFilter>" 2>&1 | head -30
```

Look for the line starting with `>>> running process:` immediately after `[EXTENSION TEST START: ...]`. This is the full `kit` command with all `--/` carb settings configured by the extension's `extension.toml`. Copy the entire command.

Example output:
```
||||||||||||||  [EXTENSION TEST START: isaacsim.sensors.rtx-enable_multitick_rendering]  ||||||||||||||
>>> running process: ./tests/../kit/kit .../omni.app.test_ext.kit --enable isaacsim.sensors.rtx-15.13.0 --/log/flushStandardStreamOutput=1 ... --/rtx/hydra/supportMultiTickRate=true
```

### Step B — Kill the test and rerun with Tracy settings

Kill the running test process (Ctrl+C or kill the PID), then rerun the captured command directly with Tracy profiling arguments appended:

```bash
<captured kit command> \
  --/app/profilerBackend=tracy \
  --/app/profileFromStart=true \
  --/plugins/carb.profiler-tracy.plugin/fibersAsThreads=false \
  --/plugins/carb.profiler-tracy.plugin/instantEventsAsMessages=true \
  --/rtx/addTileGpuAnnotations=true \
  --/rtx/fullFrameNumberInTileGpuAnnotations_=true \
  --/profiler/channels/carb.events/enabled=false \
  --/profiler/channels/carb.tasking/enabled=false \
  --/profiler/channels/omni.usd.multitick.render.profile/enabled=true
```

This runs the `kit` executable directly (not through the test runner wrapper), so Tracy can connect to the process. The profiling settings must be command-line arguments — setting them in Python `setUp()` is too late since Tracy must initialize at process startup.

### Step C — Capture with Tracy

Once the `kit` process is running with Tracy enabled, use `tracy-capture` to record the profile:

```bash
tracy-capture -o /tmp/unit_test_profile.tracy
```

Or connect the Tracy GUI profiler to the running process.

### Key differences from standalone benchmarks

- **Two-phase discovery**: You must first run the test to discover the child command, then rerun it directly.
- **Settings come from extension.toml**: The `--/` arguments are generated by the test infrastructure from the extension's test configuration — don't try to reconstruct them manually.
- **No `--gpu-frametime`**: Unit tests don't use the benchmark framework, so GPU frametime flags don't apply. Use Tracy's GPU zones instead.

## Guidelines

- **Never skip `--gpu-frametime`** for standalone benchmarks — GPU-side timing is essential for rendering-heavy workloads.
- **Keep arguments identical** across runs for fair comparison. If the user changes arguments mid-iteration, note it clearly and treat it as a new baseline.
- **Use `--csv-self`** when you need to isolate a zone's own cost from its children.
- **Read the `.log` file** for the full benchmark stdout (summary report, warnings, errors) — it is always produced alongside the `.tracy` and `.csv`.
- **Prompt for feedback** after every iteration — don't silently chain multiple changes.
