---
name: validation-diff-gifs
description: "Pixel-diff GIFs comparing validation captures to golden images. Use to triage benchmark or regression image failures."
license: Apache-2.0
metadata:
  author: Chris Dodd <info@nvidia.com>
  permissions:
    - shell
    - filesystem
---

# Validation Difference GIFs

## Purpose

Animate pixel differences between validation captures and golden images to quickly localize benchmark visual regressions.

## Limitations

- Sensitive to tolerance settings and driver/GPU differences in captures.
- Requires existing golden image directories from a benchmark run.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| No golden directory | Benchmark not run with validation | Generate captures before diffing |
| Full-frame red diff | Tolerance too tight or lighting drift | Adjust per-channel threshold; relight scene |
| GIF empty | Zero changed pixels | Confirm capture and golden paths match resolution |
| "All frames passed" (SDG) | QA report shows no failures | Thresholds may need tightening; pass `-` to diff all frames |
| jq not found (SDG) | QA report filtering requires jq | Install jq or omit the qa_report argument to diff all frames |

Generate per-camera GIF animations showing the pixel-wise difference between captured benchmark images and their golden references. Useful for debugging validation tolerance failures.

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/generate_diff_gifs.sh` | Benchmark per-camera diff GIFs (nested `Robots/` layout) | positional args per script header |
| `scripts/sdg_tolerance_diff_gifs.sh` | SDG golden-set diff GIFs (flat `rgb/` layout), optionally filtered to QA-failed frames | positional args per script header |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/generate_diff_gifs.sh", args=[])
```

From a built Isaac Sim tree, run the same file with `./python.sh` (Linux) or `python.bat` (Windows) from `_build/*/release`, or execute shell helpers directly when they do not require the simulator.

## Prerequisites

- ImageMagick (`composite`, `convert`) must be available on `PATH`.
- A completed validation capture run with matching directory structure to the golden data.

## Paths

Captures and golden data live under the Isaac Sim build release directory:

```
standalone_examples/benchmarks/validation/captures/<run_name>/
standalone_examples/benchmarks/validation/golden_data/<benchmark_name>/
```

Both share a parallel directory tree (e.g. `Robots/Robot_0/.../front_hawk/left/camera_left/rgb/`).

## Usage

### Find the latest capture

```bash
ls -td standalone_examples/benchmarks/validation/captures/benchmark_robots_nova_carter_ros2_*/ | head -1
```

### Run the script

The bundled script is resolved via `${CLAUDE_SKILL_DIR}`; falls back to the canonical repo path `skills/validation-diff-gifs/scripts/`:

```bash
bash "${CLAUDE_SKILL_DIR:-skills/validation-diff-gifs}/scripts/generate_diff_gifs.sh" \
    <captured_run_dir> \
    <golden_benchmark_dir> \
    [amplify] [fps]
```

| Argument | Default | Description |
|---|---|---|
| `captured_run_dir` | (required) | Root of the capture run |
| `golden_benchmark_dir` | (required) | Root of the golden data for the benchmark |
| `amplify` | `10` | Multiply pixel differences by this factor for visibility |
| `fps` | `5` | Frame rate for the output GIF |

### Example

```bash
LATEST=$(ls -td standalone_examples/benchmarks/validation/captures/benchmark_robots_nova_carter_ros2_*/ | head -1)
GOLDEN="standalone_examples/benchmarks/validation/golden_data/benchmark_robots_nova_carter_ros2"

bash "${CLAUDE_SKILL_DIR:-skills/validation-diff-gifs}/scripts/generate_diff_gifs.sh" "$LATEST" "$GOLDEN"
```

## Output

For each `rgb/` directory under the capture, a `diff_animation.gif` is written alongside the captured PNGs:

```
<captured_run_dir>/Robots/.../front_hawk/left/camera_left/rgb/diff_animation.gif
```

- **Black pixels** = identical between captured and golden
- **Brighter pixels** = larger difference (amplified by the `amplify` factor)

## SDG Golden-Set Tolerance Diffs

When `production_capture_qa.py` (from the `data-collection-sim` skill) produces a failing `qa_report.json`, use the SDG script to generate diffs only for the frames that tripped tolerance thresholds:

```bash
bash "${CLAUDE_SKILL_DIR:-skills/validation-diff-gifs}/scripts/sdg_tolerance_diff_gifs.sh" \
    <sdg_output_dir> \
    <sdg_golden_dir> \
    [qa_report.json] [amplify] [fps]
```

| Argument | Default | Description |
|---|---|---|
| `sdg_output_dir` | (required) | SDG capture output (flat: `rgb/`, `distance_to_image_plane/`, etc.) |
| `sdg_golden_dir` | (required) | Golden reference with the same flat layout |
| `qa_report.json` | (optional) | Path to QA report; only failed frames are diffed. Pass `-` to diff all. |
| `amplify` | `10` | Pixel difference multiplier |
| `fps` | `5` | Output GIF frame rate |

### SDG Example (CI integration)

```bash
# After a production_capture_qa.py run exits non-zero:
SDG_OUT="/data/sdg_production_run"
SDG_GOLDEN="/data/sdg_golden/warehouse_baseline"
QA_REPORT="$SDG_OUT/qa_report.json"

bash "${CLAUDE_SKILL_DIR:-skills/validation-diff-gifs}/scripts/sdg_tolerance_diff_gifs.sh" \
    "$SDG_OUT" "$SDG_GOLDEN" "$QA_REPORT"

# Outputs:
#   /data/sdg_production_run/rgb/diff_animation.gif
#   /data/sdg_production_run/rgb/diff_failed_frames.txt
#   /data/sdg_production_run/distance_to_image_plane/diff_animation.gif
```

### SDG Output Structure

```
<sdg_output_dir>/
├── rgb/
│   ├── rgb_0000.png ... rgb_0199.png
│   ├── diff_animation.gif          ← animated diff (failed frames only when filtered)
│   └── diff_failed_frames.txt      ← frame indices that tripped thresholds
├── distance_to_image_plane/
│   ├── distance_to_image_plane_0000.npy ...
│   └── diff_animation.gif          ← depth diff (grayscale)
└── qa_report.json                   ← from production_capture_qa.py
```

### Prerequisites (SDG script)

- ImageMagick (`composite`, `convert`)
- `jq` (only when passing a `qa_report.json`)
- Python 3 with `numpy` (for depth `.npy` diffing); `Pillow` optional but recommended

## Interpreting results

- Uniform low-level noise across the frame → rendering non-determinism (likely acceptable)
- Bright regions concentrated on object edges → sub-pixel movement differences
- Entire frame bright → wrong timestamp match or completely different camera pose
- One camera consistently worse than others → possible per-camera issue (tick rate, initialization)
- SDG: clustered failures at specific frame indices → seed-dependent pose or lighting issue
