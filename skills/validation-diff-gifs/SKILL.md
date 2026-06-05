---
name: validation-diff-gifs
description: Generate pixel-difference GIF animations comparing benchmark validation captures against golden images. Use when the user asks to visualize validation failures, compare captured vs golden images, generate diff GIFs, or debug image tolerance issues.
---

# Validation Difference GIFs

Generate per-camera GIF animations showing the pixel-wise difference between captured benchmark images and their golden references. Useful for debugging validation tolerance failures.

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

## Interpreting results

- Uniform low-level noise across the frame → rendering non-determinism (likely acceptable)
- Bright regions concentrated on object edges → sub-pixel movement differences
- Entire frame bright → wrong timestamp match or completely different camera pose
- One camera consistently worse than others → possible per-camera issue (tick rate, initialization)
