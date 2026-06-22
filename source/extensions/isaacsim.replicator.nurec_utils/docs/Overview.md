# Isaac Sim Replicator NuRec Utils

`isaacsim.replicator.nurec_utils` is an experimental module for rendering and checking NuRec (neural reconstruction) USD assets in Isaac Sim. It is intended for validation, regression tests, and debugging workflows where a NuRec asset must be rendered at known camera views and compared against reference imagery.

> **Experimental:** APIs, command-line behavior, supported NuRec formats, and output files are not guaranteed to stay future compatible.

## What It Provides

The module provides reusable helpers for NuRec USD rendering workflows:

- Stage detection for NuRec particle-field / gaussian-splat assets, NuRec volume assets, and SPG/PPISP render paths.
- Render setup helpers that apply the stage-specific settings required before first Hydra sync.
- Render-product and capture helpers for authored SPG render products and plain camera render products.
- Pose, timestamp, manifest, image, and sampling helpers for rendering sensor-rig keyframes or explicit TUM-format poses.
- Evaluation helpers for render-vs-ground-truth checks, including PSNR, SSIM, mean absolute difference, comparison panels, metric plots, pose heatmaps, and scoped timing.

Runnable tutorials, command lines, ground-truth directory layout, and custom-script setup examples live in the NuRec rendering utilities user guide under `docs/isaacsim/assets/nurec_utils.rst`.
