# Quality Criteria

This file evolves with user feedback. Each criterion is tagged with its origin.

## Structural (Level 2)

| ID | Criterion | Severity | Origin |
|----|-----------|----------|--------|
| S1 | SimulationApp created before Kit- or extension-dependent isaacsim imports | error | baseline |
| S2 | Headless mode enabled for server/VM environments | warning | baseline |
| S3 | simulation_app.close() called for clean shutdown | warning | baseline |
| S4 | Stage created or loaded before adding prims | error | baseline |

## Runtime (Level 3)

| ID | Criterion | Severity | Origin |
|----|-----------|----------|--------|
| R1 | Script runs to completion without unhandled exceptions | error | baseline |
| R2 | Expected output files exist and are non-empty | error | baseline |
| R3 | No GPU memory leaks for GPU-backed workloads (VRAM returns to baseline after close) | warning | baseline |

## Level 4 (evolves with feedback)

### Render Mode

| ID | Criterion | Severity | Origin |
|----|-----------|----------|--------|
| L4-RM1 | Iteration renders use `RayTracedLighting`; `PathTracing` reserved for hero shots only | error | SKILL.md baseline |
| L4-RM2 | ACES tonemap enabled (carb `rtx.post.tonemap.op` = 4) | warning | SKILL.md baseline |
| L4-RM3 | `filmIso` set per scene type: 200 (default), 600 (deep-aisle/indoor), 400 (aerial) | warning | SKILL.md baseline |
| L4-RM4 | Pixel variance > 15 (flat image = missing shadows) | warning | SKILL.md baseline |
| L4-RM5 | Mean RGB 80–160; reject < 20 (too dark) or > 240 (overexposed) | error | SKILL.md baseline |

### SimulationApp Size

| ID | Criterion | Severity | Origin |
|----|-----------|----------|--------|
| L4-SA1 | `SimulationApp` config includes explicit `width` and `height` | error | SKILL.md baseline |
| L4-SA2 | Resolution matches capture intent (render product dims = app dims); mismatch causes swapchain-capture size artifacts | error | SKILL.md baseline |
| L4-SA3 | Headless mode (`headless=True`) set when running on server/VM/CI | warning | SKILL.md baseline |

### RL Output Formats

| ID | Criterion | Severity | Origin |
|----|-----------|----------|--------|
| L4-RL1 | Checkpoint named `model_XXXX.zip` or `policy_only_XXXX.pt`; reject `policy.pt` or unnamed files | error | SKILL.md baseline |
| L4-RL2 | Training run produces reward curve (`log.csv`, `wandb` URL, or `results.json` with episode data) | error | SKILL.md baseline |
| L4-RL3 | Logs include `episode/reward_mean` and `train/value_loss` | error | SKILL.md baseline |
| L4-RL4 | Multiple seeds run for valid comparison (recommend seeds 42, 123, 456) | warning | SKILL.md baseline |
| L4-RL5 | `PYTHONUNBUFFERED=1` set so training logs stream in real time | warning | SKILL.md baseline |
| L4-RL6 | GPU memory logged (`torch.cuda.memory_allocated()/1e6`) to W&B or equivalent | warning | SKILL.md baseline |
