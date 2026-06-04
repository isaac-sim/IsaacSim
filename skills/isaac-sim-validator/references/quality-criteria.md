# Quality Criteria

This file evolves with user feedback. Each criterion is tagged with its origin.

## Structural (Level 2)

| ID | Criterion | Severity | Origin |
|----|-----------|----------|--------|
| S1 | SimulationApp created before all other isaacsim imports | error | baseline |
| S2 | Headless mode enabled for server/VM environments | warning | baseline |
| S3 | simulation_app.close() called for clean shutdown | warning | baseline |
| S4 | Stage created or loaded before adding prims | error | baseline |

## Runtime (Level 3)

| ID | Criterion | Severity | Origin |
|----|-----------|----------|--------|
| R1 | Script runs to completion without unhandled exceptions | error | baseline |
| R2 | Expected output files exist and are non-empty | error | baseline |
| R3 | No GPU memory leaks (VRAM returns to baseline after close) | warning | baseline |

## Visual Quality (Level 4) — evolves with feedback

_No criteria yet. Will be populated from user feedback._

## Data Quality (Level 4) — evolves with feedback

_No criteria yet. Will be populated from user feedback._
