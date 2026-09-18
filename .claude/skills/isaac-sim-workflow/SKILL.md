---
name: isaac-sim-workflow
description: "Scope demos, POCs, and presentation captures before implementation. Use to define deliverables, acceptance criteria, evidence, and specialist handoffs; technique execution belongs to routed skills."
license: Apache-2.0
metadata:
  author: Aaron Young <info@nvidia.com>
---

# Isaac Sim Workflow

## Purpose

Scope demos, showcases, and proof-of-concept captures before implementation: define the deliverable type, acceptance criteria, evidence, and routing into technique skills.

## Prerequisites

- Built Isaac Sim (`$ISAAC_SIM_DIR` or `_build/linux-x86_64/release`).
- NVIDIA GPU with a current driver (`nvidia-smi`).
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`, `$WORKSPACE_DIR`.

## Limitations

- Targets Isaac Sim 6 / Kit 110 unless a section states otherwise.
- Does not replace official NVIDIA documentation for unsupported edge cases.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Extension or import not found | Wrong `$ISAAC_SIM_DIR` or stale build | Point env vars at `_build/linux-x86_64/release` or rebuild |
| Black or empty frames | Missing lights or non-RTX render mode | Add dome/key light; confirm RTX / PathTracing settings |
| Hang on stage load or first render | MDL compile or oversized stage | Follow isolation steps in `isaac-sim-troubleshooting` |

Process skill for turning an Isaac Sim demo request into an implementation
contract. This skill owns scoping, acceptance criteria, evidence requirements,
and handoff. `isaac-sim-orchestrator` owns ordered multi-skill execution;
technique-specific implementation belongs in the routed specialist skills.

## Classify the deliverable

Determine the primary output before building:

| Output type | Typical request | Required result |
|---|---|---|
| Presentation capture | video, stills, partner review, showcase | behavior is visible, legible, framed, lit, and captured from the actual scene |
| Technical reference | runnable how-to, API workflow, autonomy behavior | behavior is reproducible, measured, and backed by the routed technique skill's validation gates |
| Hybrid | presentation asset plus working script | technical gates pass first; then package a presentation-quality capture |

Clarifying questions are opt-in. Ask them only when the user explicitly asks
for questions, asks to plan before building, or asks to reach consensus first.
If the user asks the agent to make assumptions or not ask for input, choose the
least misleading interpretation and record the assumptions in the handoff. Only
stop for a question when an undiscoverable answer is a hard blocker.

## Consensus Questions

Ask only when the user explicitly requests planning, interviewing, or shared
understanding. Ask one question at a time. Resolve enough to form a demo
contract:

- What final artifact should be delivered?
- What exact behavior or claim must it prove?
- Which subjects, assets, scene elements, or APIs are fixed?
- What assumptions may the agent choose?
- What shortcuts or outcomes are unacceptable?
- What evidence counts as success?

End by restating the agreed acceptance criteria, assumptions, and planned
artifacts. Then build against that contract.

If a question can be answered by exploring the codebase, explore the codebase instead.

## Define acceptance criteria

Convert the prompt into concrete checks:

- subject: robot, sensor, environment, object, or behavior being demonstrated
- claim: what the demo proves or illustrates
- output artifact: script, scene, screenshots, video, or a combination
- success condition: measured behavior, visible state change, or required frame
  content
- constraints: physics/collisions, controller/API requirements, asset source,
  runtime budget, capture resolution, or deployment target

Do not add constraints that are only stylistic preferences. Do preserve
constraints that affect truthfulness, reproducibility, or the user's stated
goal.

When the user corrects an attempt, convert the correction into a measurable
negative gate before continuing. A rejection like "not from above" or "not with
that contact surface" must become validation, not only a visual preference.

## Integrity rules

- Do not present teleported, kinematic, pose-written, or scripted motion as
  controller-driven autonomy or physics-based interaction. Only use non-physics-based interactions when the user has explicitly stated it is acceptable.
- If contact, collision, navigation, manipulation, sensing, or dynamics is part
  of the claim, enable the relevant simulation behavior and verify it with
  evidence from the run.
- Non-physical animation or presentation-only assistance is acceptable only when
  the request is explicitly illustrative and the handoff states what was
  assisted.
- Debug visuals are opt-in and must be omitted when the user says no debug visuals.
- Technical references must satisfy the validation gates of the routed
  technique skill.

## Working practices

- Use `isaac-sim-remote` for live Python-server iteration when a running sim is
  available. Create a standalone script only when needed for handoff. Avoid `python.sh`
  unless testing the standalone script. (See `isaac-sim-orchestrator` "Execution mode").
- Search existing Isaac Sim, Nucleus, SimReady, material, environment, and
  example-scene assets before authoring substitutes.
- Keep the scene no more complex than needed to satisfy the acceptance criteria.
- Capture fresh screenshots or video from the actual run.
- Inspect representative start, middle, and end frames before delivery. Check
  for black frames, missing assets, bad framing, unreadable scale, occlusion, and
  obvious behavior mismatch.
- Hand off requested artifacts with assumptions and pass/fail evidence.
- Version videos and saved assets as a means to save intermediate results for
  use after.

## Route implementation work

| Need | Skill |
|---|---|
| Overall Isaac Sim build phases | `isaac-sim-orchestrator` |
| Scene composition and USD structure | `usd-pipeline`, `usd-composition-architecture` |
| Mobile or wheeled navigation | `navigation-primitives`, `isaac-sim-robot-navigation` |
| Arm manipulation and obstacle-aware motion | `motion-generation`, `manipulation-ik` |
| Physics, rigid bodies, collisions, materials | `physics-simulation` |
| Sensors and cameras | `isaac-sim-sensor`, `isaac-camera` |
| Rendering, camera placement, video capture | `isaac-sim-rendering` |
| Live Python-server execution | `isaac-sim-remote` |
| Output validation | `isaac-sim-validator` |
