# Isaac Sim Skills Index

Routing map for the skills under `skills/`. Read it when a task spans several
skills or you need the order they compose in. For single-skill routing you do
not need this file: every `SKILL.md` carries a `description` in its YAML
frontmatter stating when to use it, and agent platforms load those
automatically.

A skill is a directory with a `SKILL.md` plus optional `scripts/` and
`references/`.


Shared shell-variable contract (`$ISAAC_SIM_DIR`, `$ISAAC_LAB_DIR`,
`$WORKSPACE_DIR`): declared in
[`isaac-sim-orchestrator/SKILL.md`](isaac-sim-orchestrator/SKILL.md).

---

## Categories

Names and links only. What each skill does, and when to use it, is stated in the
`description` in its frontmatter, which is what agents route on and is loaded
without reading this file.

<!-- BEGIN GENERATED: skill table -->

### Repo-native, public

- [`isaac-sim-assets`](isaac-sim-assets/SKILL.md)
- [`isaac-sim-installation`](isaac-sim-installation/SKILL.md)
- [`isaac-sim-migration`](isaac-sim-migration/SKILL.md)
- [`isaac-sim-remote`](isaac-sim-remote/SKILL.md)
- [`isaac-sim-web-viewer`](isaac-sim-web-viewer/SKILL.md)
- [`profile-isaac-sim`](profile-isaac-sim/SKILL.md)
- [`validation-diff-gifs`](validation-diff-gifs/SKILL.md)

### Foundations and operating loop

- [`isaac-sim-orchestrator`](isaac-sim-orchestrator/SKILL.md)
- [`isaac-sim-workflow`](isaac-sim-workflow/SKILL.md)
- [`meta-skills`](meta-skills/SKILL.md)
- [`skill-distillation`](skill-distillation/SKILL.md)
- [`isaac-sim-validator`](isaac-sim-validator/SKILL.md)
- [`isaac-sim-troubleshooting`](isaac-sim-troubleshooting/SKILL.md)

### Robot asset pipeline

- [`urdf-mjcf-to-usd-conversion`](urdf-mjcf-to-usd-conversion/SKILL.md)
- [`physx-to-newton-conversion`](physx-to-newton-conversion/SKILL.md)
- [`usd-articulation`](usd-articulation/SKILL.md)

### Physics

- [`physics-simulation`](physics-simulation/SKILL.md)

### Mobile robot navigation

- [`navigation-primitives`](navigation-primitives/SKILL.md)
- [`occupancy-map`](occupancy-map/SKILL.md)
- [`isaac-sim-robot-navigation`](isaac-sim-robot-navigation/SKILL.md)
- [`mobility-gen`](mobility-gen/SKILL.md)

### Manipulation

- [`manipulation-ik`](manipulation-ik/SKILL.md)
- [`motion-generation`](motion-generation/SKILL.md)

### Sensors and perception

- [`isaac-sim-sensor`](isaac-sim-sensor/SKILL.md)
- [`isaac-camera`](isaac-camera/SKILL.md)

### Synthetic data generation

- [`data-collection-sim`](data-collection-sim/SKILL.md)
- `mobility-gen` — mobile-robot variant; see [Mobile robot navigation](#mobile-robot-navigation)

### Action and event data generation

- [`action-and-event-data-generation`](action-and-event-data-generation/SKILL.md)
- [`actor-sdg-sweep-config`](actor-sdg-sweep-config/SKILL.md)
- [`actor-sdg-generate-lighting-variations`](actor-sdg-generate-lighting-variations/SKILL.md)
- [`object-bin-packing`](object-bin-packing/SKILL.md)
- [`generate-incident-config`](generate-incident-config/SKILL.md)
- [`run-incident-events`](run-incident-events/SKILL.md)
- [`vlm-scene-captioning`](vlm-scene-captioning/SKILL.md)
- [`behavior-tree-generation`](behavior-tree-generation/SKILL.md)
- [`place-camera-max-coverage`](place-camera-max-coverage/SKILL.md)
- [`place-camera-aim-at`](place-camera-aim-at/SKILL.md)
- [`calibrate-metropolis-camera`](calibrate-metropolis-camera/SKILL.md)

### Rendering

- [`isaac-sim-rendering`](isaac-sim-rendering/SKILL.md)
- [`isaac-sim-headless-deployment`](isaac-sim-headless-deployment/SKILL.md)

### USD pipeline

- [`spatial-reasoning`](spatial-reasoning/SKILL.md)
- [`usd-pipeline`](usd-pipeline/SKILL.md)
- [`usd-composition-architecture`](usd-composition-architecture/SKILL.md)

### ROS 2

- [`isaac-sim-ros-workspaces`](isaac-sim-ros-workspaces/SKILL.md)
- [`isaac-sim-ros2-bridge`](isaac-sim-ros2-bridge/SKILL.md)

<!-- END GENERATED: skill table -->

---

## Composition

Which skill feeds which. This is the part no single `SKILL.md` can state.

- **Robot to simulation:** `urdf-mjcf-to-usd-conversion` then `usd-articulation`
  (structure it with `usd-composition-architecture`) then `physics-simulation`,
  then `isaac-sim-orchestrator` to integrate sensors and capture.
- **Synthetic data:** static scenes go `isaac-sim-sensor` then
  `data-collection-sim`; mobile robots go `navigation-primitives` then
  `mobility-gen` (record, then replay and render).
- **Mobile navigation:** `navigation-primitives` is the substrate under both
  `isaac-sim-robot-navigation` (live control) and `mobility-gen` (SDG);
  `occupancy-map` supplies the grid, `isaac-sim-ros2-bridge` exposes it to Nav2.
- **Manipulation:** `manipulation-ik` for grasp frames and contact validation,
  `motion-generation` when the path must avoid obstacles.
- **Delivery:** `isaac-sim-rendering` produces frames, `isaac-sim-validator`
  gates them, `skill-distillation` captures what was learned.


---

## When stuck

- [`isaac-sim-troubleshooting`](isaac-sim-troubleshooting/SKILL.md) — hangs, freezes, performance stalls
- [`isaac-sim-validator`](isaac-sim-validator/SKILL.md) — refuses bad deliverables before they reach the user
