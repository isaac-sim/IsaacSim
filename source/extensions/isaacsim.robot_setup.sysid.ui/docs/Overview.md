# Overview

## System Identification UI

> **Beta:** This extension is in beta. Its workflow, run contracts, exported run specifications, and APIs are not guaranteed to stay future compatible.

This extension provides the interactive System Identification window, menus, plots, file pickers, and viewport integration.

It depends on `isaacsim.robot_setup.sysid`, which owns the headless runtime, run contracts, optimizers, and simulator bridges. Enable only the backend extension for headless workflows; enable this extension for the **Tools > Robotics > Asset Editors > System Identification** window.

The window follows a six-stage workflow:

1. **Robot** selects the articulation, simulation engine, actuator runtime, and controller settings.
2. **Data** loads telemetry and defines an analysis window plus automatic or manual training and validation partitions.
3. **Parameters** selects the physical parameters and bounds to identify.
4. **Check** validates telemetry quality and parameter identifiability.
5. **Solve** selects residual weights, optimizer settings, and output/writeback behavior.
6. **Results** shows live cost, parameter deltas, residual summaries, sensitivity, and paired untuned/tuned validation metrics.

Run remains disabled until Check has completed for the current robot, data,
parameter selection, residual configuration, and check-relevant solver settings.
Changing any of those inputs requires a fresh Check. Isaac Sim rollouts also
track the timeline state, so Play, Pause, and Stop immediately refresh run
readiness.

Numeric solver, timing, seed, weight, and substep controls use exact-entry fields rather than drag gestures.
Inputs with physical or algorithmic limits are constrained in the UI, including positive iteration counts,
non-negative residual weights, and valid optimizer population sizes.

The Data page separates the overall analysis window from dataset partitioning. **Automatic validation holdout**
accepts a training percentage and minimum partition duration. **Manual chunk editor** creates named training or
validation intervals with excitation tags; manual chunks take precedence and temporarily disable automatic
holdout. The timeline shows training, validation, and unused regions, while the scrollable table provides exact
start/end inputs, sample counts, durations, and training-only weights.

Start from a recipe when possible. Recipes populate the UI widgets but remain
fully editable. Review **Outputs & stage changes** before solving: optimized
parameters are authored to the open USD stage only when writeback is enabled
and completes successfully.

Interactive writeback authors overrides in the open stage's root layer at the
selected robot namespace. This keeps referenced multi-physics payloads unchanged,
verifies the composed joint stiffness and damping, and refreshes the live PhysX
articulation before reporting success. Save the open stage to persist the
overrides. USD instance-proxy descendants cannot receive local property opinions;
for an instanceable robot, create a derived non-instanceable asset or de-instance
it before applying the result.

Under **Advanced simulation settings**, choose the in-app physics backend, actuator runtime
(`auto`, `implicit_drive`, `newton_explicit`, or `mixed`), and missing-actuator policy
(`authored_only` or `promote_selected`). These settings are retained across execution-host changes,
exported in the RunSpec, and invalidate an older Check. `auto` resolves to mixed ownership for Newton
MuJoCo and explicit ownership for Featherstone. Enable
**Command delay** in **Parameters** when delay should be identified; the
resulting value is quantized to the nearest simulation physics step.

The backend's shared capability contract also gates recipes, Check, and direct runtime construction. In-app
Isaac-hosted Newton currently rejects joint-friction parameters and torque/contact-force residuals because its
tensor interface does not expose mutable friction, projected joint force, or incoming link force. Use Isaac Sim
PhysX for the complete interactive parameter/residual surface, or standalone Newton when Newton friction
identification is required. Standalone MuJoCo and differentiable Featherstone expose narrower residual and
parameter sets; Check reports the exact unsupported selections before rollout.

The Results page compares RMSE from the authored launch parameters with RMSE from the accepted parameters
on the exact same validation chunks. It labels the all-training-chunk fallback as training-data replay rather than
held-out evidence, and shows unavailable percentage changes when the authored RMSE is zero.
