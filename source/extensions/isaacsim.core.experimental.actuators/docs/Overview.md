# Overview

The `isaacsim.core.experimental.actuators` extension provides high-level
runtime support for **Newton actuators** on robot articulations in
Isaac Sim.  It adds a thin wrapper that owns one or more actuator
pipelines per articulation, registers a pre-physics callback, and writes the
resulting joint efforts back to the Articulation on every step.  Actuators can either be
**authored on the asset in USD** (so they travel with the robot file and can
be loaded unchanged by any Newton-aware application, including Isaac Lab) or
**built in Python at runtime** for cases where USD authoring is impractical.

```{note}
This extension is **experimental**.  Class names, USD schema attribute
names, and parsing semantics may change in future releases.
```

## Key Components

### {class}`ArticulationActuators <isaacsim.core.experimental.actuators.ArticulationActuators>`

The {class}`ArticulationActuators <isaacsim.core.experimental.actuators.ArticulationActuators>`
class wraps an
{class}`Articulation <isaacsim.core.experimental.prims.Articulation>` and is
the primary user-facing object in the extension.  On construction it either
discovers `NewtonActuator` prims under the articulation root in USD, or
accepts a list of Python-built configs via
{meth}`from_actuators <isaacsim.core.experimental.actuators.ArticulationActuators.from_actuators>`.
On each physics tick its pre-physics callback reads joint state and target,
evaluates each actuator's `delay → controller → clamping` pipeline, and
writes the resulting effort back to the articulation.  USD `DriveAPI` gains
on actuated DOFs are zeroed automatically so the implicit drive does not
fight the actuator output.

### {class}`ActuatorConfig <isaacsim.core.experimental.actuators.ActuatorConfig>`

The {class}`ActuatorConfig <isaacsim.core.experimental.actuators.ActuatorConfig>`
dataclass bundles the three components of a single actuator's pipeline: a
required Newton `Controller`, an ordered list of optional `Clamping`
stages, and an optional `Delay`.  Used together with
{meth}`from_actuators <isaacsim.core.experimental.actuators.ArticulationActuators.from_actuators>`
to attach actuators to an articulation without touching USD.  Subclasses of
`newton.actuators.Controller`, `Clamping`, or `Delay` are accepted, so
custom control laws can be plugged in by user code without modifying the
Newton library.

### USD Authoring

The USD authoring helpers — {func}`add_actuator <isaacsim.core.experimental.actuators.add_actuator>`
together with the per-component config dataclasses
{class}`PDControlConfig <isaacsim.core.experimental.actuators.PDControlConfig>`,
{class}`PIDControlConfig <isaacsim.core.experimental.actuators.PIDControlConfig>`,
{class}`NeuralControlConfig <isaacsim.core.experimental.actuators.NeuralControlConfig>`,
{class}`MaxEffortClampingConfig <isaacsim.core.experimental.actuators.MaxEffortClampingConfig>`,
{class}`DCMotorClampingConfig <isaacsim.core.experimental.actuators.DCMotorClampingConfig>`,
{class}`PositionBasedClampingConfig <isaacsim.core.experimental.actuators.PositionBasedClampingConfig>`,
and {class}`DelayConfig <isaacsim.core.experimental.actuators.DelayConfig>` —
define `NewtonActuator` prims under an `Actuators` scope on the
articulation root and apply the matching Newton USD API schemas.  An
authored actuator looks like:

```usda
def Scope "Actuators" {
    def NewtonActuator "panda_joint1_actuator" (
        prepend apiSchemas = ["NewtonPDControlAPI", "NewtonMaxEffortClampingAPI"]
    ) {
        rel newton:targets = </Franka/panda_link0/panda_joint1>
        float newton:kp = 400.0
        float newton:kd = 40.0
        float newton:maxEffort = 87.0
    }
}
```

The schema names live in the shared Newton USD schema, so any Newton-aware
application — including Isaac Lab — can parse the same prim and recover
the same actuator model.

### `Articulation Actuators` OmniGraph Node

The extension ships an `omni.graph.action.execution`-style node, registered
as `isaacsim.core.experimental.actuators.ArticulationActuators` (UI name
**Articulation Actuators**, category **Newton Actuators**).  On its first
`execIn` pulse the node lazily constructs an
{class}`ArticulationActuators <isaacsim.core.experimental.actuators.ArticulationActuators>`
for the configured `robotPath`; from then on subsequent pulses re-apply the
optional `feedforwardCommand` input.  The node only operates on USD-authored
actuators.

## Functionality

### Two Construction Paths

- **Discover from USD** — pass an articulation root to
  {class}`ArticulationActuators <isaacsim.core.experimental.actuators.ArticulationActuators>`
  and the constructor walks the USD subtree, parses every `NewtonActuator`
  prim, and rebuilds the pipeline.
- **Build in Python** — call
  {meth}`from_actuators <isaacsim.core.experimental.actuators.ArticulationActuators.from_actuators>`
  with a list of `(ActuatorConfig, dof_name)` pairs.  Useful when iterating
  on parameters or attaching custom controller subclasses that have no USD
  schema equivalent.

### Composable Per-Actuator Pipeline

Every actuator is a `delay → controller → clamping` pipeline applied to a
single DOF.  Built-in components from `newton.actuators` cover the common
cases (PD / PID / neural-MLP / neural-LSTM controllers; symmetric, DC-motor,
and position-based clampings; per-step input delay), and any subclass of the
corresponding base class can be substituted from user code.

### Autonomous Pre-Physics Stepping

Once constructed, the wrapper registers its own pre-physics callback and
runs the actuators on every physics step independent of further user code.
The callback can be disabled by passing `auto_step_pre_physics=False`, in
which case the user calls
{meth}`step_actuators <isaacsim.core.experimental.actuators.ArticulationActuators.step_actuators>`
manually — useful for tests that need deterministic single-step control.

### Feedforward Effort

{meth}`set_dof_feedforward_effort_targets <isaacsim.core.experimental.actuators.ArticulationActuators.set_dof_feedforward_effort_targets>`
adds a per-DOF effort that is summed with the controller output every tick.
With `kp` and `kd` zeroed this reduces to a pure open-loop torque drive,
making it a convenient hook for gravity-compensation, learned residual
policies, or any feed-forward torque component.  Feedforward only affects
DOFs that have an explicit actuator attached.

### Partial Coverage

Not every joint of an articulation has to be actuated.  Joints without an
explicit `NewtonActuator` prim (or matching `from_actuators` entry) keep
their authored `UsdPhysics.DriveAPI` stiffness/damping and behave exactly
as they would on the unmodified asset.

### Cross-Application Portability

Because the underlying actuator components and USD schema both live in
Newton, USD-authored actuators on a robot file are application-agnostic:
the same asset loaded in |isaac-sim_short| or in Isaac Lab will be
reconstructed with the same effective control law.  This extension is
the |isaac-sim_short| runtime side of that portability story.

## Tutorials

Hands-on tutorials covering each construction path live under
`docs/isaacsim/newton_actuators_tutorials/`:

- *Set Up Actuators from Python* — building an actuator entirely in Python
  with `from_actuators`.
- *Author and Parse Actuators from USD* — using `add_actuator` to bake
  actuators onto an asset and re-loading the saved file.
- *Drive an Actuated Robot from OmniGraph* — wiring the `Articulation
  Actuators` node into an Action Graph.
- *Tips* — practical guidance on armature, physics rate, and diagnosing
  high-frequency vibration.

Runnable companion scripts ship under
`standalone_examples/api/isaacsim.core.experimental.actuators/` and can be
invoked via `./python.sh`.
