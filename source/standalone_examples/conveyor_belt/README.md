# Conveyor Belt Sample

A standalone Isaac Sim sample that demonstrates a custom conveyor belt simulation
implemented with NVIDIA Warp.
Rather than relying on the surface velocity logic provided by USD schemas defined in
PhysxSchema, the sample computes tangential friction forces at each contact point
using a Coulomb friction model and applies them directly to the rigid bodies sitting
on the belts.

## What the sample does

At every physics post-step the pipeline:

1. Queries contact data (positions, normals, normal forces) between tracked rigid bodies
   and conveyor belt geometry.
2. Maps each contact point to a *velocity field* that defines the desired surface
   velocity at that point.
3. Groups contact points with similar normals into *patches* and optionally
   redistributes the total normal force within a patch.
4. Computes the tangential force needed to bring the contact-point velocity towards
   the target velocity, clamped by the Coulomb friction force.
5. Accumulates the resulting force and torque for each body and applies them via the
   Isaac Sim rigid-prim API.

The full pipeline (excluding Isaac Sim API calls) is implemented as Warp kernels and,
when running on CUDA, is captured into a CUDA graph after a short warm-up period so that
subsequent steps incur minimal CPU overhead.

The sample sets up a closed-loop conveyor belt circuit with straight sections, turns,
ramps and side-by-side belts. Several box-shaped rigid bodies
with different masses and friction coefficients are placed on the belts to
demonstrate transport and transitions between sections. To ensure friction on the
conveyor belt sections is solely handled by the sample custom logic, the corresponding
physics material is configured such that the inbuilt physics simulation uses a friction
coefficient of zero for interactions with the conveyor belts.

## Files

| File | Description |
|---|---|
| `cb_app.py` | Application entry point. Creates the Isaac Sim `World`, triggers scene building, registers the physics post-step callback, manages the simulation loop, and handles CUDA graph capture and reset. |
| `cb_scene.py` | Builds the full conveyor belt circuit scene. Registers all velocity fields, conveyor belts and rigid bodies with the respective managers. |
| `cb_scene_building_utils.py` | Helper functions to build and configure USD geometry and rigid body prims. |
| `cb_actuators.py` | Defines `VelocityFieldActuator`, which owns the registered velocity fields and runs the Warp kernel that computes the per-contact tangential force/torque. |
| `cb_kernels.py` | Defines all the Warp kernels (except the core actuator related kernel) used in the sample. The kernels prepare data, assign contact points to patches, redistribute normal forces, etc. |
| `cb_conveyor_belt_manager.py` | `ConveyorBeltManager` — registers conveyor belt prims and stores their associated properties. |
| `cb_body_manager.py` | `BodyManager` — registers the rigid bodies to be transported and manages the corresponding per-body data. |
| `cb_material_pair_manager.py` | `MaterialPairManager` — provides material indices. Allows to use these indices to define friction coefficients for material pairs. |
| `cb_visualizers.py` | `VelocityFieldVisualizer` — animates small point markers along the velocity field paths using the Isaac Sim debug-draw interface so belt speeds and directions can be inspected visually at runtime. |
| `cb_utils.py` | Utility functions to share basic logic. |

## Running the sample

The Isaac Sim Python script python.sh/.bat can be used to run the sample. The entry script is cb_app.py. Example:

```bash
./python.sh ./standalone_examples/conveyor_belt/cb_app.py
```

## Comparison to inbuilt surface velocity approach

As mentioned in the introduction, this sample shows an alternative to the inbuilt surface
velocity approach (PhysxSurfaceVelocityAPI USD schema) to model conveyor belts. The sample
computes the friction forces between the conveyor belts and the transported bodies and applies
them as external forces. Some advantages and disadvantages of this approach are listed in
the following:

### Pros

- Most data can stay on the GPU and does not have to be copied from/to CPU.

- Flexibility to implement alternative friction models.

- Easier to adjust to and optimize for application specific purposes or work around undesired
behavior.

- Scenarios like rigid bodies sitting on multiple conveyor belts with different speeds might
behave better for cases where the physics simulation has a tendency to not distribute the normal
forces at contact points in an equitable way.

### Cons

- Using external forces might require smaller simulation timesteps or the implementation of a
higher order Runge-Kutta integration scheme to reach the desired level of accuracy.

- The sample uses a rather crude heuristic to redistribute the total normal force of a patch
among the contact points. Depending on the distribution of the contact points, this can lead
to biases which might surface especially when the delta between the current and the target
velocity at contact points is large.

- The Isaac Sim APIs to fetch the contact data or apply the forces to the rigid bodies can
not be fused into the CUDA graph that the sample is capturing. As a consequence, some of the
performance overhead of launching CUDA kernels etc. can not be reduced.

## Parameter tuning

- Rigid bodies crossing the boundary between two belt sections can experience a brief force
discontinuity when hitting "internal" edges of the involved geometry. The rest offset and
contact offset parameters might need to be increased further to smoothen the transition. To
avoid having the rigid bodies look as if they are floating above ground, the rigid body
objects would usually use two different set of geometries, one that is used for rendering
(being inflated to cover the rest offset) and a collision geometry that is used for the
physics simulation.

- The sample has various batch size and level of parallelism count parameters. While those
might not impact the small scale scene of the sample, they might become more relevant when
running with large number of rigid bodies and conveyor belt sections. Tuning those parameters
might then help to improve throughput for the targeted hardware.

- The `max_average_contact_count_per_body` parameter in `cb_app.py` might need adjustment
as described in the code.

## Potential extensions and improvements

### Simulation quality

- **Higher-fidelity friction model** — Extend the sample's single-coefficient Coulomb model
to a richer friction model. For example, distinguishing between static and dynamic friction
coefficients would allow the sample to compare the force required to reach the target
velocity against the static friction force, then clamp by the dynamic friction force when
the body is slipping and by the static friction force otherwise.
A second improvement is to make the friction coefficient depend on slip velocity rather than
fixing it. For instance, scaling the coefficient linearly with the slip velocity within a
defined range (and saturating outside that range) addresses a known limitation of the current
model: when the delta between the target and current velocity is large, all contact points in
a patch saturate at the same force magnitude. As a result, a rigid body sitting across two
belts running at different speeds may not begin to rotate until its velocity nears the target
velocity. A slip-dependent coefficient would let the faster belt apply a larger force in this
scenario and produce the expected rotation.

- **Non-flat belt surfaces** — Generalize the contact-processing threshold to support curved
or non-planar belts. The current implementation filters contacts by comparing each contact
normal against a single per-belt surface normal, which assumes a flat belt. Supporting curved
geometry requires a more general acceptance criterion.

### Features

- **Dynamic rigid body creation** — Allow rigid bodies to enter and leave the conveyor
system at runtime. The sample currently requires up-front registration. One option is to
maintain a pool of pre-registered rigid bodies and recycle them over time. Another is to
re-create the Warp data arrays and Isaac Sim rigid prim view, and re-capture the CUDA graph,
when new rigid bodies are added.

- **Additional velocity field types** — Add more sophisticated velocity field types beyond
the constant and pivot-point fields the sample provides today.
