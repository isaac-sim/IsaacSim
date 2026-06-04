# Overview

isaacsim.replicator.grasping provides a comprehensive workflow for generating synthetic grasping datasets using Isaac Sim's Replicator framework. This extension focuses specifically on robotic manipulation scenarios, offering automated generation of diverse grasping data for training and validation of robotic manipulation models.

## Key Components

### {class}`GraspingManager <isaacsim.replicator.grasping.GraspingManager>`

The {class}`GraspingManager <isaacsim.replicator.grasping.GraspingManager>` serves as the central coordinator for all aspects of grasp evaluation workflows. It manages the complete pipeline from grasp pose generation through physics simulation to results output.

**Core workflow capabilities include:**
- Grasp pose generation using configurable samplers with antipodal grasp sampling
- Multi-phase grasp simulation with customizable joint drive targets and timing
- Physics simulation isolation for controlled testing environments
- Structured results output in YAML format with configurable file management
- Workflow management with stop controls and progress tracking

The manager operates on USD prims for both grippers and objects, automatically handling coordinate frame transformations between local object space and world coordinates. It maintains state for joint pregrasp positions, simulation parameters, and grasp evaluation results throughout the workflow.

**Simulation modes:** The manager supports both direct physics stepping and timeline-based simulation, with optional scene isolation to create controlled testing environments separate from the main scene.

### {class}`GraspPhase <isaacsim.replicator.grasping.GraspPhase>`

{class}`GraspPhase <isaacsim.replicator.grasping.GraspPhase>` represents individual phases within a grasping sequence, each containing specific joint targets and simulation parameters. Each phase defines:
- Joint drive targets with specific position values
- Number of simulation steps for the phase duration
- Simulation step timing parameters

Phases can be dynamically managed with methods to add, remove, and query joints within each phase. This allows for flexible definition of complex grasping sequences like open, approach, close, and lift phases.

## Functionality

### Grasp Pose Generation

The extension generates grasp poses using configurable sampling strategies. The antipodal grasp sampler creates candidate poses based on object geometry and gripper constraints:

```python
# Generate grasp poses with custom configuration
config = {
    "sampler_type": "antipodal",
    "num_candidates": 100,
    "gripper_maximum_aperture": 0.1,
    "num_orientations": 8
}
grasping_manager.generate_grasp_poses(config)
```

### Multi-Phase Simulation

Grasp evaluation proceeds through configurable phases, each with distinct joint targets and timing:

```python
# Create grasp phases
open_phase = grasping_manager.create_and_add_grasp_phase(
    "open", 
    {"finger_joint": 0.05}, 
    simulation_steps=60
)

close_phase = grasping_manager.create_and_add_grasp_phase(
    "close", 
    {"finger_joint": 0.0}, 
    simulation_steps=120
)
```

### Configuration Management

The extension supports comprehensive state serialization for reproducible workflows:

```python
# Save complete workflow configuration
grasping_manager.save_config("grasp_config.yaml", overwrite=True)

# Load specific components
status = grasping_manager.load_config("grasp_config.yaml", 
                                    components=["gripper", "phases", "sampler"])
```

### Batch Evaluation

Large-scale grasp evaluation with progress tracking and optional scene isolation:

```python
# Evaluate all generated poses
await grasping_manager.evaluate_grasp_poses(
    grasp_poses, 
    render=True, 
    isolate_simulation=True,
    progress_callback=my_progress_callback
)
```
