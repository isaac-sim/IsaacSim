# ROS 2 Simulation Control Extension

## Overview

The ROS 2 Simulation Control extension uses the ROS 2 Simulation Interfaces to control Isaac Sim functions. This extension is designed to be scalable, allowing multiple services and actions to run simultaneously with little performance overhead.

## Features

- Uses ROS 2 services to control Isaac Sim:
  - `isaacsim/GetSimulatorFeatures`: Lists the supported features in this Isaac Sim implementation
  - `isaacsim/SetSimulationState`: Set simulation to specific state (playing/paused/stopped)
  - `isaacsim/GetSimulationState`: Get current simulation state
  - `isaacsim/GetEntities`: Get list of all entities (prims) in the simulation
  - `isaacsim/GetEntityInfo`: Get detailed information about a specific entity (currently returns OBJECT category type)
  - `isaacsim/GetEntityState`: Get the pose, twist, and acceleration of a specific entity
  - `isaacsim/GetEntitiesStates`: Get the states (pose, twist, acceleration) of multiple entities with filtering
  - `isaacsim/DeleteEntity`: Delete a specific entity (prim) from the simulation
  - `isaacsim/SpawnEntity`: Spawn a new entity into the simulation at a specified location
  - `isaacsim/ResetSimulation`: Reset the simulation environment to its initial state
  - `isaacsim/SetEntityState`: Sets the state (pose, twist) of a specific entity in the simulation
  - `isaacsim/StepSimulation`: Step the simulation forward by a specific number of frames

- Uses ROS 2 actions to control Isaac Sim:
  - `isaacsim/SimulateSteps`: Action for stepping the simulation with progress feedback


## Requirements

- Isaac Sim 5.0 or later
- ROS 2 (Humble or later)
- [Simulation Interfaces](https://github.com/ros-simulation/simulation_interfaces) ROS 2 package installed. This extension is currently tested with [v1.0.1](https://github.com/ros-simulation/simulation_interfaces/releases/tag/1.0.1).  

## Usage

### Enabling the Extension

1. Open Isaac Sim
2. Enable the extension from the Extension Manager: `isaacsim.ros2.sim_control`

### Using the GetSimulatorFeatures Service

The GetSimulatorFeatures service lists the subset of services and actions supported by Isaac Sim from simulation_interfaces.

```bash
ros2 service call /isaacsim/GetSimulatorFeatures simulation_interfaces/srv/GetSimulatorFeatures
```

Notes:
- Returns a list of supported features (e.g., SPAWNING, DELETING, ENTITY_STATE_GETTING)
- Reports USD file format support via the spawn_formats field
- Provides custom_info with details about the implementation

### Using the SetSimulationState Service

The SetSimulationState service updates the global state of the simulation (stopped/playing/paused/quitting) corresponding to enums defined in SimulationState.msg (STATE_STOPPED, STATE_PLAYING, STATE_PAUSED, STATE_QUITTING). 

1. To set simulation state to playing:
   ```bash
   ros2 service call /isaacsim/SetSimulationState simulation_interfaces/srv/SetSimulationState "{state: {state: 1}}"  # 1=playing
   ```

2. To set simulation state to paused:
   ```bash
   ros2 service call /isaacsim/SetSimulationState simulation_interfaces/srv/SetSimulationState "{state: {state: 2}}"  # 2=paused
   ```

3. To set simulation state to stopped:
   ```bash
   ros2 service call /isaacsim/SetSimulationState simulation_interfaces/srv/SetSimulationState "{state: {state: 0}}"  # 0=stopped
   ```

4. To quit the simulator:
   ```bash
   ros2 service call /isaacsim/SetSimulationState simulation_interfaces/srv/SetSimulationState "{state: {state: 3}}"  # 3=quit
   ```

Notes:
- State 0 (Stopped) is equivalent to pausing and resetting the simulation
- State 1 (Playing) starts the simulation timeline
- State 2 (Paused) pauses the simulation at the current time
- State 3 (Quitting) shuts down Isaac Sim

### Using the GetSimulationState Service

The GetSimulationState service retrieves the current state of the entire simulation (stopped/playing/paused/quitting) corresponding to enums defined in SimulationState.msg (STATE_STOPPED, STATE_PLAYING, STATE_PAUSED, STATE_QUITTING).

```bash
ros2 service call /isaacsim/GetSimulationState simulation_interfaces/srv/GetSimulationState
```

Notes:
- Returns state 0 for stopped, 1 for playing, 2 for paused
- Used to query the simulation state before performing operations like stepping

### Using the GetEntities Service

The GetEntities service retrieves a list of all entities present in the simulation with optional filtering.

1. Get all entities in the simulation:
   ```bash
   ros2 service call /isaacsim/GetEntities simulation_interfaces/srv/GetEntities "{filters: {filter: ''}}"
   ```
   
2. Get entities with full paths or partial paths. In this case filter for prims containing 'camera' in the path:
   ```bash
   ros2 service call /isaacsim/GetEntities simulation_interfaces/srv/GetEntities "{filters: {filter: 'camera'}}"
   ```

3. Get entities with paths starting with '/World':
   ```bash
   ros2 service call /isaacsim/GetEntities simulation_interfaces/srv/GetEntities "{filters: {filter: '^/World'}}"
   ```

4. Get entities with paths ending with 'mesh':
   ```bash
   ros2 service call /isaacsim/GetEntities simulation_interfaces/srv/GetEntities "{filters: {filter: 'mesh$'}}"
   ```

Notes:
- The filter parameter accepts POSIX Extended regular expressions for matching entity names (prim paths)
- Isaac Sim uses the full USD prim paths as entity names

### Using the GetEntityInfo Service

The GetEntityInfo service provides detailed information about a specific entity, such as its type and properties.

```bash
ros2 service call /isaacsim/GetEntityInfo simulation_interfaces/srv/GetEntityInfo "{entity: '/World/robot'}"
```

Notes:
- Returns `RESULT_OK` with EntityInfo if the entity exists
- Returns `RESULT_OPERATION_FAILED` if the entity doesn't exist
- The EntityInfo contains:
  - category: Currently always set to OBJECT (EntityCategory.OBJECT)
  - description: Empty string (reserved for future use)
  - tags: Empty array (reserved for future use)

### Using the GetEntityState Service

The GetEntityState service gets the pose, twist, acceleration of a specific entity relative to a given reference frame. Currently only world frames are supported.

```bash
ros2 service call /isaacsim/GetEntityState simulation_interfaces/srv/GetEntityState "{entity: '/World/robot'}"
```

Notes:
- For entities with RigidBodyAPI, both pose and velocities will be returned
- For entities without RigidBodyAPI, only pose will be returned with zero velocities
- Acceleration values are always reported as zero (not provided by the current API)
- Returns `RESULT_OK` if successfully retrieved entity state
- Returns `RESULT_NOT_FOUND` if entity does not exist
- Returns `RESULT_OPERATION_FAILED` if error retrieving entity state

### Using the GetEntitiesStates Service

The GetEntitiesStates service fetches the states (pose, twist, acceleration) of multiple entities in the simulation.

1. Get states for all entities in the simulation:
   ```bash
   ros2 service call /isaacsim/GetEntitiesStates simulation_interfaces/srv/GetEntitiesStates "{filters: {filter: ''}}"
   ```

2. Get states for entities containing 'robot' in their path:
   ```bash
   ros2 service call /isaacsim/GetEntitiesStates simulation_interfaces/srv/GetEntitiesStates "{filters: {filter: 'robot'}}"
   ```

3. Get states for entities with paths starting with '/World':
   ```bash
   ros2 service call /isaacsim/GetEntitiesStates simulation_interfaces/srv/GetEntitiesStates "{filters: {filter: '^/World'}}"
   ```

Notes:
- Combines functionality from GetEntities and GetEntityState services
- Filters entities first using regex pattern matching
- Retrieves state for each filtered entity
- Returns list of entity paths and corresponding states
- For entities with RigidBodyAPI, both pose and velocities will be returned
- For entities without RigidBodyAPI, only pose will be returned with zero velocities
- Acceleration values are always reported as zero (not provided by the current API)
- Using this service is more efficient than making multiple GetEntityState calls when you need states for many entities
- Returns `RESULT_OK` if successfully retrieved entity states
- Returns `RESULT_OPERATION_FAILED` if error in filtering or retrieving states

### Using the DeleteEntity Service

The DeleteEntity service deletes a specified entity in the simulation.

```bash
ros2 service call /isaacsim/DeleteEntity simulation_interfaces/srv/DeleteEntity "{entity: '/World/robot'}"
```

Notes:
- The service will return `RESULT_OK` if the entity was successfully deleted
- Returns `RESULT_OPERATION_FAILED` if the entity is protected and cannot be deleted
- Uses prim_utils.is_prim_no_delete() to check if a prim can be deleted before attempting deletion

### Using the SpawnEntity Service

The SpawnEntity service spawns a new entity into the simulation at a specified location.

1. Basic entity spawn with default position:
   ```bash
   ros2 service call /isaacsim/SpawnEntity simulation_interfaces/srv/SpawnEntity "{name: 'MyEntity', allow_renaming: false, uri: '/path/to/model.usd'}"
   ```

2. Spawn with specific position and orientation:
   ```bash
   ros2 service call /isaacsim/SpawnEntity simulation_interfaces/srv/SpawnEntity "{name: 'PositionedEntity', allow_renaming: false, uri: '/path/to/model.usd', initial_pose: {pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}}}"
   ```

3. Empty Xform creation (no URI):
   ```bash
   ros2 service call /isaacsim/SpawnEntity simulation_interfaces/srv/SpawnEntity "{name: 'EmptyXform', allow_renaming: false, uri: ''}"
   ```

4. With auto-renaming enabled:
   ```bash
   ros2 service call /isaacsim/SpawnEntity simulation_interfaces/srv/SpawnEntity "{name: 'AutoRenamedEntity', allow_renaming: true, uri: '/path/to/model.usd'}"
   ```

5. With namespace specified:
   ```bash
   ros2 service call /isaacsim/SpawnEntity simulation_interfaces/srv/SpawnEntity "{name: 'NamespacedEntity', allow_renaming: false, uri: '/path/to/model.usd', entity_namespace: 'robot1'}"
   ```

Notes:
- If URI is provided, loads the USD file as a reference in the given prim path
- If URI is not provided, creates a Xform at the given prim path
- All spawned prims are marked with a simulationInterfacesSpawned attribute for tracking
- Returns `RESULT_OK` if the entity was successfully spawned
- Returns `NAME_NOT_UNIQUE (101)` if the entity name already exists and allow_renaming is false
- Returns `NAME_INVALID (102)` if the entity name is empty and allow_renaming is false
- Returns `RESOURCE_PARSE_ERROR (106)` if failed to parse or load USD file

### Using the ResetSimulation Service

The ResetSimulation service resets the simulation environment to its initial state.

```bash
ros2 service call /isaacsim/ResetSimulation simulation_interfaces/srv/ResetSimulation
```

Notes:
- Stops the simulation timeline
- Finds and removes all prims with simulationInterfacesSpawned attribute
- Uses multiple passes to ensure all spawned entities are removed
- Restarts the simulation timeline
- Returns `RESULT_OK` if successfully reset
- Returns `RESULT_OPERATION_FAILED` if error resetting simulation

### Using the SetEntityState Service

The SetEntityState service sets the state (pose, twist) of a specific entity in the simulation. Only transforms in the **world** frame are currently accepted.

1. Set only position and orientation:
   ```bash
   ros2 service call /isaacsim/SetEntityState simulation_interfaces/srv/SetEntityState "{
     entity: '/World/Cube', 
     state: {
       header: {frame_id: 'world'}, 
       pose: {
         position: {x: 1.0, y: 2.0, z: 3.0}, 
         orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
       },
       twist: {
         linear: {x: 0.0, y: 0.0, z: 0.0},
         angular: {x: 0.0, y: 0.0, z: 0.0}
       }
     }
   }"
   ```

2. Set position, orientation and velocity (for entities with rigid body physics):
   ```bash
   ros2 service call /isaacsim/SetEntityState simulation_interfaces/srv/SetEntityState "{
     entity: '/World/RigidBody', 
     state: {
       header: {frame_id: 'world'}, 
       pose: {
         position: {x: 1.0, y: 2.0, z: 3.0}, 
         orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
       },
       twist: {
         linear: {x: 0.5, y: 0.0, z: 0.0},
         angular: {x: 0.0, y: 0.0, z: 0.1}
       }
     }
   }"
   ```

Notes:
- The position and orientation are always updated for any entity
- Velocities are only set for entities with a RigidBodyAPI
- For non-rigid bodies, only position and orientation will be set (velocity settings are ignored)
- Acceleration settings are not currently supported and will be ignored
- Returns `RESULT_OK` if successfully set entity state
- Returns `RESULT_NOT_FOUND` if entity does not exist
- Returns `RESULT_OPERATION_FAILED` if error setting entity state

### Using the StepSimulation Service

The StepSimulation service simulates a finite number of steps and returns to PAUSED state.

1. Step the simulation by 1 frame (note: will use 2 steps internally):
   ```bash
   ros2 service call /isaacsim/StepSimulation simulation_interfaces/srv/StepSimulation "{steps: 1}"
   ```

2. Step the simulation by 10 frames:
   ```bash
   ros2 service call /isaacsim/StepSimulation simulation_interfaces/srv/StepSimulation "{steps: 10}"
   ```

3. Step the simulation by 100 frames:
   ```bash
   ros2 service call /isaacsim/StepSimulation simulation_interfaces/srv/StepSimulation "{steps: 100}"
   ```

Notes:
- The simulation must be in a paused state before stepping can be performed
- The service call will block until all steps are completed
- After stepping completes, the simulation will automatically return to a paused state
- **Important limitation**: When steps=1 is requested, the service will automatically use steps=2 instead. Only step values greater than 1 are available. The minimum effective step count is 2.
- Returns `RESULT_OK` if stepping completed successfully
- Returns `RESULT_INCORRECT_STATE` if the simulation is not paused when the service is called
- Returns `RESULT_OPERATION_FAILED` if any error occurs during stepping

### Using the SimulateSteps Action

The SimulateSteps action simulates a finite number of steps and returns to PAUSED state with feedback after each step.

1. Basic usage - Step the simulation by 10 frames:
   ```bash
   ros2 action send_goal /isaacsim/SimulateSteps simulation_interfaces/action/SimulateSteps "{steps: 10}"
   ```

2. With feedback - Step the simulation by 20 frames and show feedback:
   ```bash
   ros2 action send_goal /isaacsim/SimulateSteps simulation_interfaces/action/SimulateSteps "{steps: 20}" --feedback
   ```

Notes:
- The simulation must be in a paused state before stepping can be performed
- After steps are completed, the simulation will return to a paused state
- You will receive feedback after each step showing completed and remaining steps
- The action can be canceled while executing
- **Important limitation**: When steps=1 is requested, the action will automatically use steps=2 instead. Only step values greater than 1 are available. The minimum effective step count is 2.

## Technical Details

The extension uses the `omni.timeline` interface to control the simulation state and provides a clean ROS 2 interface through standard services. The implementation includes:

- A singleton `ROS2ServiceManager` to handle all ROS 2 services through a single node
- A `SimulationControl` class that interfaces with Isaac Sim's timeline
- Thread-safe implementation for ROS 2 spinning independant of Action Graph interface

## Extending

To add more simulation control services, extend the `SimulationControl` class and register additional services with the `ROS2ServiceManager`. 
