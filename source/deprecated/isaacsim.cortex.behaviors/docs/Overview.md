# Overview

```{deprecated} 6.0.0
This extension has been deprecated and will be replaced by open source equivalents and simple examples.
```

The isaacsim.cortex.behaviors extension provides a library of sample behaviors for the Isaac Cortex decision framework. These behaviors demonstrate reactive task execution on simulated robots, including block stacking, pick-and-place, and game-playing scenarios using decider networks and state machines.

## Included Behaviors

- **Block stacking (Franka)**: A reactive block tower building behavior that monitors tower state, plans grasp poses, and recovers from disturbances. Includes perception synchronization and collision-aware motion planning.
- **Peck game (Franka)**: Demonstrates decider network and state machine approaches to a simple interactive game behavior.
- **Simple behaviors (Franka)**: Minimal decider network and state machine examples for learning the Cortex framework.
- **Bin stacking (UR10)**: A pick-and-place state machine for bin manipulation using a UR10 robot.

## Key Components

- **BuildTowerContext**: Manages block world state, tower tracking, and robot perception for the stacking behavior
- **Grasp planning utilities**: Functions for generating and scoring candidate grasp transforms based on approach direction and obstacle proximity
- **State machines and decider networks**: Reusable behavior building blocks for pick, place, lift, and navigation actions
