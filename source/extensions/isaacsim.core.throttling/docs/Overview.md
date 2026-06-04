# Overview

The isaacsim.core.throttling extension automatically optimizes Isaac Sim performance by managing rendering settings based on timeline state. It provides intelligent throttling mechanisms that adjust system behavior during simulation playback to balance performance with simulation accuracy, then restore optimal settings for scene editing.

## Key Components

### Async Rendering Toggle

The extension provides automatic async rendering management that responds to timeline events. When enabled, it disables async rendering during simulation playback to ensure deterministic behavior, which is critical for accurate physics simulation and reproducible results. After stopping or pausing the timeline, async rendering is re-enabled with a frame delay to restore optimal rendering performance during scene editing.

### Manual Loop Mode

Manual loop mode allows the extension to take direct control of the application loop during simulation. This provides more precise timing control and can improve performance in specific simulation scenarios where frame-by-frame execution control is beneficial.

### Performance Optimization Features

The extension manages additional performance-related settings during simulation:

- **Eco Mode Management**: Automatically disables eco mode during simulation to ensure full system resources are available for computation
- **Gizmo Visibility Control**: Hides visual gizmos during playback to reduce rendering overhead, then restores them for improved visual feedback during scene editing

## Functionality

The extension operates through timeline event monitoring, automatically detecting when simulation starts (play), stops, or pauses. It maintains frame counting mechanisms to ensure proper timing of setting changes, particularly for the delayed re-enabling of async rendering after simulation ends.

**Automatic Response to Timeline Events**: The extension subscribes to timeline play, stop, and pause events, applying appropriate throttling settings for each state without requiring user intervention.

**Frame-Delayed Recovery**: When transitioning from simulation back to editing mode, settings are restored with appropriate frame delays to ensure system stability.

## Configuration

The extension provides two main configuration options:

- `enable_async`: Controls whether automatic async rendering toggle functionality is active
- `enable_manualmode`: Enables manual loop mode for direct application loop control during simulation

These settings allow users to enable only the throttling behaviors that benefit their specific workflow and hardware configuration.
