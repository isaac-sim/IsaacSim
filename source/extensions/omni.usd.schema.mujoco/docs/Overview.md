# Overview

The Omni USD Mujoco Schema extension hosts and registers Mujoco USD schemas for simulation workflows. This extension provides the foundational schema definitions needed to work with Mujoco physics simulation data within the USD ecosystem, enabling consistent data representation and interoperability for physics-based simulations.

## Key Components

### Schema Registration
The extension automatically registers Mujoco-specific USD schemas when loaded, making them available to the USD stage and other USD-aware components. These schemas define the structure and properties for Mujoco simulation elements, ensuring proper data validation and type safety when working with Mujoco physics data in USD format.

### Plugin System Integration
The extension integrates with USD's plugin architecture to make Mujoco schemas discoverable and usable across the USD ecosystem. The schema definitions are loaded early in the application lifecycle to ensure they are available before other extensions attempt to use Mujoco-related USD data.

## Integration

The extension serves as a dependency for other simulation-related extensions that need to work with Mujoco physics data. By providing these schema definitions at the USD level, it enables consistent data exchange between different simulation components and ensures that Mujoco physics properties are properly represented in USD stages.

The early load order ensures that Mujoco schemas are registered before other extensions that depend on them, maintaining proper initialization sequence for simulation workflows that combine USD data management with Mujoco physics simulation capabilities.
