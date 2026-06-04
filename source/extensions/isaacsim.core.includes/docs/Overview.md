# Overview

The isaacsim.core.includes extension provides essential header files and C++ plugin infrastructure for Isaac Sim development. This extension serves as a foundational component that makes common Isaac Sim headers available to other extensions that need to build against or extend Isaac Sim's core functionality.

## Key Components

### Header Files

The extension packages commonly used Isaac Sim header files that other extensions can include when building native C++ components. These headers provide access to core Isaac Sim APIs and data structures needed for robotics simulation development.

### C++ Plugin Support

The extension includes native plugin infrastructure through its binary plugin files. These plugins are automatically discovered and loaded, providing the underlying C++ functionality that supports Isaac Sim's simulation capabilities.

## Integration

This extension functions as a dependency for other Isaac Sim extensions that require native C++ development capabilities. Extensions building custom physics components, sensor implementations, or performance-critical simulation features typically depend on the headers and plugin infrastructure provided by this extension.

The extension loads early in the startup sequence to ensure the necessary headers and plugin support are available before other Isaac Sim components initialize.
