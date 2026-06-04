# Overview

```{deprecated} 6.0.0
This extension is deprecated. No replacement is provided.
```

The **omni.isaac.ml_archive** extension provides machine learning pip packages required by Isaac Sim extensions. This extension bundles essential ML libraries and dependencies into the Isaac Sim environment, making them available to other extensions that require machine learning functionality.

## Functionality

The extension serves as a pip package archive specifically tailored for machine learning workflows within Isaac Sim. It pre-bundles ML-related Python packages to ensure they are available when needed by other Isaac Sim extensions without requiring separate installation or dependency management by users.

**Package Delivery**: The extension uses a pip prebundle module to deliver pre-packaged Python libraries directly into the Isaac Sim environment. This approach ensures consistent availability of ML dependencies across different installations and platforms.

**Platform Compatibility**: The extension is designed to be platform-specific, adapting to different operating systems while maintaining consistent ML package availability.

## Key Components

### Pip Prebundle Module

The pip_prebundle module contains the actual ML packages that are made available to the Isaac Sim environment. This module handles the integration of pre-packaged Python libraries without requiring runtime pip installations.

### Archive Integration

The extension integrates with the broader Isaac Sim archive system, building upon the core archive infrastructure to provide specialized ML package support.

## Relationships

The extension builds upon **omni.isaac.core_archive** to access the main Isaac Sim pip archive infrastructure and extends **omni.kit.pip_archive** for base Python package management capabilities. This layered approach ensures that ML packages are properly integrated with the existing package management system while maintaining compatibility with the core Isaac Sim environment.

The extension loads early in the startup sequence to ensure ML packages are available before other extensions that depend on them attempt to load.
