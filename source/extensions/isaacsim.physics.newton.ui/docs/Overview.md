# Overview

The isaacsim.physics.newton.ui extension provides UI integration for Newton and Mujoco physics schemas within Isaac Sim. It registers schema definitions and property widgets that enable users to configure and interact with Newton and Mujoco physics parameters through the property panel interface.

## Key Components

### Schema Registration

The extension provides functions to retrieve schema type names for both physics systems:

- {func}`get_newton_schema_names <isaacsim.physics.newton.ui.get_newton_schema_names>` returns prim type names and API schema names for Newton physics
- {func}`get_mujoco_schema_names <isaacsim.physics.newton.ui.get_mujoco_schema_names>` returns prim type names and API schema names for Mujoco physics

These functions enable the property system to recognize and handle Newton and Mujoco specific schemas.

### UI Definitions

**{class}`NewtonUiDefinitions <isaacsim.physics.newton.ui.NewtonUiDefinitions>`** and **{class}`MujocoUiDefinitions <isaacsim.physics.newton.ui.MujocoUiDefinitions>`** classes contain comprehensive UI configuration for their respective physics systems. Each definition class includes:

- Property widget specifications for displaying physics parameters
- Property builders for creating custom UI elements
- Property ordering configurations for consistent layout
- Extension mappings and filtering rules

### Newton Scene Widget

**ExtendedNewtonSceneWidget** provides specialized property handling for Newton physics scenes. The widget automatically detects when Newton physics is the active simulation variant and dynamically adds Newton-specific properties like the "newton:solver" selection to the property panel. This allows users to configure solver settings directly through the UI when Newton physics is enabled.

## Functionality

The extension integrates Newton and Mujoco physics configuration into Isaac Sim's existing property panel system. When users select physics objects in the scene, the appropriate physics-specific properties become available for editing through the standard property interface. The extension handles the registration of custom widgets and ensures proper display ordering of physics parameters.

For Newton physics specifically, the extension provides dynamic property injection based on the active simulation variant, automatically showing relevant Newton solver options when Newton physics is selected for the scene.
