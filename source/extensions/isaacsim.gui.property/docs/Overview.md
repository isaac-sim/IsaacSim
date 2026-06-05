# Overview

The isaacsim.gui.property extension provides specialized property panel widgets for robotics workflows in Isaac Sim. It extends the USD Property Window with custom widgets designed specifically for editing robot-related schemas, arrays, custom data, and Isaac Sim-specific attributes on USD prims.

## Key Components

### Robot Schema Widgets

**Robot API Schema Management**: The {class}`RobotAPIWidget <isaacsim.gui.property.RobotAPIWidget>` provides a property interface for managing Robot API schema on USD prims, allowing users to configure robot-specific properties like description, namespace, robot type, license, and version information.

**Link and Joint Configuration**: The {class}`LinkAPIWidget <isaacsim.gui.property.LinkAPIWidget>` and {class}`JointAPIWidget <isaacsim.gui.property.JointAPIWidget>` handle link-specific and joint-specific properties respectively, including joint name overrides, degree of freedom configurations, and actuator settings for articulated robots.

**Motion Planning Integration**: The {class}`MotionPlanningAPIWidget <isaacsim.gui.property.MotionPlanningAPIWidget>` exposes IsaacMotionPlanningAPI properties for configuring motion planning parameters and constraints directly in the property panel.

### Specialized Data Widgets

**Array Property Management**: The {class}`ArrayPropertiesWidget <isaacsim.gui.property.ArrayPropertiesWidget>` provides editing functionality for USD array-based attributes including integers, floats, and multi-dimensional vectors (int[], float[], int2[], float2[], int3[], float3[], int4[], float4[]). It creates dedicated editing windows where users can view, add, remove, and modify individual array elements with appropriate input fields based on the array's data type.

**Custom Data Editor**: The {class}`CustomDataWidget <isaacsim.gui.property.CustomDataWidget>` offers a JSON text editor interface for viewing and modifying custom metadata stored on USD prims. It validates input in real-time and converts complex data types to lists for JSON compatibility while preserving original data structure.

**Name and Namespace Management**: The {class}`NameOverrideWidget <isaacsim.gui.property.NameOverrideWidget>` and {class}`NamespaceWidget <isaacsim.gui.property.NamespaceWidget>` provide interfaces for managing name override and namespace attributes on prims that don't have robot schema APIs, enabling alternative name searches and organizational categorization.

### Context Menu Integration

Each widget integrates with the property panel's context menu system, automatically adding menu entries like "Isaac/Robot Schema/Robot API" or "Isaac/NameOverride" when appropriate prims are selected. The widgets intelligently show or hide based on prim selection and existing schema presence.

## Extensibility

The extension uses a base class approach with `_RobotSchemaWidgetBase` providing unified functionality for robot schema widgets. This pattern allows consistent behavior across different robot schema types while enabling specialized attribute handling for each schema variant.

All widgets implement singleton patterns to ensure only one instance exists per session, and they automatically handle schema application, removal, and property synchronization with the underlying USD stage.
