# Overview

The isaacsim.replicator.behavior.ui extension provides UI components for managing behavior script configuration within Isaac Sim's Replicator framework. The extension integrates with the Property window to display exposed variables from behavior scripts, enabling users to configure script parameters through a specialized property widget interface.

## Key Components

### {class}`ExposedVariablesPropertyWidget <isaacsim.replicator.behavior.ui.ExposedVariablesPropertyWidget>`

**{class}`ExposedVariablesPropertyWidget <isaacsim.replicator.behavior.ui.ExposedVariablesPropertyWidget>` extends the standard USD property system to display filtered script variables.** This specialized widget focuses specifically on properties that match defined namespace prefixes, typically those marked as exposed variables from behavior scripts.

The widget creates a hierarchical organization of properties based on their namespace structure. For properties with multiple namespace levels like `exposedVar:locationRandomizer:includeChildren`, it removes the filter namespace and creates nested display groups from the remaining parts. This approach provides clear visual organization of complex behavior configurations.

When multiple prims are selected, the widget groups properties by prim path to maintain clear separation between different script instances. The widget only processes prims that have scripting capabilities and contain actual scripts, filtering out non-relevant selections automatically.

```python
# Initialize the widget with specific namespace filtering
widget = ExposedVariablesPropertyWidget(
    title="Behavior Variables",
    attribute_namespace_filter=['exposedVar'],
    collapsed=False
)
```

## Integration

The extension integrates with **omni.kit.property.usd** to extend the standard USD property system and **omni.kit.window.property** to register the widget within the Property window interface. This integration ensures that behavior script variables appear alongside other USD properties in a consistent interface while maintaining their specialized filtering and organization.
