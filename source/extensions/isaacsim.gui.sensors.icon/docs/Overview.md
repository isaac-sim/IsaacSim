# Overview

The isaacsim.gui.sensors.icon extension renders sensor icons in the viewport and stage hierarchy for visual identification of camera, lidar, IMU, and other sensor prims in Isaac Sim.

## Concepts

### Sensor Icon System

The extension automatically detects various sensor types in the USD stage and displays interactive icons at their world positions. Supported sensor types include OmniLidar, IsaacContactSensor, IsaacImuSensor, and IsaacRaycastSensor, as well as deprecated types (Lidar, IsaacLightBeamSensor, Generic) for backward compatibility.

Icons are positioned based on the sensor prim's world transform and update dynamically when the stage changes or when sensors are moved. Each icon can have custom click callbacks for interactive behavior.

### Visual Integration

The extension integrates with both the 3D viewport and stage widget to provide consistent sensor visualization. In the viewport, icons appear as overlay graphics positioned at sensor locations. In the stage hierarchy, sensor prims display custom icons to help users quickly identify different sensor types.

## Key Components

### {class}`IconModel <isaacsim.gui.sensors.icon.IconModel>`

The {class}`IconModel <isaacsim.gui.sensors.icon.IconModel>` class serves as a manipulator model that manages all sensor icons within the viewport scene. It automatically scans the USD stage for recognized sensor prims and creates corresponding icon items.

The model maintains connections to both USD and USDRT stages for efficient querying and real-time updates. It responds to stage events such as opening, closing, and frame updates to keep icons synchronized with the current scene state.

```python
model = IconModel()
model.add_sensor_icon("/World/Camera", "path/to/camera_icon.png")
model.set_icon_click_fn("/World/Camera", callback_function)
```

### {class}`IconScene <isaacsim.gui.sensors.icon.IconScene>`

{class}`IconScene <isaacsim.gui.sensors.icon.IconScene>` provides the viewport window and manipulator framework for displaying sensor icons. It manages the visual presentation layer and handles icon scaling and visibility states.

```python
scene = IconScene(title="Sensor Icons", icon_scale=1.5)
scene.visible = True  # Show/hide all sensor icons
```

### Singleton Management

The extension uses a singleton pattern accessed through {func}`get_instance <isaacsim.gui.sensors.icon.get_instance>` to ensure consistent sensor icon management across the application. This provides a central point for adding, removing, and configuring sensor icons.

## Functionality

### Automatic Sensor Detection

The extension continuously monitors the USD stage for sensor prims and automatically creates icons for newly added sensors. Icons are removed when their corresponding prims are deleted from the stage.

### Interactive Icons

Each sensor icon can have custom click callbacks, enabling interactive workflows where users can click sensor icons to open configuration panels, focus cameras, or trigger other sensor-specific actions.

### Visibility Control

Sensor icons respect both USD visibility attributes and the extension's global visibility settings. Icons can be shown or hidden individually or collectively through the viewport's "Show By Type" menu under the "Sensors" category.

### Stage Widget Integration

The extension integrates with `**omni.kit.widget.stage**` to display custom icons for sensor prims directly in the stage hierarchy, making it easier to identify different sensor types at a glance.
