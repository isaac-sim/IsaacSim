# Overview

The isaacsim.app.setup extension handles the initial configuration and layout setup for the Isaac Sim application. This extension configures window arrangements, application menus, and various Isaac Sim-specific UI components during application startup.

## Functionality

The extension manages several key aspects of Isaac Sim application initialization:

**Window Layout Management** - The {func}`load_layout <isaacsim.app.setup.load_layout>` function provides programmatic control over window arrangements by loading saved layout configurations from JSON files. This allows for consistent workspace setups across different Isaac Sim sessions.

**Application Configuration** - The extension handles core application setup including window docking order, property window delegate configuration, and menu system initialization. It establishes the standard Isaac Sim window arrangement with proper positioning for Stage, Layer, Console, Content, and Assets windows.

**Platform-Specific Settings** - The extension includes platform-specific configuration options for ROS bridge integration and simulation control extensions, allowing different setups for Windows and Linux environments.

## Key Components

### Layout Loading

The {func}`load_layout <isaacsim.app.setup.load_layout>` function accepts a JSON layout file path and applies the saved window configuration:

```python
await load_layout("path/to/layout.json", keep_windows_open=False)
```

The function includes timing coordination to avoid conflicts with main window initialization and provides options for preserving existing window states.

### Startup Configuration

The extension manages Isaac Sim-specific startup behaviors through its settings, including automatic stage creation and optional ROS bridge integration based on platform requirements.

## Integration

The extension uses `**omni.kit.quicklayout**` for layout management operations and integrates with `**omni.kit.window.property**` to configure property window delegates for Isaac Sim-specific prim types. It also leverages `**omni.kit.menu.common**` for menu system setup and `**omni.kit.stage_templates**` for initial stage configuration.
