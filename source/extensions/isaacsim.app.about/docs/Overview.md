# Overview

The Isaac Sim About Window extension provides an About dialog that displays application information and system details. This dialog shows the Isaac Sim application name and version, Kit SDK version, client library version, and a comprehensive list of all currently loaded plugins.

```{image} ../../../../source/extensions/isaacsim.app.about/data/preview.png
---
align: center
---
```


## UI Components

### About Dialog Window

The main component is an About dialog window that presents system information in a user-friendly format. The dialog consolidates key version information and plugin details into a single view, helping users understand their current Isaac Sim configuration.

The window displays:
- Isaac Sim application name and version
- Kit SDK version information
- Client library version details
- Complete list of loaded plugins

### Menu Integration

The extension integrates with Isaac Sim's menu system using `**omni.kit.menu.utils**` to provide easy access to the About dialog. Users can access the dialog through the application's menu system without requiring any programmatic interaction.

## Functionality

**Version Information Display**: The extension retrieves and caches version information from multiple sources including the Kit SDK and client libraries. This information is formatted and presented in the About dialog for easy reference.

**Plugin Enumeration**: The dialog displays all currently loaded plugins, providing insight into the active components of the Isaac Sim environment. This helps users understand what functionality is available in their current session.

**Dynamic Content Loading**: Version and plugin information is loaded dynamically when the About dialog is requested, ensuring the displayed information reflects the current state of the application.

## Relationships

This extension depends on `isaacsim.core.version` to retrieve Isaac Sim-specific version information and integrates with `**omni.client**` for client library version details. The menu integration is implemented through `**omni.kit.menu.utils**`, which handles the registration of menu items that trigger the About dialog.
