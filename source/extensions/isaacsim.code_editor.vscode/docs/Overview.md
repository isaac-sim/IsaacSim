# Overview

The isaacsim.code_editor.vscode extension provides Visual Studio Code launcher and menu integration for Isaac Sim. It adds a *Window > VS Code* menu item that opens VS Code pointed at the application directory and displays connection details for the Python execution server.

The actual Python code execution server is provided by the **isaacsim.code_editor.python_server** extension, which this extension depends on.

## Functionality

### VS Code Launcher

The extension adds a menu item under *Window > VS Code* that launches Visual Studio Code with the Isaac Sim application directory. On success, a notification displays the server host and port. On failure, a warning notification provides troubleshooting guidance.

### UI Builder

The UIBuilder manages the lifecycle of the menu item and handles launching VS Code via the `code` CLI command. It reads the Python server's host and port settings to display connection information.

## Integration

The extension integrates with:

- **isaacsim.code_editor.python_server** for the TCP code execution server
- **omni.kit.notification_manager** (optional) for user notifications
- **omni.kit.uiapp** (optional) for menu UI functionality
- **isaacsim.core.deprecation_manager** for settings migration
