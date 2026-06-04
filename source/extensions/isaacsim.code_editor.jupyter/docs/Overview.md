# Overview

The isaacsim.code_editor.jupyter extension integrates Jupyter Notebook functionality directly into Isaac Sim, providing an interactive Python development environment that seamlessly connects with the simulation. This extension creates a bridge between Jupyter's web-based notebook interface and Isaac Sim's Python environment, enabling users to develop, test, and execute code interactively while maintaining full access to Isaac Sim's APIs and capabilities.

## Functionality

**Complete Jupyter Integration**: The extension launches a full Jupyter Notebook server alongside Isaac Sim, creating a web-based development environment. Users can access standard Jupyter features including notebook creation, cell-based code execution, markdown documentation, and data visualization.

**Real-time Code Execution**: Python code written in Jupyter notebooks executes directly within Isaac Sim's Python environment. This allows immediate interaction with simulation objects, scene manipulation, and robot control without context switching between applications.

**Intelligent Code Assistance**: The extension provides enhanced development support through Jedi-powered code completion and introspection. Users receive intelligent suggestions for Isaac Sim APIs, parameter hints, and documentation lookup directly within the notebook interface.

**Output and Error Handling**: All code execution results, including standard output, error messages, and exception traces, are captured and displayed within the notebook cells. This provides immediate feedback for debugging and development workflows.

## Configuration

The extension offers comprehensive configuration options to adapt to different development environments and security requirements:

**Server Configuration**: Users can customize the host IP address and port settings for both the internal communication server (`host` and `port`) and the Jupyter server (`notebook_ip` and `notebook_port`). This enables remote access scenarios and multi-user environments.

**Authentication and Security**: The `notebook_token` setting allows users to configure token-based authentication for the Jupyter server. When left empty, the server runs without authentication for local development convenience.

**Workspace Management**: The `notebook_dir` setting specifies the root directory for Jupyter notebooks. If not configured, notebooks are stored in the extension's dedicated directory, providing organized project management.

**Advanced Options**: Additional Jupyter server parameters can be specified through `command_line_options`, allowing fine-tuned control over server behavior such as kernel management and security policies.

**Port Management**: The `kill_processes_with_port_in_use` setting controls whether the extension automatically terminates processes using required ports, enabling smooth startup in development environments.

## UI Components

### Menu Integration

The extension adds a menu item to Isaac Sim's interface that provides quick access to the Jupyter environment. The menu item displays the configured server URL and opens the Jupyter interface in the default web browser, streamlining the transition between Isaac Sim and notebook development.

## Integration

The extension establishes a socket-based communication channel between Isaac Sim and the Jupyter server, ensuring that notebook code executes within Isaac Sim's Python context. This architecture preserves access to all Isaac Sim modules, extensions, and simulation state while providing the interactive development benefits of Jupyter notebooks.

The Python environment configuration automatically includes Isaac Sim's module paths and extension APIs, enabling seamless import and usage of simulation-specific functionality within notebook cells.
