# Isaac Sim Extension Templates

Create new Isaac Sim extensions from standardized templates using the `repo template` tool.

## Available Templates

| Template | Description |
|----------|-------------|
| **Isaac Sim Python Extension** | Python-only extension with `omni.ext.IExt` lifecycle, tests, and docs |
| **Isaac Sim UI Extension** | Python extension with Examples Browser integration, scene management, physics callbacks, and custom UI |
| **Isaac Sim C++ and Python Extension** | C++ Carbonite plugin with pybind11 bindings and Python wrapper |
| **Isaac Sim OmniGraph Node Extension** | C++ and Python OmniGraph nodes with `.ogn` definitions, Carbonite plugin, and pybind11 bindings |

## Usage

### Create a new extension interactively

```bash
./repo.sh template new
```

This launches an interactive prompt that walks you through:

1. Choosing **Extension** as the template type
2. Selecting one of the four templates above
3. Entering variable values (extension name, title, version, etc.)

The generated extension is placed in `source/extensions/<extension_name>/` and is automatically discovered by the build system.

### Variables

Each template prompts for the following variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `extension_name` | `isaacsim.my.extension` | Full dotted extension name (e.g. `isaacsim.sensors.lidar`) |
| `title` | `My Extension` | Human-readable title shown in the Extension Manager |
| `version` | `0.1.0` | Initial semantic version |
| `description` | *(varies)* | One-line description for `extension.toml` |
| `category` | `Simulation` | Extension category |

The C++ template additionally prompts for:

| Variable | Default | Description |
|----------|---------|-------------|
| `binding_module` | `my_extension` | pybind11 module name (produces `_<name>.so`) |

The OmniGraph template does not need a `binding_module` — it is derived automatically from the extension name by the OGN build system.

The following variables are derived automatically from `extension_name`:

| Variable | Example |
|----------|---------|
| `python_module` | `isaacsim.sensors.lidar` |
| `python_module_path` | `isaacsim/sensors/lidar` |
| `python_module_toplevel` | `isaacsim` |
| `current_date` | `2026-03-31` |

### Non-interactive usage (automation / CI)

Generate a playback file first:

```bash
./repo.sh template new --generate-playback my_extension.toml
```

Then replay it without prompts:

```bash
./repo.sh template replay my_extension.toml
```

Example playback file:

```toml
[isaacsim-python-extension]
extension_name = "isaacsim.sensors.lidar"
title = "Lidar Sensor"
version = "0.1.0"
description = "Provides lidar sensor simulation."
category = "Sensors"
```

### List existing extensions

```bash
./repo.sh template list
```

## Build and verify

After creating an extension:

```bash
# Build
./build.sh

# Run tests
cd _build/linux-x86_64/release
./tests/tests-<extension_name>.sh
```

The new extension's `premake5.lua` is auto-discovered under `source/extensions/`.

## Naming Convention

Extensions **must** follow the `isaacsim.<category>[.<subcategory>...]` naming pattern. For example:

- `isaacsim.sensors.camera`
- `isaacsim.core.utils`
- `isaacsim.robot.controllers`

## Template Structure

Each template lives under `templates/<template-name>/` and contains files with `{{variable}}` Jinja2 placeholders that are rendered at creation time. The template catalog is defined in `templates/templates.toml`.

To add a new template, create a directory with the scaffolded files and register it in `templates.toml`. See the [repo_kit_template documentation](https://github.com/NVIDIA-Omniverse/repo_kit_template) for the full authoring guide.
