<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Isaac Sim examples

These examples show how to use the public APIs from installed Isaac Sim libraries. They do not import private
implementation packages or add library source directories to build and import paths. Windows examples require
Windows 11.

## Layout and authoring

- `benchmarks/` contains standalone performance workloads. See [benchmarking](benchmarks/README.md).
- `libraries/` contains focused examples of one public library API.
- `series/<name>/` contains independently runnable examples grouped by a `series.toml` file.
- `benchmarks/<category>/` contains independently runnable performance benchmarks measured by
  `isaacsim.performance.benchmarking`, which they declare like any other library requirement.
- Every example has a concise `README.md` and an `example.toml` manifest.

Keep each entry point focused on one workflow. Use `isaacsim.foundation` and other public APIs instead of authoring USD
directly when they support the operation. Put test-only setup and assertions in `test.py`, then select that file with
`[[test]].path`. Put longer walkthroughs in the tutorial documentation.

The runner finds `example.toml` files recursively. Examples are published by default; set `published = false` for an
incomplete or private example. IDs remain stable when directories move. Manifest validation rejects missing or
unknown fields, invalid values, unsafe paths, and incompatible build or run settings.

## Manifest reference

### Top-level fields

| Field | Required | Description |
| --- | --- | --- |
| `id` | Yes | Stable selector made of lowercase, dot-separated segments matching `[a-z][a-z0-9_]*`. |
| `title` | Yes | Non-empty user-facing title. |
| `summary` | Yes | Non-empty summary of the example's purpose. |
| `owners` | Yes | Non-empty, unique public API owner IDs, such as `isaacsim.common.logging`. |
| `published` | No | Boolean discovery switch; defaults to `true`. |
| `categories` | No | Unique discovery names matching `[a-z][a-z0-9_]*`; may be empty. |
| `topics` | No | Unique cross-cutting discovery names; may be empty. |
| `build` | Yes | Build adapter table. |
| `run` | Yes | Default entry-point table, optionally containing named run tables. |
| `requirements` | Yes | Direct library, package, and system requirements. |
| `test` | No | Array of named test configurations. |

Manifests do not need a version because the examples, runner, installer, and libraries ship together.

### Build and run adapters

| Table | Field | Required | Description |
| --- | --- | --- | --- |
| `[build]` | `adapter` | Yes | `none` or `cmake`. |
| `[build]` | `targets` | For `cmake` | Non-empty, unique targets matching `[A-Za-z0-9_][A-Za-z0-9_.+-]*`. |
| `[run]` | `adapter` | Yes | `executable`, `python`, or `command`. |
| `[run]` | `target` | For `executable` | Target listed in `build.targets`. |
| `[run]` | `path` | For `python` | Existing script beneath the example root, expressed as a relative POSIX path. |
| `[run]` | `command` | For `command` | Executable name resolved on the prepared environment's `PATH`; no paths or shell syntax. |
| `[run]` | `arguments` | No | Ordered default arguments; empty and duplicate strings are allowed. |

The `none` build adapter accepts no additional fields. The `cmake` adapter requires `CMakeLists.txt`, and executable
examples must use it. A Python-only example normally combines `build.adapter = "none"` with
`run.adapter = "python"`.

### Named run commands

Use `[run]` for the default entry point and `[run.NAME]` for related commands. Names must match
`[a-z][a-z0-9_]*` and cannot be `adapter`, `arguments`, `command`, `path`, or `target`. Each named command declares all
of its own run settings; it does not inherit fields from `[run]`. All commands share the same build and requirements.

```toml
[run]
adapter = "python"
path = "main.py"

[run.server]
adapter = "command"
command = "isaacsim-ovsim-grpc-server"
arguments = ["--listen-address", "127.0.0.1:50051"]
```

For a source build, build the required libraries and prepare the example dependencies from the repository root:

```bash
source/examples/examples.sh build falling_cube_ovsim.simulate_cube_grpc
```

Then run these commands in separate terminals:

```bash
source/examples/examples.sh run falling_cube_ovsim.simulate_cube_grpc:server
source/examples/examples.sh run falling_cube_ovsim.simulate_cube_grpc -- --endpoint 127.0.0.1:50051
```

Use `source\examples\examples.bat` on Windows. Commands run in the foreground; press **Ctrl+C** to stop the server.
The runner prepares `PATH` and native-library paths from the developer build or installed workspace. It launches
`command` executables directly, passing `arguments` followed by any extra arguments after `--`.

`test EXAMPLE_ID:NAME` selects a test configuration. Catalog `commands` includes a `run:NAME` entry for each named run.

### Requirements

| Field | Required | Description |
| --- | --- | --- |
| `system` | No | Unique capabilities from `cmake`, `c`, `cpp`, `gpu`, `python`, and `python_development`. |
| `python_packages` | No | Exact external Python requirements, such as `transitions==0.9.3`. |
| `cpp_packages` | No | Exact lowercase Pixi package pins, such as `fmt==7.0.3`. |
| `modules` | Yes | Non-empty array of direct Isaac Sim library distributions. |

Write `modules` as a list of inline records under `[requirements]`:

```toml
[requirements]
system = ["python"]
modules = [
    { name = "isaacsim_foundation", surfaces = ["python"] },
    { name = "isaacsim_ovsim", surfaces = ["native_sdk", "python"] },
]
```

Each record has a unique `name` matching `[a-z][a-z0-9_]*`. Its `surfaces` array contains `native_sdk`, `python`, or
both. These values select the distribution's development or Python files; runtime files are always installed. Declare
surfaces for every distribution, regardless of the example's `system` capabilities. You may also use the equivalent
`[[requirements.modules]]` syntax.

Declare only direct dependencies and capabilities:

- `cmake` builds require `native_sdk`, `cmake`, and at least one of `c` or `cpp`.
- Python runs require `python`.
- CMake-built Python examples also require `python_development`.
- `python_development` requires `python`.
- `python_packages` require a Python run adapter and `python`.
- `cpp_packages` require the CMake adapter and `cpp`.

`gpu` states that the example needs an NVIDIA GPU at run time. It is the one capability the developer environment
cannot install, so the runner asks the driver for the host's devices and reports the example's tests as skipped when
it finds none, rather than failing them. Declare it only when the example cannot produce a result without a GPU.

Pin each Python and C++ package to one exact version with `==`. Python requirements cannot use URLs, paths, editable
or version-control installs, markers, extras, ranges, wildcards, or pip options. If several examples use the same
package, their pins must agree. Tool and language versions come from
`source/libraries/system_requirements.toml`; do not repeat them in example manifests.

### Tests

Each `[[test]]` table defines a named configuration. It runs the normal entry point unless a Python example selects a
dedicated `test.py` with `path`.

| Field | Required | Default | Description |
| --- | --- | --- | --- |
| `name` | Yes | — | Unique name matching `[a-z][a-z0-9_]*`. |
| `path` | No | `[run].path` | Existing Python test script beneath the example root. |
| `arguments` | No | `run.arguments` | Replacement argument list; an explicit empty array passes no arguments. |
| `timeout_seconds` | No | `60` | Positive timeout before the runner terminates the process tree. |
| `expected_exit_code` | No | `0` | Required integer exit code. |
| `stdout_contains` | No | Empty | Unique output substrings that must appear on standard output. |
| `stderr_contains` | No | Empty | Unique output substrings that must appear on standard error. |
| `environment` | No | Empty | String-valued environment overlay for the test. |

Write an environment overlay as `[test.environment]` after its `[[test]]` table. Names must match
`[A-Za-z_][A-Za-z0-9_]*`, values must not contain secrets, and the overlay cannot override
`PYTHONDONTWRITEBYTECODE`. The runner reports an example with no test configuration as skipped, as it does every
test of an example that requires a `gpu` the host does not have.

### Series

A series is defined by `series/<name>/series.toml`. Use `kind = "sequence"` when examples have a meaningful order, or
`kind = "collection"` when they simply share a workflow or problem. Each `[[step]]` names an example in the same
series and assigns it a `beginner`, `intermediate`, or `expert` level. List every example under the series exactly
once.

## Run examples

In a source checkout, build the libraries first. Then use the platform launcher from the repository root. It selects
the matching developer environment, including third-party runtime packages such as `ovrtx`.

```bash
source/libraries/build.sh -r
source/examples/examples.sh list
source/examples/examples.sh build [EXAMPLE_ID ...]
source/examples/examples.sh run EXAMPLE_ID_OR_PATH[:COMMAND] -- [EXAMPLE_ARGUMENT ...]
source/examples/examples.sh run --no-build EXAMPLE_ID_OR_PATH[:COMMAND]
source/examples/examples.sh test [EXAMPLE_ID[:TEST_NAME] ...]
source/examples/examples.sh test --no-build [EXAMPLE_ID[:TEST_NAME] ...]
source/examples/examples.sh catalog --output /tmp/examples.json
```

Use `source\examples\examples.bat` with the same arguments on Windows. Use `--help` for CMake configuration and custom
build-root options.

An installed examples workspace contains `examples.py`, this README, the selected examples, and the shared system
requirements. Before using it:

1. Activate an environment containing `packaging`, the declared Python distributions, and required third-party
   runtimes.
2. For native examples, add the installed SDK prefix to `CMAKE_PREFIX_PATH`.
3. If an example links a package from the active environment, add that environment to `CMAKE_PREFIX_PATH` too.

Then run:

```bash
python examples.py build
python examples.py run hello_world.python
python examples.py test
```

`build` and normal `test` commands prepare all external packages declared by the selected examples. If an installed
workspace declares external packages, run `build` before `run` or `test --no-build`. Python packages use your pip
configuration. C++ packages require Pixi through `PIXI_EXE` or `PATH`. Keep custom build directories outside the
examples source tree.

The installer must resolve selected Isaac Sim and external dependencies together. A selected series includes all of
its steps. The source tree and installed workspace contain no generated build or dependency files.


## Development terminal frontend

Browse, build, run, and test examples from a terminal:

```bash
./source/examples/examples.sh tui
```

On Windows, use `source\examples\examples.bat tui`. The launcher prepares the
locked `examples-runner` Pixi environment and loads the frontend from source;
no manual pip installation or built SDK is needed to browse.

Options follow `tui`: use `--config Debug` for a Debug build or
`--source-root PATH` to browse another compatible checkout. The default is the
checkout containing the launcher.

- Click an example or use arrow keys to select it. Hovering leaves selection unchanged.
- Enter toggles a category or invokes the selected example's primary action.
- **Build (b)** prepares the SDK and dependencies if needed, then builds the example.
  **Run (r)** reuses current outputs; stale or missing builds require Build first.
- **Args (a)** edits extra run arguments: one exact argument per row, appended after
  the example defaults. **Test (t)** runs the selected example's declared tests.
- Output stays in the TUI. Escape closes a dialog, cancels active work, or quits
  when idle. `q` quits after cleanup.

Restart the TUI after editing frontend code or to reload the catalog. Runs do not
accept interactive stdin; use the CLI for examples that need terminal input.
This frontend is still under development.

The frontend is part of the `isaacsim-installer` package in `source/installer`,
with separate `core/`, `cli/`, and `tui/` folders. `_run.py` is the internal checkout launcher.
Build a wheel and run the backend, CLI, and TUI source tests with:

```bash
./source/installer/build.sh --test
```

Wheels are written to `_build/packages/installer/`; `--output-dir PATH` overrides
that location. Windows uses `source\installer\build.bat` with the same options.
Install the generated wheel with its `[tui]` extra, or use an editable install:

```bash
python -m pip install -e 'source/installer[tui]'
isaacsim-installer list
isaacsim-installer build hello_world.python
isaacsim-installer run hello_world.python
isaacsim-installer test hello_world.python
isaacsim-installer tui
```

Source and editable installs detect their containing checkout. Wheel installs
require `--source-root PATH` after the command, before example IDs or runner
arguments. CLI commands use the same checkout dispatcher as `examples.sh` and
`examples.bat`, including its Pixi environment and normal build/run/test behavior. Install the
base package without `[tui]` for these CLI commands; only the TUI needs Textual.
Explicit `--source-root` always overrides checkout detection.
Command options are forwarded; for global runner options such as `--build-root`,
use `examples.sh` or `examples.bat` directly.

These tests exercise UI behavior and subprocess handling without launching Isaac Sim.
