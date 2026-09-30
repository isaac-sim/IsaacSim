<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Isaac Sim examples

These focused examples consume public, installed Isaac Sim library APIs. They do not import implementation packages or
add library source directories to build or import paths. Windows examples require Windows 11.

## Layout and authoring

- `libraries/` contains focused examples of one public library API.
- `series/<name>/` contains independently runnable examples grouped by a `series.toml` file.
- Every example has a concise `README.md` and an `example.toml` manifest.

Keep each normal entry point limited to the workflow it demonstrates. Prefer `isaacsim.foundation` and other public
library APIs over direct USD authoring when they provide the needed operation. Put test-only setup and assertions in
`test.py`, then select it with `[[test]].path`. Keep extended walkthroughs in the tutorial documentation.

The runner recursively discovers `example.toml` files. `published` defaults to `true`; set it to `false` to exclude an
incomplete or private example. Stable IDs do not depend on directory paths. The parser rejects unknown or missing
fields, invalid types, duplicate unique values, unsafe paths, and incoherent adapter requirements.

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
| `run` | Yes | Single normal entry-point table. |
| `requirements` | Yes | Direct library, package, and system requirements. |
| `test` | No | Array of named test configurations. |

Manifests have no version field. Examples, the runner, the installer, and the libraries ship as one release.

### Build and run adapters

| Table | Field | Required | Description |
| --- | --- | --- | --- |
| `[build]` | `adapter` | Yes | `none` or `cmake`. |
| `[build]` | `targets` | For `cmake` | Non-empty, unique targets matching `[A-Za-z0-9_][A-Za-z0-9_.+-]*`. |
| `[run]` | `adapter` | Yes | `executable` or `python`. |
| `[run]` | `target` | For `executable` | Target listed in `build.targets`. |
| `[run]` | `path` | For `python` | Existing script beneath the example root, expressed as a relative POSIX path. |
| `[run]` | `arguments` | No | Ordered default arguments; empty and duplicate strings are allowed. |

The `none` build adapter accepts no other fields. The `cmake` adapter requires `CMakeLists.txt`. An executable requires
the CMake adapter. A Python-only example normally combines `build.adapter = "none"` with `run.adapter = "python"`.

### Requirements

| Field | Required | Description |
| --- | --- | --- |
| `system` | No | Unique capabilities from `cmake`, `c`, `cpp`, `python`, and `python_development`. |
| `python_packages` | No | Exact external Python requirements, such as `transitions==0.9.3`. |
| `cpp_packages` | No | Exact lowercase Pixi package pins, such as `fmt==7.0.3`. |
| `modules` | Yes | Non-empty array of direct Isaac Sim library distributions. |

Each `[[requirements.modules]]` entry has a unique `name` matching `[a-z][a-z0-9_]*` and a non-empty, unique
`surfaces` array containing `native_sdk`, `python`, or both. Declare only direct dependencies and capabilities:

- `cmake` builds require `native_sdk`, `cmake`, and at least one of `c` or `cpp`.
- Python runs require `python`.
- CMake-built Python examples also require `python_development`.
- `python_development` requires `python`.
- `python_packages` require a Python run adapter and `python`.
- `cpp_packages` require the CMake adapter and `cpp`.

Python package entries must use one exact, non-wildcard `==` version. URLs, paths, editable or VCS requirements,
markers, extras, ranges, and pip options are unsupported. C++ package entries also require one exact `==` version.
Repeated package pins must agree across the example collection. Tool and language versions come from
`source/libraries/system_requirements.toml`; do not repeat them in a manifest.

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
`PYTHONDONTWRITEBYTECODE`. The runner reports an example with no test configuration as skipped.

### Series

A series lives at `series/<name>/series.toml`. The manifest contains `id`, `title`, `summary`, a `kind` of `sequence` or
`collection`, and one or more `[[step]]` tables. Use a sequence when the examples have a meaningful execution order;
use a collection when they share only a workflow or problem domain. Each entry names a co-located `example` ID and
assigns a `level` of `beginner`, `intermediate`, or `expert`. The manifest must list every example beneath the series
root exactly once.

## Run examples

In a source checkout, first build the libraries for the desired configuration. Then run the platform launcher from the
repository root; it selects the matching developer environment automatically.

```bash
source/libraries/build.sh -r
source/examples/example.sh list
source/examples/example.sh build [EXAMPLE_ID ...]
source/examples/example.sh run EXAMPLE_ID_OR_PATH -- [EXAMPLE_ARGUMENT ...]
source/examples/example.sh test [EXAMPLE_ID[:TEST_NAME] ...]
source/examples/example.sh test --no-build [EXAMPLE_ID[:TEST_NAME] ...]
source/examples/example.sh catalog --output /tmp/examples.json
```

Use `source\examples\example.bat` with the same arguments on Windows. Use `--help` for CMake configuration and custom
build-root options.

An installed examples workspace contains `examples.py`, this README, the selected examples, and the release-wide
system requirements. Activate an environment containing `packaging` and the declared Python distributions, set
`CMAKE_PREFIX_PATH` to the installed SDK prefix for native examples, and invoke the runner directly:

```bash
python examples.py build
python examples.py run hello_world.python
python examples.py test
```

`build` and normal `test` calls resolve the collection-wide union of declared external packages. Run `build` before
`run` or `test --no-build` when the payload declares external packages. Python packages use the caller's pip
configuration; C++ packages require Pixi through `PIXI_EXE` or `PATH`. A custom build root must remain outside the
examples source tree.

The installer must resolve selected Isaac Sim and external dependencies together. A selected series includes all of
its steps. The source tree and installed workspace contain no generated build or dependency files.
