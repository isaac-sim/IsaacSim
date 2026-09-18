# Library packaging

Python wheels and native archives use separate, standards-appropriate workflows.
Terminology in this document follows the canonical definitions of
[carrier, adapter, and wrapper](../README.md#terminology).

## Python wheels

Every independently shippable distribution owns a `pyproject.toml` at its package root. Package discovery supports
the normal two-component layout, such as `isaacsim/common/pyproject.toml`, and named flat roots, such as
`isaacsim_deprecated/pyproject.toml`. That file is authoritative for the Python project name, dynamic version,
supported Python version, runtime dependencies, and PEP 517 backend. CMake remains authoritative for the wheel payload.
Build requirements and runtime requirements are different contracts: `[build-system].requires` advertises the backend
versions that can build the project from source, while `[project].dependencies` becomes public `Requires-Dist` wheel
metadata. Internal `source/libraries` dependency names are listed in the `isaacsim_libraries_metadata` dynamic provider,
which appends exact requirements derived from the shared `VERSION`; external requirements remain static and may use
supported ranges. Concrete tools selected for an Isaac Sim release do not replace either published contract.
Authored Python project and requirement names use canonical PEP 508 hyphens, such as `isaacsim-common`; CMake group
selectors retain their underscore form, such as `--group isaacsim_common`.

Build the libraries and assemble their wheels:

```text
cd source/libraries
./build.sh -r --wheel
```

Select distributions by repeating `--group` and override the output directory when needed:

```text
./build.sh -r --wheel --group isaacsim_common --output-dir ../../_cmake_build/my-wheelhouse
```

Exercise the supported lower bounds with separate environments and build trees:

```text
./build.sh -r --test --wheel --dependency-profile minimum
```

`locked` is the default and uses the concrete versions selected for ordinary release production. `minimum` pins
direct build/test tools to their declared lower bounds. Newer versions are evaluated only by updating and reviewing
`pixi.lock`; ordinary builds never perform a floating solve. Test and wheel subprocesses isolate `PYTHONPATH` to the
selected profile. The selected versions affect artifact production and validation, not wheel runtime metadata.

Without `--group`, the tool assembles every discovered package whose generated manifest declares at least one Python
import. It skips C- and C++-only packages. Explicitly selecting a native-only package is an error rather than an empty
wheel.

The standalone command configures and builds the complete module graph in
`_cmake_build/isaacsim-libraries-release`. Wheel assembly validates the CMake source directory, configuration, Python
executable, and required package options before installing anything. Rerun the matching standalone build after
changing compiled module sources or CMake configuration.

For each package, the wheel tool creates a unique temporary directory under
`_cmake_build/isaacsim-libraries-wheel-stage`, installs only the already-built `<group>-python` CMake component, and
invokes `python -m build --wheel` with scikit-build-core's CMake integration disabled. Scikit-build-core owns Python
core metadata, wheel tags, `WHEEL`, `RECORD`, archive layout, and future wheel-specification changes; it never
configures or compiles CMake. The tool validates the completed wheel and atomically publishes it to the output
directory. Temporary installation and backend output directories are removed after assembly, including after a
failure. Separate packages can be assembled concurrently.

Schema projects may declare named wheel variants in `[tool.isaacsim-library]`. Their standalone default installs the
matching `<group>-python-<variant>` component and receives a PEP 427 build tag such as `1usd2505`; carrier manifests
select their required variant explicitly, such as `usd2511`. Schema wheels remain `py3-none-any` because their payload
contains Python and USD data rather than native code. Keep variants in separate wheelhouses for general pip consumers:
wheel build tags distinguish artifacts but do not express runtime compatibility to pip's resolver. Carrier staging
selects the exact version, build tag, and pure-wheel tag before exposing a one-wheel local index to pip.

Validation checks the distribution name and version, exact internal package requirements, registered imports,
`py.typed` markers, implicit namespace layout, and the installed package manifest. Requirement names and specifiers
are normalized with the standard `packaging` parser. Internal wheel requirements must exist in the native package
graph and must be unconditional exact shared-version pins. A native-only dependency may be omitted from wheel
metadata. Source tests, tool caches, bytecode, and symlinks are also rejected. Additional external Python requirements
and their supported ranges remain owned by `pyproject.toml`.

Use the wheel tool instead of invoking `python -m build` directly. The frontend needs the private CMake component stage
that the tool supplies through backend configuration; `pyproject.toml` intentionally contains no source-tree or build-
tree payload paths.

For wheels containing native code, the backend-generated platform tag describes the build host. Producing a wheel for
a release portability policy, such as manylinux, is a separate CI repair and validation step performed with the
platform's standard wheel tooling; do not change the filename or metadata tag manually.

Validate a wheel by installing it into a clean environment without the source or build tree on `PYTHONPATH`, importing
each module, querying its version through `importlib.metadata`, and checking its installed files. Source tests, caches,
and symlinks must not be present.

The command writes wheels to `_cmake_build/isaacsim-libraries-artifacts/<configuration>` by default. Backend scratch
trees, CMake install stages, and generated artifacts are not part of a package's source layout. Do not commit them.

Run the packaging unit tests without a compiled library build:

```text
PYTHONPATH=<python-test-deps> <library-python> -m unittest discover -s packaging/tests -v
```

Here, `<library-python>` is the locked interpreter in `.pixi/envs/build-driver`, and `<python-test-deps>` is the
site-packages directory in the selected locked or minimum Pixi test environment.

## Kit carrier artifacts

Use a carrier build when a Kit extension needs both the Python wheel and compile-time native SDK for one distribution:

```text
./build.sh -r --test --carrier isaacsim.common
```

The carrier build configures the complete native module graph, then creates a standards-based wheel and installs only
the selected distribution's native runtime and development CMake components into the carrier stage. A carrier may
declare a disjoint Python-module projection when multiple Kit extensions reuse one complete wheel; only those leaf
packages enter that extension's `pip_prebundle`. It validates the wheel and SDK's shared package manifests. SDK
archives are produced only by the explicit native archive workflow below.
See [Kit carrier extensions](../README.md#kit-carrier-extensions) for the carrier layout, root-build integration,
dependency ownership, and validation contract.

## Native archives

`package.py` creates native artifacts from CMake install components. It uses only the Python standard library and the
CMake executable that configured the build.

- `runtime` creates a platform-specific `.tar.gz` with shared libraries, registered runtime data, package metadata,
  the repository release version, and per-module native ABI metadata.
- `sdk` combines the runtime and development components, adding public headers, import libraries, and the relocatable
  CMake package.

Build the native-only profile before packaging:

```text
./build.sh -r --profile cpp-library
python packaging/package.py runtime \
    --build-dir ../../_cmake_build/isaacsim-libraries-cpp-library \
    --output-dir ../../_cmake_build/isaacsim-libraries-artifacts/release --group isaacsim_common
python packaging/package.py sdk \
    --build-dir ../../_cmake_build/isaacsim-libraries-cpp-library \
    --output-dir ../../_cmake_build/isaacsim-libraries-artifacts/release --group isaacsim_common
```

`--build-dir`, `--output-dir`, and `--group` are required. Use `--config` with a multi-configuration generator and
`--cmake` only when the configuring CMake executable is not on `PATH`. The tool recreates a private `.staging`
directory below the output directory and removes it before returning.

The tool reads the artifact version from the installed package manifest. It has no version override, so artifact names
cannot diverge from `source/libraries/VERSION`. Artifact names are
`<group>-<version>-<platform>-runtime.tar.gz` or `<group>-<version>-<platform>-sdk.tar.gz`. Native archives are ordinary
install prefixes; validate an SDK by extracting it and using `find_package(<group> CONFIG REQUIRED)` from a separate
consumer project. An archive contains only the selected distribution. For a package with exact internal dependencies,
extract the dependency runtime or SDK archives into the same prefix, or provide their prefixes through
`CMAKE_PREFIX_PATH`; the generated package configuration discovers and version-checks them rather than copying them
into the dependent archive. Archive creation also verifies that every internal manifest dependency requires the
package's exact shared version.
