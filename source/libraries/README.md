# Isaac Sim libraries

`source/libraries` is the home for Isaac Sim code that must work without Kit. It contains C and C++ libraries, pure
Python modules, and optional Python bindings built with nanobind. Use this guide when you add, build, test, package, or
deliver one of these modules.

Public APIs must not expose Carbonite types or headers. Most modules use the shared
`isaacsim.common.logging` API instead of including Carbonite. Infrastructure modules may link Carbonite privately and
statically, but the resulting binary must not depend on Carbonite dynamically or export Carbonite symbols.

## Terminology

This document uses the following terms for different responsibilities:

| Term | Meaning |
|---|---|
| **Host adapter** | Code that connects a module to services or lifecycle events owned by the host application. For example, a host adapter can connect a logging backend or handle startup and shutdown. |
| **Compatibility wrapper (shim)** | An optional API layer that preserves or translates an older API during a migration. This is one specific kind of wrapper; the codebase also uses *wrapper* for ordinary object and binding wrappers. Simply packaging an unchanged module does not create a compatibility wrapper. |

A Kit extension can stage a library wheel and also contain a native host adapter. Add a compatibility wrapper only
when the public API differs from the underlying module API.

## Structure

```text
source/libraries/
├── build.sh                               # Standalone Linux dependency, CMake, test, and wheel entry point.
├── build.bat                              # Standalone Windows entry point with the same arguments.
├── CMakeLists.txt                         # Standalone project and explicit distribution-package inventory.
├── CMakePresets.json                     # Supported local build configurations.
├── README.md                              # Module structure and build contract.
├── VERSION                                # One canonical version shared by every distribution package.
├── package_manifest.py                    # Shared generated-package manifest validation.
├── system_requirements.toml               # Authoritative C, C++, Python, and CMake version contract.
├── cmake/
│   ├── README.md                          # Public helper reference and generated-name contract.
│   ├── IsaacSimModule.cmake               # Public entry point for build helpers.
│   ├── IsaacSimModuleCore.cmake           # C++ targets, dependencies, installation, and source boundary.
│   ├── IsaacSimPackage.cmake              # Distribution registration, metadata, and install components.
│   ├── IsaacSimPackagingFilters.cmake      # Shared exclusions for generated caches and Python bytecode.
│   ├── IsaacSimPython.cmake                # Python package staging, installation, and registration.
│   ├── IsaacSimUsdSchema.cmake             # Codeless USD schema generation and packaging.
│   ├── IsaacSimNanobind.cmake             # Bindings, runtime closure, and loader paths.
│   ├── IsaacSimTesting.cmake              # Unit, ABI, binary-boundary, and install-contract tests.
│   ├── IsaacSimDocumentationCatalog.cmake # Generates the public documentation inventory.
│   ├── IsaacSimSystemRequirements.cmake   # Loads the shared language and tool version contract.
│   ├── FindCarbonite.cmake                 # Demand-driven pinned static Carbonite discovery.
│   ├── IsaacSimPackageConfig.cmake.in     # Per-package installed CMake configuration.
│   ├── IsaacSimPackageConfigVersion.cmake.in # Per-package CMake version policy.
│   ├── IsaacSimPackageManifest.json.in    # Installed package identity and membership manifest.
│   ├── CheckBinaryDependencies.cmake      # Rejects dynamic Kit and Carbonite dependencies.
│   ├── CheckExportedSymbols.cmake         # Checks an explicitly stable C ABI.
│   ├── CopyPythonPackage.cmake            # Copies package trees without generated cache files.
│   ├── CopyRuntimeDirectory.cmake         # Copies runtime data without transient Python files.
│   ├── initialize_nanobind_runtime.py      # Initializes installed binding runtime search paths.
│   ├── run_nanobind_stubgen.py             # Enables Windows binding runtime paths during stub generation.
│   ├── run_usd_gen_schema.py               # Runs USD schema generation in the locked environment.
│   ├── StageEditableDirectory.cmake       # Links editable Python trees with a portable copy fallback.
│   ├── RunInstallContractTest.cmake       # Tests relocated C, C++, and Python installations.
│   └── tests/                             # Generic CMake helper contract tests.
├── ovsim/interfaces/
│   ├── cpp/                               # Canonical OV SIM C++ function-pointer type aliases.
│   └── python/                            # Source-only OV SIM Python callable type aliases.
├── packaging/
│   ├── README.md                          # Native archive and Python wheel workflow.
│   ├── isaacsim_libraries_metadata.py     # Exact shared-version Python dependency provider.
│   ├── package.py                         # Standard-library native archive builder.
│   ├── wheel.py                           # Packaging-only PEP 517 wheel orchestrator.
│   └── tests/                             # Unit tests for packaging orchestration.
├── tools/
│   ├── build.py                           # Cross-platform standalone build orchestration.
│   ├── prepare_root_build.py              # Discovers wheel-staging manifests for the root application build.
│   ├── stage_extension_wheel.py           # Installs and validates a wheel and SDK for a Kit extension.
│   ├── update_version.py                  # Checks or updates the lockstep library version.
│   └── tests/                             # Host, root-integration, artifact, and build-layout contract tests.
├── testing/                               # Shared test infrastructure; not module test suites.
│   ├── README.md                          # Purpose and ownership of every shared test file.
│   ├── cpp/                               # Shared compiled-test support.
│   ├── examples/                          # Repository examples integration-test driver.
│   ├── install_contract/                  # Generic downstream C, C++, and Python consumer templates.
│   └── python/                            # Shared pytest configuration and test utilities.
├── licenses/                              # Authored notices for redistributed native dependencies.
├── isaacsim_deprecated/                   # Flat compatibility distribution with an explicit module prefix.
├── isaacsim_usd_schemas/<schema>/          # USD schema distributions with explicit module prefixes.
│   ├── CMakeLists.txt                      # Registers schema inputs, import roots, and public headers.
│   └── pyproject.toml                      # Authoritative schema wheel metadata.
└── isaacsim/<distribution>/                # Derives the isaacsim_<distribution> package name.
    ├── CMakeLists.txt                     # Registers the package, then its modules in dependency order.
    ├── pyproject.toml                     # Authoritative Python distribution and wheel build metadata.
    ├── docs/                              # Optional package-level documentation rooted at index.rst.
    └── <module-suffix>/                   # Full path derives isaacsim.<distribution>.<module-suffix>.
        ├── CMakeLists.txt
        ├── CHANGELOG.md                   # Required history for this module's public APIs.
        ├── include/                       # Installed public headers under their full namespace path.
        │   └── .../details/               # Public inline/template implementation details when required.
        ├── src/                           # C/C++ implementation and private headers.
        │   └── details/                   # Private detail-namespace headers and implementations.
        ├── bindings/python/               # Optional nanobind definition and binding-only headers.
        ├── python/                        # Optional leaf package content.
        │   ├── __init__.py                # Fixed facade re-exporting impl.__all__.
        │   ├── impl/                      # Public API inventory and Python implementation.
        │   └── py.typed                   # Inline typing marker.
        ├── tests/                         # Optional tests, organized strictly by implementation language.
        │   ├── c/                         # C API doctest cases, compiled as C++.
        │   ├── cpp/                       # C++ API doctest cases.
        │   └── python/                    # pytest and Hypothesis tests.
        ├── resources/                     # Optional explicitly registered test or runtime data.
        ├── abi/                           # Optional stable C symbol baseline.
        └── docs/                          # Optional module API documentation rooted at index.rst.
```

A directory is not built or installed until its distribution registers it with a CMake helper. Choose the helper that
matches the module:

- **Compiled:** `isaacsim_add_module` creates a shared library from C11, C++17, or both.
- **Header-only:** `isaacsim_add_header_module` exports C or C++ headers and their dependencies without creating a
  binary.
- **Pure Python:** `isaacsim_add_python_module` installs a typed Python leaf package without a compiled target.
- **Compatibility Python:** `isaacsim_add_compat_python_module` keeps the module's source identity but installs it at
  an explicitly declared legacy import path.
- **USD schema:** `isaacsim_add_usd_schema_module` generates codeless schema resources, typed Python imports, and
  optional public C++ headers.
- **Native with Python bindings:** Register the native module with `isaacsim_add_module` or
  `isaacsim_add_header_module`, then add its private Python binding with `isaacsim_add_nanobind`.

For compatibility migrations, pass `PRESERVE_PACKAGE_LAYOUT` to `isaacsim_add_python_module` only when you must keep
an existing leaf package unchanged. New modules use the standard `impl/` layout.

Pass `REQUIRES_BINDINGS` to `isaacsim_add_python_module` or `isaacsim_add_compat_python_module` only when the module
cannot be imported without a nanobind extension. Install tests without bindings skip those imports but still validate
the distribution and all of its dependencies. This module option is separate from the Python test option with the
same name.

A *distribution package* is one unit that can be installed and shipped independently. Its CMake name usually comes
from its two-part source path: for example, `isaacsim/common` becomes `isaacsim_common`. A flat distribution directory
is also supported when it declares `MODULE_PREFIX`; `isaacsim_deprecated`, for example, declares
`MODULE_PREFIX isaacsim.deprecated`.

The CMake API uses the older term *group* for a distribution package, so helper names and placeholders still use
`group`. One package can contain several C, C++, header-only, pure Python, or Python-bound modules. For example,
`isaacsim/common/logging` defines the logical module `isaacsim.common.logging`. A module does not have to expose a
Python API.

Each module has its own changelog, but all packages and modules use the version in `source/libraries/VERSION`. The
native runtime, development files, Python files, and documentation are installable parts of the same package, not
separately versioned packages.

Use underscores in CMake group, component, manifest, and native archive names. Use canonical hyphenated names, such as
`isaacsim-common`, in Python project metadata and requirements. Use dotted names for logical modules and imports.

Register a group before its modules:

```cmake
isaacsim_add_group(
    NAME isaacsim_common
    MODULES
        isaacsim.common.array
        isaacsim.common.logging
        isaacsim.common.profiling
)

add_subdirectory(array)
add_subdirectory(logging)
add_subdirectory(profiling)
```

For a normal two-part root, `isaacsim/<distribution>` determines the underscore-separated distribution name. The full
module path determines the dotted logical module name. For a flat root, the directory supplies the distribution name,
`MODULE_PREFIX` supplies the dotted prefix, and the path below the root supplies the rest. You still declare CMake
names explicitly; configuration fails if a name does not match its path. A compatibility Python module keeps this
path-based logical name and declares its legacy import separately:

| Source path relative to `source/libraries` | CMake group | Logical module | Python import |
|---|---|---|---|
| `isaacsim/common/logging` | `isaacsim_common` | `isaacsim.common.logging` | `isaacsim.common.logging` |
| `isaacsim/common/profiling` | `isaacsim_common` | `isaacsim.common.profiling` | `isaacsim.common.profiling` |
| `isaacsim/navigation/route_planner` | `isaacsim_navigation` | `isaacsim.navigation.route_planner` | `isaacsim.navigation.route_planner` |
| `isaacsim/robot/kinematics/inverse` | `isaacsim_robot` | `isaacsim.robot.kinematics.inverse` | `isaacsim.robot.kinematics.inverse` |
| `isaacsim_deprecated/core/experimental/objects` | `isaacsim_deprecated` | `isaacsim.deprecated.core.experimental.objects` | `isaacsim.core.experimental.objects` |

USD schema distributions under `isaacsim_usd_schemas` follow the package boundaries of `newton-usd-schemas` and
`physx-usd-schemas`. They declare `MODULE_PREFIX` because their source layout, distribution names, and public import
names intentionally differ. Ordinary modules must still follow the source-path naming rules.

Schema generation supports two separate compatibility variants. Standalone builds generate `usd2505` resources. A
Kit extension can request `usd2511`, which uses the matching OpenUSD dependency in `_build/target-deps/usd`. Their pure
Python wheels use the build tags `1usd2505` and `1usd2511`. Wheel staging requires an exact tag instead of relying on
pip to choose a variant.

A module must live below its distribution root; the root itself cannot be a module. Intermediate namespace
directories do not need a `CMakeLists.txt`. For example, the distribution can call
`add_subdirectory(kinematics/inverse)` directly. Module roots cannot overlap: if `isaacsim.robot.kinematics` is a
module, `isaacsim.robot.kinematics.inverse` cannot also be one. The group-level `docs` directory is reserved for
package documentation.

Python packages use implicit namespaces above the module root, as defined by PEP 420. For example, the
`isaacsim.common.logging` wheel includes `isaacsim/common/logging/__init__.py`, but not `isaacsim/__init__.py` or
`isaacsim/common/__init__.py`. This allows independently installed distributions to share the `isaacsim` namespace.
The distribution root is packaging structure, not an importable API. Compatibility distributions may declare legacy
imports outside their logical namespace, but each import must still have exactly one owner.

After registering a group, declare each dependency on another package with `isaacsim_add_group_dependency`. Register
dependency groups first. All dependencies use the shared version in `source/libraries/VERSION`; you cannot provide a
different version.

The `MODULES` list must exactly match the modules registered for the package. CMake generates package metadata and
install tests only when the list matches and all dependencies can be packaged. To make an incomplete package a
configuration error, set `ISAACSIM_PACKAGE_GROUP=<name>` or `ISAACSIM_ENFORCE_COMPLETE_PACKAGES=ON`.

The package's `pyproject.toml` defines its Python metadata and runtime dependencies. List supported version ranges for
external dependencies in `[project.dependencies]`. For dependencies on other `source/libraries` packages, use the
dynamic metadata provider so every package uses the shared version:

```toml
[project]
dynamic = ["version", "dependencies"]

[[tool.dynamic-metadata]]
provider = { path = "../../packaging", module = "isaacsim_libraries_metadata" }
field = "dependencies"
names = ["isaacsim-common"]
```

The provider reads `source/libraries/VERSION` and adds exact requirements such as
`isaacsim-common==7.0.0a2` to the wheel metadata. Native package dependencies do not automatically become wheel
requirements. If a wheel declares an internal dependency, validation confirms that the dependency exists in the
native package graph and uses the shared version.

CMake does not create or edit Python project metadata. The standalone build compiles the full module graph. Wheel
assembly installs only the selected package's `<group>-python` component, then packages those files without running
CMake again.

Create only the directories a group or module uses. The build does not discover source directories automatically.
The root project adds each distribution with `add_subdirectory`, and each distribution adds its module paths.

A group may register a module conditionally when all required external targets are available. If only some required
targets are available, stop with a configuration error instead of building a reduced module. If an optional target set
is completely unavailable, report that in the configuration output.

Generated files are not source files. Standard build trees live under
`<repo>/_cmake_build/isaacsim-libraries-<configuration>`. Release trees for the other profiles live under
`<repo>/_cmake_build/isaacsim-libraries-<profile>`; Debug appends `-debug`. Populated dependencies live under
`<repo>/_cmake_build/isaacsim-libraries-target-deps`, and default wheel and native artifacts live under
`<repo>/_cmake_build/isaacsim-libraries-artifacts/<configuration>` when the documented packaging commands are used.
Wheels prepared for Kit and their validated extension stages live under
`<repo>/_cmake_build/extension-wheel-artifacts/<configuration>` and
`<repo>/_cmake_build/extension-wheel-stages/<configuration>/<extension>`. Keep the editable Python stage, generated
export headers, nanobind binaries, stubs, package manifests, and installation test files inside a build tree. Never
treat tool-created caches, bytecode, or packaging staging directories as source structure or include them in an
installed package.

After successful default extension-wheel assembly and staging, the historical
`module-carrier-artifacts/<configuration>` and `module-carriers/<configuration>/<extension>` paths under
`_cmake_build` link to those same respective outputs. An explicit `--output-dir` remains caller-owned and does not
update the historical artifact path. Generate-only prepares link-target directories and preserves any existing
ordinary legacy stage directory and its files until that extension is staged successfully.

Library dependencies are separate from Kit application dependencies. Private and patched native dependencies belong
in `deps/isaacsim-libraries.packman.xml` and are installed under
`_cmake_build/isaacsim-libraries-target-deps`. Public native archives, including the OVPhysX SDK, use Pixi recipes
pinned by checksum.

The root `pixi.toml` and `pixi.lock` define all public build and Python packages. Exact versions of Python, CMake,
Ninja, DLPack, doctest, fmt, nanobind, SDL, stb, and tsl-robin-map are recorded in `pixi.lock`. Pixi uses conda-forge
binaries when they are compatible with the standalone modules and builds selected packages from checksum-pinned
official archives where needed. CMake uses the installed files and never downloads dependencies itself.

DLPack headers are installed with their development component so the SDK is self-contained. Locked and minimum
dependency environments live under `.pixi/envs`. Packages come from public PyPI or the NVIDIA public index when
available; use NVIDIA's internal PyPI only for a pinned version that is unavailable from those sources.

Repository builds prepare extensions with `wheel-staging.toml` through `tools/repoman/repoman.py` before Repo
generation and staging. Internal Linux builds wait for the linbuild relaunch so staged binaries use the application
toolchain. Windows and `--no-docker` builds prepare them directly.
The standalone launcher collects licenses for redistributed packages from the locked `native-runtime` environment.
It stores the combined license set under `_cmake_build/PACKAGE-LICENSES/<key>`, and CMake copies it into each build
profile. Runtimes, SDKs, Python wheels, and extension wheel stages install the appropriate copy under
`share/licenses/<distribution>`.

A successful full `standard` build with locked dependencies also creates a `developer-environment` in its build tree.
It contains the runtime, development, and Python components of every complete package. Source examples use this
environment directly, so they do not install or replace dependencies. Concurrent examples hold a shared read lock;
cleanup or replacement waits for them to finish.

Do not add library-only dependencies to `deps/isaac-sim.packman.xml` or Kit Python prebundles.

Declare dependencies as close as possible to the code that uses them:

- Prefer an exact public conda package on each supported platform. Use a checksum-pinned Pixi source recipe only when
  a platform package is unavailable or conflicts with a runtime provided by the host. Do not add a CMake download
  fallback.
- Add or update private, patched, and binding packages in `deps/isaacsim-libraries.packman.xml`.
- Add or update external Python runtime, native-support, build, and test dependencies in root `pixi.toml`, then
  regenerate and review `pixi.lock`. Keep non-public package indexes scoped to the individual dependency.
- Declare target-level build dependencies in the consuming module's `CMakeLists.txt`. Use `PACKAGE_DEPENDENCIES` when
  consumers must find a package before loading an installed exported target.
- Declare dependencies on another distribution with `isaacsim_add_group_dependency`. The owning package must declare
  a dependency before one of its targets can link to that package. If the wheel also needs the package, add its name
  to the `isaacsim_libraries_metadata` provider in `pyproject.toml`.
- Declare third-party Python runtime requirements and their supported ranges in `[project.dependencies]`. Concrete
  build/test tool selections remain private build-environment inputs and do not become wheel runtime requirements.

Carbonite is a private implementation dependency with extra restrictions. Public APIs and package dependencies must
not expose its types or headers. A host-integration header may accept an opaque native handle, allowing a host adapter
to use a host-owned backend without making feature consumers include or link Carbonite.

`isaacsim.common.logging` and `isaacsim.common.profiling` contain the shared Carbonite implementation. Feature modules
link to these public APIs instead of compiling their own copy. An infrastructure module may link Carbonite privately
only when CMake identifies it as a static library, and the result must still pass binary-dependency and exported-symbol
tests. Call `find_package(Carbonite REQUIRED MODULE)` only in modules that use it.

The project requires CMake 3.26 or newer and Ninja. The standalone entry point installs and always uses the locked
Pixi CMake 4.3.2 and Ninja 1.13.2 packages. Linux host prerequisites are standard shell/archive utilities,
CA certificates, and either `curl` or `wget` for Packman bootstrap downloads. On
Windows 11, install Visual Studio 2022 and the Windows SDK. Windows 10 is not supported. The build reuses an initialized
developer environment or locates Visual Studio with `vswhere`. NVIDIA CI may provision its non-redistributable Windows
compiler separately through Packman; it is not part of the public dependency manifest.

See [cmake/README.md](cmake/README.md) for the complete public helper reference and generated-name rules.

### Vendored source notices

The gRPC protocol module includes the following third-party source file:

| Component | Source file | Copyright | License |
| --- | --- | --- | --- |
| Google APIs RPC status protocol | [`google/rpc/status.proto`](isaacsim/ovsim/protos/grpc/src/grpc/google/rpc/status.proto) | Copyright 2026 Google LLC | [Apache-2.0](licenses/googleapis-LICENSE.txt) |

The file retains its original Google copyright and license header. It matches
the [upstream Google APIs file at revision `ebd1d23ac613b177828dad42ad8dfb13ba498279`](https://github.com/googleapis/googleapis/blob/ebd1d23ac613b177828dad42ad8dfb13ba498279/google/rpc/status.proto)
except for three removed blank lines. The accompanying license text is copied unchanged from that revision.

This entry covers the vendored protocol source, not the gRPC and Protobuf libraries downloaded for a build.
Built library distributions carry the dependency license aggregate described above.

## Compiled module

```cmake
isaacsim_add_module(
    NAME isaacsim.common.logging
    EXPORT_HEADER isaacsim/common/logging/Export.h
    EXPORT_MACRO ISAACSIM_COMMON_LOGGING_API
    PUBLIC_C_HEADERS
        isaacsim/common/logging/Logging.h
    PUBLIC_CPP_HEADERS
        isaacsim/common/logging/Logging.hpp
    PUBLIC_C_SYMBOLS_FILE abi/c-api.symbols
    SOURCES
        src/Logging.cpp
)
```

`NAME` must match the module path relative to `source/libraries`, with slashes replaced by dots. CMake uses this name to
derive the native and exported targets, Python path, test name, and output directory. For example,
`isaacsim.common.logging` exports `isaacsim::common-logging`, while `isaacsim.navigation.route_planner` exports
`isaacsim::navigation-route_planner`.

Each compiled module is a C11 or C++17 shared library, or both. It has hidden symbol visibility, a generated export
header, package release metadata, and portable runtime library paths. On platforms with shared-library versioning, the
runtime includes the fully versioned file and an unversioned link, such as `libisaacsim-common-logging.so.7.0.0` and
`libisaacsim-common-logging.so`. This version identifies the library within its distribution; the module is not
released independently. The CMake package installs both public and generated export headers.

```cmake
find_package(isaacsim_common CONFIG REQUIRED)
target_link_libraries(application PRIVATE isaacsim::common-logging)
```

Use `.h` for C interfaces and `.hpp` for C++ interfaces. Use `.c` for C implementation sources and `.cpp` for C++
implementation sources. Name C and C++ headers and sources with `PascalCase` file stems. Public header extensions and
the repository-wide PEP 440 `VERSION` file are checked at configure time. A module may expose a stable C API, a C++
API, or both; C++ sources and consumers require C++17.

Keep only public, installed headers under `include/`. Put private headers under `src/` and nanobind-only adapters under
`bindings/python/`. You may use a build-only interface target to share binding code between modules, but do not export
it as part of the public C++ package. `PUBLIC_C_HEADERS` and `PUBLIC_CPP_HEADERS` must list every file under
`include/`. Only those files and the generated export header are installed.

Use target dependencies rather than ambient include or library directories:

```cmake
PUBLIC_DEPENDENCIES isaacsim::other-module ThirdParty::Core
PRIVATE_DEPENDENCIES ThirdParty::Implementation
PACKAGE_DEPENDENCIES ThirdParty
```

`PACKAGE_DEPENDENCIES` lists external packages that consumers must find before loading exported targets. CMake also
uses internal module dependencies to find all shared libraries needed by Python bindings. Declare dependencies on
other Isaac Sim distributions at group level instead of repeating them in `PACKAGE_DEPENDENCIES`.

For example, a module in another distribution that logs through `isaacsim.common.logging` declares both the package
dependency and target link:

```cmake
isaacsim_add_group(
    NAME isaacsim_feature
    MODULES isaacsim.feature.runtime
)

isaacsim_add_group_dependency(
    GROUP isaacsim_feature
    DEPENDS_ON isaacsim_common
)

isaacsim_add_module(
    NAME isaacsim.feature.runtime
    # Other required arguments omitted.
    PRIVATE_DEPENDENCIES isaacsim::common-logging
)
```

Use `PRIVATE_DEPENDENCIES` when logging is an implementation detail. Use `PUBLIC_DEPENDENCIES` only when a public
header uses logging API types. If the dependent distribution produces a wheel, add `isaacsim-common` to the
`isaacsim_libraries_metadata` provider in its `pyproject.toml`. The dependency supplies the logging library; do not
copy it into the dependent package.

Use `isaacsim_add_header_module` for a header-only C or C++ API. Use
`isaacsim_install_runtime_dependencies` to copy an imported shared library into the standalone C++ runtime. Use
`isaacsim_add_runtime_directory` for other runtime data, such as schema registries. Put these declarations in the
module that uses the files. A `resources/` directory has no automatic build behavior; register test and runtime data
explicitly.

Source checks reject Kit, OmniGraph, and pybind11 includes, imports, and dependencies. They also reject Carbonite in
public headers and dependencies, while allowing approved private static use. Tests confirm that built libraries do
not link dynamically to Carbonite or Kit. Modules with a stable C ABI may export only their declared symbols. Keep
host adapters and Kit lifecycle code outside this tree.

## Python module

A pure Python module registers its leaf package without declaring a compiled library or binding:

```cmake
isaacsim_add_python_module(
    NAME isaacsim.example.utilities
)
```

A compatibility distribution can preserve an old import without placing its logical module under that deprecated
namespace:

```cmake
isaacsim_add_compat_python_module(
    NAME isaacsim.deprecated.core.experimental.objects
    IMPORT_NAME isaacsim.core.experimental.objects
)
```

`NAME` still matches the source path and identifies the module during the build. `IMPORT_NAME` sets the legacy Python
import used for staging, installation, manifests, and wheels. Two modules cannot own the same import.

The `python/` directory contains the module's leaf package; it is not another package root. Do not repeat the namespace
as `python/isaacsim/...`. Define the public API in `python/impl/__init__.py` with `impl.__all__`, and use this fixed
public `python/__init__.py` after the required license header:

```python
from .impl import *
from .impl import __all__ as __all__
```

Pure-Python implementation files and binding adapters live under `python/impl/`; include `py.typed` at the `python/`
root. A pure Python module does not need `include/`, `src/`, `bindings/`, `abi/`, `tests/c/`, or `tests/cpp/`.

All Python modules stage into one build tree. Parent directories are implicit namespace packages; only registered leaf
modules contain the authored facade and typing marker:

```text
<build>/python/isaacsim/
├── lib/                                   # Shared-library closure needed by bindings.
└── <registered-import-path>/              # Public API, implementation, typing, and optional binding.
```

For development, the build copies `__init__.py` and `py.typed`, then links `impl/` and the tests to their source
directories when the platform supports directory symlinks. This makes Python code and test changes visible without
restaging. Otherwise, the build uses copies that update when their sources change.

This editable behavior applies only to the build tree. Installs and wheels contain regular copied files, and tests are
never installed. Tools that use the editable package must not write generated data beside `__file__`; use a temporary
directory instead.

Each group with Python imports provides a `<group>-python` install component. It contains that group's Python modules,
bindings, generated stubs, and required native libraries from the same group. Code from other groups remains a package
dependency and is not copied into the component.

Installing this CMake component creates an importable Python directory, not a complete Python distribution. The wheel
tool packages the component and adds standard distribution metadata. Modules do not define their own `__version__`.
Use `importlib.metadata.version("<group-name>")` to read the installed distribution version.

## Python binding

```cmake
isaacsim_add_nanobind(
    MODULE isaacsim.common.logging
    SOURCES bindings/python/Bindings.cpp
    DEPENDENCIES SomeBindingSupportTarget
    STUB_PRELOADS dependency.module
)
```

The helper registers the Python leaf package, then builds a private `bindings._bindings` extension with nanobind and
the stable CPython ABI. It generates `bindings/_bindings.pyi` and installs the type stub beside the binary. Register
the native module first with `isaacsim_add_module` or `isaacsim_add_header_module`.

Use `NO_STABLE_ABI` with a non-empty `STABLE_ABI_EXCEPTION` only when the binding cannot use the stable ABI. `SOURCE`
is shorthand for one file; use `SOURCES` for one or more. `DEPENDENCIES` lists extra targets used only by the binding.
`STUB_PRELOADS` lists nanobind modules that must load before stub generation so cross-module C++ types receive their
Python names. On Windows, use `WINDOWS_RUNTIME_SIBLING_DIRECTORY` only for an SDK whose runtime directory sits beside
the binding runtime directory.

Export headers, native binding binaries, and `_bindings.pyi` files are generated in the build tree. Do not check them
into a module's source directory. `py.typed` is authored source: it tells type checkers that the installed Python
package supplies typing information.

Do not maintain a second package-level `.pyi` for re-exported binding functions. Put richer annotations in Python
wrappers, or use `nb::sig` when stub generation cannot infer the public type. The `check-python-stubs` target builds
all registered stubs without modifying source files.

Relative loader paths are calculated from the full module depth. On Linux and macOS, shared libraries install under
`python/isaacsim/lib`; on Windows, required DLLs install next to the binding.

## Tests

```cmake
isaacsim_add_c_tests(
    MODULE isaacsim.common.logging
    SOURCES
        tests/c/Logging.cpp
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.common.logging
    SOURCES
        tests/cpp/Logging.cpp
    RESOURCE_DIR resources/tests
)

isaacsim_add_python_tests(
    MODULE isaacsim.example.utilities
)
```

Add `REQUIRES_BINDINGS` when a Python test imports a nanobind extension. Use `TEST_SUPPORT <module>...` when a Python
suite imports test-only helpers from another registered module. These helpers remain in the source tree and are never
installed.

Add `REQUIRES_NO_BINDINGS` to a C++ test only when a dependency has separate, incompatible native and Python-enabled
variants. Put C and C++ API doctest cases under `tests/c` and `tests/cpp`, and put Python tests under `tests/python`.
Never put tests inside the importable `python/` package. C API tests use `.cpp` because doctest is a C++ framework, but
they may include only public C headers and call only C symbols.

For native tests, `RESOURCE_DIR` sets the working directory. For Python tests, it limits resource lookup. In both
cases, the directory must contain `TEST_RESOURCES.md`.
Set `TIMEOUT <seconds>` on `isaacsim_add_python_tests` when one Python suite needs a positive per-test CTest timeout;
the standalone `--test-timeout` option sets the command-wide default for tests without their own timeout.

When `BUILD_TESTING=ON`, the build includes:

- C API doctest unit tests authored under `tests/c` and compiled as C++ so individual cases can be listed and filtered.
- C++ API doctest unit tests authored under `tests/cpp`. The test helpers add the shared doctest entry point
  automatically; module `SOURCES` lists contain only module-owned `.cpp` test sources and reject `Main.cpp`.
- Python pytest and Hypothesis tests from an isolated test tree against the staged package when Python support is
  enabled. Binding-dependent tests run only when bindings are enabled.
- A binary dependency check for every C++ module.
- An exact stable C symbol check only when `PUBLIC_C_SYMBOLS_FILE` is supplied.
- One isolated installation test for each complete distribution. It installs the package and its dependencies,
  compiles every public header independently against its exported target, runs the consumers, and imports installed
  Python modules in a fresh process. It also verifies exact internal dependencies and confirms that an incorrect
  dependency version is rejected.

These installation tests are generated only for test builds. Adding a package, public header, Python module, or
binding automatically extends the appropriate test.

## Code style and API documentation

The standalone library tree uses C11, C++17, and Python 3.12. C and C++ share the repository `.clang-format`
configuration: Allman braces, four-space indentation, a 120-column limit, and left-aligned pointers. The shared
formatter controls mechanical layout; the language rules control naming, ownership, error handling, and API design.

File extensions identify the native-language contract:

- `.h` and `.c` are C and use `PascalCase` file stems. Public C headers must compile independently as C11 and when
  included from C++17.
- `.hpp` and `.cpp` are C++ and use `PascalCase` file stems. A `.cpp` file that implements or tests a C ABI still
  follows C++ source style.
- C API doctest cases live under `tests/c` but are `.cpp` because doctest is a C++ framework. They include and call
  only the public C API; the install contract compiles the installed headers and consumer with the C compiler.

Public C APIs use the same identifier casing as C++: module-prefixed `camelCase` functions, `PascalCase` types, and
`camelCase` fields, parameters, and local variables. Dotted module names are flattened without separators, so
`isaacsim.common.logging` becomes the `isaacsimCommonLogging` function prefix and the `IsaacSimCommonLogging` type
prefix. Macros and unscoped C enum constants remain uppercase and module-prefixed. C API Doxygen comments document
pointer ownership, nullability, buffer sizes, units, result values, thread safety, callback behavior, and ABI
initialization. Public C++ APIs use Doxygen and document the same caller-visible contract, including exceptions only
when the API can throw.

Python implementation code lives under `python/impl`, owns `__all__`, uses type annotations and Google-style
docstrings, and targets Python 3.12. The outer `python/__init__.py` remains the fixed packaging facade. Python API tests
use pytest and Hypothesis under `tests/python`; build and packaging tool tests may use `unittest`. Python-facing
docstrings authored in nanobind C++ sources follow the Python documentation convention and remain consistent with the
generated stubs.

Repository guidance under `.cursor/rules` is global by language: C and C++ have separate code-style and Doxygen rules,
and Python has shared code-style and Python API docstring rules. Python-facing strings in nanobind sources follow the
same Python docstring contract while their surrounding code follows C++ style. Module-specific requirements are
limited to the structure, build, testing, packaging, and ABI contracts documented here. CMake files use the global
CMake style: target-based dependencies, explicit inventories, lowercase commands, and four-space indentation. Do not
hand-format generated export headers, generated `.pyi` files, package manifests, or ABI symbol output.

## Enforcement model

CMake and the test suite enforce rules that affect package identity, binary compatibility, installation, and
repeatable builds. Rules that require human judgment are checked during review.

| Contract | Enforcement |
|---|---|
| Module and group names, unique registration, version syntax, and exact group membership | Configure time |
| Required sources, public-header inventory and extensions, Python package shape, and documentation entry points | Configure time |
| C, C++, and Python test placement under the matching `tests/<language>` directory | Configure time |
| Cross-package native target links and exact shared-version dependencies | Configure time |
| C11/C++17, hidden visibility, group-derived binary versions, and relative loader paths | Configure and build time |
| Forbidden framework source/public dependencies and dynamic Carbonite or Kit libraries | Configure time and CTest |
| Nanobind stable ABI policy and rejection of the alternate binding framework | Configure time |
| Generated export headers remaining outside the source tree | Configure-time rejection and build layout |
| Stable C symbol baseline | Opt-in CTest check; updates require an explicit target |
| Installed C/C++ headers, relocated consumers, Python imports, metadata, and dependency closure | Install-contract CTest |
| Separation of Python tests, caches, bytecode, and symlinks from installed packages | Source layout, install rules, and install-contract CTest |
| Wheel identity, exact internal dependencies, import inventory, typing, namespace layout, and content exclusions | Wheel assembly validation |
| Wheel-staging discovery, artifact identity, direct SDK installation, and wheel/SDK agreement | Root preflight, staging, and tool tests |
| Mirroring module package dependencies in Kit extension metadata | Source review |

CMake generates package metadata, install components, and installation tests only when registered modules exactly
match `MODULES` and every dependency is complete. Set `ISAACSIM_PACKAGE_GROUP` or
`ISAACSIM_ENFORCE_COMPLETE_PACKAGES` to make an incomplete package a configuration error instead of reporting and
skipping it.

Each `pyproject.toml` owns its Python dependency declarations. Wheel validation confirms that every internal Python
requirement exists in the native dependency graph and uses the shared version. Native-only dependencies do not need to
appear in wheel metadata, and external Python dependencies may use supported version ranges. CMake does not read
Python project metadata.

Reviewers check rules that depend on intent, including dependency order, source file extensions, documentation,
changelog releases, generated-file cleanliness, and whether an API should promise a stable C ABI. Add an automated
check only when it provides enough value to justify the maintenance cost.

## Version, changelog, documentation, and compatibility policy

`source/libraries/VERSION` is the only version source for all packages and modules. `isaacsim_add_group` verifies the
module list, and `isaacsim_finalize_modules` generates release metadata only for complete groups. Every module needs a
`CHANGELOG.md`. Package-level documentation is optional, but a package `docs/` directory must contain `index.rst`.

`VERSION` contains only the package version and a trailing newline:

```text
7.0.0a2
```

Update the shared version with:

```bash
python3.12 source/libraries/tools/update_version.py <version>
```

Run `python3.12 source/libraries/tools/update_version.py --check` to verify the version and internal dynamic dependency
metadata without changing the source tree.

Versions use a limited form of [PEP 440](https://peps.python.org/pep-0440/): three release numbers, followed by optional
alpha (`aN`), beta (`bN`), release-candidate (`rcN`), post-release (`.postN`), development (`.devN`), or local
(`+local.parts`) suffixes. Use lowercase identifiers and no leading zeros in numeric fields. The first alpha of 7.0.0,
for example, is `7.0.0a1`. Other valid examples include `7.0.0.dev0` and `7.0.0rc1`.

The build writes this version to package manifests, CMake configuration, and Python distribution metadata. Native
libraries use only the three numeric release fields because platform library versions cannot represent PEP 440
suffixes. They do not use a separate `SOVERSION`. A version change rebuilds and relinks all libraries and consumers
because they are released together.

Review ABI compatibility manually against the latest published artifact. Check changed public headers, exported
symbols, data layouts, calling conventions, and public dependency types. The build can verify declared C symbol lists,
but it cannot determine C++ binary compatibility.

Library versions follow the Isaac Sim release train; individual libraries do not use independent semantic versions.
A minor Isaac Sim release may break the API or ABI of a module, while patch releases should remain compatible.

Modules in one package always ship together. Dependencies between packages require the exact full version from
`source/libraries/VERSION`, including any PEP 440 suffix. The build records and checks that version in native and
Python package metadata. CMake's numeric package lookup treats versions as compatible only within the same major and
minor release; consumers must explicitly opt into a later minor release.

Module-level `docs/` directories remain next to the API they describe and are also optional. When present, module
registration requires `docs/index.rst` and installs the documentation automatically; module `CMakeLists.txt` files do
not need a separate documentation call. The `<group>-documentation` component uses:

```text
share/isaacsim/packages/<group>/
├── VERSION
├── package.json                           # Package identity, membership, components, and exact dependencies.
├── index.rst                              # Present only when the group has docs/.
├── ...                                    # Other optional group documentation files and assets.
└── modules/<module-name>/
    ├── CHANGELOG.md                       # Always present.
    └── ...                                # Optional module documentation.
```

Use reStructuredText in package and module `docs/` directories. Keep changelogs in Markdown. The recommended module
layout is:

```text
docs/
├── index.rst                              # Required overview, local table of contents, and entry point.
├── api_c.rst                              # Optional C reference for modules exposing C.
├── api_cpp.rst                            # Optional C++ reference for modules exposing C++.
└── api_python.rst                         # Optional Python reference for modules exposing Python.
```

Put the module overview and local table of contents in `index.rst`. Link longer concept, example, or migration guides
from that table of contents, and create only the language reference files the module needs. Document public APIs in
their source headers, Python facades, binding docstrings, and type information as well. CMake installs the
reStructuredText files but does not build a Sphinx site. Do not copy Python signatures into prose; the facade or
binding docstring is the source of truth.

Follow Keep a Changelog in each module's `CHANGELOG.md`. Keep `## [Unreleased]` at the top, even when it is empty. Add
only populated `Added`, `Changed`, `Deprecated`, `Removed`, `Fixed`, or `Security` sections. At release, move entries
into a dated version section that matches `source/libraries/VERSION`. Do not add empty entries for unchanged modules.
Record changes users can observe, not implementation details or issue numbers.

During alpha, changelogs describe the net user-visible delta since the last release; an entry is not required for
every code change or merge request. Routine build fixes, internal cleanup, and test-only changes do not need entries.
Revise existing Unreleased entries as features evolve instead of adding `Fixed` entries for behavior that has not
shipped. CI checks shared-version and dependency metadata consistency when library metadata or its validation
tools change. The optional local `pre_merge_validate.py --libraries` command checks shared-version consistency
and scoped Python lint. Neither checks changelog content, formatting, or whether each changed library has an entry.

```markdown
# Changelog

## [Unreleased]

### Added
- `isaacsim.common.logging`: add the new public capability.

## [6.1.0] - 2026-07-13

### Added
- `isaacsim.common.logging`: add the initial logging APIs.
```

A symbol baseline is opt-in. Supply `PUBLIC_C_SYMBOLS_FILE` only after committing to maintain a stable C ABI. The check
compares the baseline with the exact exported set under the module's derived C prefix, such as `isaacsimCommonLogging`.
This catches removed, renamed, missing, and accidentally added exports; it does not validate function signatures.

Run `cmake --build <build> --target check-abi` for a read-only check. After approving an intentional C ABI addition or
removal, run the module-specific `update-abi-baseline-<module-target>` target. The aggregate `update-abi-baselines`
target updates every registered baseline and should be used cautiously. Normal builds and tests never rewrite a
baseline.

## Configuration

Use `source/libraries/build.sh` on Linux and `source/libraries/build.bat` on Windows. These scripts start the verified
Pixi launcher with Packman's bootstrap Python, then run the build in the locked Python 3.12.13 environment. Pixi
provides CMake, Ninja, DLPack, and the required Python packages. Profiles without Python receive only the small native
schema runtime. These standalone scripts do not run `repo.sh`, Premake, linbuild, Kit, or the extension build.

Run these commands from `source/libraries`:

```text
# Linux
./build.sh -r
./build.sh -r --test
./build.sh -r --test --coverage
./build.sh -r --test-only
./build.sh -r --wheel
./build.sh -r --test --stage-wheel isaacsim.common
./build.sh -r --test --wheel --dependency-profile minimum

# Windows
build.bat -r
build.bat -r --test
build.bat -r --test-only
build.bat -r --wheel
build.bat -r --test --stage-wheel isaacsim.common
```

The scripts download the repository-pinned Pixi release from its official GitHub URL, verify its committed SHA-256
digest, and install only locked environments. Packman continues to provide private, patched, and binary native
dependencies, as well as the Python interpreter used to start Pixi.

Release is the default when neither `-r` nor `-d` is present. Passing both builds both configurations. The remaining
entry-point options are:

| Option | Purpose |
|---|---|
| `-c`, `--clean` | Remove the selected profile and configuration build trees, then exit. |
| `-x`, `--rebuild` | Remove the selected build trees before configuring and building. |
| `-g`, `--generate` | Configure without building. |
| `-t`, `--target <name>` | Build one CMake target. |
| `-j`, `--jobs <count>` | Limit parallel build jobs. |
| `--profile <name>` | Select `standard`, `cpp-tests`, `python-modules`, `cpp-library`, or `werror`. |
| `--test` | Run CTest after the build; Python-enabled configurations always record locked test dependencies so CI may test the artifact later. |
| `--coverage` | Instrument native targets for coverage; requires GCC or Clang. |
| `--test-only` | Restore test dependencies and run CTest from the matching existing build tree without rebuilding. |
| `--junit-output <path>` | Write CTest results as JUnit XML; requires `--test` or `--test-only`. |
| `--test-timeout <seconds>` | Set the default per-test CTest timeout; requires `--test` or `--test-only`. |
| `--test-jobs <count>` | Limit parallel CTest jobs; requires `--test` or `--test-only`. |
| `--wheel` | Assemble wheels from a completed `standard` build without invoking CMake from the PEP 517 backend. |
| `--stage-wheel <extension>` | Build the distribution declared by an extension's `wheel-staging.toml`, then stage its wheel and SDK; repeat as needed. |
| `--carrier <extension>` | M1-compatible alias for `--stage-wheel`. |
| `--group <name>` | Limit `--wheel` to a distribution; repeat to select more than one. |
| `--output-dir <path>` | Override the wheel output directory for `--wheel` or `--stage-wheel`. |
| `--dependency-profile <name>` | Select `locked` (default) or `minimum` external Python dependencies. |
| `--no-pull` | Require current local Packman and locked Pixi environments without network access. |
| `--pull-only` | Populate Packman and every supported Pixi environment, including locked source-recipe packages, then exit. |

The `locked` dependency profile uses the exact versions selected for normal and release builds. The `minimum` profile
uses the lowest supported version of each direct Python dependency. Ordinary builds never resolve floating versions;
only the reviewed dependency-update workflow evaluates ranges and updates `pixi.toml` and `pixi.lock`.

Each non-default build profile has separate dependency and CMake directories, so profiles cannot reuse stale files.
Tests and wheel builds use only the selected profile on `PYTHONPATH`. Profiles choose the versions used to build and
test; they do not change the dependency requirements written into wheel metadata. Pixi recreates an environment when
its locked contents change.

`--test-only` supports CI jobs that restore a configured build from another job, possibly at a different checkout
path. It updates generated CMake paths, restores the locked build and test environments, recreates the generated
`developer-environment`, and runs CTest without rebuilding. Windows CI also restores the matching MSVC and Windows
SDK. Use `--junit-output` for GitLab reports, `--test-timeout` to stop hung tests, and the test regex options to select
or exclude CTest tests.

The profile names have the same feature selections as the CMake presets:

```text
./build.sh -r --profile standard --test
./build.sh -r --profile cpp-tests --test
./build.sh -r --profile python-modules --test
./build.sh -r --profile cpp-library
./build.sh -r --profile werror --test
```

### Reproduce the library CI pair locally

CI performs one clean build followed by a complete CTest run from the saved build output. Run the same pair before
pushing changes to `source/libraries` or `source/examples`:

```text
# Linux
./build.sh -r --rebuild --profile standard
./build.sh -r --profile standard --test-only --no-pull --test-timeout 600 \
    --junit-output ../../_cmake_build/isaacsim-libraries-release/source-libraries-tests.xml

# Windows
build.bat -r --rebuild --profile standard
build.bat -r --profile standard --test-only --no-pull --test-timeout 600 ^
    --junit-output ..\..\_cmake_build\isaacsim-libraries-release\source-libraries-tests.xml
```

The full CTest run includes the installed examples integration test. For a quicker examples-only check, add
`--test-regex '^tests-examples$'` to the second command. Remove it before pushing so you run the full CI suite.

The supported options are:

| Option | Default | Purpose |
|---|---:|---|
| `BUILD_TESTING` | `ON` | Build unit tests and the install contract. |
| `ISAACSIM_ENABLE_PYTHON` | `ON` | Stage, install, and test Python modules. |
| `ISAACSIM_ENABLE_PYTHON_BINDINGS` | `ON` | Build, stage, install, and test nanobind APIs; requires Python. |
| `ISAACSIM_WARNINGS_AS_ERRORS` | `OFF` | Promote compiler warnings to errors. |
| `ISAACSIM_ENABLE_COVERAGE` | `OFF` | Add native coverage instrumentation; requires GCC or Clang. |
| `ISAACSIM_PACKAGE_GROUP` | empty | Require one named group to be complete and packageable. |
| `ISAACSIM_ENFORCE_COMPLETE_PACKAGES` | `OFF` | Fail configuration if any registered group is incomplete. |
| `ISAACSIM_TARGET_DEPS_DIR` | `<repo>/_cmake_build/isaacsim-libraries-target-deps` | Override library dependencies. |
| `ISAACSIM_NATIVE_RUNTIME_DEPS_DIR` | empty | Native-support schema packages selected by the Pixi launcher. |
| `ISAACSIM_PUBLIC_DEPS_ROOT` | empty | Pixi prefix containing pinned public build tools, headers, and sources. |
| `ISAACSIM_PYTHON_RUNTIME_DEPS_DIR` | empty | Python runtime packages selected by the Pixi launcher. |
| `ISAACSIM_EXTENSION_USD_ROOT` | empty | Add the extension-compatible schema variant using a Kit OpenUSD root. |
| `ISAACSIM_PYTHON_TEST_DEPS_DIR` | empty | Test packages selected by the Pixi launcher. |
| `ISAACSIM_PYTHON_INSTALL_DIR` | `python` | Set the import-package root for a direct component install. |

Sanitizers and release portability matrices are CI or toolchain concerns rather than module CMake options.

The presets describe the CMake settings for each profile, but they do not prepare dependencies. Use `build.sh` or
`build.bat` for normal builds so Packman and Pixi paths are set consistently.

`standard` builds the C and C++ libraries, Python modules and bindings, and all tests. `cpp-tests` proves that the
libraries and native C and C++ tests do not depend on Python or nanobind. `python-modules` builds the compiled libraries
and pure Python packages and runs the applicable tests with nanobind disabled.
`cpp-library` builds only the independently consumable C and C++ libraries.

## Stage a library wheel in a Kit extension

Build reusable code as a distribution in `source/libraries`, then stage its wheel and, when needed, its native SDK in a
Kit extension. The extension can provide the complete distribution or a declared selection of its Python packages.
Add `wheel-staging.toml` at the extension root:

```toml
[wheel]
distribution = "isaacsim_common"
```

Normally, one extension provides the complete distribution. Multiple extensions can share one wheel by declaring
non-overlapping sets of Python leaf packages:

```toml
[wheel]
distribution = "isaacsim_asset"
python-modules = ["isaacsim.asset.transformer"]
```

Every extension that shares a distribution must declare `python-modules`, and their module paths must not overlap. The
wheel remains complete; only the files staged for each Kit extension are filtered.

Use `wheel-paths` to project additional top-level runtime payloads owned by the same package, such as a native Python
module and its private libraries. Use `platform-wheel-paths` for payloads present only on one normalized Python
platform tag:

```toml
[wheel]
distribution = "isaacsim_robot_motion"
python-modules = ["isaacsim.robot_motion.cumotion"]
wheel-paths = ["_cumotion*", "cumotion"]

[wheel.platform-wheel-paths]
win_amd64 = ["cumotion.dll"]
```

Each path is relative to the installed wheel and may use `*` only in its final path component. An extension with
`wheel-paths` must also declare `python-modules`. Staging fails when a path is unsafe, matches nothing, or overlaps
another projected payload.

Repository builds use `tools/repoman/repoman.py` to find each `wheel-staging.toml` and prepare its wheel and SDK before
Repo generates and stages the application. Internal Linux builds do this after relaunching in linbuild; Windows and
`--no-docker` builds do it directly. The generated output stays under `_cmake_build`, outside Repo's `_build`
directory, so rebuilding the application does not remove it before staging. To stage one extension directly, run
`source/libraries/build.sh -r --stage-wheel <extension>` or the equivalent Windows command.

The build configures the complete CMake module graph, then packages only the distribution named in the manifest. It
creates the wheel, installs the runtime and development components into an SDK directory, and installs the wheel into a
temporary staging directory. For a complete distribution, that installation becomes `pip_prebundle`. For a selected
set of packages, the tool copies only the declared leaf packages and omits distribution metadata because the result is
not a complete pip installation.

The intermediate artifact directory contains only the wheel. Create a separately shippable SDK archive through the
explicit packaging workflow. Staging fails when the wheel is missing or ambiguous, the package manifest is incomplete,
a selected module is absent, two extensions claim the same module, or the wheel and SDK describe different packages.

The distribution and extension have separate versions. `source/libraries/VERSION` identifies the wheel, SDK, and
package manifest. `config/extension.toml` controls the extension version. They do not need to match. Use the optional
`wheel-variant` field when an extension needs a nondefault distribution payload, such as schemas generated for Kit's
OpenUSD version.

Root build options apply consistently to wheel staging. A normal build prepares the selected Debug and Release
configurations. `--build-only` uses existing dependencies without network access. `--rebuild` or `-x` removes and
rebuilds the selected library and wheel staging output. `--clean` removes that output before cleaning the application.
`--fetch-only` downloads dependencies without configuring CMake, and `--generate` configures without compiling or
staging. `--enable-gcov` enables native coverage for the staged library modules. Help, post-build-only, and stage-only
commands do not stage library wheels.

Each extension's staged directory has two inputs for the extension build:

```text
_cmake_build/extension-wheel-stages/<configuration>/<extension>/
├── pip_prebundle/                         # Complete wheel install or selected Python leaf-package projection.
└── sdk/                                   # CMake-installed headers, libraries, and package metadata.
```

In the extension's `premake5.lua`, load `tools/isaac_build/extension_wheel.lua`.
`stage_isaacsim_extension_wheel` links the generated `pip_prebundle` into the built extension. Set its optional third
argument to `true` to also link SDK headers. If the extension adds its own Python adapters at paths inside the staged
package, use `stage_isaacsim_extension_wheel_with_python_overlays` to copy the wheel contents before adding those
files; the shared stage then stays unchanged. Both helpers put the installed files in the built extension's
`pip_prebundle`, which is packaged at `exts/<extension>/pip_prebundle`.

`use_isaacsim_module_sdk` adds SDK include and library directories to a native extension project. Enable
`add_local_runtime_rpath` only for a native project inside the extension that owns the adjacent `pip_prebundle`.
Other extensions depend on that extension and leave it disabled. Consumers must still list the libraries they use in
`links`.

In `extension.toml`, declare the installed Python path with `[[python.module]]`, preload required shared libraries with
platform-filtered `[[native.library]]` entries, and declare any runtime host adapter with `[[native.plugin]]`. A
compatibility wrapper is regular extension Python code; keep it separate from wheel staging and native host
adapters.

Native-library paths may use Kit's `${lib_prefix}`, `${lib_ext}`, `${platform}`, and `${config}` tokens. Do not use
wildcards; Kit treats them as literal characters for native-library preloads. Staging resolves the tokens and confirms
that each file exists. Reference the unversioned library name so releases do not require manifest changes.

Add the extension that stages the wheel as a dependency of every consuming extension. This loads the library runtime
before plugins that link its SDK libraries. Do not copy the SDK or wheel into extension source, and do not commit
generated staging output.

An extension stages only the wheel declared in its manifest because installation uses `pip --no-deps`. Kit must load
every dependency from another extension first. This prevents multiple extensions from loading separate copies of the
same native library.

Usually, another extension stages the dependency's wheel. Add that extension to `[dependencies]` in the current
extension's `config/extension.toml`.

If other Kit extensions provide a wheel dependency, map the distribution to those extensions in `wheel-staging.toml`.
For example:

```toml
[wheel.dependency-extensions]
isaacsim_deprecated = [
    "isaacsim.core.experimental.objects",
    "isaacsim.core.experimental.utils",
]
```

This mapping says that the two listed extensions provide the `isaacsim_deprecated` wheel dependency. Also list both
extensions in the current extension's `config/extension.toml`:

```toml
[dependencies]
"isaacsim.core.experimental.objects" = {}
"isaacsim.core.experimental.utils" = {}
```

Staging first looks for an extension with a matching `wheel-staging.toml`, then checks the explicit provider mapping.
It fails if neither exists or if `extension.toml` omits a required provider. When several extensions split one wheel,
list all of them because wheel metadata does not identify the extension that provides each module. The tool validates
these files but does not edit them.

For a native extension project that directly includes or links a staged library, call `use_isaacsim_module_sdk` to
add that library's SDK include and library directories. This applies whether the extension stages its own wheel or
depends on another extension's wheel. Enable `add_local_runtime_rpath` only when that native project owns the adjacent
`pip_prebundle`. A dependency used only inside the already-built feature library needs the Kit extension dependency
and runtime preload, but no extra host-adapter include or link path.

`werror` is a configure, build, and test preset rather than a workflow preset. Run it with:

```text
cmake --preset werror --fresh
cmake --build --preset werror
ctest --preset werror
```

## Install components

Every complete group owns four group-specific components. Replace `<group>` below with a registered distribution name:

```text
cmake --install <build> --prefix <prefix> --component <group>-runtime
cmake --install <build> --prefix <prefix> --component <group>-development
cmake --install <build> --prefix <prefix> --component <group>-python
cmake --install <build> --prefix <prefix> --component <group>-documentation
```

- `<group>-runtime` contains the package's versioned shared libraries, registered runtime data, and explicitly
  registered third-party runtime libraries.
- `<group>-development` contains public headers, import libraries, and the relocatable
  `find_package(<group> CONFIG)` export. Install it with the runtime component for a native SDK.
- `<group>-python` contains the package's Python APIs, implementation, typing, bindings, and same-group shared-library
  closure; it excludes Python distribution metadata and source tests. Install the exact-version internal dependency
  packages' Python components into the same prefix when their Python APIs or native libraries are needed.
- `<group>-documentation` contains the shared version, every module changelog, and optional group or module docs.

Each component includes `share/isaacsim/packages/<group>/package.json` and `VERSION`, allowing an installer to identify
it without inspecting file names. The manifest records modules, Python imports, component names, and exact internal
dependencies. Python metadata and requirements remain in `pyproject.toml`.

The four components are selectable parts of one versioned package, not separate packages. A native runtime archive
usually contains `runtime`; an SDK contains `runtime` and `development`; and a Python installation contains `python`
plus required dependency packages. Documentation may ship separately or with any of them.

Use the matching workflow for each release format. Build native runtime and SDK archives without bindings
(`cpp-library` to assemble them and `cpp-tests` to validate them), so they never rely on Python. Build a Python
distribution by packaging the installed `<group>-python` component with its PEP 517 backend. Wheel assembly does not
compile CMake targets.

Do not publish the runtime or development files from a bindings-enabled build as the standalone native release.
Installation tests load native libraries without bindings and test Python imports separately with bindings enabled.

The generated CMake configuration finds each internal package and checks its full version before loading its targets.
It also finds external CMake dependencies. When a wheel needs an internal package, add the distribution name to the
`isaacsim_libraries_metadata` provider in `pyproject.toml`. The provider adds the exact shared-version requirement.
Native-only dependencies may be omitted from wheel metadata.

## Package artifacts

[packaging/package.py](packaging/package.py) creates native runtime and SDK archives from an existing CMake build.
`./build.sh -r --wheel` installs each distribution's Python component and runs its PEP 517 backend without invoking
CMake. The wheel tool gives the backend a temporary private staging directory; `pyproject.toml` files do not refer to
repository staging paths. See
[packaging/README.md](packaging/README.md) for commands and validation expectations.

## Add a module

1. Select the package that will ship the module. For a new package, normally add its two-component source root, such
   as `isaacsim/navigation` for `isaacsim_navigation`, with
   `isaacsim_add_group(NAME <distribution-name> MODULES ...)`. A flat root named for the distribution may instead
   declare `MODULE_PREFIX <dotted-logical-prefix>`. Give it a `pyproject.toml`, declare already registered dependencies
   with `isaacsim_add_group_dependency`, and add the package root explicitly to `source/libraries/CMakeLists.txt`.
   Configure scikit-build-core with `wheel.cmake = false` and an
   empty `wheel.packages` list; the wheel tool supplies the installed component as packaging input.
2. Add the module below that root using its dotted-name suffix as directories. For example,
   `isaacsim.navigation.route_planner` lives at `isaacsim/navigation/route_planner`. Add `CMakeLists.txt` and
   `CHANGELOG.md`, then register it explicitly in the distribution's `CMakeLists.txt`; do not give it an independent
   release version.
3. For a compiled module, use `.h`/`.c` for C and `.hpp`/`.cpp` for C++, then declare sources, public headers, and
   target dependencies with `isaacsim_add_module`.
4. For a Python API, add a typed leaf package under `python/` and register it with `isaacsim_add_python_module`, either
   directly for a pure Python module or indirectly through `isaacsim_add_nanobind`.
5. Add C, C++, and Python tests under `tests/c`, `tests/cpp`, and `tests/python`, respectively. Mark binding-dependent
   Python tests with `REQUIRES_BINDINGS`.
6. If the module adds dependencies, update the root Pixi manifest or the library-only Packman manifest according to
   ownership; do not add them to Kit dependency manifests. Add native cross-package dependencies with
   `isaacsim_add_group_dependency` and add equivalent Python dependency names to the package's
   `isaacsim_libraries_metadata` provider when its wheel needs them. External Python runtime dependencies may use
   supported ranges in `[project.dependencies]`.
7. Add a nanobind definition only when the Python API wraps compiled code; binding stub generation is automatic.
8. Add a C symbol baseline only if the C ABI is intentionally stable.
9. Add the module name to the group's `MODULES`, update the module `CHANGELOG.md`, and add module
   reStructuredText documentation rooted at `docs/index.rst` when the
   public contract needs more than header and Python documentation.
10. Run the `standard`, `cpp-tests`, `python-modules`, `cpp-library`, and `werror` profiles through the standalone
   entry point.

### M1 wheel staging compatibility

`--carrier` remains an alias for `--stage-wheel`. Source extensions can retain
`module-carrier.toml` and its `[carrier]` table; the build and staging tools accept
that manifest when `wheel-staging.toml` is absent. If both exist, their staging
tables must match. The old `tools/isaac_build/module_carrier.lua` helper and
`source/libraries/tools/stage_carrier.py` entry point forward to the current staging
implementation and use its output directories.
