# Isaac Sim libraries

`source/libraries` contains Kit-independent Isaac Sim C and C++ libraries, pure Python modules, and optional nanobind
Python APIs. A module must work without Kit, the extension registry, OmniGraph, or an Isaac Sim application. Public
APIs do not expose Carbonite types or headers. Infrastructure modules may use a private, statically linked Carbonite
implementation when the binary does not acquire a dynamic Carbonite dependency or export Carbonite symbols; feature
modules use the shared `isaacsim.common.logging` facade instead of embedding Carbonite again.

## Terminology

The module and Kit integration layers use these terms for distinct responsibilities:

| Term | Meaning |
|---|---|
| **Carrier** | A Kit extension packaging role that makes one independently built distribution, its native runtime, and its optional SDK available to Kit. A carrier owns staging and dependency metadata; it does not imply a new API. |
| **Adapter** | Runtime integration code that connects a module to host-owned services or lifecycle. An adapter may attach a logging backend or translate startup and shutdown events without owning the feature implementation. |
| **Wrapper** | An optional API layer that delegates to or translates another API, typically to preserve compatibility during migration. A wrapper is separately identifiable and deprecatable; packaging a module unchanged does not make it a wrapper. |

A Kit extension may perform more than one role. For example, a carrier commonly contains a native adapter, while a
compatibility wrapper is needed only when the exposed API differs from the underlying module API. The sections below
use these terms according to their architectural role rather than using them interchangeably.

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
│   ├── prepare_root_build.py              # Discovers and prepares carriers for the root application build.
│   ├── stage_carrier.py                   # Installs and validates a wheel and SDK for a Kit carrier extension.
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

A directory has no build or install meaning until its distribution explicitly registers it with the appropriate CMake
helper. The supported module shapes are:

- Compiled: `isaacsim_add_module` creates a separately linkable shared library from C11 and/or C++17 sources.
- Header-only: `isaacsim_add_header_module` exports C and/or C++ headers and transitive target dependencies without a
  binary.
- Pure Python: `isaacsim_add_python_module` stages and installs a typed leaf package without a compiled target.
- Compatibility Python: `isaacsim_add_compat_python_module` keeps the source-aligned logical module identity while
  installing a typed leaf package at an explicitly declared legacy import path.
- USD schema: `isaacsim_add_usd_schema_module` generates codeless schema resources in the build tree and stages typed
  Python import roots plus optional public C++ headers.
- Native with Python: `isaacsim_add_module` or `isaacsim_add_header_module`, followed by `isaacsim_add_nanobind`, adds
  a private binding to the same package shape used by a pure Python module.

Compatibility migrations may pass `PRESERVE_PACKAGE_LAYOUT` to `isaacsim_add_python_module` when retaining an
established leaf package exactly is more important than converting it to the standard `impl/` facade. New modules use
the standard layout. Pass `REQUIRES_BINDINGS` to `isaacsim_add_python_module` or
`isaacsim_add_compat_python_module` only when importing that module inherently requires a nanobind extension. A
bindings-disabled install contract still installs and validates the distribution and its dependency closure, but omits
only those marked imports; bindings-enabled contracts continue to import every registered module. This module-level
flag describes import applicability and is separate from the identically named Python-test registration flag.

A distribution package is one independently installable and shippable unit. Its CMake package identity, such as
`isaacsim_common`, normally derives from the matching two-component source path, `isaacsim/common`. A flat distribution
root whose directory already matches the CMake package name is also supported when it declares its logical
`MODULE_PREFIX`; the compatibility package uses `isaacsim_deprecated` with `MODULE_PREFIX isaacsim.deprecated`. The
CMake API historically calls this unit a *group*, so helper names, component placeholders, and internal properties use
`group`; this documentation otherwise uses *distribution package* or *package*. A package can contain multiple C,
C++, header-only, pure Python, or bound logical modules, such as `isaacsim.common.logging` at
`isaacsim/common/logging`. A logical module does not need to provide Python. Each module owns its changelog but not an
independent version. Every distribution package and module ships at the single version in `source/libraries/VERSION`.
The package exposes native runtime, native development, Python, and documentation payload components; these are views
of one package and never carry separate versions.

CMake group, component, manifest, and native archive names use underscores. Authored Python project names and internal
requirements use their canonical hyphenated PEP 508 form, such as `isaacsim-common`; Python packaging tools normalize
these separators, but the source metadata consistently uses hyphens. Logical modules and import paths remain dotted.

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

For a normal two-component root, the `isaacsim/<distribution>` source path forms the underscore-separated distribution
name and its complete module-root path forms the dotted logical module name. For a flat root, its directory is the
distribution name and `MODULE_PREFIX` supplies the dotted logical prefix; the module's path below the root supplies
the remaining suffix. CMake names remain explicit, and configure-time validation rejects any mismatch. A compatibility
Python module retains this source-aligned logical name while declaring its legacy installed import separately:

| Source path relative to `source/libraries` | CMake group | Logical module | Python import |
|---|---|---|---|
| `isaacsim/common/logging` | `isaacsim_common` | `isaacsim.common.logging` | `isaacsim.common.logging` |
| `isaacsim/common/profiling` | `isaacsim_common` | `isaacsim.common.profiling` | `isaacsim.common.profiling` |
| `isaacsim/navigation/route_planner` | `isaacsim_navigation` | `isaacsim.navigation.route_planner` | `isaacsim.navigation.route_planner` |
| `isaacsim/robot/kinematics/inverse` | `isaacsim_robot` | `isaacsim.robot.kinematics.inverse` | `isaacsim.robot.kinematics.inverse` |
| `isaacsim_deprecated/core/experimental/objects` | `isaacsim_deprecated` | `isaacsim.deprecated.core.experimental.objects` | `isaacsim.core.experimental.objects` |

USD schema distributions mirror the `newton-usd-schemas` and `physx-usd-schemas` package boundary under
`isaacsim_usd_schemas`. They pass an explicit `MODULE_PREFIX` because their source grouping, distribution name, and
public import namespace intentionally differ. This exception retains the two-component distribution root and exact
module inventory; it does not make source-path naming optional for ordinary modules.

Schema generation has two isolated compatibility variants. Ordinary standalone builds generate `usd2505` resources
with the module OpenUSD dependency. A Kit carrier requests `usd2511`, causing CMake to generate a second package with
the configuration-matched OpenUSD dependency already pulled under `_build/target-deps/usd`. The pure Python wheels use
build tags `1usd2505` and `1usd2511`; carriers require an exact tag instead of relying on pip to choose between them.

A module must have at least one component below its distribution root; the distribution root itself is not a module.
Intermediate namespace directories need no `CMakeLists.txt`: the distribution can use
`add_subdirectory(kinematics/inverse)`. Registered module roots are non-overlapping leaves: if
`isaacsim.robot.kinematics` is a module, `isaacsim.robot.kinematics.inverse` cannot be a separate module. The
group-level `docs` directory is reserved for package documentation and cannot be registered as a module.

Python leaf packages use PEP 420 implicit namespaces above the module root. For example, the
`isaacsim.common.logging` wheel contains `isaacsim/common/logging/__init__.py` but does not add authored
`isaacsim/__init__.py` or `isaacsim/common/__init__.py` files. This lets independently installed distributions share
the `isaacsim` namespace. The distribution root is packaging structure, not an importable leaf API, and cannot itself
be registered as a logical module. Compatibility distributions may provide explicitly declared legacy leaves outside
their logical namespace, but they follow the same implicit-namespace and single-owner rules.

Declare each cross-package dependency after registering its owning group with `isaacsim_add_group_dependency`.
The dependency version is always the shared `source/libraries/VERSION`; version arguments are intentionally unsupported.
Dependency groups must be registered first. `MODULES` is the exact membership contract: package metadata
and install-contract tests are generated only when the declared and registered modules match and every dependency is
itself packageable. Use
`ISAACSIM_PACKAGE_GROUP=<name>` or `ISAACSIM_ENFORCE_COMPLETE_PACKAGES=ON` to turn an incomplete package into a
configuration error.

The package's `pyproject.toml` is the authoritative source for Python distribution metadata and runtime dependencies.
External Python runtime dependencies declare their supported PEP 508 ranges in `[project.dependencies]`. Internal
`source/libraries` dependency names use the shared-version dynamic metadata provider:

```toml
[project]
dynamic = ["version", "dependencies"]

[[tool.dynamic-metadata]]
provider = { path = "../../packaging", module = "isaacsim_libraries_metadata" }
field = "dependencies"
names = ["isaacsim-common"]
```

The provider reads `source/libraries/VERSION` and appends exact requirements such as `isaacsim-common==7.0.0a1` when
the backend resolves wheel metadata. A native package dependency does not automatically become a wheel requirement.
When the wheel declares an internal requirement, wheel validation proves that it exists in the native package graph
and uses the exact shared version. CMake does not construct or modify Python core metadata. The standalone build
compiles the complete module graph independently. Wheel assembly installs only the selected package's `<group>-python`
component and asks the backend to package those staged files without configuring or building CMake.

Only create directories a group or module uses. Groups and modules are listed explicitly with `add_subdirectory`; the
build does not discover source directories automatically. The root project adds each distribution path, for example
`add_subdirectory(isaacsim/common)` or `add_subdirectory(isaacsim_deprecated)`, and that distribution adds its module
suffix paths. A group may
conditionally register a module when its complete external target set is available. Treat a partially available
required target set as a configuration error rather than silently building a reduced module, and report fully
unavailable optional target sets in the configure output.

Generated output is not part of the source contract. Standard build trees live under
`<repo>/_cmake_build/isaacsim-libraries-<configuration>`. Release trees for the other profiles live under
`<repo>/_cmake_build/isaacsim-libraries-<profile>`; Debug appends `-debug`. Populated dependencies live under
`<repo>/_cmake_build/isaacsim-libraries-target-deps`, and default wheel and native artifacts live under
`<repo>/_cmake_build/isaacsim-libraries-artifacts/<configuration>` when the documented packaging commands are used.
Carrier wheel artifacts and their validated extension stages live under
`<repo>/_cmake_build/module-carrier-artifacts/<configuration>` and
`<repo>/_cmake_build/module-carriers/<configuration>/<extension>`, respectively. The Python editable stage, generated
export headers, nanobind binaries, stubs, package manifests, and install-contract consumers all live below a build
tree. Tool-created `.pytest_cache`, `.mypy_cache`, `__pycache__`, bytecode, and packaging staging directories are
ignored transient output; they must never be treated as module structure or included in an installed package.

Library dependencies are isolated from Kit application dependencies. `deps/isaacsim-libraries.packman.xml` installs
only the private and patched native dependencies under `_cmake_build/isaacsim-libraries-target-deps`. Public native
release archives, including the OVPhysX SDK, use checksum-pinned Pixi recipes.
The root `pixi.toml` and `pixi.lock` are the sole declarations for public build packages, Python packages,
Python 3.12.13, CMake 4.3.2, Ninja 1.13.2, DLPack 1.3, doctest 2.5.3, fmt 7.0.3, nanobind 2.12.0, SDL 3.4.14,
stb 2.30, and tsl-robin-map 1.4.0 are exact dependencies in `pixi.lock`. Public conda-forge binaries are used where
their runtime closures are compatible with the standalone modules. Pixi builds aarch64 doctest, SDL, and the
header-only stb package from checksum-pinned official archives. The shared SDL recipe avoids introducing a second
EGL/OpenGL dispatch stack ahead of the host graphics driver. CMake only consumes installed files and never downloads
them. This does not require publishing Isaac Sim to conda-forge.
DLPack headers are installed with the owning development component so its SDK remains self-contained. Separate locked
and minimum environments live below `.pixi/envs`; shared
schema-runtime pins are declared once and composed into the full and native-support runtime environments. Packages
resolve from public PyPI or the NVIDIA public index when available; an unchanged dependency version unavailable there
uses the NVIDIA internal PyPI explicitly. Repository builds delegate registered carrier preparation through
`tools/repoman/repoman.py` before Repo generation and staging. On internal Linux builds, preparation waits for Repo's
linbuild relaunch so carrier binaries use the same target toolchain; `--no-docker` and Windows builds prepare directly.
The standalone launcher collects license files and declared license metadata from the locked `native-runtime` Pixi
distributions whose payloads are redistributed by the modules. It writes the aggregate under a lock- and
platform-specific `_cmake_build/PACKAGE-LICENSES/<key>` directory. CMake snapshots that aggregate inside each
profile-specific build tree. Every module runtime and native SDK, and every Python-enabled wheel, installs its snapshot
under `share/licenses/<distribution>`. Carrier staging requires the same aggregate in its Python prebundle and SDK,
including carriers that project selected packages from a shared wheel.
A successful full `standard` build with locked dependencies also creates `developer-environment` in its build tree.
The build installs every complete package's runtime, development, and Python components into a staging directory and
publishes the unified environment only after every component succeeds. Source examples use this environment directly,
so individual example commands do not install or replace library dependencies. Concurrent examples share a read lock;
replacement and cleanup wait until those examples finish.

Library dependencies remain isolated from application dependency manifests. Do not add library-only dependencies to
`deps/isaac-sim.packman.xml` or the Kit Python prebundles.

Keep dependency declarations with the modules that consume them:

- Prefer an exact public conda package on each supported platform. Use a checksum-pinned Pixi source recipe only for a
  platform-specific gap or when the public package's runtime closure conflicts with a host-owned system runtime. Do
  not add a CMake download fallback.
- Add or update private, patched, and binding packages in `deps/isaacsim-libraries.packman.xml`.
- Add or update external Python runtime, native-support, build, and test dependencies in root `pixi.toml`, then
  regenerate and review `pixi.lock`. Keep non-public package indexes scoped to the individual dependency.
- Declare target-level build dependencies in the consuming module's `CMakeLists.txt`. Use `PACKAGE_DEPENDENCIES` when
  an installed exported target requires a package to be found before it can be loaded.
- Declare dependencies on another distribution with `isaacsim_add_group_dependency`. Cross-package target links are
  rejected unless the owning package declares that dependency. Package manifests and CMake configs require the exact
  shared version. If the wheel also needs that package, add its distribution name to the `isaacsim_libraries_metadata`
  provider's `names` list in `pyproject.toml`; the provider supplies the exact shared version.
- Declare third-party Python runtime requirements and their supported ranges in `[project.dependencies]`. Concrete
  build/test tool selections remain private build-environment inputs and do not become wheel runtime requirements.

Carbonite is a special private implementation dependency. Feature APIs and package dependencies may not expose its
types or headers. A dedicated host-integration header may accept an opaque native handle so an adapter can borrow a
host-owned backend without adding a Carbonite include or link dependency to feature consumers. The
`isaacsim.common.logging` and `isaacsim.common.profiling` shared libraries contain the Carbonite-specific
implementations behind their public facades. Feature modules link those facades and must not compile another copy of
the implementation. An infrastructure module may declare a private Carbonite target only when CMake identifies that
target as a static library. The resulting module must still pass the binary dependency and exported-symbol tests.
`carb_sdk_plugins` in the library Packman manifest supplies the pinned headers, static archive, and the NVTX plugin
packaged by `isaacsim.common.profiling`. Only a consuming module calls `find_package(Carbonite REQUIRED MODULE)`, so
unrelated module configurations do not require Carbonite discovery.

The project requires CMake 3.26 or newer and Ninja. The standalone entry point installs and always uses the locked
Pixi CMake 4.3.2 and Ninja 1.13.2 packages. Linux host prerequisites are a C11/C++17 compiler toolchain, binutils,
standard shell/archive utilities, CA certificates, and either `curl` or `wget` for Packman bootstrap downloads. On
Windows 11, install Visual Studio 2022 and the Windows SDK. Windows 10 is not supported. The build reuses an initialized
developer environment or locates Visual Studio with `vswhere`. NVIDIA CI may provision its non-redistributable
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

`NAME` must match the module root relative to `source/libraries`, with directory separators replaced by dots. The helper
derives the native target, exported target, Python path, test name, and isolated output directory. For example,
`isaacsim.common.logging` exports `isaacsim::common-logging`, while `isaacsim.navigation.route_planner` exports
`isaacsim::navigation-route_planner`.

Every compiled module is a C11 and/or C++17 shared-library target with hidden visibility, a generated export header,
group-derived release metadata, and platform-relative runtime loader paths. On platforms that use shared-object
versioning, the runtime contains the fully versioned library and an unversioned link to it, for example
`libisaacsim-common-logging.so.7.0.0` and `libisaacsim-common-logging.so`. The inherited release version identifies an
artifact inside the distribution; it does not create an independently released module. Public headers and the
generated export header are installed with a CMake package. C++ consumers use:

```cmake
find_package(isaacsim_common CONFIG REQUIRED)
target_link_libraries(application PRIVATE isaacsim::common-logging)
```

Use `.h` for C interfaces and `.hpp` for C++ interfaces. Use `.c` for C implementation sources and `.cpp` for C++
implementation sources. Name C and C++ headers and sources with `PascalCase` file stems. Public header extensions and
the repository-wide PEP 440 `VERSION` file are checked at configure time. A module may expose a stable C API, a C++
API, or both; C++ sources and consumers require C++17.

Keep only installed API headers under `include/`. Put implementation-only headers under `src/` and nanobind-only
adapters under `bindings/python/`. A build-only interface target may share a binding adapter between modules, but it
must not be exported as part of the public C++ package. The declared `PUBLIC_C_HEADERS` and `PUBLIC_CPP_HEADERS` must
exactly match every file under `include/`; only that inventory and the generated export header are installed.

Use target dependencies rather than ambient include or library directories:

```cmake
PUBLIC_DEPENDENCIES isaacsim::other-module ThirdParty::Core
PRIVATE_DEPENDENCIES ThirdParty::Implementation
PACKAGE_DEPENDENCIES ThirdParty
```

`PACKAGE_DEPENDENCIES` names packages required before loading the installed exported targets. Internal Isaac Sim module
dependencies are also used to compute the shared-library closure staged with Python bindings. Do not repeat an
internal distribution in `PACKAGE_DEPENDENCIES`; declare it once at group level so the generated exact requirement is
authoritative.

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

Use `PRIVATE_DEPENDENCIES` when logging is only an implementation detail. Use `PUBLIC_DEPENDENCIES` only when an
installed public header directly uses logging API types. If the dependent distribution produces a wheel, its
`pyproject.toml` adds `isaacsim-common` to the `isaacsim_libraries_metadata` provider so native and Python package
metadata agree. The dependent package does not copy the logging shared library into its own runtime or Python
component; the exact `isaacsim-common` package dependency supplies it.

Use `isaacsim_add_header_module` for a header-only C API, C++ API, or both. Use
`isaacsim_install_runtime_dependencies` when an
imported shared-library target must be copied into the standalone C++ runtime component, and
`isaacsim_add_runtime_directory` for non-library runtime data such as schema registries. These declarations belong in
the consuming module rather than in a top-level name-based exception. Files under `resources/` have no implicit build
meaning; register test and runtime resources explicitly. Changes to declared runtime data retrigger staging.

The source boundary rejects Kit, OmniGraph, and pybind11 includes, imports, and declared dependencies. It also rejects
Carbonite in public headers and dependencies, while allowing a private static Carbonite target. CTest checks that the
built library has no dynamic Carbonite or Kit dependency and, for modules with a stable C ABI, exports only the
declared symbols under that ABI's prefix. Adapters for extensions and Kit lifecycle behavior live outside this tree.

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

`NAME` still matches the module source path and determines its build identity. `IMPORT_NAME` determines only the
Python staging, installation, manifest, and wheel import identity. The helper rejects duplicate import ownership.

The `python/` directory is the content of the module's derived leaf package, not another Python package root. Do not
repeat the namespace as `python/isaacsim/...`. Every Python module provides `python/impl/__init__.py`, defines its
public API with `impl.__all__`, and uses this fixed public facade after the required license header:

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

Development staging copies the fixed `__init__.py` facade and `py.typed`, then links `impl/` and the isolated test tree
to their authored source directories. Python implementation and test edits are therefore immediately visible without
restaging on platforms that permit directory symlinks. When directory symlinks are unavailable, the build falls back
to dependency-tracked copies. This editable behavior is limited to the build tree: install components and wheels
always contain ordinary copied implementation files, and source tests are never installed. Helper-owned stub
generation and tests disable Python bytecode and pytest caches so the links do not dirty the source tree. Other tools
that import the editable stage should likewise avoid writing beside `__file__` and use a temporary directory for
generated data.

Each group with Python imports provides a `<group>-python` install component containing only that group's authored
Python modules, bindings, generated stubs, and same-group native runtime closure. The installed import namespace does
not change component ownership. Cross-group code is not copied into the component; it remains a package dependency. A
direct CMake component install creates an importable Python prefix, not an installed Python distribution. The wheel
tool selects that component, and the PEP 517 backend packages its staged contents with all standard metadata and
records. Python import modules do not define independent `__version__` values. Consumers of a wheel query the
distribution version with `importlib.metadata.version("<group-name>")`.

## Python binding

```cmake
isaacsim_add_nanobind(
    MODULE isaacsim.common.logging
    SOURCES bindings/python/Bindings.cpp
    DEPENDENCIES SomeBindingSupportTarget
    STUB_PRELOADS dependency.module
)
```

The helper registers the leaf package through `isaacsim_add_python_module`, then builds a private
`bindings._bindings` extension with nanobind and the CPython stable ABI. It generates `bindings/_bindings.pyi` from the
compiled extension and installs the stub beside the binary. The native module must first be registered with
`isaacsim_add_module` or `isaacsim_add_header_module`; the public package imports the private extension. Use
`NO_STABLE_ABI` together with a non-empty `STABLE_ABI_EXCEPTION` only when a binding cannot use the stable ABI.
`SOURCE` is accepted as a shorthand for a single file; use `SOURCES` for one or more files. `DEPENDENCIES` declares
additional targets needed only by the binding. `STUB_PRELOADS` lists nanobind modules whose private bindings stubgen
must import first to register cross-module C++ types under their Python-facing names; each dependency binding is staged
before the target stub is generated. On Windows, `WINDOWS_RUNTIME_SIBLING_DIRECTORY` copies an SDK runtime
directory that sits beside, rather than below, the binding runtime directory into the installed leaf package. Use it
only for an SDK with that split layout.

Export headers, native binding binaries, and `_bindings.pyi` files are generated in the build tree. Do not check them
into a module's source directory. `py.typed` is authored source: it tells type checkers that the installed Python
package supplies typing information.

Do not maintain a second package-root `.pyi` for re-exported binding functions. Put richer annotations in real Python
wrappers or use `nb::sig` on a binding when stub generation cannot infer the public type. The `check-python-stubs`
target builds every registered stub without modifying the source tree.

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

Add `REQUIRES_BINDINGS` to a Python test registration when the package imports a nanobind extension.
Use `TEST_SUPPORT <module>...` when a Python suite imports test-only helpers owned by other registered modules. The
named modules' staged `tests/python` directories are appended to the suite's isolated `PYTHONPATH` after its own tests
and before external dependencies. Test support remains source-only and is never installed.
Add `REQUIRES_NO_BINDINGS` to a C++ test registration only when a dependency provides mutually exclusive native and
Python-enabled variants. C API and C++ API doctest suites are authored as `.cpp` files under `tests/c` and `tests/cpp`,
respectively; Python tests are authored under `tests/python`, never inside the importable `python/` package. The C API
suite includes only public C headers and calls only C symbols, while compilation of an actual C consumer is enforced by
the relocated install contract. For native tests, `RESOURCE_DIR` selects the working directory; for Python tests, it
scopes resource lookup. In every case the directory requires a `TEST_RESOURCES.md` marker.
Set `TIMEOUT <seconds>` on `isaacsim_add_python_tests` when one Python suite needs a positive per-test CTest timeout;
the standalone `--test-timeout` option sets the command-wide default for tests without their own timeout.

When `BUILD_TESTING=ON`, the build registers:

- C API doctest unit tests authored under `tests/c` and compiled as C++ so individual cases can be listed and filtered.
- C++ API doctest unit tests authored under `tests/cpp`. The test helpers add the shared doctest entry point
  automatically; module `SOURCES` lists contain only module-owned `.cpp` test sources and reject `Main.cpp`.
- Python pytest and Hypothesis tests from an isolated test tree against the staged package when Python support is
  enabled. Binding-dependent tests run only when bindings are enabled.
- A binary dependency check for every C++ module.
- An exact stable C symbol check only when `PUBLIC_C_SYMBOLS_FILE` is supplied.
- One isolated relocated install contract per complete distribution package. Each contract installs that package and
  its dependency closure, compiles every package-owned public header in its own translation unit against only its
  module's exported target, runs each consumer, and imports the installed Python modules in a fresh process. Packages
  with CMake exports and dependencies also prove that a deliberately wrong dependency version is rejected. Every
  contract verifies the manifest's exact internal dependencies, including for pure-Python-only packages.

The install contracts are generated only for test builds. Their templates contain no module-specific names; adding a
package, header inventory, pure Python module, or Python binding automatically extends the applicable contract.

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

The CMake helpers and tests enforce contracts that affect package identity, binary compatibility, relocation, or
repeatable builds. Conventions that cannot be checked mechanically without interpreting intent remain review-time
policy.

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
| Carrier discovery, artifact identity, direct SDK installation, and wheel/SDK agreement | Root preflight, staging, and tool tests |
| Mirroring module package dependencies in Kit extension metadata | Source review |

Finalization emits package metadata, components, and install-contract tests only when the registered modules exactly
match `MODULES` and all package dependencies finalized successfully. Set `ISAACSIM_PACKAGE_GROUP` or
`ISAACSIM_ENFORCE_COMPLETE_PACKAGES` when an incomplete package must be a configuration error instead of being reported
and skipped.

Python dependency declarations are owned by each `pyproject.toml`. Wheel validation enforces that every declared
internal requirement belongs to the native dependency graph and uses the exact shared version; it does not require
native-only dependencies to appear in wheel metadata. Supported ranges remain available for external Python
dependencies. CMake does not parse Python project metadata.
Binding and stub source-tree cleanliness, dependency ordering, implementation source filename extensions,
doctest-runner contents, documentation prose, changelog release transitions, and the decision to adopt a stable C ABI
are also review-time conventions. Promote one to a mechanical check only when repeated mistakes justify the added
machinery.

## Version, changelog, documentation, and compatibility policy

`source/libraries/VERSION` is the only version source for every distribution package and contained module.
`isaacsim_add_group` validates complete module inventory; `isaacsim_finalize_modules` generates package metadata and
installs release metadata only for complete groups. Every module must provide `CHANGELOG.md` at its root. A group-level
`docs/` directory is optional and, when present, must contain `index.rst`; it describes the package as a whole.

`VERSION` contains only the package version and a trailing newline:

```text
7.0.0a1
```

Update the shared version with:

```bash
python3.12 source/libraries/tools/update_version.py <version>
```

Run `python3.12 source/libraries/tools/update_version.py --check` to verify the version and internal dynamic dependency
metadata without changing the source tree.

The canonical version syntax is a deliberately constrained
[PEP 440](https://peps.python.org/pep-0440/) subset: a three-part release tuple with optional `aN`, `bN`, or `rcN`,
followed by optional `.postN`, `.devN`, and `+local.parts`. PEP 440 defines `aN` as the alpha pre-release form; this
project numbers its first alpha as `a1`, so the first alpha for the 7.0.0 release is `7.0.0a1`. Identifiers are
lowercase and numeric fields have no leading zero.
Examples include `7.0.0.dev0`, `7.0.0a1`, and `7.0.0rc1`. This one value is written to the package manifest and CMake
package configuration; each package's `pyproject.toml` reads it as dynamic Python distribution metadata. Native shared
libraries use its numeric release tuple for `VERSION`, because native platform version fields cannot represent PEP 440
suffixes. They do not use a separate `SOVERSION`: packages contain only the fully versioned runtime file and its
unversioned link. Since all libraries and consumers are built and shipped together, any release-version change
rebuilds and relinks the complete set.

ABI compatibility remains a manual review responsibility. Authors must compare affected public headers, exported
symbols, layouts, calling conventions, and public dependency types with the most recent published artifact. The build
verifies declared C symbol baselines where present, but it does not infer C++ binary compatibility.

The shared library version follows the Isaac Sim release train rather than independent semantic versioning for each
library. All libraries ship at exactly the same version, and a minor Isaac Sim release may include an API or ABI break
in specific modules. Patch releases should remain compatible. Modules within a group require no constraints because
they always ship together. Dependencies crossing package boundaries require the same full
`source/libraries/VERSION`, including any PEP 440 suffix. The exact PEP 440 specifier is recorded in
`package.json`; generated native discovery verifies the dependency's full exported version after locating it, and
the `isaacsim_libraries_metadata` provider derives the exact Python requirement from the same file so the backend writes
matching `Requires-Dist` metadata. This explicit comparison also handles PEP 440 prerelease, post, development, and
local suffixes that CMake's version argument cannot express. Numeric CMake package discovery considers releases
compatible only within the same major and minor release line; consumers must opt into a later minor release explicitly.

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

Within a package or module's `docs/` directory, use reStructuredText. Keep changelogs in Markdown. The recommended
module documentation layout is:

```text
docs/
├── index.rst                              # Required overview, local table of contents, and entry point.
├── api_c.rst                              # Optional C reference for modules exposing C.
├── api_cpp.rst                            # Optional C++ reference for modules exposing C++.
└── api_python.rst                         # Optional Python reference for modules exposing Python.
```

Keep the module overview and its local table of contents in `index.rst`. Substantial concept, example, or migration
guides may use topic-specific filenames and must be linked from that table of contents. Create only the
language-reference files the module needs. Public APIs must also be documented at their source in C or C++ headers,
Python facades, binding docstrings, and typing information. CMake installs these reStructuredText sources as
documentation payload; it does not assemble a Sphinx site. Module documentation does not depend on the repository's
extension documentation tools. Do not copy Python signatures into prose; binding or facade docstrings are the
reference source.

Follow Keep a Changelog in each module `CHANGELOG.md`. Keep an empty or populated `## [Unreleased]` heading at the top;
use `Added`, `Changed`, `Deprecated`, `Removed`, `Fixed`, or `Security` only when populated. At release, move the
entries from that section into a new `## [X.Y.Z] - YYYY-MM-DD` section immediately below it, where the version matches
`source/libraries/VERSION`. Unchanged modules do not need an empty entry for every repository version. Record
user-visible API, behavior, compatibility, and migration changes rather than implementation details or issue
identifiers.

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

The primary build entry points are `source/libraries/build.sh` on Linux and `source/libraries/build.bat` on Windows.
The thin wrappers use Packman's bootstrap interpreter only to run the verified Pixi launcher. All build logic then
runs once under Python 3.12.13 in the locked `build-driver` environment; there is no pip installer, interpreter
restart, latest-dependency mode, or Packman build-Python fallback. The launcher supplies CMake, Ninja, DLPack, and
concern-specific Python package prefixes explicitly. Python-disabled profiles still receive the small
`native-runtime` schema prefix but do not install the full scientific runtime. The entry points do not call `repo.sh`,
Premake, linbuild, Kit, or the extension build.

Run these commands from `source/libraries`:

```text
# Linux
./build.sh -r
./build.sh -r --test
./build.sh -r --test --coverage
./build.sh -r --test-only
./build.sh -r --wheel
./build.sh -r --test --carrier isaacsim.common
./build.sh -r --test --wheel --dependency-profile minimum

# Windows
build.bat -r
build.bat -r --test
build.bat -r --test-only
build.bat -r --wheel
build.bat -r --test --carrier isaacsim.common
```

The wrappers bootstrap the repository-pinned Pixi release from its exact official GitHub release URL, verify its
committed SHA-256 digest, install only locked Pixi environments, and then invoke the private Python build
implementation. Pixi itself is not hosted by or downloaded through Packman. Packman remains the owner of private,
patched, and binary native dependencies and supplies only the bootstrap interpreter used by the thin wrappers.

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
| `--wheel` | Assemble wheels from a completed `standard` build without invoking CMake from the PEP 517 backend. |
| `--carrier <extension>` | Build and stage the distribution declared by a carrier extension; repeat as needed. |
| `--group <name>` | Limit `--wheel` to a distribution; repeat to select more than one. |
| `--output-dir <path>` | Override the wheel output directory for `--wheel` or `--carrier`. |
| `--dependency-profile <name>` | Select `locked` (default) or `minimum` external Python dependencies. |
| `--no-pull` | Require current local Packman and locked Pixi environments without network access. |
| `--pull-only` | Populate Packman and every supported Pixi environment, including locked source-recipe packages, then exit. |

`locked` uses the concrete versions selected for ordinary and release builds. `minimum` pins every directly declared
Python dependency to its supported lower bound. Newer versions are evaluated only by the reviewed dependency-update
workflow, which updates `pixi.toml` and regenerates `pixi.lock`; ordinary builds never perform a floating solve. The
dependency-update workflow is the only place that evaluates declared ranges. Each non-default profile has separate
dependency and CMake build directories, so switching profiles cannot reuse stale files. Test and wheel subprocesses
receive only the selected profile on `PYTHONPATH`, preventing a caller's environment from supplying different tooling.
These profiles select concrete dependencies used to build and test artifacts; they do not affect exact internal
package requirements or external runtime ranges written to wheel metadata. Pixi installs the committed lock without
changing it and recreates a prefix when its locked contents change. Transitive tools follow their direct dependency's
published compatibility contract.

`--test-only` is intended for the separate Linux x86_64, Linux aarch64, and Windows x86_64 CI test jobs that restore
their matching configured build-tree artifacts. The producer and consumer checkout paths may differ; test-only
execution validates the completed artifact and relocates generated CMake metadata to the consumer checkout. The
command restores Packman plus the locked build-driver, native-runtime, runtime, and test environments before invoking
CTest. Windows CI also restores the packaged MSVC and Windows SDK used by the build job. Dependency directories are
not portable build artifacts because Packman entries are symlinks into the runner cache. The large generated
`developer-environment` is also omitted from CI artifacts and rematerialized from the retained CMake install rules
before CTest. Use `--junit-output` for GitLab test reporting, `--test-timeout` to bound a hung top-level CTest test,
`--test-regex` to select CTest tests by name, and `--exclude-test-regex` to omit matching tests.

The profile names have the same feature selections as the CMake presets:

```text
./build.sh -r --profile standard --test
./build.sh -r --profile cpp-tests --test
./build.sh -r --profile python-modules --test
./build.sh -r --profile cpp-library
./build.sh -r --profile werror --test
```

### Reproduce the library CI pair locally

The platform CI contract is one clean artifact-producing build followed by one complete CTest consumer. Run the same
pair before pushing changes to `source/libraries` or `source/examples`:

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

The complete CTest consumer includes the installed examples integration test. Use `--test-regex '^tests-examples$'`
on the second command for a quicker examples-only iteration; omit it before pushing to exercise the same suite as CI.

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

The presets document the static CMake profile shapes. Direct preset execution is not a supported dependency bootstrap;
use `build.sh` or `build.bat` so all Packman and Pixi roots are supplied consistently.

`standard` builds the C and C++ libraries, Python modules and bindings, and all tests. `cpp-tests` proves that the
libraries and native C and C++ tests do not depend on Python or nanobind. `python-modules` builds the compiled libraries
and pure Python packages and runs the applicable tests with nanobind disabled.
`cpp-library` builds only the independently consumable C and C++ libraries.

## Kit carrier extensions

A carrier extension makes an independently built distribution, or a declared Python-package projection of one,
available to Kit without moving its implementation into `source/extensions`. The extension owns a small
`module-carrier.toml` file:

```toml
[carrier]
distribution = "isaacsim_common"
```

One extension normally carries the complete distribution. Multiple extensions may reuse one wheel artifact without
duplicating package ownership by declaring disjoint leaf-package projections:

```toml
[carrier]
distribution = "isaacsim_asset"
python-modules = ["isaacsim.asset.transformer"]
```

Every carrier sharing a distribution must declare `python-modules`, and projected module paths may not overlap. The
wheel remains complete and publishable; only its Kit prebundle view is projected.

Every repository `build` command passes through `tools/repoman/repoman.py`, which discovers each
`module-carrier.toml` and prepares the matching configuration before Repo generation and staging. Internal Linux builds
defer this work until Repo relaunches in linbuild; Windows and `--no-docker` builds run it in their current environment.
Module-owned output remains under `_cmake_build`, outside Repo's `_build` clean root, so an application rebuild cannot
remove the carrier before extension staging. For standalone iteration, run
`source/libraries/build.sh -r --carrier <extension>` or the matching Windows command explicitly.

Carrier preparation configures and builds the authoritative complete CMake module graph. It then packages only the
distribution named by the carrier: the Python build frontend creates its wheel, CMake installs its runtime and
development components into an SDK stage, and pip installs the wheel into a temporary carrier input. A complete
carrier retains that installation as its `pip_prebundle`; a projected carrier copies only its declared leaf packages
and omits distribution metadata because the projection is not a complete pip installation. This keeps CMake as the
only native dependency graph and allows native-only package dependencies to remain absent from wheel metadata. The
intermediate artifact directory contains the wheel only; independently shippable SDK archives remain an explicit
packaging operation. Staging rejects a missing or ambiguous wheel, an incomplete package manifest, a projected module
absent from the wheel inventory, overlapping projection ownership, or wheel and SDK stages produced from different
package inventories.

The distribution and extension have independent versions. `source/libraries/VERSION` identifies the wheel, SDK, and
package manifest, while the extension's `config/extension.toml` version controls extension publication. A carrier
always packages artifacts generated from the current source-library build, so these version values do not need to
match. Use the optional `wheel-variant` carrier field when an extension requires a nondefault payload declared by the
distribution, such as a schema generated for the Kit OpenUSD version.

The root build entry points preserve the relevant application-build intent: an ordinary build prepares the selected
Debug and Release configurations, `--build-only` requires already populated native and Python module dependencies
without network access, and `--rebuild` or `-x` removes the configuration's complete library build tree and persistent
carrier outputs before rebuilding and restaging them. The same rebuild argument is then forwarded unchanged to Repo.
`--clean` removes module and carrier output before Repo cleans application output. `--fetch-only` populates native,
source, and Python module dependencies without configuring CMake. `--generate` configures the module graph but does not
compile, package, or stage carrier artifacts. Root builds using `--enable-gcov` forward native coverage instrumentation
to the carrier module configuration. Help, post-build-only, and stage-only invocations do not build carriers.

The carrier stage has two inputs for the extension build:

```text
_cmake_build/module-carriers/<configuration>/<extension>/
├── pip_prebundle/                         # Complete wheel install or selected Python leaf-package projection.
└── sdk/                                   # CMake-installed headers, libraries, and package metadata.
```

Use `tools/isaac_build/module_carrier.lua` from the carrier extension's `premake5.lua`.
`stage_isaacsim_module_carrier` links the generated `pip_prebundle` into the built extension. Its optional third
argument links the carrier SDK headers into the extension when set to `true`.
`use_isaacsim_module_sdk` adds the SDK include and library directories to a native extension project. Pass `true` for
`add_local_runtime_rpath` only when that native project lives in the carrier extension and therefore owns the adjacent
`pip_prebundle`; other extensions depend on the carrier extension and leave it false. Consumer projects still name
their libraries explicitly in `links`, so the helper does not hardcode a distribution or module naming pattern.

The carrier extension must declare the installed Python path with `[[python.module]]`, preload the required shared
libraries with platform-filtered `[[native.library]]` entries, and declare any runtime adapter with
`[[native.plugin]]`. A compatibility wrapper, when required, is ordinary extension Python code and remains separate
from carrier staging and native adapter ownership.

Native-library paths may use Kit's `${lib_prefix}`, `${lib_ext}`, `${platform}`, and `${config}` tokens. Do not use
wildcards: the supported Kit runtime treats them literally for native-library preloads. Carrier staging resolves the
tokens for the current build and requires every `pip_prebundle` native-library path to identify an existing file.
Reference the unversioned shared-library name so release changes do not require carrier manifest edits. A consuming
extension adds the carrier to its extension dependencies. This ensures the carrier runtime is loaded before a consumer
plugin that links its SDK library. Do not copy the SDK or wheel into extension source, and do not commit a carrier
stage.

A carrier stages only its own wheel because installation uses `pip --no-deps`. Kit must load every wheel dependency
through another extension before it loads the carrier. This also prevents multiple extensions from loading separate
copies of the same native library.

Usually, another carrier provides the dependency. Add that carrier extension to `[dependencies]` in the current
carrier's `config/extension.toml`.

Sometimes normal Kit extensions provide a wheel dependency instead of another carrier. In that case, map the wheel
distribution to those extensions in the current carrier's `module-carrier.toml`. For example:

```toml
[carrier.dependency-extensions]
isaacsim_deprecated = [
    "isaacsim.core.experimental.objects",
    "isaacsim.core.experimental.utils",
]
```

This example says that the `isaacsim.core.experimental.objects` and `isaacsim.core.experimental.utils` extensions
provide the `isaacsim_deprecated` wheel dependency. Also list both extensions in the carrier's
`config/extension.toml`:

```toml
[dependencies]
"isaacsim.core.experimental.objects" = {}
"isaacsim.core.experimental.utils" = {}
```

The staging tool first looks for a registered carrier. If it finds none, it uses the explicit provider mapping. It
rejects a dependency when neither is available, or when a required provider is missing from `extension.toml`. When one
wheel is split across several carrier extensions, list all of them because the wheel metadata does not identify which
extension provides each module. The staging tool validates these files but does not modify them.

A dependent carrier's Premake project always calls `use_isaacsim_module_sdk` for its own SDK. It calls the same helper
for a dependency carrier with `add_local_runtime_rpath` left false only when its adapter directly includes or links
that dependency's API. A dependency used privately inside the already-built feature library needs no additional
adapter include or link path; it only needs the Kit extension dependency and runtime preload described above.

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

Each installed component also carries `share/isaacsim/packages/<group>/package.json` and `VERSION`, so an archive or
installer can identify the payload without inspecting filenames. The manifest records package membership, Python
imports, component names, and exact internal package dependencies. Python project metadata and requirements remain in
`pyproject.toml`. These components are not four packages: they are selectable payloads from one versioned package. A
native runtime archive
normally contains `runtime`; a native SDK contains `runtime` and `development`; a Python prefix contains `python` plus
the required dependency packages; documentation can ship separately or alongside any of them.

Build native and Python release payloads through their matching workflows. A native runtime or SDK archive uses a
bindings-disabled configuration (`cpp-library` for release assembly, `cpp-tests` for validation), so every native
dependency is linked and installed without relying on Python. A Python distribution uses its PEP 517 backend to
package the `<group>-python` component installed from the completed standalone build; its declared
Python dependencies provide any Python-enabled native dependencies. Wheel assembly never compiles CMake targets.
Do not treat the runtime/development view of a bindings-enabled build as the standalone native release
artifact. The install contracts force native libraries to load in bindings-disabled configurations and separately
exercise Python imports in the bindings-enabled configuration.

The generated CMake config locates every declared internal package and then compares its exported full version with
the shared version before loading exported targets. It also resolves external CMake dependencies. When a wheel needs
the same dependency, add its distribution name to the dependent package's `isaacsim_libraries_metadata` provider in
`pyproject.toml`; the provider generates its exact shared-version requirement. Wheel validation rejects
undeclared-native, conditional, ranged, or differently versioned internal requirements; omission is valid for a
native-only dependency.

## Package artifacts

[packaging/package.py](packaging/package.py) creates native runtime and SDK archives from an existing CMake build.
`./build.sh -r --wheel` installs each distribution's Python component and invokes the configured PEP 517 backend with
CMake disabled. The wheel orchestrator supplies a private, per-invocation component stage to the backend; package
`pyproject.toml` files never name a repository staging directory. The backend owns the wheel specification and
archive. See
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
