# CMake helper reference

Include `IsaacSimModule.cmake` once, register distribution groups in dependency order, explicitly add their module
directories, and call `isaacsim_finalize_modules()` after all groups. `source/libraries/CMakeLists.txt` is the canonical
working example.

In this API, *group* means one independently shippable distribution package. It is not a second packaging layer. The
term remains in helper and component names for compatibility; logical modules are the package's unversioned contents.
The terms *carrier*, *adapter*, and *wrapper* follow the canonical definitions in the
[main libraries README](../README.md#terminology).

The helper bundle resolves its scripts and templates relative to `IsaacSimModule.cmake`. A source tree may therefore
include the helpers from another location; it does not need its own `cmake/` copy. The consuming project still owns
`ISAACSIM_LIBRARIES_DIR`, `ISAACSIM_LIBRARIES_VERSION_FILE`, output paths, feature options, and dependency discovery.
Dependency acquisition and tool selection are deliberately outside these helpers. In this repository,
`source/libraries/build.sh` and `build.bat` use Packman's bootstrap Python to pull the library dependency manifest,
restart under the pinned library Python for every profile, materialize the selected Python dependency environments,
and invoke the pinned CMake executable. Python-disabled CMake profiles still use the pinned Python interpreter for
build orchestration and schema-wheel staging.

## Files

| File | Responsibility |
|---|---|
| `IsaacSimModule.cmake` | Public include point. |
| `IsaacSimModuleCore.cmake` | Native targets, public headers, runtime payloads, naming, and boundary checks. |
| `IsaacSimPackage.cmake` | Distribution registration, shared version metadata, install components, and finalization. |
| `IsaacSimPackagingFilters.cmake` | Shared exclusions for generated caches and Python bytecode in package payloads. |
| `IsaacSimPython.cmake` | Pure-Python package staging and installation. |
| `IsaacSimUsdSchema.cmake` | Codeless USD schema generation, Python staging, and SDK header installation. |
| `IsaacSimNanobind.cmake` | Nanobind targets, generated stubs, loader paths, and native runtime closure. |
| `IsaacSimTesting.cmake` | C, C++, Python, ABI, binary-boundary, and relocated install tests. |
| `IsaacSimDocumentationCatalog.cmake` | Generate the public distribution, module, header, and Python API inventory. |
| `IsaacSimSystemRequirements.cmake` | Load and validate the shared language and build-tool version contract. |
| `FindCarbonite.cmake` | Locates the pinned static Carbonite archive when a consuming module requests it. |
| `CheckBinaryDependencies.cmake` | Reject linked framework libraries outside the module boundary. |
| `CheckExportedSymbols.cmake` | Compare or explicitly update an opt-in stable C symbol baseline. |
| `initialize_nanobind_runtime.py` | Initialize binding-local Windows runtime search paths in installed packages. |
| `run_nanobind_stubgen.py` | Run nanobind stub generation with required Windows runtime directories enabled. |
| `StageEditableDirectory.cmake` | Symlink editable Python directories with a copy fallback. |
| `CopyPythonPackage.cmake` | Copy a clean Python package tree while excluding generated and cache files. |
| `CopyRuntimeDirectory.cmake` | Copy runtime payloads while excluding transient Python cache files. |
| `run_usd_gen_schema.py` | Run OpenUSD schema generation with isolated module dependencies. |
| `RunInstallContractTest.cmake` | Install a package closure and exercise downstream native and Python consumers. |
| `tests/PackageDependencies.cmake` | Exercise valid and adversarial exact package dependency declarations. |
| `tests/PythonImports.cmake` | Exercise compatibility import registration and ownership validation. |
| `tests/Testing.cmake` | Exercise native test-helper source validation. |
| `tests/CopyRuntimeDirectory.cmake` | Verify runtime staging retains payloads and excludes Python caches. |
| `IsaacSimPackageConfig.cmake.in` | Installed package discovery template. |
| `IsaacSimPackageConfigVersion.cmake.in` | Installed package version-policy template. |
| `IsaacSimPackageManifest.json.in` | Installed package identity and component manifest template. |

Names beginning with `_isaacsim_` are implementation details. Module `CMakeLists.txt` files use only the public
functions below and ordinary target-based CMake commands. Public helpers reject unknown arguments so misspelled
contract keywords fail at configure time.

## Package registration

```cmake
isaacsim_add_group(
    NAME isaacsim_common
    MODULES
        isaacsim.common.array
        isaacsim.common.logging
)
```

- `NAME` is the independently installable distribution name.
- `MODULE_PREFIX` is required only for a flat distribution root and supplies its dotted logical namespace.
- `MODULES` is its exact module inventory. Missing or undeclared modules prevent packaging.

`isaacsim_add_group_dependency` declares one previously registered internal distribution. It accepts no version
argument: every `source/libraries` package is released in lockstep, so the helper derives an exact requirement from the
shared `source/libraries/VERSION`. Registration rejects duplicate, self, unknown, and mismatched-version dependencies.
Generated package discovery locates the dependency and compares its full exported version, which preserves exactness
even for PEP 440 suffixes that cannot be passed as a CMake version argument.

`isaacsim_finalize_modules()` validates inventories and emits native manifests, install components, CMake exports, and
install-contract tests. Call it exactly once after the last group. Every group root, including a native-only package,
must provide `pyproject.toml`; that file, not the CMake helpers, owns Python project metadata and dependencies.
When a dependency is required by both native and Python artifacts, add its distribution name to the
`isaacsim_libraries_metadata` dynamic provider in `pyproject.toml`. The provider derives its exact requirement from the
shared version; wheel validation requires it to exist in the generated native package manifest and match that version.
A native-only dependency may be absent from wheel metadata. External Python dependencies may independently advertise
supported ranges.

A group may be registered from a directory one or two components below `ISAACSIM_LIBRARIES_DIR`. A two-component root
uses `isaacsim/<distribution>`; joining those components with an underscore must equal `NAME`, and the dotted path is
its module prefix. A flat root must itself equal `NAME` and explicitly declare `MODULE_PREFIX`. For example,
`isaacsim/common` registers `isaacsim_common` with the derived `isaacsim.common` prefix, while
`isaacsim_deprecated` registers `isaacsim_deprecated` with `MODULE_PREFIX isaacsim.deprecated`. Every declared module
must be below the resulting dotted namespace. The root project explicitly adds the distribution directory.

Schema distributions that intentionally mirror an external namespace may pass `MODULE_PREFIX` explicitly. The group
must still be exactly two source directories below `ISAACSIM_LIBRARIES_DIR`, and the declared modules must be below that
prefix. For example, `isaacsim_usd_schemas/robot` registers distribution `isaacsim_robot_schema` with
`MODULE_PREFIX usd.schema`. The distribution name remains authoritative in `pyproject.toml`; the override does not
infer package identity from the source directory.

Cross-group target links require a matching group dependency. For example:

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

The generated `isaacsim_featureConfig.cmake` calls `find_dependency(isaacsim_common CONFIG)` and rejects a dependency
whose full package version differs from the repository-wide version. Use a private target dependency when the linked
module is an implementation detail and a public target dependency only when installed headers require it. Do not also
list an internal distribution in `PACKAGE_DEPENDENCIES`.

## Python wheel inputs

Every standalone, wheel, and carrier build configures the complete module graph declared by the top-level
`CMakeLists.txt`. This keeps native package dependencies authoritative in CMake, including dependencies intentionally
absent from wheel metadata. The wheel tool uses `cmake --install --component <group>-python` to place only the selected
distribution in a clean staging prefix; it never asks the PEP 517 backend to configure or compile CMake. This component
must therefore contain the complete import package, bindings, same-package native closure, typing, resources, and
package manifest.

CMake does not create `.dist-info`, choose wheel tags, calculate records, or write the wheel archive. The configured
PEP 517 backend owns those responsibilities and copies the clean component stage with its CMake integration disabled.
The wheel tool passes the unique stage to the backend at invocation time; neither this helper bundle nor a package's
`pyproject.toml` contains a fixed wheel-staging path.

## Native modules

Compiled modules use portable C11 for `.c`, C++17 for `.cpp`, `.h` for C interfaces, and `.hpp` for C++ interfaces.
Public C headers must also be safe to include from C++17. The top-level project disables compiler-specific C and C++
language extensions and the install contract compiles installed C and C++ headers with their corresponding compilers.

```cmake
isaacsim_add_module(
    NAME isaacsim.common.logging
    SOURCES src/Logging.cpp
    EXPORT_HEADER isaacsim/common/logging/Export.h
    EXPORT_MACRO ISAACSIM_COMMON_LOGGING_API
    PUBLIC_C_HEADERS isaacsim/common/logging/Logging.h
    PUBLIC_CPP_HEADERS isaacsim/common/logging/Logging.hpp
    PUBLIC_DEPENDENCIES dependency::public
    PRIVATE_DEPENDENCIES dependency::implementation
    PACKAGE_DEPENDENCIES DependencyPackage
    PUBLIC_C_SYMBOLS_FILE abi/c-api.symbols
)
```

`NAME`, `SOURCES`, `EXPORT_HEADER`, and `EXPORT_MACRO` are required. Declare every installed header in exactly one of
`PUBLIC_C_HEADERS` or `PUBLIC_CPP_HEADERS`. `PUBLIC_C_SYMBOLS_FILE` is optional and commits the module to that exact C
ABI. ABI compatibility remains a manual review responsibility, especially for C++ APIs. Dependencies are targets;
`PACKAGE_DEPENDENCIES` names external packages needed by installed consumers.

Public headers containing unavoidable inline or template implementation machinery belong under the namespace-aligned
`include/.../details/` path and remain part of the declared header inventory. Private headers and implementations in a
module's `details` namespace belong under `src/details/`.

Public and package dependencies may not expose Carbonite types or headers. A Carbonite target is accepted in
`PRIVATE_DEPENDENCIES` only when it is a CMake static-library target, and public headers are checked for Carbonite
includes. The separately registered binary-boundary test still rejects a dynamic Carbonite dependency in the built
module.

The `isaacsim.common.logging` infrastructure module calls `find_package(Carbonite REQUIRED MODULE)` and privately
links `carb::static`. Feature modules that need logging link `isaacsim::common-logging`; they do not discover or link
Carbonite themselves. This keeps the static Carbonite implementation in one shared library and lets a host adapter
attach that instance to an application-owned logging interface. Discovery remains demand-driven for other legitimate
infrastructure consumers: projects containing no static Carbonite consumer do not need the SDK.
`FindCarbonite.cmake` requires exactly one populated platform directory and both Debug and Release archives, avoiding
an ambiguous cross-platform glob.

Use `isaacsim_add_header_module()` for a native module without a binary:

```cmake
isaacsim_add_header_module(
    NAME isaacsim.example.header_api
    PUBLIC_CPP_HEADERS isaacsim/example/header_api/HeaderApi.hpp
    PUBLIC_DEPENDENCIES dependency::public
    PACKAGE_DEPENDENCIES DependencyPackage
)
```

It accepts `NAME`, `PUBLIC_C_HEADERS`, `PUBLIC_CPP_HEADERS`, `PUBLIC_DEPENDENCIES`, and `PACKAGE_DEPENDENCIES`. At least
one public header is required. A compiled or header-only native module may be followed by `isaacsim_add_nanobind()`.

## Python modules and bindings

```cmake
isaacsim_add_python_module(NAME isaacsim.example.utilities)
```

This is sufficient for a pure-Python module. Its `python/` directory must contain `__init__.py`, `impl/__init__.py`, and
`py.typed`. The implementation initializer owns `__all__`; the public initializer contains the fixed
`from .impl import *` and `from .impl import __all__ as __all__` facade statements. Development staging copies the
facade and typing marker and symlinks `impl/`. A platform that cannot create a directory symlink falls back to a
dependency-tracked copy. Installation always copies ordinary files. Tests remain outside the package under
`tests/python/` and are linked to an isolated build-tree location for test execution.

For a migration that must retain an established leaf package byte-for-byte, pass `PRESERVE_PACKAGE_LAYOUT`. The
helper then requires only `__init__.py` and `py.typed`, stages the complete `python/` directory as the leaf package,
and does not impose the `impl/` facade. This option is for compatibility migrations; new modules use the standard
facade layout.

Pass `REQUIRES_BINDINGS` when a pure-Python module imports a binding-backed module at import time. The module is still
staged and installed in every Python-enabled profile, but binding-disabled install contracts omit that import because
its runtime prerequisite is intentionally unavailable. Binding-enabled install contracts continue to import it. Use
the option only for import-time requirements; ordinary optional or function-local binding use does not need it.

Use `isaacsim_add_compat_python_module` when a compatibility distribution must own a legacy import outside its
logical source namespace:

```cmake
isaacsim_add_compat_python_module(
    NAME isaacsim.deprecated.core.experimental.objects
    IMPORT_NAME isaacsim.core.experimental.objects
    REQUIRES_BINDINGS
)
```

`NAME` remains subject to the physical identity rule and determines targets, output directories, tests, changelogs,
and distribution membership. `IMPORT_NAME` must be a valid `isaacsim.*` module name and determines the Python stage,
install destination, and package-manifest import. Each import name can have only one registered provider.
`REQUIRES_BINDINGS` has the same install-contract meaning as on `isaacsim_add_python_module`.

Codeless USD schema packages use the source import layout directly and declare each import root and schema input:

```cmake
isaacsim_add_usd_schema_module(
    NAME usd.schema.isaac
    PYTHON_IMPORTS
        usd.schema.isaac
        omni.isaac.IsaacSensorSchema
    SCHEMAS
        usd/schema/isaac/robot_schema/RobotSchema.usda
    PUBLIC_CPP_HEADERS
        isaacsim/robot/schema/robot_schema.hpp
)
```

Every import root provides `__init__.py` and `py.typed`. The helper copies maintained package sources into the build
tree, excludes any source-tree `generatedSchema.usda`, runs the OpenUSD generator there, and installs only clean
generated packages. The default `usd2505` component uses the standalone dependency under
`ISAACSIM_TARGET_DEPS_DIR/openusd`. Setting `ISAACSIM_EXTENSION_USD_ROOT` adds an isolated `usd2511` component generated
with Kit's OpenUSD; the generator verifies both runtime versions before producing resources. A maintained
`plugInfo.json` beside each input schema seeds generation so intentional resource-path settings survive regeneration.
Tests use the standalone component and the same `isaacsim_add_python_tests` helper as other Python modules.

```cmake
isaacsim_add_nanobind(
    MODULE isaacsim.common.logging
    SOURCES bindings/python/Bindings.cpp
    DEPENDENCIES binding_support_target
    STUB_PRELOADS dependency.module
)
```

The compiled or header-only native module must be registered first. `SOURCE` is supported for one source; `SOURCES`
accepts multiple. `DEPENDENCIES` adds binding-only targets. `STUB_PRELOADS` names nanobind modules whose private
bindings must be imported before stub generation so cross-module C++ types retain their Python-facing annotations.
The helper stages each listed binding first. Bindings use the CPython stable ABI by default.
`NO_STABLE_ABI` requires a non-empty `STABLE_ABI_EXCEPTION` explaining why. Generated `_bindings.pyi` files remain in
the build tree and are installed beside the binding binary. The helper validates its declaration and source files even
when bindings are disabled, while binding-only target dependencies are resolved only in a binding-enabled build.
On Windows, `WINDOWS_RUNTIME_SIBLING_DIRECTORY` accepts an existing SDK runtime directory that is a sibling of the
binding runtime directory. The helper copies it to `<module>/plugins` for staging and installation and exposes both
runtime locations while generating stubs. Do not use it for runtime content already emitted beside the binding.

Compiled, header-only, pure-Python, and compatibility helpers enforce the same physical identity rule. A module
registered as `isaacsim.common.logging` must be invoked from
`<ISAACSIM_LIBRARIES_DIR>/isaacsim/common/logging`. Under a flat root, the declared `MODULE_PREFIX` replaces the root
directory when deriving that logical identity. Deeper logical modules use deeper directories, and a module must have
at least one component below its distribution root. Registered module roots must be leaves and may not
contain other registered module roots; intermediate directories are namespaces only. Compatibility modules can
install under a different explicitly registered Python import name without changing their logical identity. The
`docs` name directly below a distribution is reserved for package documentation.

## Runtime payloads

`isaacsim_install_runtime_dependencies(MODULE <name> TARGETS ... [DESTINATION ...] [NATIVE_ONLY])` installs target
files needed by the standalone native package and stages them with bindings. On platforms with SONAMEs, both the
imported target's file name and its load-time SONAME are emitted so relocated packages do not depend on the build
prefix. `DESTINATION` places the libraries below the platform's installed library directory and mirrors that relative
path under `isaacsim/lib` in Python packages. Pass only
runtime-loadable library targets; the helper validates target existence but does not infer whether installing a static
archive is meaningful. `NATIVE_ONLY` prevents the dependency from also being copied into Python packages that use the
module.

`isaacsim_add_runtime_directory(MODULE <name> SOURCE ... DESTINATION ... [NATIVE_ONLY])` registers non-library data
such as schema registries. `SOURCE` may be absolute or module-relative; `DESTINATION` is relative to the installed
library directory
and must not contain `..`. On Windows, `WINDOWS_INSTALL_DESTINATION` overrides the native install destination for
payloads that must remain beside a DLL. `WINDOWS_PYTHON_SHARED` keeps the Python payload under `isaacsim/lib` instead of
placing it beside each importing binding. Files placed under `resources/` have no effect until one of these helpers or a
test helper explicitly references them. `NATIVE_ONLY` prevents the directory from also being copied into Python
packages that use the module.

## Tests

`isaacsim_add_c_tests(MODULE <name> SOURCES ... DEPENDENCIES ... RESOURCE_DIR ... LABELS ...)` creates one C API
doctest executable from `.cpp` sources authored under `tests/c`. The suite includes the public C headers and exercises
the C ABI, but is compiled as C++ to provide doctest discovery and filtering. The install contract separately compiles
and links a real C consumer.

`isaacsim_add_cpp_tests(MODULE <name> SOURCES ... DEPENDENCIES ... RESOURCE_DIR ... LABELS ...)` creates one doctest
executable. `REQUIRES_NO_BINDINGS` limits a test to native-only configurations.

`isaacsim_add_python_tests(MODULE <name> RESOURCE_DIR ... TIMEOUT <seconds> LABELS ... TEST_SUPPORT <module>...)` runs
the editable build-tree view of `tests/python/` against the staged package. `REQUIRES_BINDINGS` omits it when nanobind
is disabled. `TIMEOUT` sets a positive per-test CTest timeout. `TEST_SUPPORT` adds the staged `tests/python`
directories of named registered modules to the isolated test `PYTHONPATH`. The suite's own tests take precedence,
followed by the support modules in declaration order and then runtime and test dependencies. This shares module-owned
test helpers without installing them or moving them into the generic `testing/` infrastructure.

The native test helpers add the shared doctest entry point from `testing/cpp/DoctestMain.cpp`. Module source lists must
contain only module-owned test sources and must not provide `Main.cpp`.

All resource options require a `TEST_RESOURCES.md` marker. The marker documents why the test depends on data outside
its source files.

The native test helpers reject non-`.cpp` sources and sources outside their matching `tests/c` or `tests/cpp`
directory. Python test registration always discovers `tests/python`; placing tests under the importable `python/`
directory is rejected.

## Generated names

For `isaacsim.common.logging`, the helpers derive:

| Use | Name |
|---|---|
| Build target | `isaacsim-common-logging` |
| Exported target | `isaacsim::common-logging` |
| Python import | `isaacsim.common.logging` |
| Output directory | `modules/isaacsim.common.logging` |
| C ABI prefix | `isaacsimCommonLogging` |

The derivation accepts any number of lowercase module segments below an `isaacsim/<distribution>` root. Logical names
are explicit and must match their source paths. Only `isaacsim_add_compat_python_module` can give a pure-Python module
a distinct installed import identity.
