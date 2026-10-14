# CMake helper reference

Include `IsaacSimModule.cmake` once. Register distribution groups in dependency order, add each module directory
explicitly, and call `isaacsim_finalize_modules()` after the last group. See `source/libraries/CMakeLists.txt` for a
working example.

In this API, *group* means an independently shippable distribution package, not another packaging layer. Helper and
component names retain this older term for compatibility. The terms *host adapter* and *compatibility wrapper* follow
the definitions in the
[main libraries README](../README.md#terminology).

The helpers find their scripts and templates relative to `IsaacSimModule.cmake`, so another source tree can use them
without copying the `cmake/` directory. The consuming project still defines library and version paths, output
directories, feature options, and dependencies.

These helpers do not download dependencies or select build tools. In this repository, `build.sh` and `build.bat`
prepare the locked dependencies and run the pinned Python and CMake versions. Profiles without Python APIs still use
the pinned Python interpreter to coordinate the build and stage schema wheels.

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
| `run_nanobind_stubgen.py` | Run nanobind stub generation with Windows runtime directories and isolated output handles. |
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

Names beginning with `_isaacsim_` are private. Module `CMakeLists.txt` files should use only the public functions below
and standard target-based CMake commands. Public helpers reject unknown arguments, so misspelled keywords fail during
configuration.

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

`isaacsim_add_group_dependency` declares a dependency on an internal distribution that has already been registered.
It has no version argument because all `source/libraries` packages use the version in
`source/libraries/VERSION`. Registration rejects duplicate, self-referencing, unknown, and mismatched dependencies.
Generated package configuration also compares the full installed version, including PEP 440 suffixes that CMake
cannot express in a version argument.

Call `isaacsim_finalize_modules()` exactly once after the last group. It verifies module lists and generates native
manifests, install components, CMake exports, and installation tests. Every group root needs a `pyproject.toml`, even
if it contains only native code. That file owns Python metadata and dependencies.

If both the native package and wheel need an internal dependency, add its distribution name to the
`isaacsim_libraries_metadata` provider in `pyproject.toml`. The provider adds the exact shared version, which wheel
validation checks against the native package manifest. Native-only dependencies may be omitted from wheel metadata.
External Python dependencies may use supported version ranges.

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

Standalone, wheel, and Kit wheel-staging builds all configure the complete module graph from the top-level
`CMakeLists.txt`. CMake therefore remains the source of truth for native dependencies, including those omitted from
wheel metadata.
The wheel tool installs only `<group>-python` into a clean staging directory; the PEP 517 backend does not configure or
compile CMake. This component must contain the complete Python package, bindings, required same-package native
libraries, typing information, resources, and package manifest.

The PEP 517 backend creates `.dist-info`, chooses wheel tags, writes records, and builds the archive. The wheel tool
passes it a temporary staging directory at build time. Neither these helpers nor `pyproject.toml` contains a fixed
staging path.

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

This is enough to register a pure Python module. Its `python/` directory must contain `__init__.py`,
`impl/__init__.py`, and `py.typed`. Define `__all__` in the implementation initializer. The public initializer contains
only `from .impl import *` and `from .impl import __all__ as __all__`.

For development, the build copies the public initializer and typing marker and links `impl/` to the source directory.
Platforms without directory symlinks use a tracked copy instead. Installed packages always contain regular files.
Keep tests outside the package under `tests/python/`.

Use `PRESERVE_PACKAGE_LAYOUT` only when a migration must retain an existing leaf package unchanged. With this option,
the helper requires `__init__.py` and `py.typed` and stages the entire `python/` directory without requiring `impl/`.
New modules use the standard layout.

Pass `REQUIRES_BINDINGS` when importing a pure Python module immediately imports a native binding. The module is still
installed in every Python-enabled profile, but installation tests without bindings skip the import. Use this option
only for import-time requirements; optional or function-local binding use does not need it.

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

Register the compiled or header-only module first. `SOURCE` accepts one file; `SOURCES` accepts one or more.
`DEPENDENCIES` adds targets used only by the binding. `STUB_PRELOADS` lists nanobind modules to import before generating
stubs so cross-module C++ types keep their Python names.

Bindings use the stable CPython ABI by default. `NO_STABLE_ABI` requires a non-empty `STABLE_ABI_EXCEPTION` explaining
the exception. Generated `_bindings.pyi` files remain in the build tree and are installed beside the binary. On
Windows, use `WINDOWS_RUNTIME_SIBLING_DIRECTORY` only when an SDK runtime directory sits beside the binding runtime
directory; the helper copies it to `<module>/plugins`.

All module helpers apply the same source-path rule. For example, register `isaacsim.common.logging` from
`<ISAACSIM_LIBRARIES_DIR>/isaacsim/common/logging`. Under a flat distribution root, `MODULE_PREFIX` replaces the root
directory when deriving the logical name. A module must live below its distribution root, and registered module roots
cannot contain one another. Intermediate directories are namespaces only. Compatibility modules may install at a
different declared Python import without changing their logical identity. The `docs` directory directly below a
distribution is reserved for package documentation.

## Runtime payloads

`isaacsim_install_runtime_dependencies(MODULE <name> TARGETS ... [DESTINATION ...] [NATIVE_ONLY])` installs libraries
needed by the standalone native package and stages them with bindings. On platforms with SONAMEs, it installs both the
target file and the name used by the runtime loader. `DESTINATION` places libraries below the installed library
directory and mirrors that path under `isaacsim/lib` in Python packages. Pass only runtime-loadable library targets.
Use `NATIVE_ONLY` to keep a dependency out of Python packages.

`isaacsim_add_runtime_directory(MODULE <name> SOURCE ... DESTINATION ... [NATIVE_ONLY])` registers non-library data
such as schema registries. `SOURCE` may be absolute or module-relative; `DESTINATION` is relative to the installed
library directory
and must not contain `..`. `WINDOWS_BESIDE_MODULE` marks a payload the module resolves from its own directory, so on
Windows it installs below the binary directory the module's DLL occupies instead, at the same relative destination.
`WINDOWS_PYTHON_SHARED` keeps the Python payload under `isaacsim/lib` instead of
placing it beside each importing binding. Files placed under `resources/` have no effect until one of these helpers or a
test helper explicitly references them. `NATIVE_ONLY` prevents the directory from also being copied into Python
packages that use the module.

Libraries registered through `isaacsim_install_runtime_dependencies` also become imported targets in the installed
CMake package, so a consumer can locate them through `$<TARGET_RUNTIME_DLLS>` or `$<TARGET_FILE_DIR>` without naming
them itself. Only their locations are exported, because these libraries are loaded rather than linked.

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

Native test helpers accept only `.cpp` files from the matching `tests/c` or `tests/cpp` directory. Python test
registration always uses `tests/python`; do not place tests under the importable `python/` directory.

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
