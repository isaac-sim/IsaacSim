# Shared module testing infrastructure

This directory provides reusable test support. Module-specific tests belong under each module: put C API tests in
`tests/c`, C++ tests in `tests/cpp`, and Python tests in `tests/python`.

Both native test suites use doctest and are written in C++ so individual cases can be listed and filtered. C API tests
include only public C headers and call only C symbols. A separate installation test compiles a real C consumer to
verify C compatibility. This document uses the [terminology from the main libraries README](../README.md#terminology).

## Unit tests and install-contract tests

Module unit tests exercise code directly from the build tree and answer: "Does this API behave correctly?"

Python tests run from an isolated build directory linked to the module's `tests/python/` directory. This makes edits
available immediately without putting tests in the import package. Platforms without directory symlinks use a tracked
copy. Neither form is installed. Tests must use pytest temporary-directory fixtures instead of writing beside their
source files. Use `TIMEOUT <seconds>` only when a suite needs its own positive CTest timeout.

When several modules share domain-specific test helpers, keep those helpers in one module's `tests/python/` directory.
Other modules reference it with `isaacsim_add_python_tests(TEST_SUPPORT ...)`. The current suite appears first on
`PYTHONPATH`, followed by the declared support modules. This does not install test code or move module-specific helpers
into this shared directory.

An *installation contract test* uses the assembled package from outside the source tree. It answers: "Can another
project use what we ship?" It catches problems that build-tree unit tests cannot, such as:

- A public header or shared library was not installed.
- An installed header depends on an undeclared include path or on another header being included first.
- An exported CMake target contains a build-machine path or omits a dependency.
- `find_package(<group> CONFIG REQUIRED)` fails after the package is moved.
- An installed Python module cannot be imported without the build tree on `PYTHONPATH`.
- Native package metadata has the wrong version or dependency requirements.
- Source tests or cache files were accidentally included in the Python package.
- The shared `VERSION` or a module `CHANGELOG.md` was omitted from the documentation component.

Files under `install_contract/` are shared templates, not tests for module authors to customize. CMake fills in the
package name, targets, public headers, Python imports, and dependencies. Adding a module normally requires no changes
here.

Pure Python and compatibility modules declare `REQUIRES_BINDINGS` when importing them requires a native binding.
Installation tests without bindings skip these imports; tests with bindings still import every registered module. Both
profiles install the module files and dependencies.

## Install-contract lifecycle

For every complete distribution group, the test performs these steps:

1. Install the group's runtime, development, documentation, and, when enabled, Python components into isolated
   temporary prefixes.
2. Install the dependency-group closure into those prefixes and preserve every exact shared-version requirement.
3. Generate a small downstream CMake project from the templates in `install_contract/`.
4. Configure that project using only the installed prefix and `find_package`.
5. Compile each installed public C and C++ header in its own source file and link it to only its exported module target.
6. Run the generated native consumers.
7. Start a fresh Python process and import every installed Python module.
8. Verify native package identity, versions, dependencies, typing markers, changelogs, and package-content exclusions.

The temporary installation directory is the key boundary. Consumers cannot use module source directories, build-tree
include paths, or the staged development package.

## Files

### `cpp/DoctestMain.cpp`

Provides the shared doctest entry point that `isaacsim_add_c_tests` and `isaacsim_add_cpp_tests` add to every native
unit-test executable. Module test directories contain only test-case sources and must not provide their own `main` or
define `DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN`.

### `cpp/IsaacSimTest.hpp`

Provides a C++ utility shared by module doctest suites. `resolveResourcePath()` resolves a path below the test's
working directory and fails clearly when the resource is absent. A module opts into a resource working directory with
`isaacsim_add_cpp_tests(RESOURCE_DIR ...)`; the directory must contain `TEST_RESOURCES.md`.

This file supports ordinary module unit tests and is unrelated to the installed-package consumer described below.

### `install_contract/CMakeLists.txt.in`

Provides the template for the downstream CMake project. The generated project calls `find_package`, declares generated
C and C++ consumer executables, and links each consumer to the package's installed exported target.

For example, the `isaacsim_common` contract discovers the installed package and links logging consumers to
`isaacsim::common-logging`. Those names are generated from package registration; they are not written into the
template.

### `install_contract/main.c.in` and `install_contract/main.cpp.in`

Provide minimal `main` functions for the generated native executables. CMake also generates one source file per public
header. Keeping headers in separate translation units proves that each installed header is self-contained.

These executables stay small because they verify packaging, not behavior. Module unit tests provide behavioral
coverage.

### `install_contract/test_python.py.in`

Provides the fresh-process Python consumer. The generated script:

- Imports every Python module owned by the distribution.
- Verifies that typed modules install `py.typed`.
- Recursively rejects source-test directories, Python and tool caches, bytecode, and symlinks in each installed leaf
  package.

The raw CMake Python component is an importable prefix, not an installed Python distribution. Wheel-build validation
separately checks the backend-generated distribution version, dependencies, metadata, and archive contents. It allows
native-only dependencies to be absent from wheel metadata, while any internal wheel requirement must belong to the
native graph and use the exact shared version.

`cmake/tests/PackageDependencies.cmake` is a CMake-helper unit test rather than shared module test data. It creates
synthetic registered package identities in memory and checks exact shared-version derivation plus self, unknown,
duplicate, explicit-version, and mismatched-version failures before a second production package exists.

Python build and test compatibility uses two profiles. `locked` uses the versions selected for normal builds;
`minimum` uses each direct tool's lowest supported version. New versions enter only through a reviewed `pixi.lock`
update. Each profile has separate dependency and build directories. These profiles do not change package requirements
published in an artifact.

### `python/conftest.py`

Provides the shared pytest session configuration for all module Python test suites. It initializes Warp before any
test runs (to keep initialization messages out of test output), registers the `isaacsim` Hypothesis settings profile
(10 examples, suppressed function-scoped-fixture health check), and ensures the `python/` directory is on `sys.path`
so helpers in `isaacsim_test.py` are importable without an explicit install step.

### `python/isaacsim_test.py`

Provides small Python utilities shared by module pytest suites.

- `finite_float_elements()` — returns a `hypothesis.strategies.floats` keyword-argument dict restricted to finite
  `float32` values in a caller-supplied range. Use with `hypothesis.extra.numpy.arrays` to generate well-conditioned
  numeric inputs.
- `resolve_resource_path(path)` — resolves a path relative to the directory named by `ISAACSIM_TEST_RESOURCE_ROOT`.
  Fails clearly when the environment variable is unset or when the sentinel `TEST_RESOURCES.md` is absent. Mirrors
  the C++ `resolveResourcePath()` helper in `cpp/IsaacSimTest.hpp`.
- `check_array(a, *, shape, dtype, device)` — asserts that a Warp array (or list of arrays) matches expected spec.
- `check_allclose(a, b, *, rtol, atol)` — asserts element-wise proximity between pairs of Warp arrays or NumPy arrays.
- `check_equal(a, b)` — asserts exact equality for Warp/NumPy arrays, strings, and nested lists thereof.

### `examples/run_examples_test.py`

Provides the repository examples integration-test driver. It copies only authored example inputs into an isolated
test tree and invokes the public examples runner to build and test the selected manifests. It removes successful
scratch trees while preserving failed trees for diagnosis. On POSIX hosts, the installed-example runtime keeps the
invoking Python environment's native libraries ahead of the installed SDK libraries. This preserves one coherent
native dependency family when the interpreter's runtime path has already selected libraries such as GLVND.

## Implementation ownership

`cmake/IsaacSimTesting.cmake` supplies the shared doctest entry point to native unit-test targets and generates one
consumer project for each complete distribution group. `cmake/tests/Testing.cmake` verifies that legacy module-owned
`Main.cpp` sources are rejected before target generation.
`cmake/RunInstallContractTest.cmake` installs the package closure, runs the native and Python consumers, and validates
the installed manifest, shared version, and module changelogs. The nested consumer reuses the parent build's CMake
generator and build program, so the test does not introduce an undeclared Make or MSBuild dependency.

Change these templates only when the package contract changes for every module. Fix module-specific headers,
dependencies, packaging, or tests in the module itself instead of adding exceptions here.

## Related tooling and Kit extension tests

Tests for the standalone orchestration and artifact tools live under `source/libraries/tools/tests`, not in this shared
template directory. They cover supported host selection, pinned build tools, effective root-build environment
selection, root-build argument translation, wheel-staging manifest discovery, deterministic artifact selection, pip
installation semantics, and direct SDK-component staging.
Tests for wheel orchestration live under `source/libraries/packaging/tests`.

The normal `build.sh --test` or `build.bat --test` path runs the applicable suites. For a focused tooling run, use the
Python executable in `.pixi/envs/build-driver` and expose only the site-packages directory from `.pixi/envs/test`.

```text
PYTHONPATH=<python-test-deps> <module-python> -m unittest discover -s tools/tests -v
PYTHONPATH=<python-test-deps> <module-python> -m unittest discover -s packaging/tests -v
```

`<module-python>` is the locked build-driver interpreter. `<python-test-deps>` is the selected locked or minimum test
environment's site-packages directory; it supplies the packaging parser used by the wheel-tool tests.

Those tests verify orchestration without starting Kit. An extension that stages a library wheel owns its Kit startup
and backend integration tests under that extension's source tree. Together, the layers answer different questions:

- Module unit tests verify API behavior against the standalone runtime.
- Install-contract tests verify relocated native and Python package consumption, including exact dependency closure.
- Packaging and staging-tool tests verify artifact construction and staging rules.
- Kit extension tests verify extension dependency order and confirm that the host adapter connects to the
  host-owned runtime. Compatibility wrappers have their own API behavior tests.
