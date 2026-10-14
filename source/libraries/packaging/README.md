# Library packaging

Python wheels and native archives use separate workflows because they follow different packaging standards.
This document uses the [terminology defined in the main libraries README](../README.md#terminology).

## Python wheels

Every shippable distribution has a `pyproject.toml` at its package root. This works for the normal two-directory layout,
such as `isaacsim/common/pyproject.toml`, and flat roots such as `isaacsim_deprecated/pyproject.toml`.

`pyproject.toml` defines the Python project name, version source, supported Python version, runtime dependencies, and
PEP 517 backend. CMake defines which files enter the wheel. `[build-system].requires` lists tools needed to build from
source, while `[project].dependencies` becomes the wheel's public runtime requirements.

List internal dependencies in the `isaacsim_libraries_metadata` provider, which adds exact requirements from the
shared `VERSION`. External requirements may use supported ranges. Use hyphenated PEP 508 names in Python metadata,
such as `isaacsim-common`, and underscore names in CMake selectors, such as `--group isaacsim_common`.

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

The default `locked` profile uses the exact versions selected for release builds. The `minimum` profile uses the
lowest supported version of each direct build and test tool. Ordinary builds never resolve floating versions; newer
versions enter through reviewed `pixi.lock` updates. These profiles affect building and testing, not wheel runtime
metadata.

Without `--group`, the tool builds a wheel for every package that declares a Python import. It skips native-only
packages. Explicitly selecting a native-only package is an error.

The standalone command configures and builds the complete module graph in
`_cmake_build/isaacsim-libraries-release`. Wheel assembly validates the CMake source directory, configuration, Python
executable, and required package options before installing anything. Rerun the matching standalone build after
changing compiled module sources or CMake configuration.

For each package, the wheel tool creates a unique temporary directory under
`_cmake_build/isaacsim-libraries-wheel-stage`, installs only the already-built `<group>-python` CMake component, and
strips staged ELF files for release artifacts with the toolchain selected by CMake. It then runs
`python -m build --wheel` without invoking CMake. Debug wheel stages retain their symbols. Scikit-build-core creates
the metadata, tags, records, layout, and archive. The tool validates the completed wheel before publishing it
atomically to the output directory and removes temporary files even after a failure. It can assemble separate packages
concurrently.

On non-Windows platforms, each distribution owns its native libraries, imported runtimes, and runtime directories. A
consumer wheel relies on its exact internal wheel requirements rather than repackaging dependency-owned payloads.
Windows wheels retain the complete binding-local DLL and data closure required by the platform loader. Linux and macOS
wheels store the runtime SONAME and omit development linker-name aliases; native SDK archives retain their normal
link-time layout. Wheel staging for Kit materializes manifest-declared runtime aliases from that SDK layout without
adding the aliases to general-consumer wheels.

Schema projects may declare named wheel variants in `[tool.isaacsim-library]`. Their standalone default installs the
matching `<group>-python-<variant>` component and receives a PEP 427 build tag such as `1usd2505`; extension manifests
select their required variant explicitly, such as `usd2511`. Schema wheels remain `py3-none-any` because their payload
contains Python and USD data rather than native code. Keep variants in separate wheelhouses for general pip consumers:
wheel build tags distinguish artifacts but do not express runtime compatibility to pip's resolver. Wheel staging
selects the exact version, build tag, and pure-wheel tag before exposing a one-wheel local index to pip.

Validation checks the distribution name and version, internal dependencies, imports, `py.typed` markers, namespace
layout, and package manifest. Internal wheel dependencies must also exist in the native package graph and use an
unconditional exact shared-version pin. Native-only dependencies may be omitted from wheel metadata. Wheels must not
contain source tests, caches, bytecode, or symlinks. `pyproject.toml` continues to own external Python requirements.

Use the wheel tool instead of running `python -m build` directly. It supplies the private CMake staging directory that
the backend needs; `pyproject.toml` intentionally contains no source or build directory paths.

For wheels with native code, the generated platform tag describes the build host. CI performs any additional repair
and validation required by a release standard such as manylinux. Do not edit wheel filenames or metadata tags by hand.

Validate a wheel by installing it into a clean environment without the source or build tree on `PYTHONPATH`, importing
each module in a fresh process, querying its version through `importlib.metadata`, and checking its installed files.
Fresh processes keep independently supported native backends from loading incompatible runtime variants together.
Source tests, caches, and symlinks must not be present.

The command writes wheels to `_cmake_build/isaacsim-libraries-artifacts/<configuration>` by default. It also writes
`wheel_sizes.json` beside the wheels with the compressed size, installed size, and file count of each artifact plus
wheelhouse totals and byte-identical duplicate content. The historical dashboard reports the total compressed wheel
size, clean install size, install time, source library build time, and the on-disk sizes of the source library and
example build trees. Backend scratch trees, CMake install stages, and generated artifacts are not part of a package's
source layout. Do not commit them.

Install a completed wheelhouse into a clean target, validate every declared import, and write installation metrics:

```text
./build.sh --validate-wheelhouse ../../_cmake_build/isaacsim-libraries-artifacts/release \
  --install-metrics-output ../../_cmake_build/isaacsim-libraries-artifacts/release/wheel_install_metrics.json
```

The metrics contain the installation duration, final installed file size and count, and validated package and import
counts. Wheel sizes are informational and do not fail a build.

Run the packaging unit tests without a compiled library build:

```text
PYTHONPATH=<python-test-deps> <library-python> -m unittest discover -s packaging/tests -v
```

Here, `<library-python>` is the locked interpreter in `.pixi/envs/build-driver`, and `<python-test-deps>` is the
site-packages directory in the selected locked or minimum Pixi test environment.

## Stage a wheel and SDK in a Kit extension

Use `--stage-wheel` when a Kit extension needs the Python wheel and compile-time native SDK for one distribution:

```text
./build.sh -r --test --stage-wheel isaacsim.common
```

The build configures the full native module graph, creates the wheel, and installs the selected distribution's runtime
and development components into the extension's stage. When several Kit extensions share a wheel, each extension may
select a non-overlapping set of Python leaf packages for its `pip_prebundle`. The build confirms that the wheel and SDK
have matching package manifests. Use the native archive workflow below to create a shippable SDK archive.

See [Stage a library wheel in a Kit extension](../README.md#stage-a-library-wheel-in-a-kit-extension) for the manifest,
root-build integration, dependency ownership, and validation rules.

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

The tool reads the artifact version from the installed manifest. You cannot override it, so artifact names always
match `source/libraries/VERSION`. Artifact names are
`<group>-<version>-<platform>-runtime.tar.gz` or `<group>-<version>-<platform>-sdk.tar.gz`. Native archives are ordinary
install prefixes. Validate an SDK by extracting it and calling `find_package(<group> CONFIG REQUIRED)` from a separate
consumer project. Each archive contains only the selected distribution. For internal dependencies, either extract
their archives into the same prefix or add their prefixes to `CMAKE_PREFIX_PATH`. Generated package configuration
finds and version-checks them instead of copying them into the dependent archive.
