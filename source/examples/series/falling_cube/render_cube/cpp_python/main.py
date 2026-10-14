# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run the C++ falling cube rendering example."""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import os
import sys
import tempfile
from pathlib import Path
from types import ModuleType

# A registered directory is revoked once the handle returned for it is collected.
_DLL_DIRECTORY_HANDLES: list[object] = []


def _ovrtx_binary_directory() -> Path | None:
    """Locate the native payload of the installed ovrtx distribution.

    Returns:
        Directory holding the ovrtx loader library, or ``None`` when ovrtx is absent.

    """
    specification = importlib.util.find_spec("ovrtx")
    locations = list(specification.submodule_search_locations or ()) if specification is not None else []
    return Path(locations[0]) / "bin" if locations else None


def _register_ovrtx_runtime() -> None:
    """Make the ovrtx renderer runtime discoverable on Windows.

    The ovrtx loader library resolves its renderer plugins relative to itself and stays
    inside its own installation, so it is reached through a registered directory rather
    than a copy. ovrtx registers that directory from its Python loader, which linking the
    library into an extension bypasses: Windows resolves the dependency while importing
    the extension, before any Python of ovrtx's could run. Linux uses an RPATH instead.
    """
    if sys.platform != "win32":
        return
    binary_directory = _ovrtx_binary_directory()
    if binary_directory is None:
        return
    # ovrtx resolves its plugins and USD schemas against these roots.
    os.environ.setdefault("OMNI_PLUGINS_BASE_PATH", str(binary_directory))
    os.environ.setdefault("OMNI_USD_PLUGINS_BASE_PATH", str(binary_directory))
    for directory in (binary_directory, binary_directory / "plugins"):
        if directory.is_dir():
            _DLL_DIRECTORY_HANDLES.append(os.add_dll_directory(str(directory)))


def _register_sdk_runtime() -> None:
    """Make the installed SDK discoverable on Windows.

    Windows resolves the extension's dependencies while importing it, before any Python
    here could redirect the search, and reuses a library already loaded under the same
    name. The SDK is read where it was installed rather than copied beside the extension,
    which would shadow the libraries the Python distributions load. The build recorded the
    directories, because nothing at runtime reveals which SDK the extension was linked
    against. Linux resolves the same libraries through an RPATH.

    Raises:
        RuntimeError: If a recorded directory is missing from this build tree.
    """
    if sys.platform != "win32":
        return

    import _render_cube_runtime

    # The build records the directories relative to the generated module, so a build tree that
    # moves after being built resolves them where it now sits.
    anchor = Path(_render_cube_runtime.__file__).parent
    for directory in _render_cube_runtime.RUNTIME_DIRECTORIES:
        resolved = (anchor / directory).resolve()
        if not resolved.is_dir():
            raise RuntimeError(f"The build recorded an SDK runtime directory this tree does not have: {resolved}")
        _DLL_DIRECTORY_HANDLES.append(os.add_dll_directory(str(resolved)))


def _load_extension() -> ModuleType:
    """Import the example extension module once its dependencies can be resolved.

    Returns:
        The example's C++ extension module.

    """
    # The Foundation OpenUSD backend resolves its OpenUSD symbols from the usd-core
    # libraries, which this import preloads into the process. The extension module links
    # that backend, so it can only be imported afterwards.
    import isaacsim.foundation.usd.openusd  # noqa: F401

    _register_sdk_runtime()
    _register_ovrtx_runtime()
    # Importing the engine package owns backend registration and process lifecycle, just
    # as it does in the Python examples.
    import isaacsim.physics_engines.ovphysx  # noqa: F401

    return importlib.import_module("_render_cube_cpp")


def _parse_arguments() -> argparse.Namespace:
    """Parse the command line.

    Returns:
        Parsed arguments.

    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Directory for the captured frames. Defaults to a new temporary directory.",
    )
    return parser.parse_args()


def main() -> None:
    """Simulate a falling cube in C++ and render each captured frame."""
    arguments = _parse_arguments()
    output_directory = arguments.output or Path(tempfile.mkdtemp(prefix="render_cube_cpp_"))
    output_directory.mkdir(parents=True, exist_ok=True)
    _load_extension().render_cube(str(output_directory))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
