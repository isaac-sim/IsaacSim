# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Test the standalone library build entry point."""

from __future__ import annotations

import importlib.util
import os
import subprocess
import sys
import tempfile
import threading
import tomllib
import unittest
import xml.etree.ElementTree as element_tree
from pathlib import Path
from unittest import mock

from packaging.requirements import Requirement
from packaging.version import Version

BUILD_TOOL_PATH = Path(__file__).resolve().parents[1] / "build.py"
BUILD_TOOL_SPEC = importlib.util.spec_from_file_location("isaacsim_library_build_tool", BUILD_TOOL_PATH)
if BUILD_TOOL_SPEC is None or BUILD_TOOL_SPEC.loader is None:
    raise RuntimeError(f"Cannot load build tool from {BUILD_TOOL_PATH}")
build_tool = importlib.util.module_from_spec(BUILD_TOOL_SPEC)
sys.modules[BUILD_TOOL_SPEC.name] = build_tool
BUILD_TOOL_SPEC.loader.exec_module(build_tool)


class LegacyCarrierTests(unittest.TestCase):
    """Keep M1 commands and manifest inputs on the current staging path."""

    def test_legacy_cli_alias(self) -> None:
        """Select the same extension list through both documented options."""
        for option in ("--carrier", "--stage-wheel"):
            with mock.patch.object(sys, "argv", ["build.py", option, "isaacsim.common"]):
                self.assertEqual(build_tool._parse_arguments().stage_wheel, ["isaacsim.common"])

    def test_legacy_manifest_metadata(self) -> None:
        """Read the legacy distribution and payload selection together."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = root / "source/extensions/acme.runtime"
            extension.mkdir(parents=True)
            (extension / "module-carrier.toml").write_text(
                '[carrier]\ndistribution="acme_runtime"\nwheel-variant="usd2511"\n', encoding="utf-8"
            )
            with mock.patch.object(build_tool, "REPOSITORY_ROOT", root):
                self.assertEqual(build_tool._load_wheel_distribution("acme.runtime"), "acme_runtime")
                self.assertEqual(build_tool._load_wheel_variant("acme.runtime"), "usd2511")


class BuildMetricsTests(unittest.TestCase):
    """Test source build storage measurements."""

    @unittest.skipIf(os.name == "nt", "Symlink identity test requires Unix filesystem semantics")
    def test_directory_storage_counts_unique_regular_files(self) -> None:
        """Exclude symlinks and count hard-linked content once."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            original = root / "original.bin"
            original.write_bytes(b"x" * 100)
            os.link(original, root / "hard-link.bin")
            (root / "symbolic-link.bin").symlink_to(original)

            metrics = build_tool._directory_storage(root)

        self.assertEqual(metrics["logical_size_bytes"], 100)
        self.assertGreaterEqual(metrics["disk_size_bytes"], 100)
        self.assertEqual(metrics["file_count"], 1)


class HostPlatformTests(unittest.TestCase):
    """Test supported host and Packman platform selection."""

    def test_detect_host_platform_supported_hosts(self) -> None:
        """Map every supported host to the expected Packman platform."""
        cases = (
            ("win32", "AMD64", "windows-x86_64", "windows-x86_64", ".exe"),
            ("linux", "x86_64", "linux-x86_64", "manylinux_2_35_x86_64", ""),
            ("linux", "aarch64", "linux-aarch64", "manylinux_2_35_aarch64", ""),
        )
        for system, machine, name, packman_platform, executable_suffix in cases:
            with self.subTest(system=system, machine=machine):
                with mock.patch.object(build_tool.sys, "platform", system):
                    with mock.patch.object(build_tool.platform, "machine", return_value=machine):
                        host = build_tool._detect_host_platform()
                self.assertEqual(host.name, name)
                self.assertEqual(host.packman_platform, packman_platform)
                self.assertEqual(host.executable_suffix, executable_suffix)

    def test_detect_host_platform_rejects_unsupported_host(self) -> None:
        """Reject an operating system outside the build contract."""
        with mock.patch.object(build_tool.sys, "platform", "darwin"):
            with mock.patch.object(build_tool.platform, "machine", return_value="arm64"):
                with self.assertRaisesRegex(RuntimeError, "Unsupported library build host"):
                    build_tool._detect_host_platform()


class WindowsCompilerEnvironmentTests(unittest.TestCase):
    """Test deterministic MSVC selection on Windows build hosts."""

    def test_existing_msvc_compiler_avoids_environment_setup(self) -> None:
        """Reuse an already initialized Visual Studio developer environment."""
        compiler = Path(r"C:\VS\bin\cl.exe")
        with (
            mock.patch.object(build_tool.sys, "platform", "win32"),
            mock.patch.object(build_tool, "WINDOWS_TOOLCHAIN_DIRECTORY", Path(r"Z:\missing-host-dependencies")),
            mock.patch.dict(build_tool.os.environ, {"INCLUDE": "include", "LIB": "lib"}, clear=False),
            mock.patch.object(build_tool.shutil, "which", return_value=str(compiler)),
            mock.patch.object(build_tool.subprocess, "run") as run,
        ):
            selected = build_tool.ensure_windows_msvc_environment()

        self.assertEqual(selected, compiler.resolve())
        run.assert_not_called()

    def test_packaged_msvc_precedes_ambient_compiler(self) -> None:
        """Accept the pinned compiler when optional setup makes its activation return nonzero."""
        installation = Path("repo") / "_build" / "host-deps" / "msvc"
        vsdevcmd = installation / "Common7" / "Tools" / "VsDevCmd.bat"
        ambient_compiler = Path(r"C:\Visual Studio\VC\Tools\MSVC\bin\HostX64\x64\cl.exe")
        compiler = installation / "VC" / "Tools" / "MSVC" / "bin" / "HostX64" / "x64" / "cl.exe"
        completed = subprocess.CompletedProcess(
            [],
            255,
            stdout="Path=msvc-path\nInclude=msvc-include\nLib=msvc-lib\n",
            stderr="",
        )

        with (
            mock.patch.object(build_tool.sys, "platform", "win32"),
            mock.patch.object(build_tool, "WINDOWS_TOOLCHAIN_DIRECTORY", installation.parent),
            mock.patch.dict(
                build_tool.os.environ,
                {"VSINSTALLDIR": r"C:\Program Files\Microsoft Visual Studio\2022\Enterprise"},
                clear=False,
            ),
            mock.patch.object(build_tool.Path, "is_file", return_value=True),
            mock.patch.object(build_tool, "_has_packaged_windows_sdk", return_value=True),
            mock.patch.object(
                build_tool.shutil,
                "which",
                side_effect=(str(ambient_compiler), str(compiler)),
            ) as which,
            mock.patch.object(build_tool.subprocess, "run", return_value=completed) as run,
            mock.patch.object(build_tool, "_activate_packaged_windows_sdk") as activate_sdk,
            mock.patch.object(build_tool, "activate_packaged_windows_debug_runtime") as activate_debug_runtime,
        ):
            selected = build_tool.ensure_windows_msvc_environment(require_debug_runtime=True)
            activated_path = build_tool._windows_environment_value(build_tool.os.environ, "PATH")

        self.assertEqual(selected, compiler.resolve())
        self.assertEqual(len(run.call_args_list), 1)
        environment_command = run.call_args.args[0]
        self.assertIn(str(vsdevcmd), environment_command)
        self.assertIn(" >nul & set", environment_command)
        self.assertFalse(any(name.casefold() == "vsinstalldir" for name in run.call_args.kwargs["env"]))
        self.assertFalse(run.call_args.kwargs["check"])
        self.assertEqual(activated_path, "msvc-path")
        activate_sdk.assert_called_once_with(installation.parent / "winsdk")
        activate_debug_runtime.assert_called_once_with(installation, installation.parent / "winsdk")
        self.assertEqual(which.call_args_list, [mock.call("cl.exe"), mock.call("cl.exe", path="msvc-path")])

    def test_packaged_msvc_failure_preserves_and_reuses_ambient_environment(self) -> None:
        """Fall back to Rattler's active compiler without leaking partial packaged activation state."""
        installation = Path("repo") / "_build" / "host-deps" / "msvc"
        ambient_compiler = Path(r"C:\Visual Studio\VC\Tools\MSVC\bin\HostX64\x64\cl.exe")
        completed = subprocess.CompletedProcess(
            [],
            255,
            stdout="Path=partial-packaged-path\nINCLUDE=partial-packaged-include\n",
            stderr="",
        )

        with (
            mock.patch.object(build_tool.sys, "platform", "win32"),
            mock.patch.object(build_tool, "WINDOWS_TOOLCHAIN_DIRECTORY", installation.parent),
            mock.patch.dict(
                build_tool.os.environ,
                {"PATH": "ambient-path", "INCLUDE": "ambient-include", "LIB": "ambient-lib"},
                clear=False,
            ),
            mock.patch.object(build_tool.Path, "is_file", return_value=True),
            mock.patch.object(build_tool, "_has_packaged_windows_sdk", return_value=True),
            mock.patch.object(
                build_tool.shutil,
                "which",
                side_effect=(str(ambient_compiler), str(installation / "VC" / "bin" / "cl.exe")),
            ),
            mock.patch.object(build_tool.subprocess, "run", return_value=completed),
            mock.patch.object(build_tool, "_activate_packaged_windows_sdk") as activate_sdk,
        ):
            selected = build_tool.ensure_windows_msvc_environment()

            self.assertEqual(build_tool.os.environ["PATH"], "ambient-path")
            self.assertEqual(build_tool.os.environ["INCLUDE"], "ambient-include")

        self.assertEqual(selected, ambient_compiler.resolve())
        activate_sdk.assert_not_called()

    def test_packaged_msvc_failure_reports_exit_code_without_fallback(self) -> None:
        """Report the activation status when neither the pinned nor ambient compiler is usable."""
        installation = Path("repo") / "_build" / "host-deps" / "msvc"
        completed = subprocess.CompletedProcess([], 255, stdout="Path=broken-path\n", stderr="activation failed")

        with (
            mock.patch.object(build_tool.sys, "platform", "win32"),
            mock.patch.object(build_tool, "WINDOWS_TOOLCHAIN_DIRECTORY", installation.parent),
            mock.patch.dict(build_tool.os.environ, {"INCLUDE": "", "LIB": ""}, clear=False),
            mock.patch.object(build_tool.Path, "is_file", return_value=True),
            mock.patch.object(build_tool, "_has_packaged_windows_sdk", return_value=True),
            mock.patch.object(
                build_tool.shutil,
                "which",
                side_effect=(str(installation / "ambient" / "cl.exe"), None),
            ),
            mock.patch.object(build_tool.subprocess, "run", return_value=completed),
        ):
            with self.assertRaisesRegex(RuntimeError, "initialization exit code: 255"):
                build_tool.ensure_windows_msvc_environment()

    def test_packaged_windows_debug_runtimes_are_exposed_to_stub_generation(self) -> None:
        """Make the portable Debug CRT and UCRT discoverable during isolated imports."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            toolchain_root = Path(temporary_directory) / "msvc"
            sdk_root = Path(temporary_directory) / "winsdk"
            debug_runtime = (
                toolchain_root
                / "VC"
                / "Redist"
                / "MSVC"
                / "14.44.35207"
                / "debug_nonredist"
                / "x64"
                / "Microsoft.VC143.DebugCRT"
            )
            debug_runtime.mkdir(parents=True)
            debug_ucrt = sdk_root / "bin" / "x64" / "ucrt"
            debug_ucrt.mkdir(parents=True)
            (debug_ucrt / "ucrtbased.dll").touch()
            with mock.patch.dict(build_tool.os.environ, {"PATH": "runner-path"}, clear=False):
                build_tool.activate_packaged_windows_debug_runtime(toolchain_root, sdk_root)

                self.assertEqual(build_tool.os.environ["PATH"], f"{debug_runtime};{debug_ucrt};runner-path")
                self.assertEqual(
                    build_tool.os.environ[build_tool.WINDOWS_DEBUG_RUNTIME_PATH_VARIABLE],
                    os.pathsep.join((str(debug_runtime), str(debug_ucrt))),
                )

    def test_packaged_windows_sdk_adds_x64_include_library_and_tool_paths(self) -> None:
        """Compose the flat Packman SDK layout with the initialized MSVC environment."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            sdk_root = Path(temporary_directory) / "winsdk"
            include_paths = tuple(sdk_root / "include" / name for name in ("ucrt", "um", "shared"))
            library_paths = tuple(sdk_root / "lib" / name / "x64" for name in ("ucrt", "um"))
            binary_path = sdk_root / "bin" / "x64"
            for path in (*include_paths, *library_paths, binary_path):
                path.mkdir(parents=True, exist_ok=True)
            (library_paths[1] / "kernel32.lib").touch()
            for executable in ("rc.exe", "mt.exe"):
                (binary_path / executable).touch()

            with mock.patch.dict(
                build_tool.os.environ,
                {"PATH": "runner-path", "INCLUDE": "msvc-include", "LIB": "msvc-lib"},
                clear=False,
            ):
                build_tool._activate_packaged_windows_sdk(sdk_root)

                self.assertEqual(build_tool.os.environ["PATH"], f"{binary_path};runner-path")
                self.assertEqual(
                    build_tool.os.environ["INCLUDE"],
                    ";".join((*map(str, include_paths), "msvc-include")),
                )
                self.assertEqual(
                    build_tool.os.environ["LIB"],
                    ";".join((*map(str, library_paths), "msvc-lib")),
                )
                self.assertEqual(build_tool.os.environ["WindowsSdkDir"], f"{sdk_root}\\")

    def test_vswhere_environment_provides_x64_msvc(self) -> None:
        """Import VsDevCmd output when CI exposes LLVM but not cl.exe on PATH."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            toolchain_directory = root / "host-deps"
            packaged_vsdevcmd = toolchain_directory / "msvc" / "Common7" / "Tools" / "VsDevCmd.bat"
            packaged_vsdevcmd.parent.mkdir(parents=True)
            packaged_vsdevcmd.touch()
            (toolchain_directory / "winsdk" / "x64" / "ucrt").mkdir(parents=True)
            vswhere = root / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
            vswhere.parent.mkdir(parents=True)
            vswhere.touch()
            installation = root / "Visual Studio" / "2022" / "Professional"
            vsdevcmd = installation / "Common7" / "Tools" / "VsDevCmd.bat"
            vsdevcmd.parent.mkdir(parents=True)
            vsdevcmd.touch()
            compiler = installation / "VC" / "Tools" / "MSVC" / "bin" / "HostX64" / "x64" / "cl.exe"

            completed = (
                subprocess.CompletedProcess([], 0, stdout=f"{installation}\n", stderr=""),
                subprocess.CompletedProcess([], 0, stdout="PATH=msvc-path\nINCLUDE=msvc-include\n", stderr=""),
            )
            with (
                mock.patch.object(build_tool.sys, "platform", "win32"),
                mock.patch.object(build_tool, "WINDOWS_TOOLCHAIN_DIRECTORY", toolchain_directory),
                mock.patch.dict(build_tool.os.environ, {"ProgramFiles(x86)": str(root)}, clear=False),
                mock.patch.object(build_tool.shutil, "which", side_effect=(None, str(compiler))),
                mock.patch.object(build_tool.subprocess, "run", side_effect=completed) as run,
                mock.patch.object(build_tool, "_activate_packaged_windows_sdk") as activate_sdk,
            ):
                selected = build_tool.ensure_windows_msvc_environment()

                self.assertEqual(build_tool.os.environ["PATH"], "msvc-path")
                self.assertEqual(build_tool.os.environ["INCLUDE"], "msvc-include")
                activate_sdk.assert_not_called()

        self.assertEqual(selected, compiler.resolve())
        vswhere_command = run.call_args_list[0].args[0]
        self.assertIn("[17.0,18.0)", vswhere_command)
        environment_command = run.call_args_list[1].args[0]
        self.assertIsInstance(environment_command, str)
        self.assertIn(str(vsdevcmd), environment_command)
        self.assertNotIn(" /s ", environment_command)
        self.assertFalse(run.call_args_list[1].kwargs["check"])

    def test_configure_forces_resolved_msvc_compiler(self) -> None:
        """Prevent an earlier clang.exe entry on PATH from winning CMake detection."""
        compiler = Path(r"C:\VS\bin\cl.exe")
        with mock.patch.object(build_tool, "_run") as run:
            build_tool._configure(
                Path(r"C:\cmake.exe"),
                Path(r"C:\ninja.exe"),
                None,
                Path(r"C:\build"),
                "Release",
                build_tool._BUILD_PROFILES["cpp-library"],
                Path(r"C:\native-runtime"),
                None,
                None,
                None,
                Path(r"C:\pixi-build"),
                Path(r"C:\licenses\PIP-LICENSES.txt"),
                msvc_compiler=compiler,
            )

        command = run.call_args.args[0]
        self.assertIn(f"-DCMAKE_C_COMPILER={compiler}", command)
        self.assertIn(f"-DCMAKE_CXX_COMPILER={compiler}", command)
        self.assertIn(r"-DISAACSIM_PUBLIC_DEPS_ROOT=C:\pixi-build", command)
        self.assertIn(r"-DISAACSIM_NATIVE_RUNTIME_DEPS_DIR=C:\native-runtime", command)
        self.assertIn(r"-DISAACSIM_COLLECTED_PYTHON_LICENSE_FILE=C:\licenses\PIP-LICENSES.txt", command)


class LinuxCompilerTests(unittest.TestCase):
    """Keep native and CUDA compilation on the provisioned Linux toolchain."""

    def test_configure_selects_pixi_compilers_over_ambient_environment(self) -> None:
        """Ignore a caller's compiler selection for C, C++, and CUDA host builds."""
        with (
            mock.patch.dict(build_tool.os.environ, {"CC": "/usr/bin/cc", "CXX": "/usr/bin/c++"}),
            mock.patch.object(build_tool.sys, "platform", "linux"),
            mock.patch.object(build_tool.platform, "machine", return_value="x86_64"),
            mock.patch.object(build_tool, "_run") as run,
        ):
            build_tool._configure(
                Path("/cmake"),
                Path("/ninja"),
                None,
                Path("/build"),
                "Release",
                build_tool._BUILD_PROFILES["cpp-library"],
                Path("/runtime"),
                None,
                None,
                None,
                Path("/pixi"),
                Path("/licenses"),
            )
        command = run.call_args.args[0]
        gcc = Path("/pixi/bin/x86_64-conda-linux-gnu-gcc")
        gxx = Path("/pixi/bin/x86_64-conda-linux-gnu-g++")
        self.assertIn(f"-DCMAKE_C_COMPILER={gcc}", command)
        self.assertIn(f"-DCMAKE_CXX_COMPILER={gxx}", command)
        self.assertIn(f"-DCMAKE_CUDA_HOST_COMPILER={gxx}", command)

    def test_changed_linux_toolchain_invalidates_configure_context(self) -> None:
        """Do not reuse discovery results from a different compiler prefix."""
        with mock.patch.object(build_tool.sys, "platform", "linux"):
            old = build_tool._configure_context({"ISAACSIM_PIXI_BUILD_PREFIX": "/old"})
            new = build_tool._configure_context({"ISAACSIM_PIXI_BUILD_PREFIX": "/new"})
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_dir = Path(temporary_directory)
            (build_dir / "CMakeCache.txt").touch()
            build_tool._write_json_atomically(build_dir / build_tool.CONFIGURE_CONTEXT_FILE, old)
            self.assertIsNotNone(build_tool._configuration_refresh_reason(build_dir, new))

    def test_changed_lock_refreshes_compiler_at_same_prefix(self) -> None:
        """Refresh compiler discovery after an in-place locked toolchain update."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            lock = root / "pixi.lock"
            environment = {"ISAACSIM_PIXI_BUILD_PREFIX": "/pixi"}
            with (
                mock.patch.object(build_tool, "REPOSITORY_ROOT", root),
                mock.patch.object(build_tool.sys, "platform", "linux"),
            ):
                lock.write_text("old toolchain", encoding="utf-8")
                old = build_tool._configure_context(environment)
                (root / "CMakeCache.txt").touch()
                build_tool._write_json_atomically(root / build_tool.CONFIGURE_CONTEXT_FILE, old)
                self.assertIsNone(build_tool._configuration_refresh_reason(root, old))
                lock.write_text("updated toolchain", encoding="utf-8")
                new = build_tool._configure_context(environment)
            self.assertIsNotNone(build_tool._configuration_refresh_reason(root, new))


class SubprocessEnvironmentTests(unittest.TestCase):
    """Test environment isolation between Pixi and Packman subprocesses."""

    def test_run_can_remove_inherited_environment_variable(self) -> None:
        """Remove a Pixi loader path before launching a tool with its own runtime."""
        with (
            mock.patch.dict(build_tool.os.environ, {"LD_LIBRARY_PATH": "/pixi/lib"}, clear=False),
            mock.patch.object(build_tool.subprocess, "run") as run,
        ):
            build_tool._run(["command"], environment={"LD_LIBRARY_PATH": None})

        environment = run.call_args.kwargs["env"]
        self.assertNotIn("LD_LIBRARY_PATH", environment)

    def test_packman_pull_removes_pixi_loader_path(self) -> None:
        """Keep Packman's bundled Python independent from the Pixi Python runtime."""
        host = build_tool._HostPlatform("linux-x86_64", "manylinux_2_35_x86_64", "")
        with mock.patch.object(build_tool, "_run") as run:
            build_tool._pull_dependencies(host)

        self.assertEqual(run.call_args.kwargs["environment"], {"LD_LIBRARY_PATH": None})


class BuildDirectoryTests(unittest.TestCase):
    """Test stable build-tree naming used by packaging commands."""

    def test_staged_wheel_artifacts_share_the_library_build_root(self) -> None:
        """Keep wheel staging artifacts outside Repo's application clean root."""
        self.assertEqual(
            build_tool.EXTENSION_WHEEL_ARTIFACT_ROOT,
            build_tool.LIBRARY_BUILD_ROOT / "extension-wheel-artifacts",
        )

    def test_get_build_directory_preserves_release_profile_contract(self) -> None:
        """Use established directory names for standard and native release builds."""
        self.assertEqual(
            build_tool._get_build_directory("standard", "Release").name,
            "isaacsim-libraries-release",
        )
        self.assertEqual(
            build_tool._get_build_directory("cpp-library", "Release").name,
            "isaacsim-libraries-cpp-library",
        )
        self.assertEqual(
            build_tool._get_build_directory("cpp-library", "Debug").name,
            "isaacsim-libraries-cpp-library-debug",
        )
        self.assertEqual(
            build_tool._get_build_directory("standard", "Release", "minimum").name,
            "isaacsim-libraries-release-minimum",
        )


class WheelStagingMetadataTests(unittest.TestCase):
    """Test wheel staging metadata loading used by root builds."""

    def test_load_wheel_metadata_with_pinned_python(self) -> None:
        """Read distribution and variant metadata from a wheel staging manifest."""
        extension = "isaacsim.robot_motion.schema"

        self.assertEqual(build_tool._load_wheel_distribution(extension), "isaacsim_robot_motion_schema")
        self.assertEqual(build_tool._load_wheel_variant(extension), "usd2511")


class ConfigureTests(unittest.TestCase):
    """Test CMake configuration arguments."""

    def test_configure_enables_native_coverage(self) -> None:
        """Forward coverage selection to the CMake library build."""
        with mock.patch.object(build_tool, "_run") as run:
            build_tool._configure(
                Path("/cmake"),
                Path("/ninja"),
                Path("/python"),
                Path("/build"),
                "Release",
                build_tool._BuildProfile(False, True, True),
                Path("/native-runtime"),
                None,
                None,
                None,
                Path("/pixi-build"),
                Path("/licenses/PIP-LICENSES.txt"),
                coverage_enabled=True,
            )

        self.assertIn("-DISAACSIM_ENABLE_COVERAGE=ON", run.call_args.args[0])
        self.assertIn(f"-DISAACSIM_PUBLIC_DEPS_ROOT={Path('/pixi-build')}", run.call_args.args[0])
        self.assertIn(f"-DISAACSIM_NATIVE_RUNTIME_DEPS_DIR={Path('/native-runtime')}", run.call_args.args[0])
        self.assertIn(
            f"-DISAACSIM_COLLECTED_PYTHON_LICENSE_FILE={Path('/licenses/PIP-LICENSES.txt')}",
            run.call_args.args[0],
        )

    def test_configure_can_discard_stale_cache(self) -> None:
        """Request CMake's targeted cache reset without deleting build outputs."""
        with mock.patch.object(build_tool, "_run") as run:
            build_tool._configure(
                Path("/cmake"),
                Path("/ninja"),
                None,
                Path("/build"),
                "Release",
                build_tool._BUILD_PROFILES["cpp-library"],
                Path("/native-runtime"),
                None,
                None,
                None,
                Path("/pixi-build"),
                Path("/licenses/PIP-LICENSES.txt"),
                fresh=True,
            )

        self.assertEqual(run.call_args.args[0][:2], [Path("/cmake"), "--fresh"])


class ConfigureContextTests(unittest.TestCase):
    """Test safe reuse of CMake caches across build environments."""

    def test_context_distinguishes_linbuild_from_host(self) -> None:
        """Do not share system-library results between the host and linbuild."""
        host = build_tool._configure_context({})
        linbuild = build_tool._configure_context({"LINBUILD_EMBEDDED": "1"})

        self.assertEqual(host["execution_environment"], "host")
        self.assertEqual(linbuild["execution_environment"], "linbuild")

    def test_existing_unrecorded_cache_is_refreshed(self) -> None:
        """Safely migrate build trees created before context tracking."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            (build_directory / "CMakeCache.txt").touch()

            reason = build_tool._configuration_refresh_reason(build_directory, build_tool._configure_context({}))

        self.assertIn("not recorded", reason or "")

    def test_missing_cached_file_is_refreshed(self) -> None:
        """Recover when a cached host dependency is absent in the active environment."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            context = build_tool._configure_context({})
            build_tool._write_json_atomically(build_directory / build_tool.CONFIGURE_CONTEXT_FILE, context)
            missing = build_directory / "missing.so"
            (build_directory / "CMakeCache.txt").write_text(
                f"GL_DISPATCH:FILEPATH={missing}\nIGNORED:FILEPATH=IGNORED-NOTFOUND\n",
                encoding="utf-8",
            )

            reason = build_tool._configuration_refresh_reason(build_directory, context)

        self.assertIn(str(missing), reason or "")


class PixiManifestTests(unittest.TestCase):
    """Test the standalone library dependency ownership contract."""

    def test_pixi_license_file_must_be_nonempty(self) -> None:
        """Reject a missing or empty launcher-selected compliance aggregate."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            license_file = Path(temporary_directory) / "PIP-LICENSES.txt"
            with mock.patch.dict(
                build_tool.os.environ,
                {build_tool.PIXI_LICENSE_FILE_VARIABLE: str(license_file)},
                clear=False,
            ):
                with self.assertRaisesRegex(RuntimeError, "missing or empty"):
                    build_tool._read_pixi_license_file()
                license_file.write_text("dependency notice\n", encoding="utf-8")
                self.assertEqual(build_tool._read_pixi_license_file(), license_file.resolve())

    def test_pixi_owns_public_build_tools_and_python(self) -> None:
        """Keep migrated tools exact and out of the Packman graph."""
        with (build_tool.REPOSITORY_ROOT / "pixi.toml").open("rb") as stream:
            pixi = tomllib.load(stream)

        build_tools = pixi["feature"]["build-tools"]
        dependency_groups = {
            "python": pixi["feature"]["python"]["dependencies"],
            "build-tools": build_tools["dependencies"],
        }
        workspace_platforms = {
            platform if isinstance(platform, str) else platform["name"] for platform in pixi["workspace"]["platforms"]
        }
        self.assertLessEqual(set(build_tools["target"]), workspace_platforms)
        for platform, configuration in build_tools["target"].items():
            dependency_groups[f"build-tools.target.{platform}"] = configuration["dependencies"]

        pixi_owned_dependencies = set()
        for group_name, dependencies in dependency_groups.items():
            for name, requirement in dependencies.items():
                with self.subTest(group=group_name, dependency=name):
                    pixi_owned_dependencies.add(name.replace("_", "-"))
                    if isinstance(requirement, str):
                        self.assertRegex(requirement, r"^==[^=].+", "dependency must use an exact version pin")
                    else:
                        self.assertEqual(set(requirement), {"path"})
                        recipe_path = build_tool.REPOSITORY_ROOT / requirement["path"]
                        self.assertTrue(recipe_path.is_file(), f"local recipe does not exist: {recipe_path}")

        self.assertNotIn("pypi-dependencies", build_tools)
        self.assertNotIn("solve-group", pixi["environments"]["build-driver"])
        packman = element_tree.parse(build_tool.DEPENDENCY_MANIFEST).getroot()
        for name in sorted(pixi_owned_dependencies):
            self.assertIsNone(packman.find(f"./dependency[@name='{name}']"))

    def test_source_recipes_are_checksum_pinned_and_limited_to_approved_packages(self) -> None:
        """Use source recipes only for reviewed portability or availability exceptions."""
        lock = (build_tool.REPOSITORY_ROOT / "pixi.lock").read_text(encoding="utf-8")
        self.assertIn("conda_source: ovphysx-sdk", lock)
        self.assertIn("conda_source: stb", lock)
        self.assertNotIn("conda_source: sdl3", lock)
        for platform in ("linux-64", "linux-aarch64", "win-64"):
            self.assertIn(f"conda-forge/{platform}/sdl3-3.4.14-", lock)
        self.assertNotIn("conda_source: doctest", lock.split("win-64:", maxsplit=1)[1].split("default:", maxsplit=1)[0])
        recipe_directory = build_tool.REPOSITORY_ROOT / "deps" / "recipes"
        self.assertEqual(
            sorted(
                path.relative_to(recipe_directory).as_posix() for path in recipe_directory.rglob("*") if path.is_file()
            ),
            [
                "doctest/recipe.yaml",
                "ovphysx-sdk/install.py",
                "ovphysx-sdk/recipe.yaml",
                "stb/install.py",
                "stb/recipe.yaml",
            ],
        )
        doctest_recipe = (recipe_directory / "doctest" / "recipe.yaml").read_text(encoding="utf-8")
        self.assertIn("https://github.com/doctest/doctest/archive/refs/tags/v${{ version }}.zip", doctest_recipe)
        self.assertIn("5ddeafa34bece65b9f2e5207186fa70cd8ca44c00f41ae2c25e6e50b427c2b79", doctest_recipe)
        ovphysx_recipe = (recipe_directory / "ovphysx-sdk" / "recipe.yaml").read_text(encoding="utf-8")
        self.assertIn("releases/download/ovphysx-${{ version }}", ovphysx_recipe)
        self.assertIn("e112b3e7ce608f85d519d8b755632a494d0f89cf2f8afb14f31833c215e1fc37", ovphysx_recipe)
        self.assertIn("54f1a322cf049d5c10a694429dbd80192ebcb6dba60d0e728ff240f0bd821233", ovphysx_recipe)
        self.assertIn("baaaf6981e52b39f29d6498524fae39a2e3a17fa5225bd6db1fc0a5fe88dea92", ovphysx_recipe)
        stb_recipe = (recipe_directory / "stb" / "recipe.yaml").read_text(encoding="utf-8")
        self.assertIn("2c980bb59875b0d32144a71867fbdebb2f77cd20.tar.gz", stb_recipe)
        self.assertIn("9a955b1b49a4410088a2e0ee2a9c057c3c907d0c1d75454144cb980aca0ba515", stb_recipe)

    def test_shared_schema_dependencies_are_not_duplicated(self) -> None:
        """Share overlapping Python package pins between native and Python runtimes."""
        with (build_tool.REPOSITORY_ROOT / "pixi.toml").open("rb") as stream:
            pixi = tomllib.load(stream)

        shared = pixi["feature"]["schema-runtime"]["pypi-dependencies"]
        self.assertEqual(
            shared,
            {
                "ovstage": {"version": "==0.2.0.377349", "index": "https://pypi.org/simple"},
                "physx-usd-schemas": "==25.11.1",
            },
        )
        for feature_name in ("runtime", "native-runtime", "minimum-runtime"):
            dependencies = pixi["feature"][feature_name]["pypi-dependencies"]
            self.assertTrue(shared.keys().isdisjoint(dependencies))

    def test_library_newton_schema_requirements_accept_locked_version(self) -> None:
        """Keep every library distribution compatible with the locked Newton schemas."""
        with (build_tool.REPOSITORY_ROOT / "pixi.toml").open("rb") as stream:
            pixi = tomllib.load(stream)

        locked_specifier = pixi["feature"]["runtime"]["pypi-dependencies"]["newton-usd-schemas"]
        self.assertRegex(locked_specifier, r"^==[^=,]+$", "Newton schema runtime dependency must be exact")
        locked_version = Version(locked_specifier.removeprefix("=="))

        libraries = build_tool.REPOSITORY_ROOT / "source" / "libraries"
        requirements: dict[Path, Requirement] = {}
        for pyproject_path in sorted(libraries.glob("*/**/pyproject.toml")):
            with pyproject_path.open("rb") as stream:
                project = tomllib.load(stream).get("project", {})
            for dependency in project.get("dependencies", []):
                requirement = Requirement(dependency)
                if requirement.name == "newton-usd-schemas":
                    requirements[pyproject_path] = requirement

        self.assertTrue(requirements)
        for pyproject_path, requirement in requirements.items():
            with self.subTest(pyproject=pyproject_path.relative_to(libraries)):
                self.assertIn(locked_version, requirement.specifier)

    def test_usd_optimize_uses_public_minimum_version(self) -> None:
        """Use the publicly available minimum supported `usd-optimize` release."""
        with (build_tool.REPOSITORY_ROOT / "pixi.toml").open("rb") as stream:
            pixi = tomllib.load(stream)

        expected = {"version": "==1.1.0", "index": "https://pypi.org/simple"}
        for feature_name in ("runtime", "minimum-runtime"):
            usd_optimize = pixi["feature"][feature_name]["pypi-dependencies"]["usd-optimize"]
            self.assertEqual(usd_optimize, expected)

        lock = (build_tool.REPOSITORY_ROOT / "pixi.lock").read_text(encoding="utf-8")
        usd_optimize_wheels = {
            line.strip()
            for line in lock.splitlines()
            if line.lstrip().startswith("- pypi: ") and "/usd_optimize-" in line
        }
        self.assertTrue(usd_optimize_wheels)
        for wheel in usd_optimize_wheels:
            self.assertTrue(wheel.startswith("- pypi: https://files.pythonhosted.org/"), wheel)
            self.assertIn("/usd_optimize-1.1.0-", wheel)


class DeveloperEnvironmentBuildTests(unittest.TestCase):
    """Test creation of the complete build-owned developer environment."""

    def _write_package(self, build_directory: Path, name: str) -> tuple[str, ...]:
        """Write one complete package manifest and return its components.

        Args:
            build_directory: Directory containing the CMake build.
            name: Name of the item.

        Returns:
            The resulting value.
        """
        components = tuple(f"{name}-{surface}" for surface in build_tool.DEVELOPER_ENVIRONMENT_SURFACES)
        package_directory = build_directory / "packages" / name
        package_directory.mkdir(parents=True)
        package_directory.joinpath("package.json").write_text(
            "{\n"
            f'  "name": "{name}",\n'
            '  "complete": true,\n'
            '  "components": {\n'
            f'    "runtime": "{components[0]}",\n'
            f'    "development": "{components[1]}",\n'
            f'    "python": "{components[2]}"\n'
            "  }\n"
            "}\n",
            encoding="utf-8",
        )
        return components

    def test_materialize_developer_environment_installs_every_surface_and_replaces_stale_tree(self) -> None:
        """Publish all components together and remove prior generated environments."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            alpha = self._write_package(build_directory, "alpha")
            beta = self._write_package(build_directory, "beta")
            environment_path = build_directory / build_tool.DEVELOPER_ENVIRONMENT_DIRECTORY
            environment_path.mkdir()
            (environment_path / "stale").touch()
            legacy_path = build_directory / "examples-dev"
            legacy_path.mkdir()

            def install(
                command: list[object],
                *,
                environment: dict[str, str | None],
                quiet: bool,
            ) -> None:
                prefix = Path(command[command.index("--prefix") + 1])
                component = str(command[command.index("--component") + 1])
                (prefix / "lib" / "cmake").mkdir(parents=True, exist_ok=True)
                (prefix / "python").mkdir(exist_ok=True)
                (prefix / component).touch()
                self.assertEqual(environment, {"CMAKE_INSTALL_MODE": None, "DESTDIR": None})
                self.assertTrue(quiet)

            with mock.patch.object(build_tool, "_run", side_effect=install) as run:
                actual_components = build_tool._materialize_developer_environment(
                    Path("/cmake"), build_directory, "Release"
                )

            expected_components = (alpha[0], beta[0], alpha[1], beta[1], alpha[2], beta[2])
            self.assertEqual(actual_components, expected_components)
            self.assertEqual(run.call_count, len(expected_components))
            self.assertFalse((environment_path / "stale").exists())
            self.assertFalse(legacy_path.exists())
            for component in expected_components:
                self.assertTrue((environment_path / component).is_file())

    def test_materialize_developer_environment_preserves_previous_tree_after_failure(self) -> None:
        """Keep the last complete environment when component installation fails."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            self._write_package(build_directory, "alpha")
            environment_path = build_directory / build_tool.DEVELOPER_ENVIRONMENT_DIRECTORY
            environment_path.mkdir()
            sentinel = environment_path / "previous"
            sentinel.touch()

            with (
                mock.patch.object(
                    build_tool,
                    "_run",
                    side_effect=subprocess.CalledProcessError(1, ["cmake", "--install"]),
                ),
                self.assertRaises(subprocess.CalledProcessError),
            ):
                build_tool._materialize_developer_environment(Path("/cmake"), build_directory, "Release")

            self.assertTrue(sentinel.is_file())
            self.assertEqual(list(build_directory.glob(".developer-environment-*")), [])

    def test_ensure_developer_environment_reuses_complete_directory(self) -> None:
        """Keep the build-owned environment during ordinary local test-only runs."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            environment_path = build_directory / build_tool.DEVELOPER_ENVIRONMENT_DIRECTORY
            (environment_path / "lib" / "cmake").mkdir(parents=True)
            (environment_path / "python").mkdir()

            with mock.patch.object(build_tool, "_materialize_developer_environment") as materialize:
                build_tool._ensure_developer_environment(Path("/cmake"), build_directory, "Release")

            materialize.assert_not_called()

    def test_ensure_developer_environment_restores_omitted_artifact_directory(self) -> None:
        """Reinstall the large generated environment omitted from CI artifacts."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            cmake = Path("/cmake")

            with mock.patch.object(build_tool, "_materialize_developer_environment") as materialize:
                build_tool._ensure_developer_environment(cmake, build_directory, "Release")

            materialize.assert_called_once_with(cmake, build_directory, "Release")

    def test_read_developer_environment_components_rejects_incomplete_package(self) -> None:
        """Refuse to publish an environment missing one declared package surface."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            package_directory = build_directory / "packages" / "alpha"
            package_directory.mkdir(parents=True)
            package_directory.joinpath("package.json").write_text(
                '{"complete": false, "components": {}}\n', encoding="utf-8"
            )

            with self.assertRaisesRegex(RuntimeError, "incomplete"):
                build_tool._read_developer_environment_components(build_directory)

    def test_remove_owned_directory_rejects_redirected_root(self) -> None:
        """Never follow a generated environment root redirected outside the build tree."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            external_directory = root / "external"
            redirected_directory = root / "redirected"
            external_directory.mkdir()
            sentinel = external_directory / "sentinel"
            sentinel.touch()
            try:
                redirected_directory.symlink_to(external_directory, target_is_directory=True)
            except OSError as error:
                self.skipTest(f"Directory symlinks are unavailable: {error}")

            with self.assertRaisesRegex(RuntimeError, "redirected"):
                build_tool._remove_owned_directory(redirected_directory)

            self.assertTrue(sentinel.is_file())

    def test_developer_environment_lock_allows_readers_and_blocks_replacement(self) -> None:
        """Allow concurrent examples while a replacement waits for all readers."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory) / "isaacsim-libraries-release"
            reader_entered = threading.Event()
            release_reader = threading.Event()
            writer_started = threading.Event()
            writer_entered = threading.Event()

            def read_environment() -> None:
                with build_tool._developer_environment_lock(build_directory, exclusive=False):
                    reader_entered.set()
                    release_reader.wait(timeout=2)

            def replace_environment() -> None:
                writer_started.set()
                with build_tool._developer_environment_lock(build_directory, exclusive=True):
                    writer_entered.set()

            reader = threading.Thread(target=read_environment, daemon=True)
            writer = threading.Thread(target=replace_environment, daemon=True)
            try:
                with build_tool._developer_environment_lock(build_directory, exclusive=False):
                    reader.start()
                    self.assertTrue(reader_entered.wait(timeout=2))
                    writer.start()
                    self.assertTrue(writer_started.wait(timeout=2))
                    self.assertFalse(writer_entered.wait(timeout=0.1))
                    release_reader.set()
                    reader.join(timeout=2)
                    self.assertFalse(reader.is_alive())
                    self.assertFalse(writer_entered.is_set())

                self.assertTrue(writer_entered.wait(timeout=2))
            finally:
                release_reader.set()
                reader.join(timeout=2)
                if writer.ident is not None:
                    writer.join(timeout=2)

            self.assertFalse(reader.is_alive())
            self.assertFalse(writer.is_alive())


class ArtifactStateTests(unittest.TestCase):
    """Test the single completed-build state contract."""

    def _create_repository(self, root: Path) -> Path:
        """Create the required state inputs and return the library source root.

        Args:
            root: Root directory to process.

        Returns:
            The resulting value.
        """
        libraries_directory = root / "source" / "libraries"
        for relative_path in build_tool.ARTIFACT_INPUT_PATHS:
            path = root / relative_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"{relative_path.as_posix()}\n", encoding="utf-8")
        libraries_directory.mkdir(parents=True, exist_ok=True)
        (libraries_directory / "authored.cpp").write_text("int value = 1;\n", encoding="utf-8")
        return libraries_directory

    def test_validate_artifact_state_accepts_matching_full_build(self) -> None:
        """Accept a completed full build while every immutable input still matches."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            libraries_directory = self._create_repository(root)
            build_directory = root / "build"
            state = build_tool._artifact_state_inputs(
                configuration="Release",
                profile="standard",
                dependency_profile="locked",
                repository_root=root,
                libraries_directory=libraries_directory,
            )
            build_tool._write_json_atomically(
                build_directory / build_tool.ARTIFACT_STATE_FILE,
                {**state, "repository_root": root.as_posix(), "targets": ["all"]},
            )

            producer_root = build_tool._validate_artifact_state(
                build_directory,
                configuration="Release",
                profile="standard",
                dependency_profile="locked",
                repository_root=root,
                libraries_directory=libraries_directory,
            )
            self.assertEqual(producer_root, root.as_posix())

    def test_validate_artifact_state_rejects_changed_input(self) -> None:
        """Reject test-only execution after launcher or manifest inputs change."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            libraries_directory = self._create_repository(root)
            build_directory = root / "build"
            state = build_tool._artifact_state_inputs(
                configuration="Release",
                profile="standard",
                dependency_profile="locked",
                repository_root=root,
                libraries_directory=libraries_directory,
            )
            build_tool._write_json_atomically(
                build_directory / build_tool.ARTIFACT_STATE_FILE,
                {**state, "repository_root": root.as_posix(), "targets": ["all"]},
            )
            (root / "tools" / "pixi_run.py").write_text("changed\n", encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "stale"):
                build_tool._validate_artifact_state(
                    build_directory,
                    configuration="Release",
                    profile="standard",
                    dependency_profile="locked",
                    repository_root=root,
                    libraries_directory=libraries_directory,
                )

    def test_validate_artifact_state_rejects_changed_license_collector(self) -> None:
        """Reject artifact reuse after the dependency-license collector changes."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            libraries_directory = self._create_repository(root)
            build_directory = root / "build"
            state = build_tool._artifact_state_inputs(
                configuration="Release",
                profile="standard",
                dependency_profile="locked",
                repository_root=root,
                libraries_directory=libraries_directory,
            )
            build_tool._write_json_atomically(
                build_directory / build_tool.ARTIFACT_STATE_FILE,
                {**state, "repository_root": root.as_posix(), "targets": ["all"]},
            )
            (root / "tools" / "pixi_licenses.py").write_text("changed\n", encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "stale"):
                build_tool._validate_artifact_state(
                    build_directory,
                    configuration="Release",
                    profile="standard",
                    dependency_profile="locked",
                    repository_root=root,
                    libraries_directory=libraries_directory,
                )

    def test_validate_artifact_state_rejects_partial_build(self) -> None:
        """Reject test-only execution after a target-limited build."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            libraries_directory = self._create_repository(root)
            build_directory = root / "build"
            state = build_tool._artifact_state_inputs(
                configuration="Release",
                profile="standard",
                dependency_profile="locked",
                repository_root=root,
                libraries_directory=libraries_directory,
            )
            build_tool._write_json_atomically(
                build_directory / build_tool.ARTIFACT_STATE_FILE,
                {**state, "repository_root": root.as_posix(), "targets": ["isaacsim_common"]},
            )

            with self.assertRaisesRegex(RuntimeError, "partial"):
                build_tool._validate_artifact_state(
                    build_directory,
                    configuration="Release",
                    profile="standard",
                    dependency_profile="locked",
                    repository_root=root,
                    libraries_directory=libraries_directory,
                )

    def test_relocate_cmake_metadata_updates_cache_test_and_install_files(self) -> None:
        """Retarget transferred build metadata to the consuming checkout."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            build_directory = root / "build"
            nested = build_directory / "module"
            nested.mkdir(parents=True)
            old_root = "C:/g/producer"
            new_root = Path("C:/g/consumer")
            old_native_root = old_root.replace("/", "\\")
            new_native_root = new_root.as_posix().replace("/", "\\")
            old_escaped_root = old_root.replace("/", "\\\\")
            new_escaped_root = new_root.as_posix().replace("/", "\\\\")
            top_level = build_directory / "CTestTestfile.cmake"
            child = nested / "CTestTestfile.cmake"
            install = nested / "cmake_install.cmake"
            install_index = build_directory / "CMakeFiles" / "InstallScripts.json"
            cache = build_directory / "CMakeCache.txt"
            top_level.write_text(
                f'add_test(test "{old_root}/.pixi/envs/build-driver/python.exe" '
                f'"--python={old_escaped_root}\\\\.pixi\\\\envs\\\\build-driver\\\\python.exe" '
                f'"--cmake={old_root}/.pixi/envs/build-driver/Library/bin/cmake.exe" '
                f'"--make-program={old_root}/.pixi/envs/build-driver/Library/bin/ninja.exe")\n',
                encoding="utf-8",
            )
            child.write_text(
                f'set_tests_properties(test PROPERTIES WORKING_DIRECTORY "{old_root}/source")\n', encoding="utf-8"
            )
            install.write_text(f'include("{old_root}/build/module/cmake_install.cmake")\n', encoding="utf-8")
            install_index.parent.mkdir()
            install_index.write_text(
                f'{{"InstallScripts":["{old_root}/build/cmake_install.cmake"]}}\n',
                encoding="utf-8",
            )
            cache.write_text(
                f"Python_EXECUTABLE:FILEPATH={old_native_root}\\.pixi\\envs\\build-driver\\python.exe\n",
                encoding="utf-8",
            )

            build_tool._relocate_cmake_metadata(build_directory, old_root, new_root)

            self.assertNotIn(old_root, top_level.read_text(encoding="utf-8"))
            self.assertNotIn(old_escaped_root, top_level.read_text(encoding="utf-8"))
            self.assertNotIn(old_root, child.read_text(encoding="utf-8"))
            self.assertNotIn(old_root, install.read_text(encoding="utf-8"))
            self.assertNotIn(old_root, install_index.read_text(encoding="utf-8"))
            self.assertNotIn(old_native_root, cache.read_text(encoding="utf-8"))
            self.assertIn(new_root.as_posix(), top_level.read_text(encoding="utf-8"))
            self.assertIn(new_escaped_root, top_level.read_text(encoding="utf-8"))
            self.assertIn(
                f"{new_root.as_posix()}/.pixi/envs/build-driver/Library/bin/ninja.exe",
                top_level.read_text(encoding="utf-8"),
            )
            self.assertIn(new_root.as_posix(), install.read_text(encoding="utf-8"))
            self.assertIn(new_root.as_posix(), install_index.read_text(encoding="utf-8"))
            self.assertIn(new_native_root, cache.read_text(encoding="utf-8"))

            build_tool._relocate_cmake_metadata(build_directory, old_root, new_root)

    def test_relocate_cmake_metadata_rejects_mismatched_artifact_state(self) -> None:
        """Fail clearly when metadata does not match its recorded producer checkout."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            build_directory = Path(temporary_directory)
            (build_directory / "CTestTestfile.cmake").write_text(
                'add_test(test "C:/g/actual/python.exe")\n',
                encoding="utf-8",
            )

            with self.assertRaisesRegex(RuntimeError, "does not reference its recorded producer root"):
                build_tool._relocate_cmake_metadata(
                    build_directory,
                    "C:/g/recorded",
                    Path("C:/g/consumer"),
                )


class ArgumentTests(unittest.TestCase):
    """Test standalone build argument contracts."""

    def test_test_jobs_require_a_test_action(self) -> None:
        """Reject CTest parallelism when no test run was requested."""
        with mock.patch.object(build_tool.sys, "argv", ["build.py", "--test-jobs", "8"]):
            with self.assertRaises(SystemExit):
                build_tool._parse_arguments()

    def test_test_jobs_must_be_positive(self) -> None:
        """Reject invalid CTest worker counts."""
        with mock.patch.object(build_tool.sys, "argv", ["build.py", "--test", "--test-jobs", "0"]):
            with self.assertRaises(SystemExit):
                build_tool._parse_arguments()

    def test_pull_only_is_a_standalone_action(self) -> None:
        """Allow dependency population without configuring a build."""
        with mock.patch.object(build_tool.sys, "argv", ["build.py", "--pull-only"]):
            arguments = build_tool._parse_arguments()

        self.assertTrue(arguments.pull_only)

    def test_pull_only_rejects_build_options(self) -> None:
        """Reject ambiguous dependency-only and build requests."""
        with mock.patch.object(build_tool.sys, "argv", ["build.py", "--pull-only", "--no-pull"]):
            with self.assertRaises(SystemExit):
                build_tool._parse_arguments()


class ExecutableTests(unittest.TestCase):
    """Test fail-closed validation of locked Pixi executables."""

    @unittest.skipIf(sys.platform == "win32", "POSIX permissions are unavailable on Windows.")
    def test_require_executable_rejects_non_executable_file(self) -> None:
        """Do not mutate a malformed locked environment to make it usable."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            executable = Path(temporary_directory) / "cmake"
            executable.write_bytes(b"test")
            executable.chmod(0o600)

            with self.assertRaisesRegex(RuntimeError, "Pinned CMake is not executable"):
                build_tool._require_executable(executable, "Pinned CMake")

    def test_require_executable_rejects_missing_file(self) -> None:
        """Report a missing locked tool without manufacturing a fallback copy."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            executable = Path(temporary_directory) / "missing"
            with self.assertRaisesRegex(RuntimeError, "Pinned CTest is missing"):
                build_tool._require_executable(executable, "Pinned CTest")


class TestRunnerTests(unittest.TestCase):
    """Test CTest command construction and test-only orchestration."""

    def test_main_default_configuration_records_test_dependencies(self) -> None:
        """Prepare CTest for a later CI job even when the build job does not run tests."""
        host = build_tool._HostPlatform("linux-x86_64", "manylinux_2_35_x86_64", "")
        with tempfile.TemporaryDirectory() as temporary_directory:
            with (
                mock.patch.object(build_tool.sys, "argv", [str(BUILD_TOOL_PATH), "--no-pull", "--generate"]),
                mock.patch.object(build_tool, "LIBRARY_BUILD_ROOT", Path(temporary_directory)),
                mock.patch.object(build_tool, "_detect_host_platform", return_value=host),
                mock.patch.object(build_tool, "_find_build_tool", side_effect=(Path("/cmake"), Path("/ninja"))),
                mock.patch.object(
                    build_tool,
                    "_read_pixi_dependency_path",
                    side_effect=lambda variable: Path(f"/{variable.lower()}"),
                ) as read_dependency_path,
                mock.patch.object(
                    build_tool,
                    "_read_pixi_license_file",
                    return_value=Path("/selected/PIP-LICENSES.txt"),
                ),
                mock.patch.object(build_tool, "_configure") as configure,
            ):
                status = build_tool._main()

        self.assertEqual(status, 0)
        read_dependency_path.assert_any_call(build_tool.PIXI_TEST_DEPS_VARIABLE)
        self.assertEqual(configure.call_args.args[7], Path(f"/{build_tool.PIXI_TEST_DEPS_VARIABLE.lower()}"))
        self.assertEqual(configure.call_args.args[11], Path("/selected/PIP-LICENSES.txt"))

    def test_run_tests_writes_junit_and_applies_timeout(self) -> None:
        """Request CI reporting, a timeout, and a nonempty test selection."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            build_directory = root / "build"
            build_directory.mkdir()
            (build_directory / "CTestTestfile.cmake").write_text("", encoding="utf-8")
            cmake = root / "cmake"
            ctest = root / "ctest"
            ctest.write_text("", encoding="utf-8")
            junit_output = Path("_cmake_build/test-reports/modules.xml")

            with (
                mock.patch.object(build_tool, "_require_executable", return_value=ctest),
                mock.patch.object(build_tool, "_run") as run,
            ):
                build_tool._run_tests(
                    cmake,
                    build_directory,
                    "Release",
                    junit_output,
                    900,
                    8,
                    "^tests-examples$",
                    "^tests-known-failure$",
                )

        command = run.call_args.args[0]
        self.assertIn("--no-tests=error", command)
        self.assertEqual(command[command.index("--output-junit") + 1], junit_output.resolve())
        self.assertEqual(command[command.index("--timeout") + 1], 900)
        self.assertEqual(command[command.index("--parallel") + 1], 8)
        self.assertEqual(command[command.index("-R") + 1], "^tests-examples$")
        self.assertEqual(command[command.index("-E") + 1], "^tests-known-failure$")

    def test_run_tests_rejects_missing_build_tree(self) -> None:
        """Fail clearly when test-only has no configured CTest build."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            with self.assertRaisesRegex(RuntimeError, "Run the matching profile build"):
                build_tool._run_tests(
                    Path(temporary_directory) / "cmake",
                    Path(temporary_directory) / "missing",
                    "Release",
                    None,
                    None,
                    None,
                    None,
                    None,
                )

    def test_main_test_only_skips_configure_and_build(self) -> None:
        """Restore dependencies and run CTest without rebuilding the artifact."""
        host = build_tool._HostPlatform("windows-x86_64", "windows-x86_64", ".exe")
        cmake = Path("/cmake")
        arguments = [
            str(BUILD_TOOL_PATH),
            "--profile",
            "werror",
            "--no-pull",
            "--test-only",
            "--test-timeout",
            "900",
            "--test-jobs",
            "4",
        ]
        with (
            mock.patch.object(build_tool.sys, "argv", arguments),
            mock.patch.object(build_tool, "_detect_host_platform", return_value=host),
            mock.patch.object(build_tool, "_pull_dependencies"),
            mock.patch.object(build_tool, "_find_build_tool", side_effect=(cmake, Path("/ninja"))),
            mock.patch.object(build_tool, "_read_pixi_dependency_path", return_value=Path("/dependencies")),
            mock.patch.object(build_tool, "ensure_windows_msvc_environment") as ensure_msvc,
            mock.patch.object(
                build_tool,
                "_validate_artifact_state",
                return_value=build_tool.REPOSITORY_ROOT.as_posix(),
            ) as validate_state,
            mock.patch.object(build_tool, "_relocate_cmake_metadata") as relocate_metadata,
            mock.patch.object(build_tool, "_configure") as configure,
            mock.patch.object(build_tool, "_build") as build,
            mock.patch.object(build_tool, "_run_tests") as run_tests,
        ):
            status = build_tool._main()

        self.assertEqual(status, 0)
        ensure_msvc.assert_called_once_with(require_debug_runtime=False)
        validate_state.assert_called_once_with(
            build_tool.LIBRARY_BUILD_ROOT / "isaacsim-libraries-werror",
            configuration="Release",
            profile="werror",
            dependency_profile="locked",
        )
        configure.assert_not_called()
        build.assert_not_called()
        relocate_metadata.assert_called_once_with(
            build_tool.LIBRARY_BUILD_ROOT / "isaacsim-libraries-werror",
            build_tool.REPOSITORY_ROOT.as_posix(),
            build_tool.REPOSITORY_ROOT,
        )
        run_tests.assert_called_once_with(
            cmake,
            build_tool.LIBRARY_BUILD_ROOT / "isaacsim-libraries-werror",
            "Release",
            None,
            900,
            4,
            None,
            None,
        )


class WheelBuildTests(unittest.TestCase):
    """Test wheel command construction."""

    def test_wheel_staging_build_removes_stale_default_artifacts(self) -> None:
        """Replace the internal wheel staging directory instead of mixing packaging shapes across builds."""
        host = build_tool._HostPlatform("linux-x86_64", "manylinux_2_35_x86_64", "")
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_root = Path(temporary_directory)
            artifact_root = temporary_root / "extension-wheel-artifacts"
            stale_artifact = artifact_root / "release" / "isaacsim_asset-7.0.0a1-cp312-abi3-linux_x86_64.whl"
            stale_artifact.parent.mkdir(parents=True)
            stale_artifact.touch()
            current_artifact = artifact_root / "release" / "isaacsim_asset-7.0.0a1-py3-none-any.whl"

            def write_wheel(*arguments: object) -> None:
                current_artifact.parent.mkdir(parents=True, exist_ok=True)
                current_artifact.touch()

            arguments = [
                str(BUILD_TOOL_PATH),
                "--stage-wheel",
                "isaacsim.asset.importer.utils",
                "--no-pull",
            ]
            with (
                mock.patch.object(build_tool.sys, "argv", arguments),
                mock.patch.object(build_tool, "LIBRARY_BUILD_ROOT", temporary_root),
                mock.patch.object(build_tool, "EXTENSION_WHEEL_ARTIFACT_ROOT", artifact_root),
                mock.patch.object(build_tool, "_detect_host_platform", return_value=host),
                mock.patch.object(build_tool, "_find_build_tool", side_effect=(Path("/cmake"), Path("/ninja"))),
                mock.patch.object(
                    build_tool,
                    "_read_pixi_dependency_path",
                    side_effect=lambda variable: Path(f"/{variable.lower()}"),
                ),
                mock.patch.object(
                    build_tool,
                    "_read_pixi_license_file",
                    return_value=Path("/selected/PIP-LICENSES.txt"),
                ),
                mock.patch.object(build_tool, "_configure"),
                mock.patch.object(build_tool, "_build"),
                mock.patch.object(build_tool, "_materialize_developer_environment", return_value=()),
                mock.patch.object(build_tool, "_write_artifact_state"),
                mock.patch.object(build_tool, "_build_wheels", side_effect=write_wheel) as build_wheels,
                mock.patch.object(build_tool, "_stage_extension_wheel"),
            ):
                status = build_tool._main()

            self.assertEqual(status, 0)
            self.assertFalse(stale_artifact.exists())
            build_wheels.assert_called_once()
            self.assertTrue(
                (temporary_root / "module-carrier-artifacts/release" / current_artifact.name).samefile(current_artifact)
            )

    def test_wheel_staging_build_preserves_explicit_output_directory(self) -> None:
        """Leave caller-owned wheel collections intact when `--output-dir` is explicit."""
        host = build_tool._HostPlatform("linux-x86_64", "manylinux_2_35_x86_64", "")
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_root = Path(temporary_directory)
            output_directory = temporary_root / "custom"
            existing_artifact = output_directory / "other-platform.whl"
            output_directory.mkdir()
            existing_artifact.touch()
            arguments = [
                str(BUILD_TOOL_PATH),
                "--stage-wheel",
                "isaacsim.asset.importer.utils",
                "--no-pull",
                "--output-dir",
                str(output_directory),
            ]
            with (
                mock.patch.object(build_tool.sys, "argv", arguments),
                mock.patch.object(build_tool, "LIBRARY_BUILD_ROOT", temporary_root),
                mock.patch.object(build_tool, "_detect_host_platform", return_value=host),
                mock.patch.object(build_tool, "_find_build_tool", side_effect=(Path("/cmake"), Path("/ninja"))),
                mock.patch.object(
                    build_tool,
                    "_read_pixi_dependency_path",
                    side_effect=lambda variable: Path(f"/{variable.lower()}"),
                ),
                mock.patch.object(
                    build_tool,
                    "_read_pixi_license_file",
                    return_value=Path("/selected/PIP-LICENSES.txt"),
                ),
                mock.patch.object(build_tool, "_configure"),
                mock.patch.object(build_tool, "_build"),
                mock.patch.object(build_tool, "_materialize_developer_environment", return_value=()),
                mock.patch.object(build_tool, "_write_artifact_state"),
                mock.patch.object(build_tool, "_build_wheels"),
                mock.patch.object(build_tool, "_stage_extension_wheel"),
            ):
                status = build_tool._main()

            self.assertEqual(status, 0)
            self.assertTrue(existing_artifact.is_file())
            self.assertFalse((temporary_root / "module-carrier-artifacts/release").exists())

    def test_main_routes_wheel_artifact_directories(self) -> None:
        """Select configuration-specific defaults while preserving an explicit CLI override."""
        host = build_tool._HostPlatform("linux-x86_64", "manylinux_2_35_x86_64", "")
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_root = Path(temporary_directory)
            artifact_root = temporary_root / "isaacsim-libraries-artifacts"
            cases = (
                ([], "Release", artifact_root / "release"),
                (["--debug"], "Debug", artifact_root / "debug"),
                (["--output-dir", str(temporary_root / "custom")], "Release", temporary_root / "custom"),
            )
            for extra_arguments, configuration, expected_output in cases:
                with self.subTest(arguments=extra_arguments):
                    arguments = [
                        str(BUILD_TOOL_PATH),
                        "--wheel",
                        "--group",
                        "isaacsim_common",
                        "--no-pull",
                        *extra_arguments,
                    ]
                    with (
                        mock.patch.object(build_tool.sys, "argv", arguments),
                        mock.patch.object(build_tool, "LIBRARY_BUILD_ROOT", temporary_root),
                        mock.patch.object(build_tool, "LIBRARY_ARTIFACT_ROOT", artifact_root),
                        mock.patch.object(build_tool, "_detect_host_platform", return_value=host),
                        mock.patch.object(
                            build_tool,
                            "_find_build_tool",
                            side_effect=(Path("/cmake"), Path("/ninja")),
                        ),
                        mock.patch.object(
                            build_tool,
                            "_read_pixi_dependency_path",
                            side_effect=lambda variable: Path(f"/{variable.lower()}"),
                        ),
                        mock.patch.object(
                            build_tool,
                            "_read_pixi_license_file",
                            return_value=Path("/selected/PIP-LICENSES.txt"),
                        ),
                        mock.patch.object(build_tool, "_configure"),
                        mock.patch.object(build_tool, "_build"),
                        mock.patch.object(build_tool, "_materialize_developer_environment", return_value=()),
                        mock.patch.object(build_tool, "_write_artifact_state"),
                        mock.patch.object(build_tool, "_build_wheels") as build_wheels,
                    ):
                        status = build_tool._main()

                self.assertEqual(status, 0)
                build_wheels.assert_called_once()
                self.assertEqual(build_wheels.call_args.args[3], configuration)
                self.assertEqual(build_wheels.call_args.args[5], expected_output)

    def test_build_wheels_uses_default_output_directory(self) -> None:
        """Publish wheels outside the source tree by default."""
        with mock.patch.object(build_tool, "_run") as run:
            with mock.patch.dict(os.environ, {}, clear=True):
                build_tool._build_wheels(
                    Path("/cmake"),
                    Path("/python"),
                    Path("/build"),
                    "Release",
                    ("isaacsim_common",),
                    None,
                    Path("/python-build"),
                    {"isaacsim_robot_schema": "usd2511"},
                )

        command = run.call_args.args[0]
        output_index = command.index("--output-dir")
        self.assertEqual(command[output_index + 1], build_tool.LIBRARY_ARTIFACT_ROOT / "release")
        dependency_index = command.index("--build-deps-dir")
        self.assertEqual(command[dependency_index + 1], Path("/python-build"))
        self.assertEqual(run.call_args.kwargs["environment"], {"PYTHONPATH": str(Path("/python-build"))})
        self.assertIn("isaacsim_robot_schema=usd2511", command)


if __name__ == "__main__":
    unittest.main()
