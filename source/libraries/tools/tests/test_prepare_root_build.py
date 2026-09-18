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

"""Test module-carrier preparation for the root repository build."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import tomllib
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

PREPARE_TOOL_PATH = Path(__file__).resolve().parents[1] / "prepare_root_build.py"
PREPARE_TOOL_SPEC = importlib.util.spec_from_file_location("isaacsim_prepare_root_build", PREPARE_TOOL_PATH)
if PREPARE_TOOL_SPEC is None or PREPARE_TOOL_SPEC.loader is None:
    raise RuntimeError(f"Cannot load root-build preparation tool from {PREPARE_TOOL_PATH}")
prepare_tool = importlib.util.module_from_spec(PREPARE_TOOL_SPEC)
sys.modules[PREPARE_TOOL_SPEC.name] = prepare_tool
PREPARE_TOOL_SPEC.loader.exec_module(prepare_tool)

REPO_BUILD_TOOL_PATH = prepare_tool.REPOSITORY_ROOT / "tools" / "isaac_build" / "repo_build.py"
REPO_BUILD_TOOL_SPEC = importlib.util.spec_from_file_location("isaacsim_repo_build", REPO_BUILD_TOOL_PATH)
if REPO_BUILD_TOOL_SPEC is None or REPO_BUILD_TOOL_SPEC.loader is None:
    raise RuntimeError(f"Cannot load Repo Build integration from {REPO_BUILD_TOOL_PATH}")
repo_build_tool = importlib.util.module_from_spec(REPO_BUILD_TOOL_SPEC)
sys.modules[REPO_BUILD_TOOL_SPEC.name] = repo_build_tool
REPO_BUILD_TOOL_SPEC.loader.exec_module(repo_build_tool)


class ArgumentTests(unittest.TestCase):
    """Test root argument translation."""

    def test_default_build_prepares_release_configuration(self) -> None:
        """Match the root build's Release-only default."""
        selection = prepare_tool._parse_root_build_arguments(("--no-docker",))

        self.assertEqual(selection.configurations, ("release",))
        self.assertFalse(selection.skipped)

    def test_explicit_configurations_preserve_selection(self) -> None:
        """Prepare only explicitly selected root configurations."""
        selection = prepare_tool._parse_root_build_arguments(("--config", "release", "debug", "--target", "app"))

        self.assertEqual(selection.configurations, ("release", "debug"))

    def test_non_build_actions_are_skipped(self) -> None:
        """Do not compile carriers for help or post-build requests."""
        for argument in ("--help", "--post-build-only", "--stage-only"):
            with self.subTest(argument=argument):
                self.assertTrue(prepare_tool._parse_root_build_arguments((argument,)).skipped)

    def test_fetch_only_pulls_library_dependencies(self) -> None:
        """Preserve dependency-only semantics for the root fetch operation."""
        selection = prepare_tool._parse_root_build_arguments(("--fetch-only",))

        self.assertTrue(selection.dependencies_only)
        self.assertFalse(selection.skipped)

    def test_build_only_reuses_library_dependencies(self) -> None:
        """Preserve the root build-only request for library dependencies."""
        selection = prepare_tool._parse_root_build_arguments(("--release", "--build-only"))

        self.assertEqual(selection.configurations, ("release",))
        self.assertTrue(selection.reuse_dependencies)

    def test_generate_only_configures_without_packaging(self) -> None:
        """Preserve root generate-only behavior for the module graph."""
        selection = prepare_tool._parse_root_build_arguments(("--release", "--generate"))

        self.assertEqual(selection.configurations, ("release",))
        self.assertTrue(selection.generate_only)

    def test_clean_and_rebuild_remain_distinct(self) -> None:
        """Clean only when requested, but rebuild after removing prior output."""
        clean = prepare_tool._parse_root_build_arguments(("--debug", "--clean"))
        rebuild = prepare_tool._parse_root_build_arguments(("--debug", "--clean", "--rebuild"))
        short_rebuild = prepare_tool._parse_root_build_arguments(("--debug", "-x"))

        self.assertTrue(clean.clean_only)
        self.assertFalse(clean.rebuild)
        self.assertFalse(rebuild.clean_only)
        self.assertTrue(rebuild.rebuild)
        self.assertTrue(short_rebuild.rebuild)

    def test_clustered_short_flags_match_argparse_tokenization(self) -> None:
        """Read `-rx` as `-r -x`, the way Repo Build's own parser reads it."""
        for arguments in (("-rx",), ("-xr",), ("-r", "-x")):
            with self.subTest(arguments=arguments):
                selection = prepare_tool._parse_root_build_arguments(arguments)

                self.assertEqual(selection.configurations, ("release",))
                self.assertTrue(selection.rebuild)

    def test_clustered_configurations_select_both(self) -> None:
        """Keep the combined configuration shorthand working."""
        self.assertEqual(prepare_tool._parse_root_build_arguments(("-rd",)).configurations, ("release", "debug"))
        self.assertEqual(prepare_tool._parse_root_build_arguments(("-dr",)).configurations, ("debug", "release"))

    def test_clusters_tolerate_options_this_tool_does_not_model(self) -> None:
        """Ignore unrelated letters instead of failing on them."""
        for arguments in (("-rv",), ("-ru",), ("-rq",)):
            with self.subTest(arguments=arguments):
                selection = prepare_tool._parse_root_build_arguments(arguments)

                self.assertEqual(selection.configurations, ("release",))

    def test_value_taking_short_options_are_not_split(self) -> None:
        """Never read an option's value, or a multi-letter option, as more flags."""
        for arguments in (("-mc", "32"), ("-mpc", "4"), ("-r", "-j8"), ("-t", "carb.dictionary.plugin")):
            with self.subTest(arguments=arguments):
                selection = prepare_tool._parse_root_build_arguments(arguments)

                self.assertFalse(selection.clean_only)
                self.assertFalse(selection.rebuild)

    def test_attached_value_still_yields_preceding_flags(self) -> None:
        """Recover the leading flags of a cluster that ends in an attached value."""
        selection = prepare_tool._parse_root_build_arguments(("-rj8",))

        self.assertEqual(selection.configurations, ("release",))

    def test_short_actions_match_their_long_forms(self) -> None:
        """Accept the short spelling of every action this tool reacts to."""
        self.assertTrue(prepare_tool._parse_root_build_arguments(("-S",)).skipped)
        self.assertTrue(prepare_tool._parse_root_build_arguments(("-r", "-g")).generate_only)
        self.assertTrue(prepare_tool._parse_root_build_arguments(("-r", "-b")).reuse_dependencies)
        self.assertTrue(prepare_tool._parse_root_build_arguments(("-r", "-c")).clean_only)

    def test_coverage_is_selected_by_argument(self) -> None:
        """Propagate root coverage instrumentation into carrier library builds."""
        selection = prepare_tool._parse_root_build_arguments(("--release", "--enable-gcov"))

        self.assertTrue(selection.coverage)

    def test_rebuild_is_selected_by_argument(self) -> None:
        """Let callers request a clean carrier rebuild explicitly."""
        selection = prepare_tool._parse_root_build_arguments(("--release", "--rebuild"))

        self.assertTrue(selection.rebuild)

    def test_combined_release_rebuild_selects_only_release(self) -> None:
        """Interpret Repo's common ``-xr`` spelling as a release rebuild."""
        selection = prepare_tool._parse_root_build_arguments(("-xr",))

        self.assertEqual(selection.configurations, ("release",))
        self.assertTrue(selection.rebuild)


class RootBuildIntegrationTests(unittest.TestCase):
    """Test carrier preparation at each supported root build entry point."""

    def test_linux_host_defers_to_linbuild(self) -> None:
        """Avoid compiling carrier binaries with the host toolchain before Repo enters linbuild."""
        self.assertFalse(
            prepare_tool._should_prepare_repository_build(("--release",), platform_name="linux", environment={})
        )

    def test_effective_build_environment_prepares_carriers(self) -> None:
        """Prepare directly for no-docker, linbuild, and Windows repository builds."""
        scenarios = (
            (("--release", "--no-docker"), "linux", {}),
            (("--release",), "linux", {"LINBUILD_EMBEDDED": "1"}),
            (("--release",), "linux", {"OMNI_REPO_BUILD_NO_DOCKER": "1"}),
            (("--release",), "win32", {}),
        )
        for arguments, platform_name, environment in scenarios:
            with self.subTest(arguments=arguments, platform_name=platform_name, environment=environment):
                self.assertTrue(
                    prepare_tool._should_prepare_repository_build(
                        arguments,
                        platform_name=platform_name,
                        environment=environment,
                    )
                )

    def test_repo_build_entry_owns_carrier_preparation(self) -> None:
        """Prepare carriers from Repo Build instead of the pre-bootstrap entry point."""
        repoman = prepare_tool.REPOSITORY_ROOT / "tools" / "repoman" / "repoman.py"
        repo_config = prepare_tool.REPOSITORY_ROOT / "repo.toml"

        self.assertNotIn("prepare_module_carriers", repoman.read_text(encoding="utf-8"))
        self.assertIn(
            "tools/isaac_build/repo_build.py:setup_repo_tool",
            repo_config.read_text(encoding="utf-8").replace("\\", "/"),
        )
        self.assertTrue(repo_build_tool.PREPARE_TOOL.samefile(PREPARE_TOOL_PATH))

        legacy_entry_points = (
            prepare_tool.REPOSITORY_ROOT / "build.sh",
            prepare_tool.REPOSITORY_ROOT / "build.bat",
        )
        for entry_point in legacy_entry_points:
            with self.subTest(entry_point=entry_point.name):
                self.assertNotIn("prepare_root_build.py", entry_point.read_text(encoding="utf-8"))

    @unittest.skipUnless(
        (prepare_tool.REPOSITORY_ROOT / "repo_internal.toml").is_file(),
        "Internal repository configuration is not included in GitHub exports",
    )
    def test_internal_ci_entries_do_not_prepare_carriers(self) -> None:
        """Keep carrier preparation in Repo Build for internal CI entry points."""
        for name in ("build_isaac.py", "build_isaac_coverage.py"):
            with self.subTest(entry_point=name):
                entry_point = prepare_tool.REPOSITORY_ROOT / "tools" / "ci" / name
                self.assertNotIn("prepare_root_build.py", entry_point.read_text(encoding="utf-8"))

    def test_carrier_premake_helper_is_a_public_build_tool(self) -> None:
        """Keep carrier Premake integration outside the GitHub-excluded `tools/isaac` tree."""
        repository_root = prepare_tool.REPOSITORY_ROOT
        helper = repository_root / "tools" / "isaac_build" / "module_carrier.lua"

        self.assertTrue(helper.is_file())
        self.assertFalse((repository_root / "tools" / "isaac" / "module_carrier.lua").exists())
        for manifest in sorted((repository_root / "source" / "extensions").glob("*/module-carrier.toml")):
            premake = manifest.parent / "premake5.lua"
            with self.subTest(extension=manifest.parent.name):
                contents = premake.read_text(encoding="utf-8")
                self.assertIn('dofile(root .. "/tools/isaac_build/module_carrier.lua")', contents)
                self.assertNotIn('dofile(root .. "/tools/isaac/module_carrier.lua")', contents)

    def test_third_party_notice_assembler_is_a_public_build_tool(self) -> None:
        """Keep packaged-notice assembly available in the staged GitHub repository."""
        repository_root = prepare_tool.REPOSITORY_ROOT
        helper = repository_root / "tools" / "isaac_build" / "assemble_third_party_notice.py"

        self.assertTrue(helper.is_file())
        self.assertTrue(repo_build_tool.ASSEMBLE_NOTICE_TOOL.samefile(helper))

    @unittest.skipUnless(
        (prepare_tool.REPOSITORY_ROOT / "repo_internal.toml").is_file(),
        "Internal repository configuration is not included in GitHub exports",
    )
    def test_windows_ci_jobs_pull_internal_library_toolchain(self) -> None:
        """Share Repo Build's pinned host toolchain with standalone Windows build and test jobs."""
        ci_config = (prepare_tool.REPOSITORY_ROOT / ".gitlab-ci.yml").read_text(encoding="utf-8")
        repo_config = (prepare_tool.REPOSITORY_ROOT / "repo.toml").read_text(encoding="utf-8")

        self.assertEqual(ci_config.count("!reference [.windows-module-toolchain, before_script]"), 2)
        self.assertIn("packman.cmd pull ./deps/isaacsim-libraries-msvc-ci.packman.xml", ci_config)
        self.assertEqual(
            tomllib.loads(repo_config)["repo_build"]["fetch"]["packman_host_files_to_pull"],
            ["${root}/deps/isaacsim-libraries-msvc-ci.packman.xml"],
        )
        self.assertIn('"token:in_ci==true".link_host_toolchain = false', repo_config)
        self.assertNotIn('"token:in_ci==true".vs_path', repo_config)

    @unittest.skipUnless(
        (prepare_tool.REPOSITORY_ROOT / "repo_internal.toml").is_file(),
        "Internal repository configuration is not included in GitHub exports",
    )
    def test_source_library_change_rules_include_license_collector(self) -> None:
        """Run standalone library CI whenever its license collector changes."""
        ci_config = (prepare_tool.REPOSITORY_ROOT / ".gitlab-ci.yml").read_text(encoding="utf-8")
        change_rules = ci_config.split(".source-libraries-change-rules:", 1)[1].split(".windows-module-toolchain:", 1)[
            0
        ]

        conditional_rules = change_rules.split("      - if:")[1:]
        self.assertEqual(len(conditional_rules), 2)
        for rule in conditional_rules:
            self.assertIn("          - tools/pixi_licenses.py", rule)

    @unittest.skipUnless(
        (prepare_tool.REPOSITORY_ROOT / "repo_internal.toml").is_file(),
        "Internal repository configuration is not included in GitHub exports",
    )
    def test_library_artifact_omits_rematerializable_developer_environment(self) -> None:
        """Keep the platform handoff artifact focused on irreproducible build outputs."""
        ci_config = (prepare_tool.REPOSITORY_ROOT / ".gitlab-ci.yml").read_text(encoding="utf-8")
        library_build = ci_config.split(".source-libraries-build-common:", 1)[1].split(
            "build-linux-x86_64-libraries:", 1
        )[0]

        self.assertIn("isaacsim-libraries-release/developer-environment/**/*", library_build)

    def test_repo_build_preparation_runs_once(self) -> None:
        """Run carrier and notice preparation once without overriding dependencies."""
        preparation = repo_build_tool._BuildPreparation()

        with (
            mock.patch.object(repo_build_tool, "_get_build_arguments", return_value=("--release",)),
            mock.patch.object(repo_build_tool.subprocess, "run") as run,
        ):
            preparation.run()
            preparation.run()

        self.assertEqual(run.call_count, 2)
        carrier_command = run.call_args_list[0].args[0]
        notice_command = run.call_args_list[1].args[0]
        self.assertEqual(carrier_command[-2:], ["--repo-entry", "--release"])
        self.assertEqual(notice_command, [sys.executable, "-s", repo_build_tool.ASSEMBLE_NOTICE_TOOL])
        for call in run.call_args_list:
            self.assertNotIn("env", call.kwargs)

    def test_repo_build_exposes_packaged_debug_runtimes_to_later_kit_tools(self) -> None:
        """Restore Debug runtime lookup in the parent after the carrier child exits."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            host_dependencies = Path(temporary_directory)
            (host_dependencies / "msvc").mkdir()
            (host_dependencies / "winsdk").mkdir()
            preparation = repo_build_tool._BuildPreparation()

            with (
                mock.patch.object(repo_build_tool.sys, "platform", "win32"),
                mock.patch.object(repo_build_tool, "WINDOWS_TOOLCHAIN_DIRECTORY", host_dependencies),
                mock.patch.object(repo_build_tool, "_get_build_arguments", return_value=("--debug",)),
                mock.patch.object(repo_build_tool.subprocess, "run"),
                mock.patch.object(repo_build_tool, "_activate_packaged_windows_debug_runtime") as activate_runtime,
            ):
                preparation.run()

        activate_runtime.assert_called_once_with()

    def test_repo_build_marks_host_preparation(self) -> None:
        """Tell carrier preparation when Repo Build will remain on the host."""
        preparation = repo_build_tool._BuildPreparation(prepare_on_host=True)

        with (
            mock.patch.object(repo_build_tool, "_get_build_arguments", return_value=("--release",)),
            mock.patch.object(repo_build_tool.subprocess, "run") as run,
        ):
            preparation.run()

        command = run.call_args_list[0].args[0]
        self.assertEqual(command[-3:], ["--repo-entry", "--release", "--no-docker"])

    def test_repo_build_arguments_follow_command(self) -> None:
        """Forward only arguments owned by the Repo Build command."""
        arguments = repo_build_tool._get_build_arguments(("repoman.py", "build", "--release", "--generate"))

        self.assertEqual(arguments, ("--release", "--generate"))

    def test_repo_build_prepares_after_dependencies_before_staging(self) -> None:
        """Enter the standalone library build only after Repo Build prepares dependencies."""
        events: list[str] = []
        fake_repo_build = SimpleNamespace()
        fake_repo_build.TOOL_CONFIG = {}
        fake_repo_build.setup_argument_parser = mock.Mock()
        fake_repo_build.pull_dependencies = lambda **kwargs: events.append("pull") or {}
        fake_repo_build.stage_files = lambda **kwargs: events.append("stage")
        fake_repo_build.build = lambda **kwargs: events.append("build")

        def run_build(*args: object, **kwargs: object) -> None:
            kwargs["pull_dependencies_fn"]()
            kwargs["stage_files_fn"]()
            kwargs["build_fn"]()

        fake_repo_build.run_build = run_build
        options = SimpleNamespace(clean=False, rebuild=False, fetch_only=False)
        config = {"repo": {"folders": {"host_deps": r"C:\repo\_build\host-deps"}}, "repo_build": {}}

        with (
            mock.patch.object(repo_build_tool.importlib, "import_module", return_value=fake_repo_build),
            mock.patch.object(repo_build_tool, "_get_build_arguments", return_value=("--release",)),
            mock.patch.object(
                repo_build_tool.subprocess,
                "run",
                side_effect=lambda command, **kwargs: events.append(
                    "assemble" if repo_build_tool.ASSEMBLE_NOTICE_TOOL in command else "prepare"
                ),
            ) as prepare,
        ):
            run_repo_tool = repo_build_tool.setup_repo_tool(mock.Mock(), config)
            run_repo_tool(options, config)

        self.assertEqual(events, ["pull", "prepare", "assemble", "stage", "build"])
        self.assertEqual(prepare.call_args_list[0].args[0][-3:], ["--repo-entry", "--release", "--no-docker"])


class DiscoveryTests(unittest.TestCase):
    """Test generic carrier discovery."""

    def test_discover_carriers_uses_manifests_instead_of_names(self) -> None:
        """Discover arbitrary extension names in deterministic order."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extensions = root / "source" / "extensions"
            for name in ("acme.zeta", "acme.alpha"):
                extension = extensions / name
                extension.mkdir(parents=True)
                (extension / "module-carrier.toml").touch()
            (extensions / "acme.unrelated").mkdir()

            discovered = prepare_tool._discover_carrier_extensions(root)

        self.assertEqual(discovered, ("acme.alpha", "acme.zeta"))


class PreparationTests(unittest.TestCase):
    """Test generated module commands."""

    def test_prepare_routes_library_builds_through_public_launcher(self) -> None:
        """Keep Pixi activation behind the supported build wrapper."""
        with mock.patch.object(prepare_tool, "_discover_carrier_extensions", return_value=("acme.common",)):
            with mock.patch.object(prepare_tool, "_run_library_build") as run_library_build:
                prepare_tool._prepare(("--release",))

        command = run_library_build.call_args.args[0]
        self.assertEqual(command[0], prepare_tool.LIBRARY_BUILD_LAUNCHER)
        self.assertNotIn(prepare_tool.sys.executable, command)

    def test_prepare_invokes_one_carrier_build_per_configuration(self) -> None:
        """Keep carrier packaging configuration-specific."""
        with mock.patch.object(prepare_tool, "_discover_carrier_extensions", return_value=("acme.common",)):
            with mock.patch.object(prepare_tool, "_run_library_build") as run_library_build:
                prepare_tool._prepare(("--config", "debug", "release"))

        commands = [call.args[0] for call in run_library_build.call_args_list]
        self.assertEqual(len(commands), 2)
        self.assertIn("-d", commands[0])
        self.assertIn("-r", commands[1])
        for command in commands:
            self.assertIn("--carrier", command)
            self.assertIn("acme.common", command)

    def test_prepare_forwards_coverage_to_library_build(self) -> None:
        """Instrument the module library when the root coverage build requests it."""
        with mock.patch.object(prepare_tool, "_discover_carrier_extensions", return_value=("acme.common",)):
            with mock.patch.object(prepare_tool, "_run_library_build") as run_library_build:
                prepare_tool._prepare(("--release", "--enable-gcov"))

        self.assertIn("--coverage", run_library_build.call_args.args[0])

    def test_generate_configures_modules_without_packaging_carriers(self) -> None:
        """Avoid compiling and staging carrier artifacts during root project generation."""
        with mock.patch.object(prepare_tool, "_discover_carrier_extensions", return_value=("acme.common",)):
            with mock.patch.object(prepare_tool, "_run_library_build") as run_library_build:
                prepare_tool._prepare(("--release", "--generate"))

        command = run_library_build.call_args.args[0]
        self.assertIn("--generate", command)
        self.assertNotIn("--carrier", command)
        self.assertNotIn("acme.common", command)

    def test_generate_creates_carrier_link_targets(self) -> None:
        """Let root staging resolve carrier links that generation does not populate."""
        with tempfile.TemporaryDirectory() as directory:
            build_root = Path(directory)
            with mock.patch.object(prepare_tool, "LIBRARY_BUILD_ROOT", build_root):
                with mock.patch.object(prepare_tool, "_discover_carrier_extensions", return_value=("acme.common",)):
                    with mock.patch.object(prepare_tool, "_run_library_build"):
                        prepare_tool._prepare(("--release", "--generate"))

            stage = build_root / "module-carriers" / "release" / "acme.common"
            for relative_path in ("pip_prebundle", "sdk/include"):
                with self.subTest(relative_path=relative_path):
                    self.assertTrue((stage / relative_path).is_dir())

    def test_packaging_build_leaves_carrier_staging_to_the_library_build(self) -> None:
        """Do not fabricate carrier output when the library build stages it for real."""
        with tempfile.TemporaryDirectory() as directory:
            build_root = Path(directory)
            with mock.patch.object(prepare_tool, "LIBRARY_BUILD_ROOT", build_root):
                with mock.patch.object(prepare_tool, "_discover_carrier_extensions", return_value=("acme.common",)):
                    with mock.patch.object(prepare_tool, "_run_library_build"):
                        prepare_tool._prepare(("--release",))

            self.assertFalse((build_root / "module-carriers").exists())

    def test_clean_removes_library_tree_and_carrier_outputs(self) -> None:
        """Keep root clean behavior symmetric across module and carrier output."""
        with mock.patch.object(prepare_tool, "_discover_carrier_extensions", return_value=("acme.common",)):
            with mock.patch.object(prepare_tool, "_run_library_build") as run_library_build:
                with mock.patch.object(prepare_tool, "_clean_carrier_outputs") as clean_carrier_outputs:
                    prepare_tool._prepare(("--release", "--clean"))

        command = run_library_build.call_args.args[0]
        self.assertIn("-r", command)
        self.assertIn("--clean", command)
        clean_carrier_outputs.assert_called_once_with("release")

    def test_rebuild_refreshes_carrier_outputs_and_library_tree(self) -> None:
        """Forward rebuild to CMake and remove the persistent handoff before restaging."""
        with mock.patch.object(prepare_tool, "_discover_carrier_extensions", return_value=("acme.common",)):
            with mock.patch.object(prepare_tool, "_run_library_build") as run_library_build:
                with mock.patch.object(prepare_tool, "_clean_carrier_outputs") as clean_carrier_outputs:
                    prepare_tool._prepare(("-x", "--release"))

        clean_carrier_outputs.assert_called_once_with("release")
        command = run_library_build.call_args.args[0]
        self.assertIn("--rebuild", command)
        self.assertIn("--carrier", command)

    def test_combined_release_rebuild_prepares_only_release_carriers(self) -> None:
        """Do not fall back to both configurations when Repo forwards ``-xr``."""
        with mock.patch.object(prepare_tool, "_discover_carrier_extensions", return_value=("acme.common",)):
            with mock.patch.object(prepare_tool, "_run_library_build") as run_library_build:
                with mock.patch.object(prepare_tool, "_clean_carrier_outputs") as clean_carrier_outputs:
                    prepare_tool._prepare(("-xr",))

        clean_carrier_outputs.assert_called_once_with("release")
        run_library_build.assert_called_once()
        command = run_library_build.call_args.args[0]
        self.assertIn("-r", command)
        self.assertNotIn("-d", command)
        self.assertIn("--rebuild", command)

    def test_fetch_only_populates_dependencies_without_discovery(self) -> None:
        """Populate native and Python dependencies without requiring a carrier."""
        with mock.patch.object(prepare_tool, "_discover_carrier_extensions") as discover:
            with mock.patch.object(prepare_tool, "_run_library_build") as run_library_build:
                prepare_tool._prepare(("--fetch-only",))

        discover.assert_not_called()
        self.assertIn("--pull-only", run_library_build.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
