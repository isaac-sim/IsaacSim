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

"""Verify that OvPhysX import keeps OVStage's OpenUSD runtime private."""

from __future__ import annotations

import subprocess
import sys
import textwrap


def _run_isolation_script(script: str) -> None:
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(script)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"Child process failed.\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
    dependency_error = "Dependency: [carb::dictionary::IDictionary v2.1] failed to be resolved."
    assert (
        dependency_error not in result.stdout + result.stderr
    ), f"Child process reported a Carbonite teardown error.\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"


def test_import_registers_ovphysx_without_importing_pxr() -> None:
    """Register OvPhysX on import without loading ``pxr``."""
    _run_isolation_script("""
        import sys

        from isaacsim.physics.manager import PhysicsManager
        from isaacsim.physics.registration import get_simulation_ids, get_simulation_name

        assert "isaacsim.physics_engines.ovstage" not in sys.modules
        assert not any(name == "pxr" or name.startswith("pxr.") for name in sys.modules)
        assert "ovphysx" not in {get_simulation_name(simulation_id) for simulation_id in get_simulation_ids()}

        import isaacsim.physics_engines.ovphysx

        manager = PhysicsManager.get_instance()
        assert manager.switch_physics_engine("ovphysx")
        assert "ovphysx" in {get_simulation_name(simulation_id) for simulation_id in get_simulation_ids()}
        assert not any(name == "pxr" or name.startswith("pxr.") for name in sys.modules)

        from isaacsim.foundation.objects import Stage
        from pxr import Usd

        stage = Stage("openusd").create_stage()
        try:
            assert stage.get_stage_id() >= 0
            assert Usd.Stage.CreateInMemory()
        finally:
            stage.close_stage()
        """)


def test_import_preserves_foundation_pxr_and_populates_ovstage() -> None:
    """Preserve Foundation's ``pxr`` modules while populating private OVStage USD."""
    _run_isolation_script("""
        import sys

        from isaacsim.foundation.objects import Stage
        from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils

        stage = Stage("openusd").create_stage()
        stage_id = stage.get_stage_id()
        usd_stage = UsdUtils.StageCache.Get().Find(Usd.StageCache.Id.FromLongInt(stage_id))
        assert usd_stage
        UsdGeom.Cube.Define(usd_stage, "/World/Cube")
        UsdPhysics.RigidBodyAPI.Apply(usd_stage.GetPrimAtPath("/World/Cube"))
        stage_text = usd_stage.Flatten().ExportToString()
        pxr_modules = {
            name: module
            for name, module in sys.modules.items()
            if name == "pxr" or name.startswith("pxr.")
        }

        import isaacsim.physics_engines.ovphysx

        assert pxr_modules == {
            name: module
            for name, module in sys.modules.items()
            if name == "pxr" or name.startswith("pxr.")
        }

        from ovstage import PopulationDomain
        from ovstage import Stage as OvStage
        from ovstage import population

        ovstage_stage = OvStage("runtime-isolation-test")
        try:
            population.open_usd_from_string(
                ovstage_stage,
                stage_text,
                domains=PopulationDomain.RENDERING | PopulationDomain.PHYSICS,
            )
        finally:
            ovstage_stage.destroy()
            stage.close_stage()
        """)


def test_setup_before_import_uses_the_backend_ovstage_instance() -> None:
    """Reconcile a setup-first loader hint before attaching an OVStage object."""
    _run_isolation_script("""
        import os
        import sys
        from pathlib import Path

        from isaacsim.physics_engines.ovstage import get_native_handle, setup

        setup()

        import isaacsim.physics_engines.ovphysx

        from isaacsim.physics.manager import PhysicsManager
        from ovstage import PopulationDomain
        from ovstage import Stage as OvStage
        from ovstage import population

        stage = OvStage("setup-before-import-test")
        simulation_initialized = False
        try:
            population.open_usd(
                stage,
                os.path.join(os.environ["ISAACSIM_TEST_RESOURCE_ROOT"], "CartPole.usda"),
                domains=PopulationDomain.RENDERING | PopulationDomain.PHYSICS,
            )
            stage.advance_write_floor(1).wait()
            manager = PhysicsManager.get_instance()
            assert manager.switch_physics_engine("ovphysx")
            manager.setup(1.0 / 60.0)
            assert manager.initialize(get_native_handle(stage), 0)
            simulation_initialized = True
            manager.step()

            if sys.platform == "linux":
                mapped_libraries = {
                    Path(line.split()[-1]).resolve()
                    for line in Path("/proc/self/maps").read_text(encoding="utf-8").splitlines()
                    if line.split() and line.split()[-1].endswith("/libovstage.so")
                }
                assert len(mapped_libraries) == 1, mapped_libraries
        finally:
            if simulation_initialized:
                manager.invalidate()
            stage.destroy()
        """)
