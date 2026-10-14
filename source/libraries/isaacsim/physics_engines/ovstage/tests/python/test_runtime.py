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

"""Validate process-level OVStage runtime setup."""

from __future__ import annotations

import subprocess
import sys
import textwrap
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


def _run_isolation_script(script: str) -> None:
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(script)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"Child process failed.\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"


def test_setup_registers_schemas_before_loading_ovphysx(monkeypatch: pytest.MonkeyPatch) -> None:
    """Register OVNewton's schemas exactly once before OvPhysX can read them."""
    from isaacsim.physics_engines.ovstage.impl import runtime

    events: list[str] = []
    package_roots = {
        "ovstage": Path("/packages/ovstage"),
        "ovphysx": Path("/packages/ovphysx"),
    }
    ovstage = ModuleType("ovstage")

    def register_usd_schemas(descriptors: list[Path]) -> None:
        assert descriptors == [
            Path("newton_usd_schemas/plugInfo.json"),
            Path("physx_usd_schemas/PhysxSchema/resources/plugInfo.json"),
        ]
        events.append("register_usd_schemas")

    ovstage.population = SimpleNamespace(register_usd_schemas=register_usd_schemas)
    monkeypatch.setattr(runtime, "distribution", lambda name: SimpleNamespace(locate_file=lambda path: Path(path)))

    monkeypatch.setattr(runtime, "_done", False)
    monkeypatch.setattr(runtime, "_package_root", package_roots.__getitem__)
    monkeypatch.setattr(runtime, "_load_library", lambda path: events.append(path.name))
    monkeypatch.setitem(sys.modules, "ovstage", ovstage)

    runtime.setup()
    runtime.setup()

    assert events == [
        runtime._OVSTAGE_LOADER_NAME,
        runtime._OVSTAGE_LIBRARY_NAME,
        "register_usd_schemas",
        runtime._OVPHYSX_LIBRARY_NAME,
    ]


def test_setup_can_retry_after_schema_registration_failure(monkeypatch: pytest.MonkeyPatch) -> None:
    """Leave setup incomplete when schema registration fails so callers can retry."""
    from isaacsim.physics_engines.ovstage.impl import runtime

    attempts = 0
    ovstage = ModuleType("ovstage")

    def register_usd_schemas(descriptors: list[Path]) -> None:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise RuntimeError("schema registration failed")

    ovstage.population = SimpleNamespace(register_usd_schemas=register_usd_schemas)
    monkeypatch.setattr(runtime, "distribution", lambda name: SimpleNamespace(locate_file=lambda path: Path(path)))
    monkeypatch.setattr(runtime, "_done", False)
    monkeypatch.setattr(runtime, "_package_root", lambda package: Path(f"/packages/{package}"))
    monkeypatch.setattr(runtime, "_load_library", lambda path: None)
    monkeypatch.setitem(sys.modules, "ovstage", ovstage)

    try:
        runtime.setup()
    except RuntimeError as exc:
        assert str(exc) == "schema registration failed"
    else:
        raise AssertionError("setup() did not propagate the schema registration failure")

    assert not runtime._done
    runtime.setup()
    assert runtime._done
    assert attempts == 2


def test_setup_exposes_registered_schema_fallbacks_without_importing_pxr() -> None:
    """Populate applied Newton and PhysX schemas through OVStage's private USD runtime."""
    _run_isolation_script("""
        import sys

        import numpy as np

        from isaacsim.physics_engines.ovstage import setup

        assert not any(name == "pxr" or name.startswith("pxr.") for name in sys.modules)
        setup()
        assert not any(name == "pxr" or name.startswith("pxr.") for name in sys.modules)

        import ovstage

        stage_text = '''#usda 1.0

        def PhysicsScene "Scene" (
            prepend apiSchemas = ["NewtonSceneAPI", "PhysxSceneAPI"]
        )
        {
        }
        '''

        with ovstage.Stage("newton-schema-registration-test") as stage:
            ovstage.population.open_usd_from_string(
                stage,
                stage_text,
                ordinal=1,
                domains=ovstage.PopulationDomain.PHYSICS,
            )
            stage.advance_write_floor(1).wait()
            with ovstage.PathDictionary(stage) as paths:
                attributes = [
                    paths.intern_token("newton:timeStepsPerSecond"),
                    paths.intern_token("physxScene:timeStepsPerSecond"),
                ]
                with paths.create_path_list_from_strings(["/Scene"]) as prims:
                    with stage.query_from_path_list(prims) as query:
                        assert query.result().total_prim_count == 1
                        with stage.read_attributes(query, attributes, ovstage.OrdinalRange.latest(1)) as read:
                            read.wait()
                            values_by_attribute = {}
                            for group in read.groups():
                                with group:
                                    assert group.tensor_count == 1
                                    values_by_attribute[group.attribute] = np.array(group.array(0), copy=True)

                            assert set(values_by_attribute) == set(attributes)
                            np.testing.assert_array_equal(
                                values_by_attribute[attributes[0]], np.array([1000], dtype=np.int32)
                            )
                            np.testing.assert_array_equal(
                                values_by_attribute[attributes[1]], np.array([60], dtype=np.uint32)
                            )

        assert not any(name == "pxr" or name.startswith("pxr.") for name in sys.modules)
        """)
