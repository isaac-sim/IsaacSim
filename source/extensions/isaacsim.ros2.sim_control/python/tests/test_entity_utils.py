# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression tests for ROS 2 simulation-control entity utilities."""

import omni.kit.test
from isaacsim.ros2.sim_control.impl.entity_utils import get_filtered_entities


class _FakePath:
    def __init__(self, path: str) -> None:
        self.pathString = path


class _FakePrim:
    def __init__(self, path: str) -> None:
        self._path = _FakePath(path)

    def GetPrimPath(self) -> _FakePath:
        return self._path


class _FakeStage:
    def __init__(self, paths: list[str]) -> None:
        self._prims = [_FakePrim(path) for path in paths]

    def Traverse(self) -> list[_FakePrim]:
        return self._prims


class TestEntityUtils(omni.kit.test.TestCase):
    def test_get_filtered_entities_keeps_first_traversed_prim(self) -> None:
        stage = _FakeStage(["/First", "/Second", "/Render/Products"])

        entities, error = get_filtered_entities(stage)

        self.assertEqual(error, "")
        self.assertEqual(entities, ["/First", "/Second"])

    def test_get_filtered_entities_applies_regex_to_all_prims(self) -> None:
        stage = _FakeStage(["/RobotA", "/RobotB", "/Sensor"])

        entities, error = get_filtered_entities(stage, r"Robot")

        self.assertEqual(error, "")
        self.assertEqual(entities, ["/RobotA", "/RobotB"])
