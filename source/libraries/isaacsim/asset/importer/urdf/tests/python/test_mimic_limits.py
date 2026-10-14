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

"""Preserve independent mimic limits when importing inferred URDF bounds."""

from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from isaacsim.asset.importer.urdf.impl.drive_reconstruction import (
    SourceDriveInfo,
    parse_source_drive_breadcrumbs,
    reconstruct_source_drives,
)
from pxr import Sdf, Usd, UsdPhysics


class TestMimicLimits(unittest.TestCase):
    """Restore only limits explicitly marked as inferred for a mimic follower."""

    def test_restore_inferred_limits_from_xml(self) -> None:
        """Parse exported metadata and remove redundant solver limits."""
        stage = Usd.Stage.CreateInMemory()
        joint = UsdPhysics.PrismaticJoint.Define(stage, "/follower")
        prim = joint.GetPrim()
        prim.AddAppliedSchema("NewtonMimicAPI")
        joint.CreateLowerLimitAttr(-1)
        joint.CreateUpperLimitAttr(2)
        prim.CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(0.5)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot.urdf"
            metadata = {
                "inferred_mimic_limits": {
                    "joint_type": "prismatic",
                    "mimic": {"joint": "source"},
                    "limits": {"velocity": "0.5", "lower": "-1"},
                }
            }
            path.write_text(
                '<robot name="test"><joint name="follower" type="prismatic">'
                '<limit lower="-1" upper="2" velocity="0.5" effort="0"/>'
                '<mimic joint="source"/>'
                f"<!-- isaac:source_drive {json.dumps(metadata)} -->"
                "</joint></robot>"
            )
            records = parse_source_drive_breadcrumbs(str(path))
        self.assertEqual(reconstruct_source_drives(stage, records), 1)
        self.assertEqual(joint.GetLowerLimitAttr().Get(), -math.inf)
        self.assertEqual(joint.GetUpperLimitAttr().Get(), 2)
        self.assertEqual(prim.GetAttribute("newton:velocityLimit").Get(), math.inf)
        self.assertEqual(prim.GetAttribute("physxJoint:maxJointVelocity").Get(), math.inf)

    def test_unmarked_limits_preserved(self) -> None:
        """Keep explicit follower limits when no inference metadata is present."""
        stage = Usd.Stage.CreateInMemory()
        prim = UsdPhysics.PrismaticJoint.Define(stage, "/follower").GetPrim()
        prim.AddAppliedSchema("NewtonMimicAPI")
        prim.CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(0.5)
        reconstruct_source_drives(stage, [SourceDriveInfo(joint_name="follower")])
        self.assertEqual(prim.GetAttribute("newton:velocityLimit").Get(), 0.5)

    def test_non_mimic_ignores_inference_marker(self) -> None:
        """Do not remove limits from a joint that is no longer a mimic."""
        stage = Usd.Stage.CreateInMemory()
        prim = UsdPhysics.PrismaticJoint.Define(stage, "/follower").GetPrim()
        prim.CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(0.5)
        record = SourceDriveInfo(joint_name="follower", inferred_mimic_limits=["velocity"])
        reconstruct_source_drives(stage, [record])
        self.assertEqual(prim.GetAttribute("newton:velocityLimit").Get(), 0.5)

    def test_edited_limits_and_relationships_are_preserved(self) -> None:
        """Do not trust stale provenance after a user edits the exported URDF."""
        import xml.etree.ElementTree as ET

        from isaacsim.asset.importer.urdf.impl.drive_reconstruction import _validate_inferred_mimic_limits

        joint = ET.fromstring(
            '<joint type="prismatic"><limit velocity="0.5" lower="-1"/>'
            '<mimic joint="source" multiplier="2"/></joint>'
        )
        metadata = {
            "joint_type": "prismatic",
            "mimic": {"joint": "source", "multiplier": "2"},
            "limits": {"velocity": "0.5", "lower": "-1"},
        }
        self.assertEqual(set(_validate_inferred_mimic_limits(metadata, joint)), {"velocity", "lower"})
        joint.find("limit").set("velocity", "0.25")
        self.assertEqual(_validate_inferred_mimic_limits(metadata, joint), ["lower"])
        joint.find("mimic").set("multiplier", "3")
        self.assertEqual(_validate_inferred_mimic_limits(metadata, joint), [])
        joint.find("mimic").set("multiplier", "2")
        joint.set("type", "revolute")
        self.assertEqual(_validate_inferred_mimic_limits(metadata, joint), [])
        self.assertEqual(_validate_inferred_mimic_limits(["velocity"], joint), [])

    def test_unauthored_effort_requires_matching_provenance(self) -> None:
        """Preserve edited effort and reject stale or absent provenance."""
        import xml.etree.ElementTree as ET

        from isaacsim.asset.importer.urdf.impl.drive_reconstruction import _validate_unauthored_effort

        joint = ET.fromstring('<joint type="revolute"><limit effort="1000000"/></joint>')
        metadata = {"joint_type": "revolute", "effort": "1000000", "mimic": None}
        self.assertTrue(_validate_unauthored_effort(metadata, joint))
        self.assertFalse(_validate_unauthored_effort(None, joint))
        joint.find("limit").set("effort", "10")
        self.assertFalse(_validate_unauthored_effort(metadata, joint))
        joint.find("limit").set("effort", "1000000")
        ET.SubElement(joint, "mimic", joint="source")
        self.assertFalse(_validate_unauthored_effort(metadata, joint))
