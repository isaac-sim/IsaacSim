# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Static information panel with title, doc link, and overview text."""

from __future__ import annotations

from isaacsim.gui.components.ui_utils import setup_ui_headers

from ..constants import EXTENSION_NAME

_DOC_LINK = (
    "https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup_tutorials/tutorial_generate_robot_config.html"
)

_OVERVIEW = (
    "This utility helps you author an XRDF file describing your robot for use with Isaac cuMotion, NVIDIA's "
    "GPU-accelerated motion planner.  The XRDF contains a collision sphere representation of the robot used "
    "for collision avoidance, along with the joint metadata cuMotion needs to interpret the robot URDF.  "
    "Authoring a Lula Robot Description YAML file is also supported for backwards compatibility with legacy "
    "Lula-based algorithms such as RmpFlow, RRT, and Lula Kinematics; for new work, the cuMotion XRDF "
    "format is recommended.\n\n"
    "To begin using this editor, load a robot USD file onto the stage and press the 'Play' button.  In the "
    "'Selection Panel', select your robot from the 'Select Articulation' drop-down menu.  The 'Select Link' "
    "drop-down menu will populate once an Articulation has been selected.  The user may create collision "
    "spheres for the robot one link at a time.\n\n"
    "Joint Properties Panel:\nIn the Joint Properties Panel, the user chooses which joints are 'Active "
    "Joints' and specifies their default positions.  'Active Joints' are considered directly controllable, "
    "while 'Fixed Joints' are assumed to never move.  The 'Active Joints' set becomes the configuration "
    "space (``cspace``) that cuMotion plans over, and the default positions become the default cspace "
    "configuration the planner uses as its seed and resting pose.  For a typical robot arm with a gripper, "
    "the arm joints are marked 'Active' and the gripper joints are left 'Fixed' at a chosen default.  "
    "Default positions are also used by legacy Lula algorithms.  By default, all joints are marked as "
    "'Fixed Joints' and the user must explicitly choose which joints should be 'Active'.  "
    "Maximum jerk and acceleration limits must be specified for each active joint; cuMotion uses these as "
    "hard constraints when generating time-parameterized trajectories.\n\n"
    "Adding Collision Spheres:\nIn the 'Link Sphere Editor' panel paired with 'Editor Tools', the user may "
    "add collision spheres on a per-link basis.  Spheres are added with positions specified relative to the "
    "base of the selected link, with their position relative to the link being fixed.  Once a sphere has "
    "been created, the user may move it around, resize or delete it on the USD stage until it looks right. "
    "Additionally, the user may generate spheres for a link automatically or select any two spheres under a "
    "link and linearly interpolate to create more spheres connecting them.  In general, the user will want "
    "to fully cover the robot in spheres, using around 40-60 spheres total.  It is easiest to create such a "
    "set of spheres when individual spheres are allowed to slightly exceed the volume of the robot.\n\n"
    "Importing and Exporting:\nThe Robot Description Editor imports and exports cuMotion XRDF files as its "
    "primary format, and also supports legacy Lula robot description YAML files for backwards "
    "compatibility.  The editor does not represent every possible field in an XRDF file, so when exporting "
    "to an existing XRDF file path the user is given an option to pull data from the existing file that "
    "should not be overwritten by leaving the default setting to 'Merge With Existing XRDF'."
)


class InfoPanel:
    """Renders the static header / overview text for the extension window."""

    def __init__(self, ext_id: str, source_file: str) -> None:
        self._ext_id = ext_id
        self._source_file = source_file

    def build(self) -> None:
        """Build the static info section."""
        setup_ui_headers(self._ext_id, self._source_file, EXTENSION_NAME, _DOC_LINK, _OVERVIEW)
