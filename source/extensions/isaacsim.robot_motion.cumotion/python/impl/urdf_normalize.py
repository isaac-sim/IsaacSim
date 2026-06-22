# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""URDF normalization helpers for the urdfdom parser used by cuMotion.

cuMotion statically links ``urdfdom`` 4.0.1, which enforces several
strictnesses that go beyond what the URDF spec or other parsers
(``PyBullet``, ``FastUrdfLoader``, MoveIt's earlier loader, ...) require.
The most common one observed in practice is that ``<limit>`` elements lacking
an ``effort`` or ``velocity`` attribute cause the parse to abort with
``joint limit: no effort`` / ``joint limit: no velocity``, even though the
URDFs are otherwise valid and accepted everywhere else.

The functions in this module patch a URDF document in-memory so it satisfies
``urdfdom``'s strictness without changing kinematically meaningful values
(positions, axes, parents, geometry, limits that are already present, ...).
They are intentionally idempotent: a URDF that is already ``urdfdom``-clean
passes through unmodified.

This is the pre-processing step used by
:func:`isaacsim.robot_motion.cumotion.load_cumotion_robot`.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

#: Default ``effort`` attribute injected into ``<limit>`` elements that lack it.
#:
#: Matches the value used by
#: ``isaacsim.replicator.teleop.pink_urdf_export`` when generating URDFs for
#: ``Pinocchio`` / ``urdfdom`` consumption, so behavior is consistent across
#: the repository's URDF post-processing paths.
DEFAULT_EFFORT = 1000.0

#: Default ``velocity`` attribute injected into ``<limit>`` elements that lack it.
DEFAULT_VELOCITY = 1000.0

#: Joint types that ``urdfdom`` requires to carry a ``<limit>`` child.
_TYPES_REQUIRING_LIMIT = frozenset({"revolute", "prismatic"})


def _ensure_limit_attrs(limit_elem: ET.Element) -> bool:
    """Add ``effort`` / ``velocity`` attributes to a ``<limit>`` element if absent.

    ``urdfdom`` requires both ``effort`` and ``velocity`` whenever a ``<limit>``
    element is present, and rejects the URDF otherwise. ``lower`` and ``upper``
    are left untouched because ``urdfdom`` already defaults them to ``0``.

    Args:
        limit_elem: The ``<limit>`` element to patch in place.

    Returns:
        ``True`` if at least one attribute was added, ``False`` otherwise.
    """
    modified = False
    if "effort" not in limit_elem.attrib:
        limit_elem.set("effort", str(DEFAULT_EFFORT))
        modified = True
    if "velocity" not in limit_elem.attrib:
        limit_elem.set("velocity", str(DEFAULT_VELOCITY))
        modified = True
    return modified


def _ensure_limit_for_joint(joint_elem: ET.Element) -> bool:
    """Insert a default ``<limit>`` child for ``revolute`` / ``prismatic`` joints.

    ``urdfdom`` rejects ``revolute`` and ``prismatic`` joints that omit
    ``<limit>`` entirely with messages such as
    ``Joint X is of type REVOLUTE but it does not specify limits``. When no
    ``<limit>`` is present this function inserts a wide-open one
    (``lower=0 upper=0`` plus the default effort/velocity), which is the same
    fallback used by :mod:`isaacsim.replicator.teleop.pink_urdf_export`.

    If a ``<limit>`` already exists this delegates to
    :func:`_ensure_limit_attrs`.

    Args:
        joint_elem: The ``<joint>`` element to patch in place.

    Returns:
        ``True`` if the element was modified, ``False`` otherwise.
    """
    joint_type = joint_elem.get("type")
    limit = joint_elem.find("limit")

    if joint_type in _TYPES_REQUIRING_LIMIT and limit is None:
        ET.SubElement(
            joint_elem,
            "limit",
            attrib={
                "lower": "0",
                "upper": "0",
                "effort": str(DEFAULT_EFFORT),
                "velocity": str(DEFAULT_VELOCITY),
            },
        )
        return True

    if limit is not None:
        return _ensure_limit_attrs(limit)

    return False


def _ensure_safety_controller(joint_elem: ET.Element) -> bool:
    """Add ``k_velocity="0"`` to ``<safety_controller>`` elements that lack it.

    ``urdfdom`` aborts with ``joint safety: no k_velocity`` if the attribute
    is missing. The other safety attributes (``k_position``, ``soft_lower_limit``,
    ``soft_upper_limit``) are optional in ``urdfdom`` and default to ``0``.

    Args:
        joint_elem: The ``<joint>`` element whose ``<safety_controller>`` child
            (if any) should be patched.

    Returns:
        ``True`` if an attribute was added, ``False`` otherwise.
    """
    safety = joint_elem.find("safety_controller")
    if safety is None or "k_velocity" in safety.attrib:
        return False
    safety.set("k_velocity", "0")
    return True


def _drop_empty_dynamics(joint_elem: ET.Element) -> bool:
    """Remove ``<dynamics/>`` elements that carry neither ``damping`` nor ``friction``.

    ``urdfdom`` rejects empty ``<dynamics>`` elements outright. Dropping the
    element keeps the joint dynamically default (which is the same effect
    callers presumably intended by writing an empty ``<dynamics/>``).

    Args:
        joint_elem: The ``<joint>`` element to patch in place.

    Returns:
        ``True`` if an empty ``<dynamics>`` was removed, ``False`` otherwise.
    """
    dynamics = joint_elem.find("dynamics")
    if dynamics is None:
        return False
    if "damping" in dynamics.attrib or "friction" in dynamics.attrib:
        return False
    joint_elem.remove(dynamics)
    return True


def _drop_malformed_mimic(joint_elem: ET.Element) -> bool:
    """Remove ``<mimic>`` elements that do not specify a ``joint`` attribute.

    ``urdfdom`` aborts with ``joint mimic: no mimic joint specified`` for such
    elements. Without a target there is nothing meaningful to mirror, so the
    element is dropped rather than fabricated.

    Args:
        joint_elem: The ``<joint>`` element to patch in place.

    Returns:
        ``True`` if a malformed ``<mimic>`` was removed, ``False`` otherwise.
    """
    mimic = joint_elem.find("mimic")
    if mimic is None or "joint" in mimic.attrib:
        return False
    joint_elem.remove(mimic)
    return True


def _normalize_joint(joint_elem: ET.Element) -> bool:
    """Apply every joint-scoped urdfdom-compatibility fixup to ``joint_elem``.

    Args:
        joint_elem: URDF joint element to normalize.

    Returns:
        ``True`` if the joint was modified by any fixup, ``False`` otherwise.
    """
    modified = False
    modified |= _ensure_limit_for_joint(joint_elem)
    modified |= _ensure_safety_controller(joint_elem)
    modified |= _drop_empty_dynamics(joint_elem)
    modified |= _drop_malformed_mimic(joint_elem)
    return modified


def normalize_urdf_for_urdfdom(urdf_text: str) -> str:
    """Return a URDF string normalized for ``urdfdom`` 4.0.1.

    Applies the following idempotent fixups:

    * For every ``<joint>``:

      * If type is ``revolute`` or ``prismatic`` and the joint has no
        ``<limit>``, insert ``<limit lower="0" upper="0" effort="1000"
        velocity="1000"/>``.
      * Otherwise, if ``<limit>`` is present, add ``effort`` and/or
        ``velocity`` attributes with the defaults from :data:`DEFAULT_EFFORT`
        / :data:`DEFAULT_VELOCITY` when they are missing.
      * Add ``k_velocity="0"`` to any ``<safety_controller>`` that lacks it.
      * Drop empty ``<dynamics/>`` elements.
      * Drop ``<mimic>`` elements that have no ``joint`` attribute.

    If ``urdf_text`` cannot be parsed as XML it is returned unchanged - the
    downstream parser is then responsible for surfacing the malformed-XML
    error. The function is otherwise idempotent: a clean URDF round-trips with
    only formatting differences introduced by ``ElementTree``'s serializer.

    Args:
        urdf_text: The URDF document as a string.

    Returns:
        The normalized URDF document as a string, ready to be passed to
        :func:`cumotion.load_robot_from_memory`.
    """
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        # Defer to urdfdom so the user gets the canonical XML diagnostic.
        return urdf_text

    modified = False
    for joint in root.iter("joint"):
        modified |= _normalize_joint(joint)

    if not modified:
        return urdf_text

    return ET.tostring(root, encoding="unicode")
