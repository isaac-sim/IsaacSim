# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verifies fixed-joint merging for URDF models. Covers link removal, visual and collision transform composition, downstream joint reparenting, mass and inertia accumulation, and preservation of non-fixed joints."""

import math
import os
import shutil
import tempfile
import textwrap
import xml.etree.ElementTree as ET

import numpy as np
import omni.kit.test
from isaacsim.asset.importer.urdf.impl.urdf_utils import (
    _rewrite_relative_mesh_paths_to_absolute,
    _rotation_matrix_to_rpy,
    _rpy_to_rotation_matrix,
    merge_fixed_joints,
)


def _write_urdf(content: str, tmp_dir: str, name: str = "input.urdf") -> str:
    path = os.path.join(tmp_dir, name)
    with open(path, "w") as f:
        f.write(content)
    return path


def _parse_output(output_path: str) -> ET.Element:
    return ET.parse(output_path).getroot()


def _parse_output_with_comments(output_path: str) -> ET.Element:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(output_path, parser=parser).getroot()


def _comment_texts(elem: ET.Element) -> list[str]:
    return [(node.text or "").strip() for node in elem.iter() if node.tag is ET.Comment]


def _link_names(root: ET.Element) -> list[str]:
    return [link.get("name") for link in root.findall("link")]


def _joint_names(root: ET.Element) -> list[str]:
    return [j.get("name") for j in root.findall("joint")]


def _joint_types(root: ET.Element) -> dict[str, str]:
    return {j.get("name"): j.get("type") for j in root.findall("joint")}


def _get_origin_xyz(elem: ET.Element) -> list[float]:
    origin = elem.find("origin")
    if origin is None:
        return [0.0, 0.0, 0.0]
    return [float(v) for v in origin.get("xyz", "0 0 0").split()]


def _get_origin_rpy(elem: ET.Element) -> list[float]:
    origin = elem.find("origin")
    if origin is None:
        return [0.0, 0.0, 0.0]
    return [float(v) for v in origin.get("rpy", "0 0 0").split()]


def _find_link(root: ET.Element, name: str) -> ET.Element | None:
    for link in root.findall("link"):
        if link.get("name") == name:
            return link
    return None


class TestUrdfUtils(omni.kit.test.AsyncTestCase):
    """Test URDF pre-processing utility ``merge_fixed_joints``.

    Example:

    .. code-block:: python

        >>> import omni.kit.test
        >>> class Example(omni.kit.test.AsyncTestCase):
        ...     pass
        ...
    """

    async def setUp(self) -> None:
        """Create a temporary directory for URDF I/O.

        Example:

        .. code-block:: python

            >>> import tempfile
            >>> tempfile.mkdtemp()  # doctest: +SKIP
        """
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
        extension_path = ext_manager.get_extension_path(ext_id)
        self._comment_fixtures_dir = os.path.join(extension_path, "data", "urdf", "tests", "comment_preservation")
        self._tmpdir = tempfile.mkdtemp(prefix="urdf_utils_test_")
        self._output_path = os.path.join(self._tmpdir, "output.urdf")
        self._success = False

    def _comment_fixture(self, name: str) -> str:
        """Return the path to a comment-preservation URDF fixture."""
        return os.path.join(self._comment_fixtures_dir, name)

    async def tearDown(self) -> None:
        """Clean up temporary files.

        Example:

        .. code-block:: python

            >>> import shutil
            >>> shutil.rmtree("/tmp/test")  # doctest: +SKIP
        """
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    # -- no-op cases ---------------------------------------------------------

    async def test_no_fixed_joints_unchanged(self) -> None:
        """URDF with only revolute joints should be returned unchanged."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="link1">
                <visual><geometry><box size="1 1 1"/></geometry></visual>
              </link>
              <joint name="j1" type="revolute">
                <parent link="base"/>
                <child link="link1"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        self.assertEqual(_link_names(root), ["base", "link1"])
        self.assertEqual(_joint_names(root), ["j1"])
        self._success = True

    async def test_no_fixed_joints_preserves_xml_comments(self) -> None:
        """A no-op merge should preserve arbitrary comments and all breadcrumb types."""
        inp = self._comment_fixture("no_fixed_joints.urdf")
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output_with_comments(self._output_path)

        comments = _comment_texts(root)
        self.assertEqual(len(comments), 4)
        self.assertIn("arbitrary robot comment", comments)
        self.assertTrue(any("isaac:source_geometry" in text for text in comments))
        self.assertTrue(any("isaac:source_joint" in text for text in comments))
        self.assertTrue(any("isaac:source_drive" in text for text in comments))
        self._success = True

    async def test_relative_mesh_rewrite_preserves_xml_comments(self) -> None:
        """Rewriting a relative mesh path should not discard XML comments."""
        inp = os.path.join(self._tmpdir, "relative_mesh.urdf")
        shutil.copyfile(self._comment_fixture("relative_mesh.urdf"), inp)
        _rewrite_relative_mesh_paths_to_absolute(inp, self._tmpdir)
        root = _parse_output_with_comments(inp)

        mesh = root.find("link/visual/geometry/mesh")
        self.assertIsNotNone(mesh)
        self.assertEqual(mesh.get("filename"), os.path.join(self._tmpdir, "meshes", "body.obj"))
        comments = _comment_texts(root)
        self.assertEqual(len(comments), 2)
        self.assertIn("arbitrary robot comment", comments)
        self.assertTrue(any("isaac:source_geometry" in text for text in comments))
        self._success = True

    async def test_package_mesh_rewrite_after_relocation(self) -> None:
        """Relocated URDFs should resolve package meshes from their source package."""
        package_root = os.path.join(self._tmpdir, "test_package")
        source_urdf_dir = os.path.join(package_root, "urdf")
        mesh_dir = os.path.join(package_root, "meshes")
        os.makedirs(source_urdf_dir)
        os.makedirs(mesh_dir)
        mesh_path = os.path.join(mesh_dir, "body.obj")
        with open(mesh_path, "w") as f:
            f.write("o body\n")

        source_urdf = _write_urdf(
            textwrap.dedent("""\
                <robot name="test">
                  <link name="base"/>
                  <link name="child">
                    <visual>
                      <geometry><mesh filename="package://test_package/meshes/body.obj"/></geometry>
                    </visual>
                  </link>
                  <joint name="fixed" type="fixed">
                    <parent link="base"/><child link="child"/>
                  </joint>
                </robot>
            """),
            source_urdf_dir,
            "robot.urdf",
        )
        relocated_urdf = os.path.join(self._tmpdir, "scratch", "robot_merged.urdf")
        os.makedirs(os.path.dirname(relocated_urdf))
        merge_fixed_joints(source_urdf, relocated_urdf)

        _rewrite_relative_mesh_paths_to_absolute(relocated_urdf, source_urdf_dir)

        mesh = _parse_output(relocated_urdf).find("link/visual/geometry/mesh")
        self.assertIsNotNone(mesh)
        self.assertEqual(mesh.get("filename"), mesh_path)
        self._success = True

    async def test_package_mesh_rewrite_falls_back_from_invalid_mapping(self) -> None:
        """Invalid ROS package mappings should fall back to the source package."""
        package_root = os.path.join(self._tmpdir, "test_package")
        source_urdf_dir = os.path.join(package_root, "urdf")
        mesh_dir = os.path.join(package_root, "meshes")
        os.makedirs(source_urdf_dir)
        os.makedirs(mesh_dir)
        mesh_path = os.path.join(mesh_dir, "body.obj")
        with open(mesh_path, "w") as f:
            f.write("o body\n")

        relocated_urdf = _write_urdf(
            (
                '<robot name="test"><link name="base"><visual><geometry>'
                '<mesh filename="package://test_package/meshes/body.obj"/>'
                "</geometry></visual></link></robot>"
            ),
            self._tmpdir,
            "relocated.urdf",
        )
        invalid_package_root = os.path.join(self._tmpdir, "invalid_package")
        os.makedirs(invalid_package_root)
        _rewrite_relative_mesh_paths_to_absolute(
            relocated_urdf,
            source_urdf_dir,
            [{"name": "test_package", "path": invalid_package_root}],
        )

        mesh = _parse_output(relocated_urdf).find("link/visual/geometry/mesh")
        self.assertIsNotNone(mesh)
        self.assertEqual(mesh.get("filename"), mesh_path)
        self._success = True

    async def test_package_mesh_rewrite_prefers_explicit_mapping(self) -> None:
        """Explicit ROS package mappings should override source-directory discovery."""
        source_urdf_dir = os.path.join(self._tmpdir, "source", "urdf")
        mapped_package_root = os.path.join(self._tmpdir, "mapped_package")
        mesh_dir = os.path.join(mapped_package_root, "meshes")
        os.makedirs(source_urdf_dir)
        os.makedirs(mesh_dir)
        mesh_path = os.path.join(mesh_dir, "body.obj")
        with open(mesh_path, "w") as f:
            f.write("o body\n")

        relocated_urdf = _write_urdf(
            (
                '<robot name="test"><link name="base"><visual><geometry>'
                '<mesh filename="package://test_package/meshes/body.obj"/>'
                "</geometry></visual></link></robot>"
            ),
            self._tmpdir,
            "relocated.urdf",
        )
        _rewrite_relative_mesh_paths_to_absolute(
            relocated_urdf,
            source_urdf_dir,
            [{"name": "test_package", "path": mapped_package_root}],
        )

        mesh = _parse_output(relocated_urdf).find("link/visual/geometry/mesh")
        self.assertIsNotNone(mesh)
        self.assertEqual(mesh.get("filename"), mesh_path)
        self._success = True

    async def test_empty_robot(self) -> None:
        """A robot with no joints or links should not crash."""
        urdf = '<robot name="empty"></robot>'
        inp = _write_urdf(urdf, self._tmpdir)
        result = merge_fixed_joints(inp, self._output_path)
        self.assertEqual(result, self._output_path)
        root = _parse_output(self._output_path)
        self.assertEqual(len(root.findall("link")), 0)
        self._success = True

    async def test_returns_output_path(self) -> None:
        """merge_fixed_joints should return the output path."""
        urdf = '<robot name="r"><link name="a"/></robot>'
        inp = _write_urdf(urdf, self._tmpdir)
        result = merge_fixed_joints(inp, self._output_path)
        self.assertEqual(result, self._output_path)
        self._success = True

    # -- single fixed joint --------------------------------------------------

    async def test_single_fixed_joint_removes_child_link(self) -> None:
        """A single fixed joint should remove the child link and the joint."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="child">
                <visual><geometry><box size="1 1 1"/></geometry></visual>
              </link>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        self.assertEqual(_link_names(root), ["base"])
        self.assertEqual(len(root.findall("joint")), 0)
        self._success = True

    async def test_visual_transferred_to_parent(self) -> None:
        """Child's <visual> elements should appear on the parent after merge."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base">
                <visual><geometry><sphere radius="0.1"/></geometry></visual>
              </link>
              <link name="child">
                <visual><geometry><box size="1 1 1"/></geometry></visual>
              </link>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        visuals = base.findall("visual")
        self.assertEqual(len(visuals), 2)
        self._success = True

    async def test_fixed_joint_merge_preserves_breadcrumb_comments(self) -> None:
        """Moved geometry and reparented joints should retain their breadcrumbs."""
        inp = self._comment_fixture("fixed_joint_merge.urdf")
        with self.assertNoLogs("isaacsim.asset.importer.urdf.impl.urdf_utils", level="WARNING"):
            merge_fixed_joints(inp, self._output_path)
        root = _parse_output_with_comments(self._output_path)

        base = _find_link(root, "base")
        self.assertIsNotNone(base)
        visuals = base.findall("visual")
        self.assertEqual(len(visuals), 1)
        self.assertTrue(any("isaac:source_geometry" in text for text in _comment_texts(visuals[0])))

        collisions = base.findall("collision")
        self.assertEqual(len(collisions), 1)
        self.assertTrue(any("isaac:source_geometry" in text for text in _comment_texts(collisions[0])))

        moving_joint = next(joint for joint in root.findall("joint") if joint.get("name") == "moving_j")
        self.assertEqual(moving_joint.find("parent").get("link"), "base")
        joint_comments = _comment_texts(moving_joint)
        self.assertTrue(any("isaac:source_joint" in text for text in joint_comments))
        self.assertTrue(any("isaac:source_drive" in text for text in joint_comments))
        self._success = True

    async def test_collision_transferred_to_parent(self) -> None:
        """Child's <collision> elements should appear on the parent after merge."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="child">
                <collision><geometry><box size="1 1 1"/></geometry></collision>
                <collision><geometry><sphere radius="0.5"/></geometry></collision>
              </link>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        self.assertEqual(len(base.findall("collision")), 2)
        self._success = True

    # -- transform composition -----------------------------------------------

    async def test_visual_origin_composed(self) -> None:
        """Visual origin in child frame should be composed with the fixed joint transform."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="child">
                <visual>
                  <origin xyz="0.5 0 0" rpy="0 0 0"/>
                  <geometry><box size="1 1 1"/></geometry>
                </visual>
              </link>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        visual = base.findall("visual")[0]
        xyz = _get_origin_xyz(visual)
        np.testing.assert_allclose(xyz, [1.5, 0.0, 0.0], atol=1e-9)
        self._success = True

    async def test_collision_origin_composed(self) -> None:
        """Collision origin should be composed with the fixed joint transform."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="child">
                <collision>
                  <origin xyz="0 0.2 0" rpy="0 0 0"/>
                  <geometry><sphere radius="0.1"/></geometry>
                </collision>
              </link>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="0 0 3" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        collision = base.findall("collision")[0]
        xyz = _get_origin_xyz(collision)
        np.testing.assert_allclose(xyz, [0.0, 0.2, 3.0], atol=1e-9)
        self._success = True

    async def test_rotation_composition(self) -> None:
        """A 90-deg yaw in the joint plus a child translation should be rotated."""
        yaw = math.pi / 2
        urdf = textwrap.dedent(f"""\
            <robot name="test">
              <link name="base"/>
              <link name="child">
                <visual>
                  <origin xyz="1 0 0" rpy="0 0 0"/>
                  <geometry><box size="1 1 1"/></geometry>
                </visual>
              </link>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="0 0 0" rpy="0 0 {yaw}"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        visual = base.findall("visual")[0]
        xyz = _get_origin_xyz(visual)
        np.testing.assert_allclose(xyz, [0.0, 1.0, 0.0], atol=1e-9)
        self._success = True

    async def test_rotation_composition_at_gimbal_lock(self) -> None:
        """A -90-deg pitch in the joint should not flip re-parented joint or merged geometry rotations."""
        pitch = -math.pi / 2
        urdf = textwrap.dedent(f"""\
            <robot name="test">
              <link name="base"/>
              <link name="adapter">
                <visual>
                  <origin xyz="0 0 0" rpy="0.4 0 0"/>
                  <geometry><box size="1 1 1"/></geometry>
                </visual>
                <collision>
                  <origin xyz="0 0 0" rpy="0.4 0 0"/>
                  <geometry><box size="1 1 1"/></geometry>
                </collision>
              </link>
              <link name="end"/>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="adapter"/>
                <origin xyz="0 0 0" rpy="0.2 {pitch} 0.3"/>
              </joint>
              <joint name="revolute_j" type="revolute">
                <parent link="adapter"/>
                <child link="end"/>
                <origin xyz="0 0 0" rpy="0.4 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        expected = _rpy_to_rotation_matrix([0.2, pitch, 0.3]) @ _rpy_to_rotation_matrix([0.4, 0.0, 0.0])
        base = _find_link(root, "base")
        for elem in (root.findall("joint")[0], base.find("visual"), base.find("collision")):
            rotation = _rpy_to_rotation_matrix(_get_origin_rpy(elem))
            np.testing.assert_allclose(rotation, expected, atol=1e-9, err_msg=elem.tag)
        self._success = True

    async def test_rotation_matrix_to_rpy_gimbal_lock(self) -> None:
        """Converting a rotation with a +/-90-deg pitch to RPY and back should reproduce the rotation."""
        # +/-1.5707963 falls inside the gimbal-lock tolerance, where the pitch is snapped to +/-pi/2
        for pitch in (math.pi / 2, -math.pi / 2, 1.5707963, -1.5707963):
            rotation = _rpy_to_rotation_matrix([0.2, pitch, 0.3])
            rpy = _rotation_matrix_to_rpy(rotation)
            np.testing.assert_allclose(
                _rpy_to_rotation_matrix(list(rpy)), rotation, atol=1e-6, err_msg=f"pitch={pitch}"
            )
        self._success = True

    # -- joint re-parenting --------------------------------------------------

    async def test_revolute_reparented_to_parent(self) -> None:
        """A revolute joint from the child link should be re-parented to the parent."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="adapter"/>
              <link name="wheel"/>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="adapter"/>
                <origin xyz="0 0 1" rpy="0 0 0"/>
              </joint>
              <joint name="wheel_j" type="revolute">
                <parent link="adapter"/>
                <child link="wheel"/>
                <origin xyz="0 0 0.5" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        self.assertNotIn("adapter", _link_names(root))
        self.assertIn("base", _link_names(root))
        self.assertIn("wheel", _link_names(root))

        wheel_joint = root.findall("joint")[0]
        self.assertEqual(wheel_joint.get("name"), "wheel_j")
        self.assertEqual(wheel_joint.find("parent").get("link"), "base")
        self.assertEqual(wheel_joint.find("child").get("link"), "wheel")
        self._success = True

    async def test_reparented_joint_origin_composed(self) -> None:
        """Re-parented joint's origin should be composed with the fixed joint transform."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="adapter"/>
              <link name="end"/>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="adapter"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
              </joint>
              <joint name="revolute_j" type="revolute">
                <parent link="adapter"/>
                <child link="end"/>
                <origin xyz="2 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        revolute_joint = root.findall("joint")[0]
        xyz = _get_origin_xyz(revolute_joint)
        np.testing.assert_allclose(xyz, [3.0, 0.0, 0.0], atol=1e-9)
        self._success = True

    async def test_multiple_downstream_joints_reparented(self) -> None:
        """All downstream joints from the merged child should be re-parented."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="adapter"/>
              <link name="arm"/>
              <link name="hand"/>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="adapter"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
              </joint>
              <joint name="arm_j" type="revolute">
                <parent link="adapter"/>
                <child link="arm"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
              <joint name="hand_j" type="revolute">
                <parent link="adapter"/>
                <child link="hand"/>
                <origin xyz="0 1 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        self.assertNotIn("adapter", _link_names(root))
        for j in root.findall("joint"):
            self.assertEqual(j.find("parent").get("link"), "base")
        self._success = True

    # -- chains of fixed joints ----------------------------------------------

    async def test_chain_of_two_fixed_joints(self) -> None:
        """Base --(fixed)--> mid --(fixed)--> tip should collapse to just base."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="mid">
                <visual><geometry><box size="1 1 1"/></geometry></visual>
              </link>
              <link name="tip">
                <visual><geometry><sphere radius="0.1"/></geometry></visual>
              </link>
              <joint name="j1" type="fixed">
                <parent link="base"/>
                <child link="mid"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
              </joint>
              <joint name="j2" type="fixed">
                <parent link="mid"/>
                <child link="tip"/>
                <origin xyz="0 2 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        self.assertEqual(_link_names(root), ["base"])
        self.assertEqual(len(root.findall("joint")), 0)

        base = _find_link(root, "base")
        visuals = base.findall("visual")
        self.assertEqual(len(visuals), 2)
        self._success = True

    async def test_chain_transform_accumulation(self) -> None:
        """Transforms should accumulate correctly through a chain of fixed joints."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="mid"/>
              <link name="tip">
                <visual>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <geometry><box size="1 1 1"/></geometry>
                </visual>
              </link>
              <joint name="j1" type="fixed">
                <parent link="base"/>
                <child link="mid"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
              </joint>
              <joint name="j2" type="fixed">
                <parent link="mid"/>
                <child link="tip"/>
                <origin xyz="0 2 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        visual = base.findall("visual")[0]
        xyz = _get_origin_xyz(visual)
        np.testing.assert_allclose(xyz, [1.0, 2.0, 0.0], atol=1e-9)
        self._success = True

    async def test_chain_with_trailing_revolute(self) -> None:
        """Base --(fixed)--> mid --(fixed)--> adapter --(revolute)--> end."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="mid"/>
              <link name="adapter"/>
              <link name="end"/>
              <joint name="j1" type="fixed">
                <parent link="base"/>
                <child link="mid"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
              </joint>
              <joint name="j2" type="fixed">
                <parent link="mid"/>
                <child link="adapter"/>
                <origin xyz="0 1 0" rpy="0 0 0"/>
              </joint>
              <joint name="j3" type="revolute">
                <parent link="adapter"/>
                <child link="end"/>
                <origin xyz="0 0 1" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        self.assertEqual(sorted(_link_names(root)), ["base", "end"])
        joints = root.findall("joint")
        self.assertEqual(len(joints), 1)
        self.assertEqual(joints[0].get("name"), "j3")
        self.assertEqual(joints[0].find("parent").get("link"), "base")

        xyz = _get_origin_xyz(joints[0])
        np.testing.assert_allclose(xyz, [1.0, 1.0, 1.0], atol=1e-9)
        self._success = True

    # -- inertial merging ----------------------------------------------------

    async def test_mass_is_summed(self) -> None:
        """Total mass should be the sum of parent and child masses."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base">
                <inertial>
                  <mass value="2.0"/>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
                </inertial>
              </link>
              <link name="child">
                <inertial>
                  <mass value="3.0"/>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <inertia ixx="0.2" iyy="0.2" izz="0.2" ixy="0" ixz="0" iyz="0"/>
                </inertial>
              </link>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        mass_elem = base.find("inertial/mass")
        self.assertAlmostEqual(float(mass_elem.get("value")), 5.0)
        self._success = True

    async def test_com_weighted_average(self) -> None:
        """Combined CoM should be the mass-weighted average."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base">
                <inertial>
                  <mass value="1.0"/>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
                </inertial>
              </link>
              <link name="child">
                <inertial>
                  <mass value="1.0"/>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
                </inertial>
              </link>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="2 0 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        com_xyz = _get_origin_xyz(base.find("inertial"))
        np.testing.assert_allclose(com_xyz, [1.0, 0.0, 0.0], atol=1e-9)
        self._success = True

    async def test_inertia_parallel_axis_theorem(self) -> None:
        """Inertia tensor should reflect parallel axis shifts for both bodies."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base">
                <inertial>
                  <mass value="1.0"/>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
                </inertial>
              </link>
              <link name="child">
                <inertial>
                  <mass value="1.0"/>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/>
                </inertial>
              </link>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="2 0 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        inertia = base.find("inertial/inertia")

        self.assertAlmostEqual(float(inertia.get("ixx")), 0.0, places=6)
        self.assertAlmostEqual(float(inertia.get("iyy")), 2.0, places=6)
        self.assertAlmostEqual(float(inertia.get("izz")), 2.0, places=6)
        self.assertAlmostEqual(float(inertia.get("ixy")), 0.0, places=6)
        self.assertAlmostEqual(float(inertia.get("ixz")), 0.0, places=6)
        self.assertAlmostEqual(float(inertia.get("iyz")), 0.0, places=6)
        self._success = True

    async def test_child_without_inertial(self) -> None:
        """If the child has no <inertial>, parent's inertial should be unchanged."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base">
                <inertial>
                  <mass value="5.0"/>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
                </inertial>
              </link>
              <link name="child"/>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        self.assertAlmostEqual(float(base.find("inertial/mass").get("value")), 5.0)
        self._success = True

    async def test_parent_without_inertial_inherits_child(self) -> None:
        """If the parent has no <inertial>, it should get the child's."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="child">
                <inertial>
                  <mass value="3.0"/>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <inertia ixx="0.5" iyy="0.5" izz="0.5" ixy="0" ixz="0" iyz="0"/>
                </inertial>
              </link>
              <joint name="fixed_j" type="fixed">
                <parent link="base"/>
                <child link="child"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        base = _find_link(root, "base")
        self.assertAlmostEqual(float(base.find("inertial/mass").get("value")), 3.0)
        com_xyz = _get_origin_xyz(base.find("inertial"))
        np.testing.assert_allclose(com_xyz, [1.0, 0.0, 0.0], atol=1e-9)
        self._success = True

    # -- mixed topology ------------------------------------------------------

    async def test_fixed_between_two_revolutes(self) -> None:
        """Base --(revolute)--> A --(fixed)--> B --(revolute)--> C."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="A"/>
              <link name="B">
                <visual><geometry><box size="1 1 1"/></geometry></visual>
              </link>
              <link name="C"/>
              <joint name="j1" type="revolute">
                <parent link="base"/>
                <child link="A"/>
                <origin xyz="1 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
              <joint name="j_fixed" type="fixed">
                <parent link="A"/>
                <child link="B"/>
                <origin xyz="0 0 0.5" rpy="0 0 0"/>
              </joint>
              <joint name="j2" type="revolute">
                <parent link="B"/>
                <child link="C"/>
                <origin xyz="0 0 1" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        self.assertEqual(sorted(_link_names(root)), ["A", "C", "base"])
        types = _joint_types(root)
        self.assertEqual(types, {"j1": "revolute", "j2": "revolute"})

        j2 = [j for j in root.findall("joint") if j.get("name") == "j2"][0]
        self.assertEqual(j2.find("parent").get("link"), "A")
        xyz = _get_origin_xyz(j2)
        np.testing.assert_allclose(xyz, [0.0, 0.0, 1.5], atol=1e-9)

        link_a = _find_link(root, "A")
        self.assertEqual(len(link_a.findall("visual")), 1)
        self._success = True

    async def test_only_fixed_joints_removed(self) -> None:
        """Non-fixed joints should never be removed."""
        urdf = textwrap.dedent("""\
            <robot name="test">
              <link name="base"/>
              <link name="A"/>
              <link name="B"/>
              <link name="C"/>
              <joint name="j_rev" type="revolute">
                <parent link="base"/>
                <child link="A"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
              <joint name="j_cont" type="continuous">
                <parent link="A"/>
                <child link="B"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
                <axis xyz="0 0 1"/>
              </joint>
              <joint name="j_fixed" type="fixed">
                <parent link="B"/>
                <child link="C"/>
                <origin xyz="0 0 0" rpy="0 0 0"/>
              </joint>
            </robot>
        """)
        inp = _write_urdf(urdf, self._tmpdir)
        merge_fixed_joints(inp, self._output_path)
        root = _parse_output(self._output_path)

        types = _joint_types(root)
        self.assertNotIn("fixed", types.values())
        self.assertIn("revolute", types.values())
        self.assertIn("continuous", types.values())
        self.assertNotIn("C", _link_names(root))
        self._success = True
