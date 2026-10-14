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

"""Sim-free unit tests for the typed policy environment config views."""

from __future__ import annotations

import math
import tempfile
import unittest
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

import omni.kit.test
from isaacsim.robot.policy.examples.env_config import PolicyEnvConfig
from isaacsim.robot.policy.examples.runtime import initialize_articulation
from isaacsim.robot.policy.examples.usd_props import apply_newton_robot_defaults
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

_JOINT_NAMES = ["joint_a", "joint_b"]


def _make_config() -> dict[str, Any]:
    """Build a minimal valid exported env config covering every typed view.

    Returns:
        A parsed-config-shaped dictionary.
    """
    return {
        "decimation": 4,
        "episode_length_s": 8.0,
        "sim": {
            "dt": 0.005,
            "render_interval": 4,
            "physx": {"solver_type": 1},
            "physics": {"default_shape_cfg": {"gap": 0.01, "ke": 2500.0}},
        },
        "scene": {
            "robot": {
                "init_state": {
                    "pos": [0.0, 0.0, 0.6],
                    "rot": [0.1, 0.2, 0.3, 0.9],
                    "joint_pos": {"joint_a": 0.25, "joint_b": -0.5},
                    "joint_vel": 0.0,
                },
                "actuators": {
                    "legs": {
                        "joint_names_expr": ["joint_a", "joint_b"],
                        "effort_limit_sim": 80.0,
                        "velocity_limit_sim": None,
                        "stiffness": {"joint_a": 60.0, "joint_b": 40.0},
                        "damping": 5.0,
                        "armature": 0.01,
                    }
                },
                "spawn": {
                    "usd_path": "/assets/robot.usd",
                    "articulation_props": {"enabled_self_collisions": False},
                    "rigid_props": {"disable_gravity": False},
                    "joint_drive_props": {"ensure_drives_exist": True},
                },
            },
            "contact_forces": {"prim_path": "/World/robot/.*"},
            "cabinet": {
                "spawn": {"usd_path": "/assets/cabinet.usd"},
                "init_state": {
                    "pos": [0.8, 0.0, 0.4],
                    "rot": [0.0, 0.0, 1.0, 0.0],
                    "joint_pos": {"drawer_joint": 0.1},
                    "joint_vel": 0.0,
                },
            },
        },
        "events": {
            "robot_material": {
                "mode": "startup",
                "func": "isaaclab.envs.mdp.events:randomize_rigid_body_material",
                "params": {
                    "asset_cfg": {"name": "robot", "body_names": ".*"},
                    "static_friction_range": [0.8, 1.2],
                    "dynamic_friction_range": [1.1, 1.5],
                    "restitution_range": [0.0, 0.2],
                    "make_consistent": True,
                    "num_buckets": 16,
                },
            }
        },
        "observations": {
            "policy": {
                "concatenate_terms": True,
                "concatenate_dim": -1,
                "enable_corruption": True,
                "history_length": None,
                "flatten_history_dim": True,
                "base_lin_vel": {
                    "func": "isaaclab.envs.mdp.observations:base_lin_vel",
                    "params": {},
                    "modifiers": None,
                    "noise": {"n_min": -0.1, "n_max": 0.1},
                    "clip": None,
                    "scale": None,
                    "history_length": 0,
                    "flatten_history_dim": True,
                },
                "velocity_commands": {
                    "func": "isaaclab.envs.mdp.observations:generated_commands",
                    "params": {"command_name": "base_velocity"},
                    "modifiers": None,
                    "noise": None,
                    "clip": [-10.0, 10.0],
                    "scale": 2.0,
                    "history_length": 0,
                    "flatten_history_dim": True,
                },
                "height_scan": None,
            }
        },
        "actions": {
            "joint_pos": {
                "class_type": "isaaclab.envs.mdp.actions.joint_actions:JointPositionAction",
                "asset_name": "robot",
                "debug_vis": False,
                "clip": None,
                "joint_names": [".*"],
                "scale": 0.5,
                "offset": 0.0,
                "preserve_order": False,
                "use_default_offset": True,
            }
        },
    }


class TestPolicyEnvConfigConstruction(omni.kit.test.AsyncTestCase):
    """Validate construction, timing, spawn, and root-pose views."""

    def test_timing_and_spawn_views(self) -> None:
        """The timing and spawn views expose their exported configuration."""
        config = PolicyEnvConfig(_make_config())
        self.assertEqual(config.timing.decimation, 4)
        self.assertEqual(config.timing.physics_dt, 0.005)
        self.assertEqual(config.timing.render_interval, 4)
        self.assertEqual(config.spawn.usd_path, "/assets/robot.usd")
        self.assertEqual(dict(config.spawn.articulation_props), {"enabled_self_collisions": False})
        self.assertEqual(dict(config.spawn.rigid_body_props), {"disable_gravity": False})
        self.assertEqual(dict(config.spawn.joint_drive_props), {"ensure_drives_exist": True})
        self.assertEqual(dict(config.newton_shape_defaults), {"gap": 0.01, "ke": 2500.0})

    def test_initial_root_pose_converts_xyzw_to_wxyz(self) -> None:
        """The exported xyzw root quaternion converts to wxyz at this boundary."""
        config = PolicyEnvConfig(_make_config())
        position, orientation = config.initial_root_pose
        self.assertEqual(position, [0.0, 0.0, 0.6])
        self.assertEqual(orientation, [0.9, 0.1, 0.2, 0.3])

    def test_legacy_initial_root_pose_preserves_wxyz(self) -> None:
        """A legacy PhysX config's root quaternion is already in wxyz order."""
        data = _make_config()
        data["sim"].pop("physics")
        data["scene"]["robot"]["init_state"]["rot"] = [0.9, 0.1, 0.2, 0.3]

        _, orientation = PolicyEnvConfig(data).initial_root_pose

        self.assertEqual(orientation, [0.9, 0.1, 0.2, 0.3])

    def test_task_scene_views_are_deterministic(self) -> None:
        """Task asset, reset state, episode length, and material ranges resolve once."""
        config = PolicyEnvConfig(_make_config())

        self.assertEqual(config.episode_length_s, 8.0)
        self.assertEqual(config.scene_entity_usd_path("cabinet"), "/assets/cabinet.usd")
        self.assertEqual(config.scene_entity_root_pose("cabinet"), ([0.8, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0]))
        self.assertEqual(
            config.scene_entity_joint_default_state("cabinet", ["drawer_joint", "door_joint"]),
            ((0.1, 0.0), (0.0, 0.0)),
        )
        self.assertEqual(len(config.startup_material_events), 1)
        event = config.startup_material_events[0]
        self.assertEqual(event.name, "robot_material")
        self.assertEqual(event.entity_name, "robot")
        self.assertEqual(event.body_patterns, (".*",))
        self.assertEqual(event.static_friction, 1.0)
        self.assertEqual(event.dynamic_friction, 1.0)
        self.assertEqual(event.restitution, 0.1)


class TestJointProperties(omni.kit.test.AsyncTestCase):
    """Validate strict per-joint property resolution."""

    def test_joint_properties_resolve_in_order(self) -> None:
        """Properties resolve per joint in the requested order."""
        config = PolicyEnvConfig(_make_config())
        properties = config.joint_properties(_JOINT_NAMES)
        self.assertEqual(properties.joint_names, ("joint_a", "joint_b"))
        self.assertEqual(properties.effort_limits, (80.0, 80.0))
        self.assertEqual(properties.velocity_limits, (None, None))
        self.assertEqual(properties.stiffness, (60.0, 40.0))
        self.assertEqual(properties.damping, (5.0, 5.0))
        self.assertEqual(properties.armature, (0.01, 0.01))
        self.assertEqual(properties.default_positions, (0.25, -0.5))
        self.assertEqual(properties.default_velocities, (0.0, 0.0))

    def test_unlisted_initial_joint_position_defaults_to_zero(self) -> None:
        """A partial Isaac Lab initial-position mapping leaves unmatched joints at zero."""
        data = _make_config()
        data["scene"]["robot"]["init_state"]["joint_pos"] = {"joint_a": 0.25}

        properties = PolicyEnvConfig(data).joint_properties(_JOINT_NAMES)

        self.assertEqual(properties.default_positions, (0.25, 0.0))

    def test_mapping_properties_resolve_independently_of_group_pattern(self) -> None:
        """Narrow property patterns resolve within a broadly matched actuator group."""
        data = _make_config()
        actuator = data["scene"]["robot"]["actuators"]["legs"]
        actuator["joint_names_expr"] = ["joint_.*"]
        actuator["stiffness"] = {"joint_a": 60.0, "joint_b": 40.0}
        actuator["damping"] = {"joint_a": 6.0, "joint_b": 4.0}

        properties = PolicyEnvConfig(data).joint_properties(_JOINT_NAMES)

        self.assertEqual(properties.stiffness, (60.0, 40.0))
        self.assertEqual(properties.damping, (6.0, 4.0))

    def test_unmatched_joint_rejected(self) -> None:
        """A joint matching no actuator group pattern fails naming the joint."""
        config = PolicyEnvConfig(_make_config())
        with self.assertRaisesRegex(ValueError, "'joint_c'"):
            config.joint_properties(["joint_a", "joint_c"])

    def test_overlapping_actuator_groups_rejected(self) -> None:
        """A joint matched by two actuator groups fails naming both groups."""
        data = _make_config()
        data["scene"]["robot"]["actuators"]["arms"] = {
            "joint_names_expr": ["joint_b"],
            "stiffness": 10.0,
            "damping": 1.0,
            "armature": 0.0,
        }
        config = PolicyEnvConfig(data)
        with self.assertRaisesRegex(ValueError, "legs.*arms|arms.*legs"):
            config.joint_properties(_JOINT_NAMES)

    def test_incomplete_pattern_keyed_property_rejected(self) -> None:
        """A pattern-keyed gain mapping that does not cover a joint fails."""
        data = _make_config()
        data["scene"]["robot"]["actuators"]["legs"]["stiffness"] = {"joint_a": 60.0}
        config = PolicyEnvConfig(data)
        with self.assertRaisesRegex(ValueError, "'joint_b'.*stiffness"):
            config.joint_properties(_JOINT_NAMES)

    def test_literal_joint_pattern_matches_uniquified_usd_name(self) -> None:
        """A literal config name remains compatible with an imported USD suffix.

        Migrated from ``test_config_loader.py``: USD imports may uniquify a joint name (the
        Menagerie ``torso`` joint imports as ``torso_1``), and the exported config still keys
        every property by the trained literal name.
        """
        data = {
            "decimation": 4,
            "sim": {"dt": 0.005, "render_interval": 4},
            "scene": {
                "robot": {
                    "actuators": {
                        "torso": {
                            "joint_names_expr": ["torso"],
                            "effort_limit": 300,
                            "velocity_limit": 7.5,
                            "velocity_limit_sim": 100,
                            "stiffness": {"torso": 200},
                            "damping": {"torso": 5},
                            "armature": 0.1,
                        }
                    },
                    "init_state": {
                        "joint_pos": {"torso": 0.25},
                        "joint_vel": {"torso": 0.0},
                    },
                }
            },
        }
        properties = PolicyEnvConfig(data).joint_properties(["torso_1"])
        self.assertEqual(properties.effort_limits, (300.0,))
        self.assertEqual(properties.velocity_limits, (100.0,))
        self.assertEqual(properties.stiffness, (200,))
        self.assertEqual(properties.damping, (5,))
        self.assertEqual(properties.armature, (0.1,))
        self.assertEqual(properties.default_positions, (0.25,))
        self.assertEqual(properties.default_velocities, (0.0,))

        with self.assertRaisesRegex(ValueError, "'torso_extra'.*does not match"):
            PolicyEnvConfig(data).joint_properties(["torso_extra"])

    def test_literal_joint_pattern_does_not_match_arbitrary_prefix(self) -> None:
        """A literal group does not steal a longer joint name that merely shares its prefix."""
        data = _make_config()
        legs = data["scene"]["robot"]["actuators"]["legs"]
        legs["joint_names_expr"] = ["joint_a"]
        data["scene"]["robot"]["actuators"]["arms"] = {
            "joint_names_expr": ["joint_ab"],
            "effort_limit_sim": 40.0,
            "velocity_limit_sim": None,
            "stiffness": 20.0,
            "damping": 2.0,
            "armature": 0.0,
        }
        data["scene"]["robot"]["init_state"]["joint_pos"]["joint_ab"] = 0.75

        properties = PolicyEnvConfig(data).joint_properties(["joint_a", "joint_ab"])

        self.assertEqual(properties.stiffness, (60.0, 20.0))
        self.assertEqual(properties.default_positions, (0.25, 0.75))

    def test_actuator_velocity_limit_does_not_replace_simulation_limit(self) -> None:
        """An actuator-model velocity limit does not overwrite an imported simulation limit.

        Migrated from ``test_config_loader.py``: only ``velocity_limit_sim`` configures
        physics; a configured actuator-model ``velocity_limit`` must not leak into it.
        """
        data = _make_config()
        data["scene"]["robot"]["actuators"]["legs"]["velocity_limit"] = 7.5
        properties = PolicyEnvConfig(data).joint_properties(_JOINT_NAMES)
        self.assertEqual(properties.velocity_limits, (None, None))

    def test_unset_armature_preserves_imported_value(self) -> None:
        """A null exported armature remains unset rather than becoming zero."""
        data = _make_config()
        data["scene"]["robot"]["actuators"]["legs"]["armature"] = None
        properties = PolicyEnvConfig(data).joint_properties(_JOINT_NAMES)
        self.assertEqual(properties.armature, (None, None))

    @patch("isaacsim.robot.policy.examples.runtime.get_physics_simulation_interface")
    @patch("isaacsim.robot.policy.examples.runtime.SimulationManager.get_active_physics_engine", return_value="newton")
    def test_initialize_writes_only_configured_armatures(
        self, active_engine: MagicMock, simulation_interface: MagicMock
    ) -> None:
        """Initialization leaves imported armatures untouched where the config is null.

        Args:
            active_engine: Mocked active-engine query.
            simulation_interface: Mocked physics simulation interface.
        """
        data = _make_config()
        data["scene"]["robot"]["actuators"]["legs"]["armature"] = {"joint_a": None, "joint_b": 0.02}
        articulation = MagicMock()
        articulation.dof_names = list(_JOINT_NAMES)

        initialize_articulation(
            articulation,
            PolicyEnvConfig(data),
            _JOINT_NAMES,
            {"joint_a": "position", "joint_b": "position"},
        )

        articulation.set_dof_armatures.assert_called_once_with([0.02], dof_indices=[1])

    def test_explicit_actuator_effort_limit_does_not_replace_simulation_limit(self) -> None:
        """An explicit actuator clamp does not replace the imported solver effort limit.

        Migrated from ``test_config_loader.py``: for exported explicit actuator classes the
        ``effort_limit`` clamps the actuator model, so an unset ``effort_limit_sim`` preserves
        the imported USD value instead of inheriting the clamp.
        """
        data = _make_config()
        data["scene"]["robot"]["actuators"]["legs"] = {
            "class_type": "isaaclab.actuators.actuator_net:ActuatorNetLSTM",
            "joint_names_expr": ["joint_a", "joint_b"],
            "effort_limit": 80,
            "effort_limit_sim": None,
            "network_file": "omniverse://dummy/net.pt",
            "stiffness": 0,
            "damping": 0,
            "armature": 0,
        }
        properties = PolicyEnvConfig(data).joint_properties(_JOINT_NAMES)
        self.assertEqual(properties.effort_limits, (None, None))


class TestActuatorModelSpecs(omni.kit.test.AsyncTestCase):
    """Validate strict actuator model spec resolution."""

    def test_implicit_groups_produce_no_specs(self) -> None:
        """Implicit and classless actuator groups produce no actuator models."""
        config = PolicyEnvConfig(_make_config())
        self.assertEqual(config.actuator_model_specs(_JOINT_NAMES), [])

    def test_non_implicit_group_resolves_spec(self) -> None:
        """A DC motor group resolves per-joint parameters into one spec."""
        data = _make_config()
        data["scene"]["robot"]["actuators"] = {
            "legs": {
                "class_type": "isaaclab.actuators.actuator_pd:DCMotor",
                "joint_names_expr": ["joint_a", "joint_b"],
                "effort_limit": 23.5,
                "velocity_limit": 7.5,
                "saturation_effort": 30.0,
                "stiffness": 25.0,
                "damping": 0.5,
                "armature": 0.0,
            }
        }
        config = PolicyEnvConfig(data)
        specs = config.actuator_model_specs(_JOINT_NAMES)
        self.assertEqual(len(specs), 1)
        self.assertEqual(specs[0].class_name, "DCMotor")
        self.assertEqual(specs[0].joints, ("joint_a", "joint_b"))
        self.assertEqual(specs[0].effort_limit, (23.5, 23.5))
        self.assertEqual(specs[0].velocity_limit, (7.5, 7.5))
        self.assertEqual(specs[0].saturation_effort, (30.0, 30.0))

    def test_delayed_dc_motor_resolves_dc_clamping_and_delay(self) -> None:
        """Compose an exported delayed DC motor from PD control, DC clamping, and delay."""
        from isaacsim.robot.policy.examples.utils.newton_actuators import build_newton_actuator_configs
        from newton.actuators import ClampingDCMotor, Delay, DrivePD

        data = _make_config()
        data["scene"]["robot"]["actuators"] = {
            "legs": {
                "class_type": "agile.rl_env.mdp.actuators.actuators:DelayedDCMotor",
                "joint_names_expr": ["joint_a", "joint_b"],
                "effort_limit": None,
                "velocity_limit": None,
                "effort_limit_sim": {"joint_a": 23.5, "joint_b": 24.5},
                "velocity_limit_sim": {"joint_a": 7.5, "joint_b": 8.5},
                "saturation_effort": 30.0,
                "stiffness": {"joint_a": 25.0, "joint_b": 35.0},
                "damping": 0.5,
                "armature": 0.0,
                "min_delay": 0,
                "max_delay": 4,
            }
        }

        specs = PolicyEnvConfig(data).actuator_model_specs(_JOINT_NAMES)
        self.assertEqual(len(specs), 1)
        spec = specs[0]
        self.assertEqual(spec.class_name, "DelayedDCMotor")
        self.assertEqual(spec.joints, ("joint_a", "joint_b"))
        self.assertEqual(spec.effort_limit, (23.5, 24.5))
        self.assertEqual(spec.velocity_limit, (7.5, 8.5))
        self.assertEqual(spec.saturation_effort, (30.0, 30.0))
        self.assertEqual(spec.max_delay, 4)

        actuator_pairs = build_newton_actuator_configs(spec, num_robots=1, device="cpu")
        self.assertEqual([joint_names for _, joint_names in actuator_pairs], [_JOINT_NAMES])
        self.assertEqual(len(actuator_pairs), 1)
        actuator, _ = actuator_pairs[0]
        self.assertIsInstance(actuator.controller, DrivePD)
        self.assertEqual(len(actuator.clamping), 1)
        self.assertIsInstance(actuator.clamping[0], ClampingDCMotor)
        self.assertIsInstance(actuator.delay, Delay)

    def test_actuator_specs_ignore_groups_outside_policy_joint_subset(self) -> None:
        """Ignore explicit actuator groups that own none of the requested joints."""
        data = _make_config()
        data["scene"]["robot"]["actuators"] = {
            "policy_joints": {
                "class_type": "agile.rl_env.mdp.actuators.actuators:DelayedDCMotor",
                "joint_names_expr": ["joint_a", "joint_b"],
                "effort_limit_sim": 80.0,
                "velocity_limit_sim": 10.0,
                "saturation_effort": 100.0,
                "stiffness": 25.0,
                "damping": 0.5,
                "max_delay": 4,
            },
            "upper_body": {
                "class_type": "agile.rl_env.mdp.actuators.actuators:DelayedDCMotor",
                "joint_names_expr": ["shoulder_.*"],
            },
        }

        specs = PolicyEnvConfig(data).actuator_model_specs(_JOINT_NAMES)

        self.assertEqual(len(specs), 1)
        self.assertEqual(specs[0].class_name, "DelayedDCMotor")
        self.assertEqual(specs[0].joints, ("joint_a", "joint_b"))


class TestNewtonRobotDefaults(omni.kit.test.AsyncTestCase):
    """Validate robot-local Newton import-default authoring."""

    def test_defaults_preserve_authored_values_and_environment(self) -> None:
        """Fallbacks apply only below the robot and preserve authored source properties."""
        stage = Usd.Stage.CreateInMemory()
        template = UsdGeom.Xform.Define(stage, "/Template").GetPrim()
        template_collision = UsdGeom.Cube.Define(stage, "/Template/Collision").GetPrim()
        UsdPhysics.CollisionAPI.Apply(template_collision)
        template_collision.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.002)

        source_material = UsdShade.Material.Define(stage, "/Materials/Source")
        UsdPhysics.MaterialAPI.Apply(source_material.GetPrim())
        source_material.GetPrim().ApplyAPI("NewtonMaterialAPI")
        source_material.GetPrim().GetAttribute("newton:contactStiffness").Set(9000.0)
        source_material.GetPrim().GetAttribute("physics:restitution").Set(0.4)
        visual_material = UsdShade.Material.Define(stage, "/Materials/Visual")
        binding_api = UsdShade.MaterialBindingAPI.Apply(template_collision)
        binding_api.Bind(visual_material)
        binding_api.Bind(source_material, materialPurpose="physics")

        robot = UsdGeom.Xform.Define(stage, "/Robot").GetPrim()
        link = stage.DefinePrim("/Robot/Link")
        link.GetReferences().AddInternalReference(template.GetPath())
        link.SetInstanceable(True)
        default_joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/DefaultJoint").GetPrim()
        linear_joint = UsdPhysics.PrismaticJoint.Define(stage, "/Robot/LinearJoint").GetPrim()
        d6_joint = UsdPhysics.Joint.Define(stage, "/Robot/D6Joint").GetPrim()
        UsdPhysics.LimitAPI.Apply(d6_joint, "rotX")
        UsdPhysics.LimitAPI.Apply(d6_joint, "transZ")
        authored_joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/AuthoredJoint").GetPrim()
        authored_joint.CreateAttribute("mjc:armature", Sdf.ValueTypeNames.Float).Set(0.2)
        authored_joint.CreateAttribute("mjc:solreflimit", Sdf.ValueTypeNames.DoubleArray).Set([0.02, 1.0])
        environment_collision = UsdGeom.Cube.Define(stage, "/Environment/Collision").GetPrim()
        UsdPhysics.CollisionAPI.Apply(environment_collision)

        counts = apply_newton_robot_defaults(
            stage,
            robot.GetPath(),
            {
                "ke": 2500.0,
                "kd": 100.0,
                "kf": 1000.0,
                "ka": 0.0,
                "mu": 1.0,
                "restitution": 0.0,
                "margin": 0.0,
                "gap": 0.01,
            },
            {"armature": 0.0, "limit_ke": 10000.0, "limit_kd": 10.0},
        )

        robot_collision = stage.GetPrimAtPath("/Robot/Link/Collision")
        local_material, _ = UsdShade.MaterialBindingAPI(robot_collision).ComputeBoundMaterial("physics")
        bound_visual_material, _ = UsdShade.MaterialBindingAPI(robot_collision).ComputeBoundMaterial()
        self.assertEqual(counts, {"collision_shapes": 1, "materials": 1, "joints": 3})
        self.assertFalse(link.IsInstance())
        self.assertFalse(robot_collision.GetAttribute("newton:contactMargin").HasAuthoredValueOpinion())
        self.assertAlmostEqual(robot_collision.GetAttribute("newton:contactGap").Get(), 0.01)
        self.assertEqual(bound_visual_material.GetPath(), visual_material.GetPath())
        self.assertTrue(str(local_material.GetPath()).startswith("/Robot/PolicyNewtonDefaults/Materials/"))
        self.assertAlmostEqual(local_material.GetPrim().GetAttribute("newton:contactStiffness").Get(), 9000.0)
        self.assertAlmostEqual(local_material.GetPrim().GetAttribute("newton:contactDamping").Get(), 100.0)
        self.assertAlmostEqual(local_material.GetPrim().GetAttribute("physics:restitution").Get(), 0.4)
        self.assertAlmostEqual(local_material.GetPrim().GetAttribute("physics:dynamicFriction").Get(), 1.0)
        self.assertAlmostEqual(default_joint.GetAttribute("newton:armature").Get(), 0.0)
        self.assertAlmostEqual(
            default_joint.GetAttribute("newton:limitStiffness").Get(), 10000.0 * math.pi / 180.0, places=4
        )
        self.assertAlmostEqual(default_joint.GetAttribute("newton:limitDamping").Get(), 10.0 * math.pi / 180.0)
        self.assertAlmostEqual(linear_joint.GetAttribute("newton:limitStiffness").Get(), 10000.0)
        self.assertAlmostEqual(linear_joint.GetAttribute("newton:limitDamping").Get(), 10.0)
        self.assertAlmostEqual(d6_joint.GetAttribute("newton:armature").Get(), 0.0)
        self.assertFalse(d6_joint.GetAttribute("newton:limitStiffness").HasAuthoredValueOpinion())
        self.assertFalse(authored_joint.GetAttribute("newton:armature").HasAuthoredValueOpinion())
        self.assertEqual(list(authored_joint.GetAttribute("mjc:solreflimit").Get()), [0.02, 1.0])
        self.assertFalse(authored_joint.GetAttribute("newton:limitStiffness").HasAuthoredValueOpinion())
        self.assertFalse(authored_joint.GetAttribute("newton:limitDamping").HasAuthoredValueOpinion())
        self.assertFalse(environment_collision.HasAPI("NewtonCollisionAPI"))


class TestFromFile(omni.kit.test.AsyncTestCase):
    """Validate file loading against injected-dict parity."""

    _YAML = """\
decimation: 2
sim:
  dt: 0.01
  render_interval: 2
scene:
  robot:
    init_state:
      pos: [0.0, 0.0, 0.5]
      rot: !!python/tuple [0.0, 0.0, 0.0, 1.0]
      joint_pos: 0.0
      joint_vel: 0.0
    actuators:
      all:
        joint_names_expr: [".*"]
        stiffness: 10.0
        damping: 1.0
        armature: 0.0
    spawn:
      usd_path: /assets/bot.usd
      semantic_tags: !custom_tag {kind: robot}
observations:
  policy:
    joint_pos:
      func: isaaclab.envs.mdp.observations:joint_pos_rel
      params: {}
actions:
  act:
    class_type: isaaclab.envs.mdp.actions.joint_actions:JointPositionAction
    asset_name: robot
    scale: 1.0
"""

    def _write_config(self, directory: str) -> str:
        """Write the YAML fixture into a directory.

        Args:
            directory: Directory receiving the file.

        Returns:
            The written file path.
        """
        path = Path(directory) / "env.yaml"
        path.write_text(self._YAML)
        return str(path)

    def test_from_file_parses_tuples_and_unknown_tags(self) -> None:
        """File loading constructs python tuples and tolerates unknown tags."""
        with tempfile.TemporaryDirectory() as directory:
            config = PolicyEnvConfig.from_file(self._write_config(directory))
        position, orientation = config.initial_root_pose
        self.assertEqual(position, [0.0, 0.0, 0.5])
        self.assertEqual(orientation, [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(config.timing.decimation, 2)
        self.assertEqual(config.spawn.usd_path, "/assets/bot.usd")


if __name__ == "__main__":
    unittest.main()
