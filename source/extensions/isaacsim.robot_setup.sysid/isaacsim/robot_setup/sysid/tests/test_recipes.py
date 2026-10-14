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

# ruff: noqa: D101, D102

"""Tests for recipe application, auto residual weights, and the Newton solver enum."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import numpy as np
import omni.kit.test
from isaacsim.robot_setup.sysid.preflight import SysIdRunPreflight, _check_residuals
from isaacsim.robot_setup.sysid.preset_loader import (
    ParameterPreset,
    load_builtin_parameter_preset,
)
from isaacsim.robot_setup.sysid.provenance import TrajectoryDatasetMetadata
from isaacsim.robot_setup.sysid.recipes import (
    RECIPE_DEFAULT_CONTACT_WEIGHT,
    RECIPE_DEFAULT_TORQUE_WEIGHT,
    apply_recipe_to_run_spec,
    derive_residual_weights,
    infer_telemetry_source_type,
    synthesize_run_spec_from_quickstart,
)
from isaacsim.robot_setup.sysid.residual_config import ResidualWeightConfig
from isaacsim.robot_setup.sysid.run_spec import (
    ParameterRunSpec,
    SysIdRunSpec,
)
from isaacsim.robot_setup.sysid.schema_validation import validate_sysid_run_spec_payload
from isaacsim.robot_setup.sysid.trajectory_csv import TrajectoryDataset


def _trajectory(
    *,
    with_torques: bool = False,
    with_contacts: bool = False,
    torque_semantics: str | None = None,
) -> TrajectoryDataset:
    values = np.zeros((4, 2), dtype=np.float64)
    metadata = None
    if torque_semantics is not None:
        metadata = TrajectoryDatasetMetadata(
            source_type="csv",
            source_path="test.csv",
            num_samples=4,
            num_joints=2,
            duration_seconds=0.03,
            extra={"torque_semantics": torque_semantics},
        )
    return TrajectoryDataset(
        times=np.arange(4, dtype=np.float64) * 0.01,
        positions=values.copy(),
        velocities=values.copy(),
        commands=values.copy(),
        metadata=metadata,
        torques=values.copy() if with_torques else None,
        contact_forces=values.copy() if with_contacts else None,
    )


class DeriveResidualWeightsTests(omni.kit.test.AsyncTestCase):
    async def test_missing_channels_are_zeroed(self) -> None:
        base = ResidualWeightConfig(torque_weight=0.4, end_effector_pose_weight=0.5, contact_force_weight=0.8)
        derived = derive_residual_weights(_trajectory(), base)
        self.assertEqual(derived.torque_weight, 0.0)
        self.assertEqual(derived.end_effector_pose_weight, 0.0)
        self.assertEqual(derived.contact_force_weight, 0.0)
        self.assertEqual(derived.position_weight, base.position_weight)
        self.assertEqual(derived.velocity_weight, base.velocity_weight)

    async def test_present_channels_keep_positive_base_weights(self) -> None:
        base = ResidualWeightConfig(torque_weight=0.4, contact_force_weight=0.8)
        derived = derive_residual_weights(
            _trajectory(with_torques=True, with_contacts=True, torque_semantics="link_side"),
            base,
        )
        self.assertEqual(derived.torque_weight, 0.4)
        self.assertEqual(derived.contact_force_weight, 0.8)

    async def test_present_channels_with_zero_base_get_defaults(self) -> None:
        derived = derive_residual_weights(
            _trajectory(with_torques=True, with_contacts=True, torque_semantics="link_side")
        )
        self.assertEqual(derived.torque_weight, RECIPE_DEFAULT_TORQUE_WEIGHT)
        self.assertEqual(derived.contact_force_weight, RECIPE_DEFAULT_CONTACT_WEIGHT)

    async def test_external_torque_channel_is_not_auto_enabled(self) -> None:
        trajectory = _trajectory(with_torques=True, torque_semantics="external")
        derived = derive_residual_weights(trajectory, ResidualWeightConfig(torque_weight=0.4))
        self.assertEqual(derived.torque_weight, 0.0)

    async def test_unspecified_torque_channel_is_not_auto_enabled(self) -> None:
        derived = derive_residual_weights(_trajectory(with_torques=True), ResidualWeightConfig(torque_weight=0.4))
        self.assertEqual(derived.torque_weight, 0.0)

    async def test_base_config_is_not_mutated(self) -> None:
        base = ResidualWeightConfig(torque_weight=0.4)
        derive_residual_weights(_trajectory(), base)
        self.assertEqual(base.torque_weight, 0.4)


class TorqueResidualPreflightTests(omni.kit.test.AsyncTestCase):
    async def test_external_torque_semantics_are_rejected(self) -> None:
        spec = SysIdRunSpec()
        spec.residuals.torque_weight = 0.2
        result = SysIdRunPreflight()
        _check_residuals(
            spec,
            result,
            trajectory=_trajectory(with_torques=True, torque_semantics="external"),
        )
        self.assertTrue(any(issue.code == "residual_torque_semantics" for issue in result.errors))

    async def test_unspecified_torque_semantics_are_rejected(self) -> None:
        spec = SysIdRunSpec()
        spec.residuals.torque_weight = 0.2
        result = SysIdRunPreflight()
        _check_residuals(spec, result, trajectory=_trajectory(with_torques=True))
        self.assertTrue(any(issue.code == "residual_torque_semantics" for issue in result.errors))


class ApplyRecipeTests(omni.kit.test.AsyncTestCase):
    async def test_quadruped_preset_maps_backend_and_flags(self) -> None:
        spec = SysIdRunSpec()
        preset = load_builtin_parameter_preset("quadruped")
        notes = apply_recipe_to_run_spec(spec, preset)
        self.assertEqual(spec.solver.optimizer, "cma_es")
        self.assertAlmostEqual(spec.solver.cma_sigma, 0.25)
        self.assertTrue(spec.parameters.space.include_joint_limit_scales)
        self.assertTrue(any("telemetry" in note.lower() for note in notes))

    async def test_apply_with_trajectory_zeroes_missing_contact_channel(self) -> None:
        spec = SysIdRunSpec()
        preset = load_builtin_parameter_preset("quadruped")
        notes = apply_recipe_to_run_spec(
            spec,
            preset,
            trajectory=_trajectory(with_torques=True, torque_semantics="link_side"),
        )
        self.assertEqual(spec.residuals.contact_force_weight, 0.0)
        self.assertGreater(spec.residuals.torque_weight, 0.0)
        self.assertTrue(any("contact force" in note.lower() for note in notes))

    async def test_apply_explains_incompatible_external_torque(self) -> None:
        spec = SysIdRunSpec()
        preset = load_builtin_parameter_preset("drive_calibration_inair")
        notes = apply_recipe_to_run_spec(
            spec,
            preset,
            trajectory=_trajectory(with_torques=True, torque_semantics="external"),
        )
        self.assertEqual(spec.residuals.torque_weight, 0.0)
        self.assertTrue(any("not link-side" in note.lower() for note in notes))

    async def test_flag_change_with_selection_adds_note(self) -> None:
        spec = SysIdRunSpec()
        spec.parameters.selected = [ParameterRunSpec(param_type="joint_friction", dof_index=0, min=0.0, max=1.0)]
        preset = ParameterPreset(name="test", include_com_offsets=True)
        notes = apply_recipe_to_run_spec(spec, preset)
        self.assertTrue(any("re-select" in note.lower() for note in notes))
        self.assertEqual(spec.parameters.selected, [])


class RecipeYamlTests(omni.kit.test.AsyncTestCase):
    async def test_drive_calibration_recipe_applies_full_workflow(self) -> None:
        spec = SysIdRunSpec()
        recipe = load_builtin_parameter_preset("drive_calibration_inair")
        apply_recipe_to_run_spec(spec, recipe)
        self.assertEqual(spec.simulation.engine, "newton")
        self.assertEqual(spec.simulation.physics_backend, "newton")
        self.assertEqual(spec.simulation.actuator_runtime, "auto")
        self.assertEqual(spec.simulation.newton.solver, "featherstone_diff")
        self.assertEqual(spec.solver.optimizer, "gradient_descent")
        self.assertEqual(spec.solver.max_iterations, 60)
        self.assertEqual(spec.solver.max_rollout_steps, 600)
        self.assertTrue(spec.telemetry.auto_split)
        self.assertAlmostEqual(spec.telemetry.auto_split_train_fraction, 0.8)

    async def test_full_inertial_recipe_enables_per_link_families(self) -> None:
        recipe = load_builtin_parameter_preset("full_inertial_inair")
        self.assertTrue(recipe.include_per_link_mass)
        self.assertTrue(recipe.include_inertia_log_cholesky)
        self.assertEqual(recipe.residual_weights_mode, "auto")

    async def test_solver_section_overrides_backend_fields(self) -> None:
        spec = SysIdRunSpec()
        preset = ParameterPreset(
            name="test",
            optimizer_backend={"backend": "cma_es", "cma_sigma": 0.3, "cma_seed": 11, "bo_seed": 13, "gd_seed": 17},
            solver={"optimizer": "gradient_descent", "max_iterations": 7, "gd_seed": 19},
        )
        apply_recipe_to_run_spec(spec, preset)
        self.assertEqual(spec.solver.optimizer, "gradient_descent")
        self.assertEqual(spec.solver.max_iterations, 7)
        self.assertEqual(spec.solver.cma_seed, 11)
        self.assertEqual(spec.solver.bo_seed, 13)
        self.assertEqual(spec.solver.gd_seed, 19)

    async def test_simulation_section_applies_valid_feedforward(self) -> None:
        spec = SysIdRunSpec()
        preset = ParameterPreset(
            name="test",
            simulation={"engine": "newton", "newton": {"solver": "featherstone_diff", "feedforward": "gravity"}},
        )
        apply_recipe_to_run_spec(spec, preset)
        self.assertEqual(spec.simulation.newton.feedforward, "gravity")

    async def test_simulation_section_skips_unknown_feedforward_with_note(self) -> None:
        spec = SysIdRunSpec()
        preset = ParameterPreset(
            name="test",
            simulation={"engine": "newton", "newton": {"feedforward": "warp_drive"}},
        )
        notes = apply_recipe_to_run_spec(spec, preset)
        self.assertEqual(spec.simulation.newton.feedforward, "none")
        self.assertTrue(any("feedforward" in note.lower() for note in notes))

    async def test_inair_presets_declare_explicit_feedforward(self) -> None:
        for name in ("drive_calibration_inair", "full_inertial_inair"):
            with self.subTest(preset=name):
                spec = SysIdRunSpec()
                apply_recipe_to_run_spec(spec, load_builtin_parameter_preset(name))
                self.assertEqual(spec.simulation.newton.feedforward, "none")
                self.assertEqual(spec.simulation.physics_backend, "newton")
                self.assertEqual(spec.simulation.actuator_runtime, "auto")
                self.assertEqual(spec.residuals.end_effector_pose_weight, 0.0)
                self.assertEqual(spec.residuals.contact_force_weight, 0.0)

    async def test_benchmark_scenes_author_physics_gravity_and_contact_colliders(self) -> None:
        from pxr import Gf, Usd, UsdPhysics

        scene_dir = Path(__file__).resolve().parents[4] / "data" / "scenes"
        for name in ("free_space", "gravity_comp", "contact_probing", "benchtop_force"):
            with self.subTest(scene=name):
                stage = Usd.Stage.Open(str(scene_dir / f"{name}.usda"))
                self.assertIsNotNone(stage)
                physics_scene = UsdPhysics.Scene(stage.GetPrimAtPath("/World/physicsScene"))
                self.assertEqual(physics_scene.GetGravityDirectionAttr().Get(), Gf.Vec3f(0.0, 0.0, -1.0))
                self.assertAlmostEqual(physics_scene.GetGravityMagnitudeAttr().Get(), 9.81, places=6)

        for name, path in (
            ("contact_probing", "/World/ProbeSurface/RigidPad"),
            ("benchtop_force", "/World/ForceGauge/LoadCellFace"),
        ):
            with self.subTest(collider=name):
                stage = Usd.Stage.Open(str(scene_dir / f"{name}.usda"))
                self.assertTrue(stage.GetPrimAtPath(path).HasAPI(UsdPhysics.CollisionAPI))


class QuickstartSynthesisTests(omni.kit.test.AsyncTestCase):
    async def test_synthesis_selects_supported_families_with_default_bounds(self) -> None:
        recipe = load_builtin_parameter_preset("drive_calibration_inair")
        spec, notes = synthesize_run_spec_from_quickstart(
            robot_prim_path="/World/robot",
            telemetry_path="data/traj.csv",
            recipe=recipe,
            stage_path="robot.usd",
            trajectory=_trajectory(with_torques=True, torque_semantics="link_side"),
            num_joints=3,
            num_links=4,
        )
        self.assertEqual(spec.simulation.robot_prim_path, "/World/robot")
        self.assertFalse(spec.simulation.parallel_clones)
        self.assertEqual(spec.telemetry.source_type, "csv")
        self.assertTrue(spec.parameters.selected)
        types = {item.param_type for item in spec.parameters.selected}
        # The differentiable bridge rejects armature; it must be filtered out.
        self.assertNotIn("joint_armature", types)
        self.assertIn("joint_friction", types)
        for item in spec.parameters.selected:
            self.assertLess(item.min, item.max)
            self.assertLessEqual(item.min, item.initial)
            self.assertLessEqual(item.initial, item.max)
        self.assertTrue(any("registry defaults" in note for note in notes))
        self.assertTrue(any("unsupported" in note.lower() for note in notes))

    async def test_synthesis_filters_joint_limits_for_standalone_mujoco(self) -> None:
        """Quick-start must not emit a run spec that its selected bridge rejects."""
        base = load_builtin_parameter_preset("manipulator")
        recipe = replace(
            base,
            simulation={"engine": "newton", "physics_backend": "newton", "newton": {"solver": "mujoco"}},
        )

        spec, notes = synthesize_run_spec_from_quickstart(
            robot_prim_path="/World/robot",
            telemetry_path="data/traj.csv",
            recipe=recipe,
            num_joints=2,
            num_links=3,
        )

        types = {item.param_type for item in spec.parameters.selected}
        self.assertNotIn("joint_limit_lower_scale", types)
        self.assertNotIn("joint_limit_upper_scale", types)
        # Match on the dropped parameter rather than the backend display label,
        # which is presentation text and free to change.
        self.assertTrue(any("joint_limit_lower_scale" in note for note in notes))

    async def test_synthesis_enables_clones_only_with_explicit_source_environment(self) -> None:
        recipe = load_builtin_parameter_preset("manipulator")
        spec, notes = synthesize_run_spec_from_quickstart(
            robot_prim_path="/World/envs/env_0/Robot",
            source_env_path="/World/envs/env_0",
            telemetry_path="data/traj.csv",
            recipe=recipe,
        )

        self.assertTrue(spec.simulation.parallel_clones)
        self.assertEqual(spec.simulation.source_env_path, "/World/envs/env_0")
        self.assertFalse(any("clones disabled" in note.lower() for note in notes))

    async def test_source_type_inference(self) -> None:
        self.assertEqual(infer_telemetry_source_type("a/b/data.csv"), "csv")
        self.assertEqual(infer_telemetry_source_type("a/b/data.mcap"), "mcap")
        self.assertEqual(infer_telemetry_source_type("a/b/rosbag.db3"), "ros2_bag")
        with self.assertRaisesRegex(ValueError, "Could not infer"):
            infer_telemetry_source_type("a/b/data.unknown")


class NewtonSolverEnumTests(omni.kit.test.AsyncTestCase):
    async def test_default_serialization_is_canonical_and_schema_clean(self) -> None:
        payload = SysIdRunSpec().to_dict()

        self.assertEqual(payload["simulation"]["newton"]["feedforward"], "none")
        self.assertNotIn("gravity_compensation", payload["simulation"]["newton"])
        issues = validate_sysid_run_spec_payload(payload)
        self.assertEqual([issue.to_dict() for issue in issues], [])

    async def test_cma_seed_is_known_and_must_be_an_integer(self) -> None:
        payload = SysIdRunSpec().to_dict()
        payload["solver"]["cma_seed"] = "seed"

        issues = validate_sysid_run_spec_payload(payload)

        self.assertEqual(
            [issue.path for issue in issues if issue.severity == "error"],
            ["$.solver.cma_seed"],
        )
        self.assertFalse(any(issue.severity == "warning" and "cma_seed" in issue.path for issue in issues))

    async def test_schema_rejects_removed_surrogate_and_legacy_bool(self) -> None:
        payload = SysIdRunSpec().to_dict()
        payload["simulation"]["newton"]["solver"] = "surrogate"
        payload["simulation"]["newton"]["use_newton_physics"] = True
        issues = validate_sysid_run_spec_payload(payload)
        self.assertTrue(any("solver" in issue.path and issue.severity == "error" for issue in issues))
        self.assertTrue(any("use_newton_physics" in issue.path for issue in issues))
