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

"""Solver panel: a small basic tier plus advanced and output disclosures.

The basic tier holds the three decisions a first-time user actually makes
(optimizer backend, iteration count, rollout cap). Solver internals — sampling,
residual weights, backend hyperparameters — live in a collapsed "Advanced
solver settings" frame. What happens with the results (USD write-back,
provenance, run-spec export, viewport pause) lives in a separate visible
"Outputs & stage changes" frame so consequential actions are clear before Run.
Backend-specific hyperparameter groups (LM, CMA-ES, Bayesian, gradient
descent) are shown only for the selected backend via visibility toggles on
pre-built frames (never rebuilt, so field state survives switching).
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Optional

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CheckBox, StringField
from isaacsim.gui.components.ui_utils import dropdown_builder
from isaacsim.robot_setup.sysid.optimizer_config import (
    _BACKEND_FROM_LABEL,
    _BACKEND_TO_LABEL,
    OPTIMIZER_BACKEND_LABELS,
    OptimizerBackend,
    OptimizerBackendConfig,
)
from isaacsim.robot_setup.sysid.residual_config import ResidualWeightConfig, load_residual_weight_config
from isaacsim.robot_setup.sysid.run_spec import SamplingRunSpec
from isaacsim.robot_setup.sysid.settings import CarbOptimizerSettingsStore
from isaacsim.robot_setup.sysid.ui.numeric_inputs import ExactFloatField, ExactIntField
from isaacsim.robot_setup.sysid.ui.styles import HINT_LABEL_STYLE as _HINT_LABEL_STYLE

_SAMPLING_MODE_LABELS = ("Auto", "Off", "Always")
_SAMPLING_MODE_VALUES = {"Auto": "auto", "Off": "off", "Always": "always"}
_SAMPLING_COMMAND_LABELS = ("Hold", "Linear")
_SAMPLING_COMMAND_VALUES = {"Hold": "zero_order_hold", "Linear": "linear"}


def visible_groups_for_backend(backend: OptimizerBackend) -> set[str]:
    """Return which backend-specific hyperparameter groups are visible.

    Auto shows every group (auto resolution may pick any backend); a specific
    backend shows only its own group.

    Args:
        backend: Backend value.

    Returns:
        The resulting value.
    """
    if backend == OptimizerBackend.AUTO:
        return {"lm", "cma", "bo", "gd"}
    return {
        OptimizerBackend.LEVENBERG_MARQUARDT: {"lm"},
        OptimizerBackend.CMA_ES: {"cma"},
        OptimizerBackend.BAYESIAN: {"bo"},
        OptimizerBackend.GRADIENT_DESCENT: {"gd"},
    }.get(backend, {"lm", "cma", "bo", "gd"})


class SolverPanel:
    """Builds and manages the Solver collapsible section.

    Args:
        on_backend_changed: Optional callback fired with the new backend label after
            the selection is persisted (used for engine/backend recommendation hints).
        on_export_run_spec: Optional callback fired when the user clicks Export RunSpec.
        on_check_inputs_changed: Optional callback fired when residual or solver
            settings used by the Check report change.
    """

    def __init__(
        self,
        on_backend_changed: Callable[[str], None] | None = None,
        on_export_run_spec: Callable[[], None] | None = None,
        on_check_inputs_changed: Callable[[], None] | None = None,
    ) -> None:
        self._on_backend_changed_callback = on_backend_changed
        self._on_export_run_spec_callback = on_export_run_spec
        self._on_check_inputs_changed_callback = on_check_inputs_changed
        self._export_run_spec_button: Optional[ui.Button] = None
        self._settings_store = CarbOptimizerSettingsStore()
        self._backend_config = self._settings_store.load()
        self._optimizer_backend_label: str = _BACKEND_TO_LABEL.get(self._backend_config.backend, "Auto")
        self._differentiable_bridge_hint = False

        self._backend_model = None
        self._backend_hint_label: Optional[ui.Label] = None
        self._lambda_field: Optional[ExactFloatField] = None
        self._epsilon_field: Optional[ExactFloatField] = None
        self._max_iter_field: Optional[ExactIntField] = None
        self._max_rollout_steps_field: Optional[ExactIntField] = None
        self._analytical_presolve_cb: Optional[CheckBox] = None
        self._sampling_mode_label = "Auto"
        self._sampling_command_label = "Hold"
        self._sampling_max_steps_field: Optional[ExactIntField] = None

        self._weight_pos_field: Optional[ExactFloatField] = None
        self._weight_vel_field: Optional[ExactFloatField] = None
        self._weight_torque_field: Optional[ExactFloatField] = None
        self._weight_ee_field: Optional[ExactFloatField] = None
        self._weight_contact_field: Optional[ExactFloatField] = None
        self._residual_weights_path_field: Optional[StringField] = None
        self._ee_link_index_field: Optional[ExactIntField] = None

        self._cma_sigma_field: Optional[ExactFloatField] = None
        self._cma_batch_size_field: Optional[ExactIntField] = None
        self._cma_seed_field: Optional[ExactIntField] = None
        self._bo_initial_samples_field: Optional[ExactIntField] = None
        self._bo_candidate_count_field: Optional[ExactIntField] = None
        self._bo_batch_size_field: Optional[ExactIntField] = None
        self._bo_seed_field: Optional[ExactIntField] = None
        self._gd_learning_rate_field: Optional[ExactFloatField] = None
        self._gd_num_restarts_field: Optional[ExactIntField] = None
        self._gd_seed_field: Optional[ExactIntField] = None

        self._apply_parameters_to_usd_cb: Optional[CheckBox] = None
        self._write_usd_provenance_cb: Optional[CheckBox] = None
        self._export_provenance_sidecar_cb: Optional[CheckBox] = None
        self._pause_viewport_rendering_cb: Optional[CheckBox] = None
        self._provenance_sidecar_field: Optional[StringField] = None

        self._lm_group: Optional[ui.Frame] = None
        self._cma_group: Optional[ui.Frame] = None
        self._bo_group: Optional[ui.Frame] = None
        self._gd_group: Optional[ui.Frame] = None
        self.wrapped_ui_elements: list = []

    # ------------------------------------------------------------------ build

    def build(self) -> None:
        """Handle build."""
        with ui.VStack(spacing=6):
            backend_default = OPTIMIZER_BACKEND_LABELS.index(self._optimizer_backend_label)
            self._backend_model = dropdown_builder(
                label="Optimizer",
                default_val=backend_default,
                items=list(OPTIMIZER_BACKEND_LABELS),
                tooltip=(
                    "Auto picks the recommended backend: gradient descent on differentiable bridges, "
                    "CMA-ES for contact-heavy or high-dimensional problems, Levenberg-Marquardt otherwise"
                ),
                on_clicked_fn=self._handle_backend_changed,
            )
            self._backend_hint_label = ui.Label("", word_wrap=True, visible=False, style=_HINT_LABEL_STYLE)
            self._max_iter_field = ExactIntField(
                "Max iterations", default_value=20, lower_limit=1, tooltip="Maximum solver iterations"
            )
            self._max_rollout_steps_field = ExactIntField(
                "Max rollout steps",
                default_value=300,
                lower_limit=2,
                tooltip="Cap physics steps per rollout (min 2; also limits residual length)",
                on_value_changed_fn=self._notify_check_inputs_changed,
            )

            with ui.CollapsableFrame("Advanced solver settings", collapsed=True):
                with ui.VStack(spacing=6):
                    self._build_lm_group()
                    self._build_cma_group()
                    self._build_bo_group()
                    self._build_gd_group()
                    self._build_presolve_and_sampling_group()
                    self._build_residual_group()

            with ui.CollapsableFrame("Outputs & stage changes", collapsed=False):
                with ui.VStack(spacing=6):
                    self._build_output_group()
                    with ui.HStack(spacing=8, height=28):
                        ui.Spacer(width=ui.Fraction(1))
                        self._export_run_spec_button = ui.Button(
                            "Export RunSpec",
                            width=130,
                            height=24,
                            tooltip=(
                                "Write the current System Identification settings to a JSON run spec "
                                "(reusable with the headless solve tool)"
                            ),
                            clicked_fn=self._on_export_run_spec_clicked,
                        )

            self.wrapped_ui_elements.extend(
                [
                    self._lambda_field,
                    self._epsilon_field,
                    self._max_iter_field,
                    self._max_rollout_steps_field,
                    self._analytical_presolve_cb,
                    self._sampling_max_steps_field,
                    self._weight_pos_field,
                    self._weight_vel_field,
                    self._weight_torque_field,
                    self._weight_ee_field,
                    self._weight_contact_field,
                    self._residual_weights_path_field,
                    self._ee_link_index_field,
                    self._cma_sigma_field,
                    self._cma_batch_size_field,
                    self._cma_seed_field,
                    self._bo_initial_samples_field,
                    self._bo_candidate_count_field,
                    self._bo_batch_size_field,
                    self._bo_seed_field,
                    self._gd_learning_rate_field,
                    self._gd_num_restarts_field,
                    self._gd_seed_field,
                    self._apply_parameters_to_usd_cb,
                    self._write_usd_provenance_cb,
                    self._export_provenance_sidecar_cb,
                    self._pause_viewport_rendering_cb,
                    self._provenance_sidecar_field,
                ]
            )
        self._refresh_backend_group_visibility()

    def _build_lm_group(self) -> None:
        self._lm_group = ui.Frame()
        with self._lm_group:
            with ui.VStack(spacing=6):
                self._lambda_field = ExactFloatField(
                    "Initial damping (lambda)",
                    default_value=1e-2,
                    lower_limit=0.0,
                    tooltip="Levenberg-Marquardt initial lambda",
                )
                self._epsilon_field = ExactFloatField(
                    "Finite difference step (epsilon)",
                    default_value=1e-4,
                    step=1e-5,
                    format="%.6f",
                    lower_limit=1e-12,
                    tooltip="Parameter perturbation for Jacobian columns",
                )

    def _build_cma_group(self) -> None:
        self._cma_group = ui.Frame()
        with self._cma_group:
            with ui.VStack(spacing=6):
                self._cma_sigma_field = ExactFloatField(
                    "CMA-ES sigma",
                    default_value=float(self._backend_config.cma_sigma),
                    lower_limit=1e-12,
                    tooltip="Initial step size for CMA-ES (normalized search space)",
                )
                self._cma_batch_size_field = ExactIntField(
                    "CMA-ES batch size",
                    default_value=int(self._backend_config.cma_batch_size or 0),
                    lower_limit=0,
                    tooltip=(
                        "Population candidates evaluated per rollout batch; 0 evaluates the full population together"
                    ),
                )
                self._cma_seed_field = ExactIntField(
                    "CMA-ES seed",
                    default_value=int(self._backend_config.cma_seed),
                    lower_limit=0,
                    tooltip="Non-negative random seed for reproducible CMA-ES populations",
                )

    def _build_bo_group(self) -> None:
        self._bo_group = ui.Frame()
        with self._bo_group:
            with ui.VStack(spacing=6):
                self._bo_initial_samples_field = ExactIntField(
                    "BO initial samples",
                    default_value=int(self._backend_config.bo_initial_samples),
                    lower_limit=1,
                    tooltip="Random evaluations before GP-guided Bayesian optimization",
                )
                self._bo_candidate_count_field = ExactIntField(
                    "BO acquisition candidates",
                    default_value=int(self._backend_config.bo_candidate_count),
                    lower_limit=1,
                    tooltip=("Random query points scored by expected improvement before choosing rollout candidates"),
                )
                self._bo_batch_size_field = ExactIntField(
                    "BO batch size",
                    default_value=int(self._backend_config.bo_batch_size),
                    lower_limit=1,
                    tooltip="Rollout candidates evaluated together when parallel clones are available",
                )
                self._bo_seed_field = ExactIntField(
                    "BO seed",
                    default_value=int(self._backend_config.bo_seed),
                    lower_limit=0,
                    tooltip="Non-negative random seed for reproducible Bayesian candidate generation",
                )

    def _build_gd_group(self) -> None:
        self._gd_group = ui.Frame()
        with self._gd_group:
            with ui.VStack(spacing=6):
                self._gd_learning_rate_field = ExactFloatField(
                    "GD learning rate",
                    default_value=float(self._backend_config.gd_learning_rate),
                    step=1e-3,
                    format="%.4f",
                    lower_limit=1e-12,
                    tooltip="Adam learning rate for gradient descent on differentiable bridges",
                )
                self._gd_num_restarts_field = ExactIntField(
                    "GD restarts",
                    default_value=int(self._backend_config.gd_num_restarts),
                    lower_limit=1,
                    tooltip="Random multi-start rows optimized in parallel (batched on the differentiable bridge)",
                )
                self._gd_seed_field = ExactIntField(
                    "GD seed",
                    default_value=int(self._backend_config.gd_seed),
                    lower_limit=0,
                    tooltip="Non-negative random seed for reproducible gradient-descent restarts",
                )

    def _build_presolve_and_sampling_group(self) -> None:
        self._analytical_presolve_cb = CheckBox(
            "Analytical presolve seed",
            default_value=bool(self._backend_config.use_analytical_presolve),
            tooltip=(
                "Seed the optimizer with a classical linear least-squares identification before rollout "
                "optimization: CMA-ES/Bayesian start from the seed, and gradient descent places it in "
                "restart row 0. The pre-presolve initials still compete at rollout cost (extra restart "
                "row / first-generation candidate / first-batch sample), so a poor seed cannot make the "
                "solve worse than not seeding. Fixed-base, contact-free; link-side torque telemetry "
                "improves the seed."
            ),
            on_click_fn=self._notify_check_inputs_changed,
        )
        dropdown_builder(
            label="Sampling",
            default_val=_SAMPLING_MODE_LABELS.index(self._sampling_mode_label),
            items=list(_SAMPLING_MODE_LABELS),
            tooltip="Resample low-rate or irregular telemetry before physics rollout",
            on_clicked_fn=self._on_sampling_mode_changed,
        )
        dropdown_builder(
            label="Command sampling",
            default_val=_SAMPLING_COMMAND_LABELS.index(self._sampling_command_label),
            items=list(_SAMPLING_COMMAND_LABELS),
            tooltip="Hold commands between recorded samples or linearly interpolate smooth references",
            on_clicked_fn=self._on_sampling_command_changed,
        )
        self._sampling_max_steps_field = ExactIntField(
            "Max resampled steps",
            default_value=0,
            lower_limit=0,
            tooltip="0 uses Max rollout steps; otherwise caps generated samples per chunk",
        )

    def _build_residual_group(self) -> None:
        self._weight_pos_field = ExactFloatField(
            "Weight: position",
            default_value=1.0,
            lower_limit=0.0,
            tooltip="Residual weight for joint position errors",
            on_value_changed_fn=self._notify_check_inputs_changed,
        )
        self._weight_vel_field = ExactFloatField(
            "Weight: velocity",
            default_value=1.0,
            lower_limit=0.0,
            tooltip="Residual weight for joint velocity errors",
            on_value_changed_fn=self._notify_check_inputs_changed,
        )
        self._weight_torque_field = ExactFloatField(
            "Weight: torque",
            default_value=0.0,
            lower_limit=0.0,
            tooltip="Residual weight for joint torque errors (requires torque telemetry)",
            on_value_changed_fn=self._notify_check_inputs_changed,
        )
        self._weight_ee_field = ExactFloatField(
            "Weight: end-effector pose",
            default_value=0.0,
            lower_limit=0.0,
            tooltip="Residual weight for EE pose errors (requires EE telemetry)",
            on_value_changed_fn=self._notify_check_inputs_changed,
        )
        self._weight_contact_field = ExactFloatField(
            "Weight: contact force",
            default_value=0.0,
            lower_limit=0.0,
            tooltip="Residual weight for contact force errors",
            on_value_changed_fn=self._notify_check_inputs_changed,
        )
        self._residual_weights_path_field = StringField(
            "Residual weights JSON",
            default_value="",
            tooltip="Optional JSON file with channel weights and per-sample weights",
            on_value_changed_fn=self._notify_check_inputs_changed,
        )
        self._ee_link_index_field = ExactIntField(
            "EE link index",
            default_value=-1,
            lower_limit=-1,
            tooltip="Link index for EE pose/contact (-1 uses last link)",
            on_value_changed_fn=self._notify_check_inputs_changed,
        )

    def _build_output_group(self) -> None:
        self._apply_parameters_to_usd_cb = CheckBox(
            "Apply optimized parameters to USD",
            default_value=True,
            tooltip=(
                "Author final optimized values as verified instance-local overrides in the open stage; "
                "save the stage to persist them"
            ),
        )
        self._write_usd_provenance_cb = CheckBox(
            "Write USD provenance",
            default_value=True,
            tooltip="Store isaac:sysid:* attributes on the articulation root when optimization completes",
        )
        self._export_provenance_sidecar_cb = CheckBox(
            "Export provenance JSON sidecar",
            default_value=False,
            tooltip="Write a JSON sidecar with full optimization provenance",
        )
        self._pause_viewport_rendering_cb = CheckBox(
            "Pause viewport rendering during run",
            default_value=True,
            tooltip="Freeze the active viewport while optimization runs",
        )
        self._provenance_sidecar_field = StringField(
            "Provenance sidecar path",
            default_value="",
            tooltip="Optional output path (defaults next to telemetry source)",
        )

    # ---------------------------------------------------------------- getters

    def get_backend_label(self) -> str:
        """Get backend label.

        Returns:
            The resulting value.
        """
        return self._optimizer_backend_label

    def get_backend_config(self) -> OptimizerBackendConfig:
        """Get backend config.

        Returns:
            The resulting value.
        """
        backend = _BACKEND_FROM_LABEL.get(self._optimizer_backend_label, OptimizerBackend.AUTO)
        return OptimizerBackendConfig(
            backend=backend,
            cma_sigma=float(self._cma_sigma_field.get_value()) if self._cma_sigma_field else 0.3,
            cma_batch_size=(
                int(self._cma_batch_size_field.get_value())
                if self._cma_batch_size_field and int(self._cma_batch_size_field.get_value()) > 0
                else None
            ),
            cma_seed=max(0, int(self._cma_seed_field.get_value())) if self._cma_seed_field else 0,
            bo_initial_samples=int(self._bo_initial_samples_field.get_value()) if self._bo_initial_samples_field else 5,
            bo_candidate_count=(
                int(self._bo_candidate_count_field.get_value()) if self._bo_candidate_count_field else 64
            ),
            bo_batch_size=int(self._bo_batch_size_field.get_value()) if self._bo_batch_size_field else 4,
            bo_seed=max(0, int(self._bo_seed_field.get_value())) if self._bo_seed_field else 0,
            gd_learning_rate=(
                float(self._gd_learning_rate_field.get_value()) if self._gd_learning_rate_field else 0.05
            ),
            gd_num_restarts=int(self._gd_num_restarts_field.get_value()) if self._gd_num_restarts_field else 1,
            gd_seed=max(0, int(self._gd_seed_field.get_value())) if self._gd_seed_field else 0,
            use_analytical_jacobian=False,
            use_analytical_presolve=(
                bool(self._analytical_presolve_cb.get_value()) if self._analytical_presolve_cb else False
            ),
            differentiable_bridge=self._differentiable_bridge_hint,
        )

    def get_residual_config(self) -> ResidualWeightConfig:
        """Get residual config.

        Returns:
            The resulting value.
        """
        path = self._residual_weights_path_field.get_value().strip() if self._residual_weights_path_field else ""
        if path:
            cfg = load_residual_weight_config(path)
            if self._ee_link_index_field is not None:
                cfg.end_effector_link_index = int(self._ee_link_index_field.get_value())
            return cfg
        return ResidualWeightConfig(
            position_weight=float(self._weight_pos_field.get_value()) if self._weight_pos_field else 1.0,
            velocity_weight=float(self._weight_vel_field.get_value()) if self._weight_vel_field else 1.0,
            torque_weight=float(self._weight_torque_field.get_value()) if self._weight_torque_field else 0.0,
            end_effector_pose_weight=float(self._weight_ee_field.get_value()) if self._weight_ee_field else 0.0,
            contact_force_weight=float(self._weight_contact_field.get_value()) if self._weight_contact_field else 0.0,
            end_effector_link_index=int(self._ee_link_index_field.get_value()) if self._ee_link_index_field else -1,
        )

    def get_max_rollout_steps(self) -> int:
        """Get max rollout steps.

        Returns:
            The resulting value.
        """
        raw = self._max_rollout_steps_field.get_value() if self._max_rollout_steps_field else 300
        return max(2, int(raw))

    def get_sampling_spec(self) -> SamplingRunSpec:
        """Get sampling spec.

        Returns:
            The resulting value.
        """
        return SamplingRunSpec(
            mode=_SAMPLING_MODE_VALUES.get(self._sampling_mode_label, "auto"),
            command_interpolation=_SAMPLING_COMMAND_VALUES.get(self._sampling_command_label, "zero_order_hold"),
            max_resampled_steps=(
                max(0, int(self._sampling_max_steps_field.get_value())) if self._sampling_max_steps_field else 0
            ),
        )

    def get_solver_floats(self) -> tuple[float, float, int]:
        """Returns (damping_initial, epsilon, max_iterations).

        Returns:
            The resulting value.
        """
        damping = float(self._lambda_field.get_value()) if self._lambda_field else 1e-2
        epsilon = float(self._epsilon_field.get_value()) if self._epsilon_field else 1e-4
        max_iter = int(self._max_iter_field.get_value()) if self._max_iter_field else 20
        return damping, epsilon, max_iter

    def get_output_flags(self) -> dict:
        """Get output flags.

        Returns:
            The resulting value.
        """
        return {
            "apply_params_to_usd": (
                bool(self._apply_parameters_to_usd_cb.get_value()) if self._apply_parameters_to_usd_cb else True
            ),
            "write_usd_provenance": (
                bool(self._write_usd_provenance_cb.get_value()) if self._write_usd_provenance_cb else True
            ),
            "export_provenance_sidecar": (
                bool(self._export_provenance_sidecar_cb.get_value()) if self._export_provenance_sidecar_cb else False
            ),
            "provenance_sidecar_path": (
                self._provenance_sidecar_field.get_value().strip() if self._provenance_sidecar_field else ""
            ),
        }

    def is_viewport_pause_enabled(self) -> bool:
        """Return whether viewport pause enabled.

        Returns:
            The resulting value.
        """
        return bool(self._pause_viewport_rendering_cb.get_value()) if self._pause_viewport_rendering_cb else True

    # ---------------------------------------------------------------- setters

    def set_backend_config(self, config: OptimizerBackendConfig) -> None:
        """Write backend settings into the panel widgets (recipe application).

        Args:
            config: Config value.
        """
        if self._cma_sigma_field is not None:
            self._cma_sigma_field.set_value(float(config.cma_sigma))
        if self._cma_batch_size_field is not None:
            self._cma_batch_size_field.set_value(int(config.cma_batch_size or 0))
        if self._cma_seed_field is not None:
            self._cma_seed_field.set_value(max(0, int(config.cma_seed)))
        if self._bo_initial_samples_field is not None:
            self._bo_initial_samples_field.set_value(int(config.bo_initial_samples))
        if self._bo_candidate_count_field is not None:
            self._bo_candidate_count_field.set_value(int(config.bo_candidate_count))
        if self._bo_batch_size_field is not None:
            self._bo_batch_size_field.set_value(int(config.bo_batch_size))
        if self._bo_seed_field is not None:
            self._bo_seed_field.set_value(max(0, int(config.bo_seed)))
        if self._gd_learning_rate_field is not None:
            self._gd_learning_rate_field.set_value(float(config.gd_learning_rate))
        if self._gd_num_restarts_field is not None:
            self._gd_num_restarts_field.set_value(int(config.gd_num_restarts))
        if self._gd_seed_field is not None:
            self._gd_seed_field.set_value(max(0, int(config.gd_seed)))
        if self._analytical_presolve_cb is not None:
            self._analytical_presolve_cb.set_value(bool(config.use_analytical_presolve))
        # Dropdown last: the item-changed callback persists the config from the widgets.
        label = _BACKEND_TO_LABEL.get(config.backend, "Auto")
        self._set_backend_dropdown(label)

    def set_residual_config(self, config: ResidualWeightConfig) -> None:
        """Write residual weights into the panel widgets (recipe application).

        Args:
            config: Config value.
        """
        if self._weight_pos_field is not None:
            self._weight_pos_field.set_value(float(config.position_weight))
        if self._weight_vel_field is not None:
            self._weight_vel_field.set_value(float(config.velocity_weight))
        if self._weight_torque_field is not None:
            self._weight_torque_field.set_value(float(config.torque_weight))
        if self._weight_ee_field is not None:
            self._weight_ee_field.set_value(float(config.end_effector_pose_weight))
        if self._weight_contact_field is not None:
            self._weight_contact_field.set_value(float(config.contact_force_weight))
        if self._ee_link_index_field is not None:
            self._ee_link_index_field.set_value(int(config.end_effector_link_index))
        if self._residual_weights_path_field is not None:
            # Explicit weights supersede any weights file.
            self._residual_weights_path_field.set_value("")

    def set_solver_floats(self, damping: float, epsilon: float, max_iterations: int) -> None:
        """Set solver floats.

        Args:
            damping: Damping value.
            epsilon: Epsilon value.
            max_iterations: Max iterations value.
        """
        if self._lambda_field is not None:
            self._lambda_field.set_value(float(damping))
        if self._epsilon_field is not None:
            self._epsilon_field.set_value(float(epsilon))
        if self._max_iter_field is not None:
            self._max_iter_field.set_value(int(max_iterations))

    def set_max_rollout_steps(self, steps: int) -> None:
        """Set max rollout steps.

        Args:
            steps: Steps value.
        """
        if self._max_rollout_steps_field is not None:
            self._max_rollout_steps_field.set_value(max(2, int(steps)))

    def set_backend_hint(self, text: str | None) -> None:
        """Show or clear the engine/backend recommendation hint under the dropdown.

        Args:
            text: Text to display.
        """
        if self._backend_hint_label is None:
            return
        self._backend_hint_label.text = text or ""
        self._backend_hint_label.visible = bool(text)

    def set_differentiable_bridge_hint(self, flag: bool) -> None:
        """Record whether the active engine exposes autograd rollouts (steers Auto to GD).

        Args:
            flag: Flag value.
        """
        self._differentiable_bridge_hint = bool(flag)

    def set_export_run_spec_enabled(self, enabled: bool) -> None:
        """Set export run spec enabled.

        Args:
            enabled: Whether the option is enabled.
        """
        if self._export_run_spec_button is not None:
            self._export_run_spec_button.enabled = bool(enabled)

    def set_enabled(self, enabled: bool) -> None:
        """Set enabled.

        Args:
            enabled: Whether the option is enabled.
        """
        for element in self.wrapped_ui_elements:
            if element is not None:
                element.enabled = enabled

    # ---------------------------------------------------------------- cleanup

    def cleanup(self) -> None:
        """Release resources."""
        for element in self.wrapped_ui_elements:
            if element is not None and hasattr(element, "cleanup"):
                element.cleanup()
        self.wrapped_ui_elements.clear()
        self._backend_model = None
        self._backend_hint_label = None
        self._lambda_field = None
        self._epsilon_field = None
        self._max_iter_field = None
        self._max_rollout_steps_field = None
        self._analytical_presolve_cb = None
        self._sampling_max_steps_field = None
        self._weight_pos_field = None
        self._weight_vel_field = None
        self._weight_torque_field = None
        self._weight_ee_field = None
        self._weight_contact_field = None
        self._residual_weights_path_field = None
        self._ee_link_index_field = None
        self._cma_sigma_field = None
        self._cma_batch_size_field = None
        self._cma_seed_field = None
        self._bo_initial_samples_field = None
        self._bo_candidate_count_field = None
        self._bo_batch_size_field = None
        self._bo_seed_field = None
        self._gd_learning_rate_field = None
        self._gd_num_restarts_field = None
        self._gd_seed_field = None
        self._apply_parameters_to_usd_cb = None
        self._write_usd_provenance_cb = None
        self._export_provenance_sidecar_cb = None
        self._pause_viewport_rendering_cb = None
        self._provenance_sidecar_field = None
        self._export_run_spec_button = None
        self._lm_group = None
        self._cma_group = None
        self._bo_group = None
        self._gd_group = None

    # --------------------------------------------------------------- private

    def _set_backend_dropdown(self, label: str) -> None:
        if label not in OPTIMIZER_BACKEND_LABELS:
            return
        if self._backend_model is not None:
            # Fires the item-changed callback, which updates the cached label,
            # persists the config, and refreshes group visibility.
            self._backend_model.get_item_value_model().set_value(OPTIMIZER_BACKEND_LABELS.index(label))
        else:
            self._optimizer_backend_label = label

    def _handle_backend_changed(self, label: str) -> None:
        self._optimizer_backend_label = label
        self._settings_store.save(self.get_backend_config())
        self._refresh_backend_group_visibility()
        if self._on_backend_changed_callback is not None:
            self._on_backend_changed_callback(label)

    def _refresh_backend_group_visibility(self) -> None:
        backend = _BACKEND_FROM_LABEL.get(self._optimizer_backend_label, OptimizerBackend.AUTO)
        visible = visible_groups_for_backend(backend)
        for key, group in (
            ("lm", self._lm_group),
            ("cma", self._cma_group),
            ("bo", self._bo_group),
            ("gd", self._gd_group),
        ):
            if group is not None:
                group.visible = key in visible

    def _on_export_run_spec_clicked(self) -> None:
        if self._on_export_run_spec_callback is not None:
            self._on_export_run_spec_callback()

    def _on_sampling_mode_changed(self, label: str) -> None:
        self._sampling_mode_label = label

    def _on_sampling_command_changed(self, label: str) -> None:
        self._sampling_command_label = label

    def _notify_check_inputs_changed(self, _value: object = None) -> None:
        if self._on_check_inputs_changed_callback is not None:
            self._on_check_inputs_changed_callback()
