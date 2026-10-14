# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Main UI builder class for the Gain Tuner extension interface that manages robot selection, gain parameter adjustment, testing functionality, and results visualization."""

from __future__ import annotations

import asyncio
import math
import os
import time
import traceback
from collections.abc import Callable
from dataclasses import dataclass
from enum import IntEnum
from functools import partial

import carb
import numpy as np
import omni
import omni.kit.notification_manager as nm
import omni.physics.core
import omni.timeline
import omni.ui as ui
import pxr
from isaacsim.gui.components.element_wrappers import (
    Button,
    CheckBox,
    CollapsableFrame,
    DropDown,
    FloatField,
    TextBlock,
    XYPlot,
)
from isaacsim.robot_setup import gain_tuner
from omni.kit.window.file import StageSaveDialog
from omni.physics.tensors import DofType
from omni.ui import color as cl
from omni.usd import StageEventType
from pxr import Sdf, Tf, Usd, UsdPhysics
from usd.schema.isaac.robot_schema import ApplyRobotAPI, Classes

from .backend_context import BackendContext
from .chart_widget import JointGraphWidget
from .color_table_widget import ColorJointWidget
from .dropdown_widget import create_combo_list_model
from .frame_widget import CustomCollapsableFrame as CollapsableFrame
from .gain_display import (
    DAMPING_RATIO_LABEL,
    NATURAL_FREQUENCY_LABEL,
    AdvancedParamDisplay,
    advanced_cell_value,
    advanced_gain_column,
    advanced_param_display,
    gain_dof_unit,
    joint_param_info_text,
    max_velocity_engine_text,
)
from .gain_table_model import (
    DETAIL_PANEL_MULTI,
    DETAIL_PANEL_SINGLE,
    GAIN_COLUMN_KD,
    GAIN_COLUMN_KI,
    GAIN_COLUMN_KP,
    build_gain_table_rows,
    detail_panel_mode,
    gain_column_spec,
    grouped_columns_for_menu,
    is_angular_dof,
    menu_backend_rows,
    menu_has_mjc,
    resolve_visible_columns,
    source_option_label,
)
from .gain_table_view import HEADER_H as _TABLE_HEADER_H
from .gain_table_view import ROW_H as _TABLE_ROW_H
from .gain_table_view import GainTableView
from .gains_tuner_backend import GainsTestMode, GainTuner
from .global_variables import (
    GT_ADVANCED_PARAMS_TITLE,
    GT_BACKEND_LABEL,
    GT_CANCEL_TEST_BUTTON,
    GT_CHARTS_TAB,
    GT_CONTROLLER_GAINS_TITLE,
    GT_COPY_ALL_JOINT_PARAMS_BUTTON,
    GT_COPY_ALL_JOINT_PARAMS_TOOLTIP,
    GT_COPY_JOINT_PARAMS_BUTTON,
    GT_COPY_JOINT_PARAMS_TOOLTIP,
    GT_DT_SWEEP_CHARTS_CAPTION,
    GT_DT_SWEEP_CLIFF_TOOLTIP,
    GT_DT_SWEEP_COL_CLIFF,
    GT_DT_SWEEP_COL_SETTLE,
    GT_DT_SWEEP_COL_SS_ERROR,
    GT_DT_SWEEP_DEGRAD_TOOLTIP,
    GT_DT_SWEEP_GROUP_PROBE,
    GT_DT_SWEEP_GROUP_RANGE,
    GT_DT_SWEEP_GROUP_THRESHOLDS,
    GT_DT_SWEEP_HEADER_CONTEXT,
    GT_DT_SWEEP_SETTLE_CHART_TITLE,
    GT_DT_SWEEP_SS_ERROR_CHART_TITLE,
    GT_DT_SWEEP_TABLE_TITLE,
    GT_DT_SWEEP_TIP_DT_MAX,
    GT_DT_SWEEP_TIP_DT_MIN,
    GT_DT_SWEEP_TIP_ERROR_DEGRAD,
    GT_DT_SWEEP_TIP_HOLD,
    GT_DT_SWEEP_TIP_SETTLE_DEGRAD,
    GT_DT_SWEEP_TIP_STEPS,
    GT_DT_SWEEP_TIP_TARGET_DT,
    GT_DT_SWEEP_TIP_TIMEOUT,
    GT_DT_SWEEP_TIP_TOLERANCE,
    GT_DT_SWEEP_VERDICT_ALL_ACCURATE,
    GT_DT_SWEEP_VERDICT_DEGRADED,
    GT_DT_SWEEP_VERDICT_DID_NOT_SETTLE,
    GT_DT_SWEEP_VERDICT_INDETERMINATE,
    GT_EFFORT_CHART_TITLE,
    GT_GAIN_SETTINGS_TAB,
    GT_HAND_TUNED_GAINS_TITLE,
    GT_JOINT_PARAM_BACKEND_UNSUPPORTED,
    GT_JOINT_PARAM_PER_BACKEND_NOTE,
    GT_JOINT_PARAM_REPLAY_TO_APPLY,
    GT_JOINT_PARAM_WRITE_FAILED,
    GT_MIRROR_TOGGLE_LABEL,
    GT_MIRROR_TOGGLE_TOOLTIP,
    GT_MJC_TARGET_LABEL,
    GT_NATURAL_FREQUENCY_NEEDS_INERTIA,
    GT_NEWTON_TARGET_LABEL,
    GT_NO_SCHEMA_BUTTON,
    GT_NO_SCHEMA_HEADING,
    GT_NO_SCHEMA_MESSAGE,
    GT_NO_SCHEMA_MESSAGE_EMPTY_STAGE,
    GT_POSITION_CHART_TITLE,
    GT_RESULTS_COL_JOINT,
    GT_RESULTS_COL_RESULT,
    GT_ROBOT_LABEL,
    GT_RUN_TEST_BUTTON,
    GT_SAVE_BUTTON_LABEL,
    GT_SAVE_TARGET_LABEL,
    GT_SNAP_COL_LOWER_MAX,
    GT_SNAP_COL_LOWER_MEAN,
    GT_SNAP_COL_LOWER_SETTLE,
    GT_SNAP_COL_UPPER_MAX,
    GT_SNAP_COL_UPPER_MEAN,
    GT_SNAP_COL_UPPER_SETTLE,
    GT_SNAP_TABLE_TITLE,
    GT_SNAP_VERDICT_ALL_PASSED,
    GT_SNAP_VERDICT_SOME_BLOCKED,
    GT_SNAP_VERDICT_SOME_FAILED,
    GT_STRESS_COL_MAX_VELOCITY,
    GT_STRESS_COL_TRIGGER_TIME,
    GT_STRESS_COL_TRIGGER_VELOCITY,
    GT_STRESS_TABLE_TITLE,
    GT_STRESS_VERDICT_ALL_STABLE,
    GT_STRESS_VERDICT_UNSTABLE,
    GT_TEST_CONTROLS_TITLE,
    GT_TEST_DURATION_LABEL,
    GT_TEST_INFO_ACCURACY_CLIFF,
    GT_TEST_INFO_ERROR_DEGRAD,
    GT_TEST_INFO_LOWER_MAX_ERROR,
    GT_TEST_INFO_LOWER_MEAN_ERROR,
    GT_TEST_INFO_LOWER_SETTLE,
    GT_TEST_INFO_MAX_VELOCITY,
    GT_TEST_INFO_OVERSHOOT,
    GT_TEST_INFO_PEAK_ERROR,
    GT_TEST_INFO_RESULT,
    GT_TEST_INFO_RMS_ERROR,
    GT_TEST_INFO_SETTLE_AT_TARGET,
    GT_TEST_INFO_SS_ERROR,
    GT_TEST_INFO_TARGET_DT,
    GT_TEST_INFO_TEST_TYPE,
    GT_TEST_INFO_TITLE,
    GT_TEST_INFO_TRIGGER_TIME,
    GT_TEST_INFO_TRIGGER_VELOCITY,
    GT_TEST_INFO_UPPER_MAX_ERROR,
    GT_TEST_INFO_UPPER_MEAN_ERROR,
    GT_TEST_INFO_UPPER_SETTLE,
    GT_TEST_MODE_LABEL,
    GT_TUNING_MODE_NATURAL_FREQUENCY_LABEL,
    GT_TUNING_MODE_STIFFNESS_LABEL,
    GT_TUNING_MODE_TOOLTIP,
    GT_WARN_INACTIVE_GAINS,
    GT_WARN_SAVE_TARGET_UNRESOLVED,
)
from .joint_param_sync import (
    copy_joint_params_to_other_backend,
    copy_summary_text,
    joint_param_copy_targets,
)
from .plot_widget import CustomXYPlot
from .results_panels import DtSweepResultsPanel, SnapResultsPanel, StressResultsPanel
from .style import (
    CHART_CMD_COLOR,
    CHART_GRID_COLOR,
    CHART_OBS_COLOR,
    CHART_STROKE,
    CHART_VIEWPORT_BG,
    DIVIDER_COLOR,
    EXTENSION_FOLDER_PATH,
    FAIL_COLOR,
    FONT_SIZE,
    FRAME_BG_COLOR,
    HEADER_BG_COLOR,
    HEADER_FONT_SIZE,
    INFO_COLOR,
    LABEL_COLOR,
    MUTED_LABEL_COLOR,
    NAV_BUTTON_ACTIVE_BG,
    NAV_BUTTON_DISABLED_BG,
    NAV_BUTTON_DISABLED_TEXT,
    NAV_BUTTON_INACTIVE_BG,
    PASS_COLOR,
    SAVE_ROW_BG,
    SUB_TAB_ACTIVE_BG,
    SUB_TAB_INACTIVE_BG,
    TAB_ACTIVE_BG,
    TAB_INACTIVE_BG,
    TREEVIEW_BG_COLOR,
    WARNING_COLOR,
    get_style,
)
from .test_table_widget import TestJointWidget
from .ui_utils import set_wrapped_tooltip

_COPY_TOOLTIP_OFFSET_Y = 24
"""Pixels below the copy buttons' own top edge that their tooltip is anchored at.

Their 20 px height plus a small gap, so a tooltip that wraps to several lines opens
underneath the button instead of over it."""


@dataclass
class _InlineTextRow:
    """One inline explanation line under a detail field, kept so it can be updated.

    The sentences under an advanced field state what is authored, which schema the
    running engine reads it from, and what an edit would do -- all of which an edit
    can change, including into having nothing to say at all.  So the row is built
    whether or not it has text and hidden when it does not: ``omni.ui`` gives an
    invisible child neither its own height nor the stack's spacing, so a hidden row
    collapses completely, while a visible row holding ``""`` would leave a blank
    line behind.
    """

    row: ui.Widget
    """The row container, hidden and shown as the text comes and goes."""

    label: ui.Label
    """The wrapping label the sentence is written to."""

    def set_text(self, text: str) -> None:
        """Write one sentence into the row, hiding it when there is none.

        Assigning ``text`` is what a repaint can afford: the caller is the
        value-changed handler, which fires on every frame of a drag, and
        ``omni.ui`` re-emits a label from its current text and re-flows the stack
        around its new wrapped height without the row being rebuilt.
        """
        self.label.text = text
        self.row.visible = bool(text)


@dataclass(frozen=True)
class _AdvFieldRendering:
    """Everything one advanced field's rows render, re-derived after an edit.

    Both halves come out of one resolve of the parameter, because they have to
    agree: the field's ``(default)`` marker and the sentence under it are two
    statements about the same authored-or-not question, and deriving them from two
    reads of the stage is how they came to contradict each other.
    """

    display: object
    """The :class:`~.gain_display.AdvancedParamDisplay` the field renders from."""

    explanations: tuple[str, ...]
    """The sentences for the field's inline rows, in the order they were built.

    An empty string is a row with nothing to say, which is a state the rows have to
    reach: authoring a value answers the question its explanation was asking."""


@dataclass
class _DetailAdvField:
    """One live Advanced Actuator Parameters field, and how to re-read it.

    A detail field carries three independent pieces of state.  The *number* lives
    in :attr:`model`, which a table edit refreshes through :attr:`read_value`.
    Whether that number reads as the user's value or as the engine's default lives
    in the widget's ``format`` and ``style``, which are set once when the field is
    built -- so registering the model alone left a field that had just been
    authored still rendering ``"(default)"`` over the value the user typed.  And
    the sentences under the field say the same thing in words, so a field whose
    marker followed the edit sat directly above a line that still did not.
    :attr:`field`, :attr:`explain_rows` and :attr:`read_rendering` are what let
    both be re-derived.
    """

    read_value: Callable[[], float | None]
    """Re-read the number from USD, through the resolution the field was built with."""

    model: ui.SimpleFloatModel | None = None
    """The field's value model.

    Filled in after :meth:`UIBuilder._gain_field_row` returns, which is also where
    the model is created -- so the record is built first, to give the widget
    callback something to attach to."""

    field: ui.Widget | None = None
    """The built drag widget, or None for a row that has no re-derivable rendering."""

    read_rendering: Callable[[], _AdvFieldRendering] | None = None
    """Re-derive the field's display and the sentences under it.

    None for fields whose rendering cannot go stale (a plain single attribute such
    as the DriveAPI max force, which is either authored or absent)."""

    explain_rows: tuple[_InlineTextRow, ...] = ()
    """The field's inline explanation rows, in :attr:`_AdvFieldRendering.explanations` order."""


# Expected error types for the panel's best-effort defensive guards. These wrap
# USD/attribute reads and writes, omni.ui model/widget calls, and small numeric
# conversions that can legitimately fail on missing/invalid prims, stale widget
# references during rebuild/shutdown, duck-typed fakes in tests, or bad values.
# Catching exactly this set keeps the panel resilient for those known cases while
# letting any *unexpected* error propagate instead of being silently swallowed.
_UI_GUARD_ERRORS = (
    AttributeError,
    IndexError,
    KeyError,
    RuntimeError,
    TypeError,
    ValueError,
    Tf.ErrorException,
)

# Snap-to-limits progress estimation.  The core (SnapToLimitsTest.run) runs three
# approach+hold cycles per sequence (lower, upper, home).  Each cycle is one
# variable-length approach phase (bounded by the core's 10 s _APPROACH_TIMEOUT, but
# usually ending early once joints settle) followed by a fixed hold of
# `hold_duration` seconds.  Runtime is therefore data-dependent, so the progress
# bar uses a per-approach estimate to size the denominator.
_SNAP_CYCLES_PER_SEQUENCE = 3
#: Number of approach+hold cycles the snap-to-limits test performs per sequence.
_SNAP_APPROACH_ESTIMATE_S = 2.0
#: Estimated seconds a snap-to-limits approach phase takes before joints settle.
_MAX_RUNNING_PROGRESS = 0.99
#: Progress-bar ceiling while a test is still running.  Snap-to-limits and dt-sweep
#: denominators are estimates, so a run whose joints never settle (weak gains, no
#: damping) can outlast its estimate by several times.  Holding the bar below 100%
#: until the run actually reports done keeps it from looking finished and stopped
#: early, which would discard the results.

# dt physics sweep progress estimation.  The sweep runs a single-timestep accuracy
# probe (approach + hold) at each of `num_dt_steps` timesteps.  Each probe's approach
# is data-dependent (it ends when joints settle or the `timeout` elapses), so the
# progress bar sizes each level as `hold_duration + _DISCRETIZATION_SETTLE_TIMEOUT_FRACTION
# * timeout`: the fixed hold plus a settle allowance derived from the timeout.  This
# tracks the real per-level runtime far better than a flat constant.
_DISCRETIZATION_SETTLE_TIMEOUT_FRACTION = 0.5
#: Fraction of the approach `timeout` budgeted as the per-level settle allowance.

# Combo-index -> GainsTestMode mapping for the Test Gains Settings mode combo.  The
# combo labels are ``["Snap to Limits", "Sinusoidal", "Step Function", "Stress",
# "dt Sweep"]`` and this tuple gives the mode each index selects.  Exposed at module
# scope so the mapping can be asserted without constructing the combo widget.
_TEST_MODE_COMBO_MODES = (
    GainsTestMode.SNAP_TO_LIMITS,
    GainsTestMode.SINUSOIDAL,
    GainsTestMode.STEP,
    GainsTestMode.STRESS_TEST,
    GainsTestMode.DISCRETIZATION,
)
#: Ordered combo-index -> ``GainsTestMode`` mapping for the test-mode combo.


class TuningMode(IntEnum):
    """How PhysX PhysicsDrive gains are edited in the per-joint detail panel."""

    STIFFNESS = 0
    """Edit drive stiffness and damping directly."""

    NATURAL_FREQUENCY = 1
    """Edit natural frequency (Hz) and damping ratio, converted to stiffness /
    damping using the joint's effective inertia."""


class UIBuilder:
    """Main UI builder class for the Gain Tuner extension interface.

    This class manages the complete user interface for robot joint gain tuning, including robot selection,
    gain parameter adjustment, testing functionality, and results visualization through charts. It handles
    UI lifecycle events such as timeline play/pause/stop, stage changes, physics steps, and render updates
    to maintain synchronization between the simulation state and the interface.

    The interface is organized into two main pages:
    - Gains Settings: Per-joint two-column view (joint source list + per-joint gain detail editor)
    - Test Gains: Configuration and execution of gain validation tests, with result charts

    The class integrates with the GainTuner backend to perform actual gain calculations and test execution,
    while managing UI state transitions and user interactions through various callback functions.
    """

    def __init__(self) -> None:
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self._gains_tuner = GainTuner()
        # Rebuild the per-joint detail editor when the effective joint inertia is
        # (re)computed so natural-frequency values reflect the real inertia.
        self._gains_tuner.add_inertia_updated_callback(self._on_inertia_updated)

        # Tuning mode for PhysX PhysicsDrive gains (stiffness/damping vs natural
        # frequency/damping ratio).  Persists across joint selection like the
        # legacy table-wide mode toggle.
        self._detail_tuning_mode = TuningMode.STIFFNESS

        self._test_mode = GainsTestMode.SNAP_TO_LIMITS
        self._test_running = False

        # Combo-index -> GainsTestMode mapping for the Test Gains Settings mode combo.
        self._test_mode_combo_modes = list(_TEST_MODE_COMBO_MODES)
        self._test_mode_combo = None

        # Per-mode settings frames + fields (built in `_build_test_controls_frame`).
        self._test_duration_frame = None
        self._snap_settings_frame = None
        self._hold_duration_field = None
        self._tolerance_field = None
        self._disable_self_collisions_cb = None
        self._disable_velocity_limits_cb = None
        self._stress_test_settings_frame = None
        self._stress_test_submode_combo = None
        self._stress_test_duration_field = None
        self._stress_test_seed_field = None
        self._stress_test_vel_threshold_field = None
        self._stress_test_sigma_frame = None
        self._stress_test_sigma_field = None
        self._stress_test_snap_interval_frame = None
        self._stress_test_snap_interval_field = None
        self._stress_test_disable_self_collisions_cb = None
        self._stress_test_disable_velocity_limits_cb = None

        # dt physics sweep settings frame + fields (built in `_build_test_controls_frame`).
        self._discretization_settings_frame = None
        self._discretization_dt_max_field = None
        self._discretization_dt_min_field = None
        self._discretization_dt_steps_field = None
        self._discretization_timeout_field = None
        self._discretization_hold_duration_field = None
        self._discretization_tolerance_field = None
        self._discretization_target_dt_field = None
        self._discretization_settle_degrad_field = None
        self._discretization_error_degrad_field = None
        # dt-sweep orchestration state.  The sweep runs the DISCRETIZATION probe once
        # per timestep from an async loop that stops/plays the timeline and changes
        # the physics dt between levels; these flags coordinate that loop with the
        # per-level completion handled by `_update_gains_test`.
        self._discretization_sweep_active = False
        self._discretization_cancel_requested = False
        self._dt_sweep_level_dts: list[float] = []
        self._dt_sweep_level_index = 0

        # `enabledSelfCollisions` is a cook-time PhysX property, so applying the
        # override requires a timeline stop/play recook. The override itself is
        # owned by the core tuner; this only tracks that recook.
        self._restarting_for_override = False

        self._reset_ui_next_frame = False
        self._make_plot_on_next_frame = False

        self._test_inline_widget = None  # TestJointWidget embedded in the tab panel
        self._test_button_ui = None
        self._test_button_is_running = False
        self._test_physics_sub = None
        # Deferred test-start bookkeeping.  When the timeline has to be started
        # first, the actual start is deferred a couple of frames via an async task.
        # Retain the task so a cancel/reset can abort it, and bump the run
        # generation on cancel so a stale deferred start can't install a physics
        # subscription for a test the user already cancelled.
        self._start_test_task = None
        self._test_run_generation = 0
        self._color_joint_widget = None
        self._position_frame = None
        self._effort_frame = None
        self._velocity_frame = None
        self._plotting_indices = []
        self._plotting_group_colors = {}
        # Per-step effort (torque) capture during a test — recorded on the
        # frontend because the shipped backend doesn't log efforts.  Temporary
        # until the backend exposes effort history directly.
        self._test_effort_history = []  # list of per-DOF effort arrays (one per step)
        self._test_effort_times = []  # accumulated sim time for each sample
        # Defensive guard: set once if get_dof_projected_joint_forces() raises so the
        # per-step effort capture stops probing (Newton is already skipped up front by
        # _measured_forces_available; this covers any other backend gap without
        # flooding the log).  Reset per test run so switching backends re-probes.
        self._projected_forces_unavailable = False
        self.force_query_mass = True
        self._save_stage_prompt = None
        self._articulation_menu_model = None
        # Robot combo contents last pushed to the menu; used to skip redundant
        # refresh_list() calls when ASSETS_LOADED fires but the robot set is
        # unchanged (part of the flicker fix — see on_stage_event).
        self._robot_menu_items = None
        # Signatures of the currently-selected robot used to detect a genuine
        # change on the repeated ASSETS_LOADED stage events (checked ONLY while the
        # timeline is stopped, so the play-time event spam never triggers a rebuild
        # — this preserves the flicker guard).  ``_robot_content_signature``
        # fingerprints the joint set (changes when a different robot is swapped in
        # at the same prim path -> full re-setup + reset); ``_gain_source_signature``
        # fingerprints the authored gain sources (changes on a Physics variant
        # switch that composes mjc:* / actuator prims in or out -> repaint only).
        self._robot_content_signature = None
        self._gain_source_signature = None
        self._backend_combo_model = None
        # Active backend index (0 = PhysX, 1 = Newton).  Follows the live engine
        # (see _refresh_backend_from_app) and drives the backend-gated columns and
        # the Advanced Actuator Parameters accordion.  The backend is displayed as a
        # read-only info tag (self._backend_label_widget); it is not user-switchable.
        self._backend_active_idx = 0
        # Signature (idx, solver) of the backend state last applied to the Gain
        # Settings view.  ASSETS_LOADED can fire many times per second while the
        # timeline plays; we only rebuild the per-joint editor when the active
        # backend actually changes, never on every event (avoids a rebuild loop).
        self._backend_applied_signature = None
        self._backend_label_widget = None
        # USD path selected in the combo that lacks Robot Schema API (GT_NoSchema state)
        self._selected_path_lacks_schema = None
        self._robot_api_target_picker = None
        # Test Gains column visibility, applied when the test table is (re)built.
        # Missing entries default to visible.
        self._test_column_visible: dict = {}
        # Nav tabs: False = Gain Settings page; True = Charts page.
        self._nav_show_charts = False
        self._no_schema_frame = None
        self._nav_buttons_frame = None
        # Viewport height (ScrollingFrame visible area).  Kept in sync each render
        # step via sync_viewport_from_scroll_frame().  Default is a sane starting guess.
        self._viewport_height = 476
        # The gains data-table HStack and its row count.  sync_viewport_from_scroll_frame
        # resizes this HStack in place (via _apply_table_height) instead of rebuilding
        # the whole gains frame, so a viewport resize never re-runs the table build.
        self._gains_table_hstack = None
        self._gains_table_row_count = 0
        # Per-joint Gain Settings view state.  The table selects a joint; the
        # detail panel (_joint_detail_frame) rebuilds to edit that joint's gains.
        self._selected_joint_index = None
        # Number of joints currently selected in the table.  The per-joint detail
        # panel is shown only for a single selection (see detail_panel_mode); 0 or
        # 2+ selected shows a placeholder instead.
        self._detail_selection_count = 0
        # Per-joint viewed-source override, shared by the table's "Source" column
        # and the detail panel's viewed-source toggle so they stay in sync:
        # joint prim path -> forced GainSource.  Persists per joint (only reset on
        # robot swap); a joint absent from the map uses the active-source default.
        self._joint_viewed_source = {}
        self._detail_gain_ctx = None  # GainReadContext shared by list badges + detail editor
        self._joint_detail_frame = None
        # Live detail-panel field models for the currently shown single joint, so a
        # TABLE cell edit can refresh the detail fields in place (mirror of the
        # detail -> table sync) without rebuilding the panel.  ``_detail_gain_models``
        # holds the Controller-Gains / Natural-Frequency models keyed by "kp" / "kd"
        # / "ki" / "nf" / "dr"; ``_detail_adv_models`` holds (model, read_value)
        # pairs for the advanced params, where ``read_value`` re-reads USD through
        # the same resolution the field was built with.  ``_detail_nf_params`` are
        # the (use_force, m_eq, is_angular) inputs needed to recompute nf/dr from
        # the fresh stiffness/damping.  All are repopulated on every detail rebuild.
        self._detail_gain_models = {}
        self._detail_adv_models = []
        self._detail_nf_params = None
        # Guard against a table<->detail update loop: True while the detail field
        # models are being refreshed from a table edit, so their value-changed
        # handlers do not write back to USD / re-sync the table.
        self._suspend_detail_writes = False
        # Gain table state.  A single TreeView lists every tunable joint; rows are
        # multi-selected (Ctrl/Shift click) and editing a gain applies to the whole
        # selection.  Column visibility auto-selects from the shown joints' schemas
        # and can be overridden per column via the hamburger menu.
        self._gain_table_frame = None  # rebuildable Frame holding the whole table
        self._gain_table_view = None  # active GainTableView (TreeView) widget
        # True after a robot is selected but setup() enumerated zero joints because
        # the articulation's DOF view had not cooked yet (the Newton backend builds
        # the physics-tensor articulation lazily, so num_dofs is 0 for the first
        # frame(s) after construction).  on_render_step re-runs setup once the view
        # reports DOFs so the joints populate, then clears the flag.
        self._awaiting_articulation_dofs = False
        # Multi-selection to restore on the next table rebuild (set before a source
        # switch so the selection survives the in-place rebuild); None otherwise.
        self._pending_table_selection = None
        self._gain_column_overrides = {}  # column key -> forced visibility (hamburger)
        self._gain_columns_menu = None  # ui.Menu for the column picker
        self._name_search_query = ""  # current joint-name search filter
        # Reference to the ScrollingFrame that holds the scrollable content.  Its
        # computed_height is the STABLE viewport height (it does not grow with
        # content), which we poll each render step to size the tables.
        self._scroll_frame_ref = None
        self._gains_settings_button = None
        self._charts_button = None
        # The active BackendContext stub; swapped by engineering when real API is ready.
        self._backend_ctx = BackendContext()
        # Identifier of the DriveAPI save-target layer the user has selected in the
        # save-target dropdown (None => use the resolved neutral default).
        self._selected_save_target_identifier = None
        self._save_target_combo = None
        # Non-DriveAPI writebacks.  Newton actuator writeback is ON by default so
        # tuned actuator gains persist to their own layer.  The DriveAPI->MuJoCo
        # mirror is a separate, explicit opt-in that is OFF by default so a save
        # never touches mjc:* actuator prims unless the user asks for it.
        # The selected identifiers override the resolved default mirror targets.
        self._mirror_writeback_enabled = True
        self._mirror_drive_to_mjc = False
        self._selected_mjc_target_identifier = None
        self._selected_newton_target_identifier = None
        self._mirror_toggle_checkbox = None
        self._mjc_target_combo = None
        self._newton_target_combo = None
        self._save_target_frame = None
        # Frames for the two top-level pages.
        self._gain_settings_page = None
        self._charts_page = None
        # Collapsible sections within Gain Settings page.
        self._gains_tuning_frame = None
        self._charts_frame = None
        self._advanced_params_frame = None
        self._advanced_params_container = None  # wraps Advanced Params accordion (backend-gated)
        self._test_controls_frame = None
        self._test_info_frame = None
        # Solver label widget (shown/hidden based on backend)
        self._solver_label_widget = None
        self._save_target_label_widget = None
        # Test progress tracking
        self._accordions_container = None  # wraps all three accordions; hidden while test runs
        # The same test-in-progress overlay is shown on both the Gain Settings page
        # and the Test Gains page so progress is visible regardless of the active
        # tab. Every overlay's widgets are tracked in lists and updated together in
        # _update_gains_test (no per-frame rebuilds — only in-place value updates).
        self._test_in_progress_panels = []  # frames toggled visible while a test runs
        self._test_progress_fills = []  # custom progress fill Rectangles
        self._test_progress_gaps = []  # remaining-space Spacers after each fill
        self._test_progress_labels = []  # "Sequence X/N   Time: Y.Ys/Z.Zs"
        self._test_progress_pct_labels = []  # "30%" text inside each bar
        self._test_start_time = 0.0
        self._test_total_duration = 0.0
        self._test_num_sequences = 0
        self._test_seq_duration = 0.0
        self._test_elapsed_sim = 0.0  # accumulated sim time driving progress
        # Mode of the last completed validation run (None until a test finishes),
        # used to select the per-joint results metric set (stress / snap / step).
        self._last_test_mode = None

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def set_scroll_frame_ref(self, scroll_frame) -> None:
        """Store a reference to the content ScrollingFrame.

        Its ``computed_height`` is the stable viewport height of the scrollable
        area (it does NOT grow with content), so it is the correct value to use
        for sizing the tables to fill the window.
        """
        self._scroll_frame_ref = scroll_frame

    def _compute_table_data_height(self, row_count: int) -> int:
        """Compute the pixel height for the two-column Gain Settings body.

        Fills the available viewport space: starts from the content ScrollingFrame's
        stable ``computed_height`` (mirrored in ``self._viewport_height``) and
        subtracts the fixed chrome on the Gain Settings page (accordion header +
        search toolbar above, Save Target row below).  Both columns scroll
        internally, so a generous minimum keeps the editor usable in a short window.
        """
        _ABOVE = 34 + 26 + 12  # accordion header + search toolbar + spacing
        # Save-target row + spacing + the retained focused-joint detail area, which
        # now sits below the table inside the same (scrollable) frame.
        _BELOW = 30 + 12 + self._DETAIL_H
        _MARGIN = 40  # frame margins the estimates above don't capture exactly
        _MIN_H = 200
        return max(int(self._viewport_height) - _ABOVE - _BELOW - _MARGIN, _MIN_H)

    def _gains_table_data_height(self, row_count: int) -> int:
        """Height for the gains data-table HStack: content-sized, capped at the viewport.

        The table body is sized to its content (header + one row per joint) rather
        than stretched to fill the whole viewport, so making the window taller no
        longer opens a growing empty band between the last row and the "Joint:"
        detail area below -- the leftover space simply becomes blank canvas at the
        bottom of the scroll area.  When there are more joints than fit, the height
        is capped at the viewport-available height (:meth:`_compute_table_data_height`)
        so the table scrolls internally instead of pushing the detail area off-screen.

        Because the content height is a constant for a given robot (independent of
        the window height), a taller window recomputes to the *same* value in the
        common case, so the in-place height update stays a no-op relayout and never
        reintroduces the earlier scroll-frame height oscillation / flicker.
        """
        if row_count <= 0:
            content = 120  # room for the "No tunable joints" placeholder message
        else:
            # +8 keeps the last row's border off the frame edge; header is the
            # column-title row drawn once above the rows.
            content = _TABLE_HEADER_H + row_count * _TABLE_ROW_H + 8
        return min(self._compute_table_data_height(row_count), content)

    def sync_viewport_from_scroll_frame(self, force: bool = False) -> None:
        """Keep the gains table sized to the ScrollingFrame viewport, in place.

        Called once after docking (force=True) and then every render step.  Reads
        the ScrollingFrame's stable ``computed_height`` (the viewport, not the
        content) and, when it changes, updates the stored data-table HStack's
        ``.height`` DIRECTLY.

        Crucially it does NOT rebuild ``_gains_tuning_frame``: setting a widget's
        height only triggers a cheap relayout, whereas rebuilding re-runs the whole
        table build (re-traversing USD, recreating the TreeView, re-fetching icons)
        on every measured pixel change — the cause of the icon-refetch flood and the
        window flicker.
        """
        # Bail out if the widgets this method reads/writes have been torn down
        # (e.g. after `cleanup()` niled the refs), so we never access
        # ``.computed_height`` / ``.height`` on a destroyed widget. A native
        # use-after-free would not surface as a catchable Python exception.
        if self._scroll_frame_ref is None or getattr(self, "_gains_table_hstack", None) is None:
            return
        # Never resize the gains table while a test is running: the table is hidden
        # behind the test-in-progress panel and physics is actively stepping.  We
        # resync once the test finishes.
        if self._test_running:
            return
        try:
            vh = self._scroll_frame_ref.computed_height
        except _UI_GUARD_ERRORS:
            return
        if vh <= 50:
            return  # not laid out yet / spurious

        # Hysteresis: ignore sub-pixel / tiny jitter in the measured height (unless
        # an explicit resync is forced after docking / backend change).
        if not force and abs(vh - self._viewport_height) <= 8:
            return

        self._viewport_height = vh
        self._apply_table_height()

    def _apply_table_height(self) -> None:
        """Resize the gains data table to the current viewport height, in place."""
        hstack = getattr(self, "_gains_table_hstack", None)
        if hstack is None:
            return
        try:
            hstack.height = ui.Pixel(self._gains_table_data_height(self._gains_table_row_count))
        except _UI_GUARD_ERRORS:
            pass

    def on_menu_callback(self) -> None:
        """Callback for when the UI is toggled from the toolbar menu.

        The extension owns all subscription setup/teardown in its window
        visibility handler, so there is nothing to do here on toggle.
        """

    def on_timeline_event(self, event) -> None:
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event: Event Type
        """
        # Stopping the timeline ends a running test without going through cancel
        # or completion, so the override has to be dropped here too. The recook
        # the override itself performs also stops the timeline; that one is
        # flagged, and must not tear down the override it is installing.
        if self._timeline.is_stopped() and not self._restarting_for_override:
            self._restore_self_collision_override()

        if not self._articulation_menu_model or not self._articulation_menu_model.has_item():
            return
        # NOTE: We intentionally do NOT force accordion collapse/expand on play/stop.
        # The user wants Gains Settings, Advanced Actuator Parameters, and Test Controls
        # all open by default and to stay under user control.

    def on_physics_step(self, step: float) -> None:
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step: Size of physics step
        """
        pass

    @staticmethod
    def _should_repopulate_deferred_joints(awaiting: bool, num_dofs: int) -> bool:
        """Return True when a deferred joint-entry re-setup should run now.

        The Newton backend constructs the physics-tensor articulation lazily, so
        ``num_dofs`` is 0 for the first frame(s) after ``setup()`` and
        ``_setup_joint_entries`` caches zero joints ("no joints found").  Once the
        view has cooked (``num_dofs > 0``) the joints must be re-enumerated.  This
        is only pending when ``awaiting`` is set (a prior setup produced no joints),
        so a normal robot (joints enumerated on the first try, PhysX) never takes
        the deferred path.

        Args:
            awaiting: True when the last setup produced zero joint entries.
            num_dofs: The articulation's current reported DOF count.

        Returns:
            True only when a re-setup is pending *and* the articulation now reports
            at least one DOF.
        """
        return bool(awaiting) and int(num_dofs) > 0

    @staticmethod
    def _should_await_articulation_dofs(num_dofs: int, has_joint_entries: bool) -> bool:
        """Return True when a setup that found no joints should wait and retry.

        A zero DOF count means the articulation view has not cooked yet, so the
        joints may still appear once it does.  That is the only case worth
        waiting for, and the wait lasts as long as the view takes to cook.

        A robot that already reports DOFs and still enumerates no joints has
        joints the robot schema does not reach, which re-running the same
        enumeration cannot resolve.  Waiting on it would retry on every render
        step, and each retry restarts the timeline from ``_setup_robot_links``.

        Args:
            num_dofs: The articulation's currently reported DOF count.
            has_joint_entries: True when the setup enumerated at least one joint.

        Returns:
            True only when no joints were enumerated and no DOFs exist yet.
        """
        return not has_joint_entries and int(num_dofs) <= 0

    def _articulation_num_dofs(self) -> int:
        """Return the bound articulation's DOF count, or 0 when it cannot be read.

        An articulation view that has not cooked yet can raise rather than report
        zero, and for the deferred-joint wait the two mean the same thing.

        Returns:
            The reported DOF count, or 0 when no articulation is bound or the
            query failed.
        """
        art = getattr(self._gains_tuner, "_articulation", None)
        if art is None:
            return 0
        try:
            return int(art.num_dofs)
        except _UI_GUARD_ERRORS:
            return 0

    def _arm_deferred_joint_wait(self) -> None:
        """Arm or clear the deferred joint-entry wait for the setup that just ran."""
        self._awaiting_articulation_dofs = self._should_await_articulation_dofs(
            self._articulation_num_dofs(), bool(self._gains_tuner.get_joint_entries())
        )

    def _maybe_repopulate_deferred_joints(self) -> None:
        """Re-run joint-entry setup once the articulation's DOF view has cooked.

        Handles the Newton lazy-articulation case (see
        :meth:`_should_repopulate_deferred_joints`): when the initial ``setup()``
        found ``num_dofs == 0`` and cached zero joints, re-run the selection once
        the view reports DOFs so the table populates.

        The re-setup re-arms the wait through
        :meth:`_should_await_articulation_dofs`, which arms again only while the
        view still reports no DOFs, so a robot that reports DOFs but enumerates
        no joints is left alone after this one re-setup.
        """
        if not self._awaiting_articulation_dofs:
            return
        tuner = self._gains_tuner
        path = getattr(tuner, "_robot_prim_path", None) if tuner is not None else None
        if not path:
            self._awaiting_articulation_dofs = False
            return
        if not self._should_repopulate_deferred_joints(True, self._articulation_num_dofs()):
            return  # view not cooked yet; keep waiting (cheap early-out per frame)
        # View is ready -> re-run the selection so joints are re-enumerated.
        # force=True re-runs setup()+rebuild.
        self._awaiting_articulation_dofs = False
        self._on_articulation_selection(path, force=True)

    def on_render_step(self, e: carb.events.IEvent) -> None:
        """Render event set up to cancel physics subscriptions that run the gains test.

        Args:
            e: Event object
        """
        # Keep the tables sized to fill the scroll viewport.  This is safe to run
        # every frame: ScrollingFrame.computed_height is the viewport (stable), and
        # sync_viewport_from_scroll_frame only rebuilds when it actually changes.
        self.sync_viewport_from_scroll_frame()

        # Suppress frame rebuilds while a self-collision override is recooking the
        # scene (the timeline stop/play would otherwise trigger a rebuild that
        # resets the test controls mid-launch).  The backend poll and the deferred
        # re-setup below are held back with them: both re-run selection, which
        # resets the tuner and would drop the override being installed.  The
        # recook is a few frames, and each is picked up on the next one.
        if self._restarting_for_override:
            return

        # The physics engine can change with no event to subscribe to, so it is
        # polled here.  The per-frame cost is one string comparison against the
        # simulation manager's cached engine name; everything expensive happens
        # only on an actual change.
        self._sync_backend_if_engine_changed()

        if not self._articulation_menu_model or not self._articulation_menu_model.has_item():
            return
        # If a robot was selected before its (Newton, lazy) articulation view had
        # cooked, setup() cached zero joints; re-populate now that DOFs are ready.
        self._maybe_repopulate_deferred_joints()

        if self._reset_ui_next_frame:
            if self._timeline.is_stopped() and self._gains_tuning_frame:
                self._gains_tuning_frame.rebuild()
            if self._make_plot_on_next_frame:
                if self._charts_frame:
                    self._charts_frame.enabled = True
                    self._charts_frame.rebuild()
                self._apply_nav_panel_visibility()

            if self._gains_tuning_frame:
                self._gains_tuning_frame.enabled = True
            if self._test_controls_frame:
                self._test_controls_frame.enabled = True
            self._reset_ui_next_frame = False

    def on_stage_event(self, event) -> None:
        """Callback for Stage Events

        Args:
            event: Event Type
        """
        if event.event_name == omni.usd.get_context().stage_event_name(
            omni.usd.StageEventType.ASSETS_LOADED
        ):  # Any asset added or removed
            items = self._populate_robot_menu()
            menu_items = items if items else [self._NO_ROBOT_PLACEHOLDER]
            # ASSETS_LOADED fires repeatedly while the timeline plays; only push the
            # combo contents when the robot set actually changed to avoid redundant
            # dropdown rebuilds (and any downstream selection churn).
            if self._articulation_menu_model and menu_items != self._robot_menu_items:
                self._robot_menu_items = list(menu_items)
                self._articulation_menu_model.refresh_list(menu_items)
            # A new scene may use a different physics engine — re-detect (the tags
            # update cheaply; the Gain Settings view only rebuilds on real change).
            self._refresh_backend_from_app()
            # Re-populate / repaint the gain views when the selected robot's content
            # or authored gain sources changed (a robot swapped at the same prim
            # path, or a Physics variant switched in/out of MuJoCo).  The helper is
            # gated to a stopped timeline, so the play-time ASSETS_LOADED spam never
            # triggers a rebuild (preserves the flicker guard).
            self._maybe_refresh_on_stage_change()
        elif event.event_name == omni.usd.get_context().stage_event_name(
            omni.usd.StageEventType.SIMULATION_START_PLAY
        ):  # Timeline played
            # The active engine is resolved at play; sync the backend/solver tags.
            self._refresh_backend_from_app()
        elif event.event_name == omni.usd.get_context().stage_event_name(
            omni.usd.StageEventType.SIMULATION_STOP_PLAY
        ):  # Timeline stopped
            self._reset_ui_next_frame = True

    def reset(self) -> None:
        """Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that
        should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()
        # Abort a deferred test-start still pending on timeline warm-up so it cannot
        # fire after a stage close / hot reload.
        self._cancel_pending_start()
        self._selected_path_lacks_schema = None
        self._awaiting_articulation_dofs = False
        # Invalidate the idempotency caches so a fresh stage re-detects its robot
        # set and backend (and rebuilds once) instead of being suppressed as a
        # no-op by a stale signature.
        self._backend_applied_signature = None
        self._robot_menu_items = None
        self._robot_content_signature = None
        self._gain_source_signature = None
        self._gains_tuner.reset()

    def cleanup(self) -> None:
        """Called when the extension is closed.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that
        should be called
        """
        self.reset()
        self._test_physics_sub = None
        # Drop the per-run effort capture and the live detail-panel field models so a
        # teardown/hot-reload does not retain stale test data or dangling widget models.
        self._test_effort_history = []
        self._test_effort_times = []
        self._detail_gain_models = {}
        self._detail_adv_models = []
        self._gains_tuning_frame = None
        # Drop the viewport-sizing widget refs so a later `sync_viewport_from_scroll_frame`
        # (e.g. from a still-live render-step subscription) short-circuits instead of
        # touching the destroyed ScrollingFrame / HStack (native use-after-free).
        self._scroll_frame_ref = None
        self._gains_table_hstack = None
        self._gain_table_frame = None
        self._gain_table_view = None
        self._gain_columns_menu = None
        self._joint_detail_frame = None
        self._charts_frame = None
        self._advanced_params_frame = None
        self._advanced_params_container = None
        self._test_controls_frame = None
        self._test_info_frame = None
        self._no_schema_frame = None
        self._nav_buttons_frame = None
        self._accordions_container = None
        self._test_in_progress_panels = []
        self._test_progress_fills = []
        self._test_progress_gaps = []
        self._test_progress_labels = []
        self._test_progress_pct_labels = []
        self._gains_settings_button = None
        self._charts_button = None
        self._gain_settings_page = None
        self._charts_page = None
        self._articulation_menu_model = None
        self._backend_combo_model = None
        self._test_inline_widget = None
        self._robot_api_target_picker = None
        self._solver_label_widget = None
        self._save_target_label_widget = None
        self._save_target_combo = None
        self._mirror_toggle_checkbox = None
        self._mjc_target_combo = None
        self._newton_target_combo = None
        self._save_target_frame = None

    def _should_show_gt_no_schema(self) -> bool:
        if self._selected_path_lacks_schema is not None:
            return True
        return self._gains_tuner.get_robot_prim_path() is None

    def _gt_no_schema_body_text(self) -> str:
        if self._selected_path_lacks_schema:
            return GT_NO_SCHEMA_MESSAGE
        return GT_NO_SCHEMA_MESSAGE_EMPTY_STAGE

    def _on_robot_api_picker_targets_selected(self, selected_paths: list) -> None:
        """Callback from RelationshipTargetPicker when user confirms a prim."""
        self._robot_api_target_picker = None
        if not selected_paths:
            return
        paths = [str(p) for p in selected_paths]
        self._apply_robot_api_to_paths(paths)

    def _apply_robot_api_to_paths(self, paths: list[str]) -> None:
        """Apply Isaac Robot API like Property panel: Add → Isaac / Robot Schema / Robot API."""
        _EXCLUSIVE_SCHEMAS = (Classes.ROBOT_API, Classes.LINK_API, Classes.JOINT_API)

        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_warn("[Gain Tuner] No stage loaded; cannot add Robot API.")
            return

        applied_paths: list[str] = []
        for path_str in paths:
            prim = stage.GetPrimAtPath(path_str)
            if not prim.IsValid():
                carb.log_warn(f"[Gain Tuner] Invalid prim path, skipping: {path_str}")
                continue
            if any(prim.HasAPI(s.value) for s in _EXCLUSIVE_SCHEMAS):
                carb.log_warn(
                    f"[Gain Tuner] Skipping {path_str}: prim already has Robot / Link / Joint API "
                    "(same rule as the Property panel Add menu)."
                )
                continue
            try:
                ApplyRobotAPI(prim)
                instanceable = [p for p in Usd.PrimRange(prim) if p.IsInstanceable()]
                if instanceable:
                    prim.SetInstanceable(True)
                    prim.ClearMetadata("instanceable")
                applied_paths.append(path_str)
            except Exception as e:
                carb.log_error(f"[Gain Tuner] ApplyRobotAPI failed for {path_str}: {e}")
                traceback.print_exc()

        if not applied_paths:
            return

        self._selected_path_lacks_schema = None
        items = self._populate_robot_menu()
        if self._articulation_menu_model:
            if items:
                self._articulation_menu_model.refresh_list(items)
            else:
                self._articulation_menu_model.refresh_list(["No Robot found in scene with Robot Schema"])
            primary = applied_paths[0]
            if items and primary in items:
                self._articulation_menu_model.set_current_string(primary)
        self._on_articulation_selection(applied_paths[0])
        try:
            import omni.kit.window.property as prop_win

            w = prop_win.get_window()
            if w and getattr(w, "_window", None) and w._window.frame:
                w._window.frame.rebuild()
        except _UI_GUARD_ERRORS:
            pass

    def _on_add_robot_schema_clicked(self) -> None:
        """Always open the stage prim picker (modal). Do not apply from dropdown or viewport
        selection — that skipped the dialog whenever the no-schema articulation was the combo
        selection (_selected_path_lacks_schema).
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            carb.log_warn("[Gain Tuner] No stage loaded; cannot add Robot API.")
            return

        from omni.kit.property.usd.relationship import RelationshipTargetPicker

        async def _open_picker_next_frame() -> None:
            await omni.kit.app.get_app().next_update_async()
            self._robot_api_target_picker = RelationshipTargetPicker(
                stage,
                [],
                None,
                {
                    "target_name": "Robot",
                    "target_plural_name": "Robots",
                    "modal_window": True,
                },
            )
            self._robot_api_target_picker.show(1, on_targets_selected=self._on_robot_api_picker_targets_selected)

        asyncio.ensure_future(_open_picker_next_frame())

    def _build_gt_no_schema_panel(self) -> None:
        """Empty state matching Figma GT_NoSchema: centered card, info row, CTA."""
        CARD_WIDTH = 468
        SIDE_PAD = 8
        BG_CARD = cl("#2E3032")
        TEXT_TITLE = cl("#CCCCCC")
        TEXT_BODY = cl("#D8D8D8")

        with ui.ZStack(height=ui.Pixel(420), width=ui.Fraction(1)):
            with ui.VStack():
                ui.Spacer(height=ui.Fraction(1))
                with ui.HStack():
                    ui.Spacer(width=ui.Fraction(1))
                    with ui.ZStack(width=ui.Pixel(CARD_WIDTH), height=0):
                        ui.Rectangle(style={"background_color": BG_CARD, "border_radius": 4})
                        with ui.VStack(spacing=0, height=0):
                            ui.Spacer(height=ui.Pixel(46))
                            with ui.HStack(height=0):
                                ui.Spacer(width=ui.Fraction(1))
                                ui.Label(
                                    GT_NO_SCHEMA_HEADING,
                                    width=0,
                                    alignment=ui.Alignment.CENTER,
                                    style={"color": TEXT_TITLE, "font_size": 14},
                                )
                                ui.Spacer(width=ui.Fraction(1))
                            ui.Spacer(height=ui.Pixel(24))
                            _info_icon = f"{EXTENSION_FOLDER_PATH}/icons/ico_more_info.svg"
                            with ui.HStack(height=0):
                                ui.Spacer(width=ui.Pixel(SIDE_PAD))
                                ui.Image(
                                    _info_icon,
                                    width=20,
                                    height=20,
                                    style={"Image": {"image_url": _info_icon}},
                                )
                                ui.Spacer(width=ui.Pixel(8))
                                ui.Label(
                                    self._gt_no_schema_body_text(),
                                    width=ui.Pixel(CARD_WIDTH - 2 * SIDE_PAD - 28),
                                    style={"color": TEXT_BODY, "font_size": 12},
                                )
                                ui.Spacer(width=ui.Pixel(SIDE_PAD))
                            ui.Spacer(height=ui.Pixel(24))
                            with ui.HStack():
                                ui.Spacer(width=ui.Fraction(1))
                                ui.Button(
                                    GT_NO_SCHEMA_BUTTON,
                                    width=ui.Pixel(232),
                                    height=ui.Pixel(24),
                                    clicked_fn=self._on_add_robot_schema_clicked,
                                    style={
                                        "Button": {
                                            "background_color": cl("#3D5A80"),
                                            "border_radius": 2,
                                        },
                                        "Button.Label": {"color": TEXT_BODY, "font_size": 14},
                                    },
                                )
                                ui.Spacer(width=ui.Fraction(1))
                            ui.Spacer(height=ui.Pixel(32))
                    ui.Spacer(width=ui.Fraction(1))
                ui.Spacer(height=ui.Fraction(1))

    def _on_help_click(self, b) -> None:
        """Opens an extension's documentation in a Web Browser

        Args:
            b: Button event parameter
        """
        import webbrowser

        doc_link = (
            "https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html"
        )
        try:
            webbrowser.open(doc_link, new=2)
        except Exception as e:
            carb.log_warn(f"Could not open browswer with url: {doc_link}, {e}")

    def _apply_schema_visibility(self) -> None:
        """Toggle between the no-schema panel and the normal working UI."""
        no_schema = self._should_show_gt_no_schema()
        if self._no_schema_frame:
            self._no_schema_frame.visible = no_schema
        if self._nav_buttons_frame:
            self._nav_buttons_frame.visible = not no_schema
        if self._gain_settings_page:
            self._gain_settings_page.visible = not no_schema
        if self._charts_page:
            self._charts_page.visible = False  # always start on Gain Settings
        if not no_schema:
            self._apply_nav_panel_visibility()

    def _apply_nav_panel_visibility(self) -> None:
        """Show Gain Settings page or Test Gains page based on active tab."""
        gs = self._gain_settings_page
        ch = self._charts_page
        if not gs or not ch:
            return
        if self._nav_show_charts:
            gs.visible = False
            ch.visible = True
            if self._charts_frame:
                self._charts_frame.collapsed = False
        else:
            gs.visible = True
            ch.visible = False

    def _update_nav_button_styles(self) -> None:
        """Apply active/inactive/disabled background to the nav tab buttons."""
        if self._gains_settings_button:
            bg = TAB_ACTIVE_BG if not self._nav_show_charts else TAB_INACTIVE_BG
            self._gains_settings_button.set_style({"Button": {"background_color": bg, "border_radius": 2}})
        if self._charts_button:
            if not self._charts_button.enabled:
                self._charts_button.set_style(
                    {
                        "Button": {"background_color": TAB_INACTIVE_BG, "border_radius": 2},
                        "Button:disabled": {"background_color": TAB_INACTIVE_BG},
                        "Button.Label:disabled": {"color": NAV_BUTTON_DISABLED_TEXT},
                    }
                )
            else:
                bg = TAB_ACTIVE_BG if self._nav_show_charts else TAB_INACTIVE_BG
                self._charts_button.set_style({"Button": {"background_color": bg, "border_radius": 2}})

    def _on_nav_gains_settings(self) -> None:
        self._nav_show_charts = False
        self._update_nav_button_styles()
        self._apply_nav_panel_visibility()

    def _on_nav_charts(self) -> None:
        self._nav_show_charts = True
        self._update_nav_button_styles()
        self._apply_nav_panel_visibility()
        if self._charts_frame:
            self._charts_frame.rebuild()

    def _refresh_backend_from_app(self) -> None:
        """Sync the header backend/solver info tags to the live physics engine.

        The backend and solver name the physics engine and solver Isaac Sim
        currently has active.  ``_backend_active_idx`` follows the active engine
        (0 = PhysX, 1 = Newton) and drives the backend-gated columns and the
        Advanced Actuator Parameters accordion.  The solver is load-bearing rather
        than decorative: Newton's schema-resolver order differs between the MuJoCo
        and XPBD/VBD solvers, so it selects which chain the advanced joint
        parameters resolve through, which is why the stage is passed in.
        """
        try:
            self._backend_ctx = BackendContext.from_app(omni.usd.get_context().get_stage())
        except _UI_GUARD_ERRORS:
            self._backend_ctx = BackendContext()
        self._backend_active_idx = 1 if self._backend_ctx.backend == "NewtonAPI" else 0
        self._apply_backend_selection(self._backend_active_idx)

    def _apply_backend_selection(self, idx: int) -> None:
        """Refresh the info-only backend/solver tags and rebuild the per-joint editor.

        The (cheap) header text tags are updated unconditionally, but the Gain
        Settings view is only rebuilt when the active backend/solver actually
        changed.  ``_refresh_backend_from_app`` is called on every ASSETS_LOADED
        stage event, which fires many times per second while the timeline plays;
        rebuilding the whole two-column view on each of those events produced a
        continuous rebuild loop and the visible window flicker.
        """
        # Backend and solver are read-only info tags naming the active engine; the
        # Gain Tuner does not change the active backend (SRD REQ-1/REQ-6).
        if self._backend_label_widget:
            self._backend_label_widget.text = self._backend_ctx.backend_display
        if self._solver_label_widget:
            self._solver_label_widget.text = f"Solver: {self._backend_ctx.solver_short}"
        # Newton actuators are backend-independent, so the active backend does not
        # decide whether an actuator is the active source.  The backend/solver
        # still affects the field labels and the editable-vs-read-only gating
        # (notably whether the MuJoCo solver makes mjc:* editable), so rebuild the
        # Gain Settings view when — and only when — the backend/solver changes.
        signature = (idx, self._backend_ctx.solver_short)
        if signature == self._backend_applied_signature:
            return
        self._backend_applied_signature = signature
        if self._gains_tuning_frame is not None and not self._test_running:
            try:
                self._gains_tuning_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass

    def _build_gain_read_context(self) -> gain_tuner.GainReadContext:
        """Build the gain-source read context for the current robot and backend.

        Traverses the articulation subtree for backend-independent Newton actuator
        gains and the composed stage for MuJoCo-native gains, and records the live
        Newton solver so the resolver can pick the active source (actuator, else mjc
        under the MuJoCo solver, else DriveAPI), gate editability accordingly, and
        resolve the advanced joint parameters through that solver's schema chain.
        """
        try:
            stage = omni.usd.get_context().get_stage()
            root = self._gains_tuner.get_articulation_root()
            actuator_map = gain_tuner.build_actuator_gain_map(stage, root) if root else {}
            # MuJoCo-native gains are a property of the composed stage/variant and
            # are independent of the active physics ENGINE, so they are detected
            # from the authored attributes rather than gated on backend == Newton.
            mjc_map = gain_tuner.build_mjc_gain_map(stage) if stage else {}
            viewed = "NewtonAPI" if self._backend_active_idx == 1 else "PhysX"
            return gain_tuner.GainReadContext(
                viewed_backend=viewed,
                active_backend=self._backend_ctx.backend,
                actuator_map=actuator_map,
                mjc_map=mjc_map,
                mujoco_solver_active=self._backend_ctx.is_mujoco_solver,
                solver=self._active_solver(),
            )
        except _UI_GUARD_ERRORS:
            return gain_tuner.GainReadContext()

    def _populate_robot_menu(self) -> list:
        """Populates the robot selection menu with prims that have the Robot Schema API.

        Returns:
            Sorted USD paths for the combo box (only prims with Robot Schema).
        """
        items_schema = []
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return []
        root = stage.GetPrimAtPath("/")
        if not root.IsValid():
            return []
        for prim in pxr.Usd.PrimRange(root):
            if not prim.IsValid():
                continue
            if prim.HasAPI(Classes.ROBOT_API.value):
                items_schema.append(str(prim.GetPath()))
        return sorted(set(items_schema))

    _NO_ROBOT_PLACEHOLDER = "No Robot found in scene with Robot Schema"

    def build_header_ui(self) -> None:
        """Build the sticky header rows (called by extension.py outside the ScrollingFrame)."""
        self._nav_show_charts = False

        def _update_articulation_selection(m, n) -> None:
            if m.has_item():
                val = m.get_current_string()
                if val == self._NO_ROBOT_PLACEHOLDER:
                    self._on_articulation_selection(None)
                else:
                    self._on_articulation_selection(val)
            else:
                self._on_articulation_selection(None)

        self._update_articulation_fn = _update_articulation_selection

        with ui.VStack(style=get_style(), spacing=0, height=0):
            self._build_header_section(self._NO_ROBOT_PLACEHOLDER, _update_articulation_selection)

    def build_content_ui(self) -> None:
        """Build the scrollable content area (called by extension.py inside the ScrollingFrame)."""
        # Reset overlay tracking before (re)building the two test-in-progress panels.
        self._test_in_progress_panels = []
        self._test_progress_fills = []
        self._test_progress_gaps = []
        self._test_progress_labels = []
        self._test_progress_pct_labels = []
        with ui.VStack(style=get_style(), spacing=0, height=0):
            # ---- No-schema placeholder ----
            self._no_schema_frame = ui.Frame(visible=False)
            with self._no_schema_frame:
                self._build_gt_no_schema_panel()

            # ---- Gain Settings page (two columns: joint list | per-joint detail) ----
            self._gain_settings_page = ui.Frame(height=0, visible=True)
            with self._gain_settings_page:
                with ui.VStack(style=get_style(), spacing=0, height=0):
                    self._accordions_container = ui.Frame(height=0, visible=True)
                    with self._accordions_container:
                        with ui.VStack(spacing=0, height=0):
                            self._gains_tuning_frame = CollapsableFrame(
                                GT_HAND_TUNED_GAINS_TITLE,
                                collapsed=False,
                                enabled=True,
                                build_fn=self._build_gains_tuning_frame,
                                show_copy_button=False,
                            )
                    _gains_page_progress_panel = ui.Frame(height=0, visible=False)
                    with _gains_page_progress_panel:
                        self._build_test_in_progress_panel()
                    self._test_in_progress_panels.append(_gains_page_progress_panel)

            # ---- Test Gains page (validation: test controls + result charts) ----
            # Validation lives here, not in the per-joint Gain Settings view.
            self._charts_page = ui.Frame(height=0, visible=False)
            with self._charts_page:
                with ui.VStack(style=get_style(), spacing=0, height=0):
                    # Same test-in-progress overlay as the Gain Settings page, so a
                    # running test's progress is visible from the Test Gains tab too.
                    _charts_page_progress_panel = ui.Frame(height=0, visible=False)
                    with _charts_page_progress_panel:
                        self._build_test_in_progress_panel()
                    self._test_in_progress_panels.append(_charts_page_progress_panel)
                    self._test_controls_frame = CollapsableFrame(
                        GT_TEST_CONTROLS_TITLE,
                        collapsed=False,
                        enabled=True,
                        build_fn=self._build_test_controls_frame,
                        show_copy_button=False,
                    )
                    self._charts_frame = CollapsableFrame(
                        "Charts",
                        collapsed=False,
                        enabled=True,
                        build_fn=self._build_charts_frame,
                        show_copy_button=False,
                    )

        # ---- Wire up callbacks (outside VStack so no widgets are created here) ----
        self._articulation_menu_model.add_item_changed_fn(self._update_articulation_fn)

        # Populate the robot menu; the item_changed callback fires from refresh_list
        # and calls _apply_schema_visibility / _apply_nav_panel_visibility itself,
        # so we only need to call them explicitly for the no-robot fallback.
        items = self._populate_robot_menu()
        if self._articulation_menu_model:
            if items:
                self._articulation_menu_model.refresh_list(items)
            else:
                self._articulation_menu_model.refresh_list([self._NO_ROBOT_PLACEHOLDER])
                self._apply_schema_visibility()

        self._apply_nav_panel_visibility()
        self._update_nav_button_styles()

    def build_ui(self) -> None:
        """Single-call path: header + scrollable content in one VStack."""
        self.build_header_ui()
        try:
            self.build_content_ui()
        except Exception as exc:
            import traceback as _tb

            carb.log_error(f"[GainTuner] build_content_ui error: {exc}\n{_tb.format_exc()}")
            ui.Label(f"Content error: {exc}", word_wrap=True, style={"color": 0xFF4444FF})
        # Default the backend/solver info tags to the live physics engine.
        self._refresh_backend_from_app()

    # ------------------------------------------------------------------
    # Header section builder (always-visible top bar)
    # ------------------------------------------------------------------

    def _build_header_section(self, no_robot_placeholder: str, articulation_changed_fn) -> None:
        """Builds the sticky header: title row, robot/backend/solver row, and nav tabs."""
        _help_icon = f"{EXTENSION_FOLDER_PATH}/icons/help.svg"

        # ---- Row 1: help icon (the active-stage name is already obvious from the
        #      Isaac Sim app title/UI, so it is not repeated here) ----
        with ui.ZStack(height=34):
            ui.Rectangle(style={"background_color": HEADER_BG_COLOR})
            with ui.HStack(height=34):
                ui.Spacer(width=8)
                ui.Spacer()
                with ui.VStack(width=0):
                    ui.Spacer()
                    ui.Image(
                        name="help",
                        height=24,
                        width=24,
                        mouse_pressed_fn=lambda x, y, b, a: self._on_help_click(b),
                    )
                    ui.Spacer()
                ui.Spacer(width=8)

        # ---- Row 2: Robot | Backend | Solver ----
        self._articulation_menu_model = create_combo_list_model([no_robot_placeholder], 0)

        with ui.ZStack(height=28):
            ui.Rectangle(style={"background_color": HEADER_BG_COLOR})
            with ui.HStack(height=28):
                ui.Spacer(width=8)
                # Robot label
                with ui.VStack(width=0):
                    ui.Spacer()
                    ui.Label(
                        GT_ROBOT_LABEL,
                        width=0,
                        style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                    )
                    ui.Spacer()
                ui.Spacer(width=4)
                # Robot combo — fills all remaining space so it shrinks/grows with window
                with ui.VStack(width=ui.Fraction(1)):
                    ui.Spacer()
                    ui.ComboBox(
                        self._articulation_menu_model,
                        name="header_combo",
                        height=24,
                    )
                    ui.Spacer()
                ui.Spacer(width=8)
                # Vertical divider
                with ui.VStack(width=ui.Pixel(1)):
                    ui.Spacer(height=4)
                    ui.Rectangle(
                        width=1,
                        height=ui.Pixel(20),
                        style={"background_color": DIVIDER_COLOR},
                    )
                    ui.Spacer(height=4)
                ui.Spacer(width=8)
                # Backend label
                with ui.VStack(width=0):
                    ui.Spacer()
                    ui.Label(
                        GT_BACKEND_LABEL,
                        width=0,
                        style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                    )
                    ui.Spacer()
                ui.Spacer(width=4)
                # Backend is informational only: it names the physics engine Isaac
                # Sim currently has active.  It is deliberately NOT a toggle so users
                # don't mistake it for changing the active backend from the Gain
                # Tuner — the backend is selected in the app (SRD REQ-1/REQ-6).
                with ui.VStack(width=0):
                    ui.Spacer()
                    self._backend_label_widget = ui.Label(
                        self._backend_ctx.backend_display,
                        width=0,
                        style={"color": LABEL_COLOR, "font_size": FONT_SIZE},
                    )
                    ui.Spacer()
                ui.Spacer(width=8)
                # Vertical divider
                with ui.VStack(width=ui.Pixel(1)):
                    ui.Spacer(height=4)
                    ui.Rectangle(
                        width=1,
                        height=ui.Pixel(20),
                        style={"background_color": DIVIDER_COLOR},
                    )
                    ui.Spacer(height=4)
                ui.Spacer(width=8)
                # Solver label — fixed width so "Solver: Featherstone" never clips
                with ui.VStack(width=180):
                    ui.Spacer()
                    self._solver_label_widget = ui.Label(
                        "Solver: PhysX",
                        width=0,
                        style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                    )
                    ui.Spacer()
                ui.Spacer(width=8)

        # ---- Row 3: Nav tabs ----
        self._nav_buttons_frame = ui.Frame(height=0)
        with self._nav_buttons_frame:
            with ui.HStack(height=28, style=get_style()):
                self._gains_settings_button = ui.Button(
                    GT_GAIN_SETTINGS_TAB,
                    height=28,
                    width=ui.Fraction(1),
                    clicked_fn=self._on_nav_gains_settings,
                    style={"Button": {"background_color": TAB_ACTIVE_BG, "border_radius": 2}},
                )
                # The Test Gains tab now hosts the validation workflow (test controls
                # + result charts), so it is always enabled — the user runs tests here.
                self._charts_button = ui.Button(
                    GT_CHARTS_TAB,
                    height=28,
                    width=ui.Fraction(1),
                    clicked_fn=self._on_nav_charts,
                    enabled=True,
                    style={
                        "Button": {"background_color": TAB_INACTIVE_BG, "border_radius": 2},
                        "Button:disabled": {"background_color": TAB_INACTIVE_BG},
                        "Button.Label:disabled": {"color": NAV_BUTTON_DISABLED_TEXT},
                    },
                )

    # Search-toolbar geometry (the search field's fixed left width).
    _LIST_W = 240

    # Room the retained focused-joint detail/advanced/validation area is assumed to
    # want, used only to cap the table so the detail area starts on screen.  The
    # section itself is content-sized, and expanding Advanced Actuator Parameters
    # takes it well past this; the page then scrolls, which is the intended outcome.
    _DETAIL_H = 300

    def _get_tunable_joint_entries(self) -> list:
        """Return the joint entries that populate the tunable joint list.

        Mimic joints are excluded here (not in the core tuner) so they never
        appear as selectable/tunable joints while the core keeps every entry with
        its articulation DOF index intact (each entry carries its own
        ``dof_index``, so filtering the list never misaligns the remaining
        joints' DOF-index mapping used by tests and joint-state reads).
        """
        entries = self._gains_tuner.get_joint_entries()
        tunable = []
        for entry in entries:
            try:
                if gain_tuner.is_joint_mimic(entry.joint):
                    continue
            except _UI_GUARD_ERRORS:
                # A joint we cannot classify is kept so it stays tunable.
                pass
            tunable.append(entry)
        return tunable

    def _build_gains_tuning_frame(self) -> None:
        """Build the Gain Settings view: a single multi-select TreeView gain table
        above a retained focused-joint detail / test / save area.

        The table lists every tunable joint as one row; rows are multi-selected with
        Ctrl+click / Shift+click and editing a gain field applies to the whole
        selection.  Visible columns (Stiffness/Kp, Damping/Kd, optional Ki, dynamic
        advanced columns) auto-select from the shown joints' schemas and can be
        toggled via the hamburger column picker; their units follow the backend.
        """
        _search_icon = f"{EXTENSION_FOLDER_PATH}/icons/search_icon_16px.svg"
        _filter_icon = f"{EXTENSION_FOLDER_PATH}/icons/ico_filter.svg"
        _TOOLBAR_H = 26

        joint_entries = self._get_tunable_joint_entries()
        # Clamp/default the selected joint to a valid index.
        if not joint_entries:
            self._selected_joint_index = None
        elif self._selected_joint_index is None or self._selected_joint_index >= len(joint_entries):
            self._selected_joint_index = 0

        # Gain-source read context shared by the list badges and the detail editor.
        self._detail_gain_ctx = self._build_gain_read_context()

        with self._gains_tuning_frame:
            with ui.VStack(style=get_style(), spacing=4, height=0):

                # -- Search toolbar (aligned above the joint list column) --
                with ui.HStack(height=_TOOLBAR_H, spacing=0):
                    with ui.ZStack(width=self._LIST_W, height=_TOOLBAR_H):
                        ui.Rectangle(style={"background_color": TREEVIEW_BG_COLOR})
                        with ui.HStack(height=_TOOLBAR_H):
                            ui.Spacer(width=6)
                            with ui.VStack(width=14):
                                ui.Spacer()
                                ui.Image(
                                    _search_icon,
                                    width=14,
                                    height=14,
                                    style={"Image": {"image_url": _search_icon, "color": MUTED_LABEL_COLOR}},
                                )
                                ui.Spacer()
                            ui.Spacer(width=4)
                            self._name_search_model = ui.SimpleStringModel("")
                            with ui.ZStack():
                                self._name_search_placeholder = ui.Label(
                                    "Search joints...",
                                    height=_TOOLBAR_H,
                                    style={
                                        "color": MUTED_LABEL_COLOR,
                                        "font_size": FONT_SIZE,
                                        "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
                                    },
                                )
                                _sf = ui.StringField(
                                    self._name_search_model,
                                    height=_TOOLBAR_H,
                                    style={
                                        "Field": {
                                            "background_color": 0x00000000,
                                            "color": LABEL_COLOR,
                                            "font_size": FONT_SIZE,
                                            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
                                        }
                                    },
                                )
                                _sf.model.add_value_changed_fn(self._on_name_search_changed)
                            ui.Spacer(width=4)
                            with ui.VStack(width=18):
                                ui.Spacer()
                                ui.Image(
                                    _filter_icon,
                                    width=16,
                                    height=16,
                                    style={"Image": {"image_url": _filter_icon, "color": MUTED_LABEL_COLOR}},
                                    mouse_pressed_fn=lambda x, y, b, m: None,
                                )
                                ui.Spacer()
                            ui.Spacer(width=4)
                    # Flexible gap, then the column-picker hamburger on the right.
                    ui.Spacer()
                    with ui.VStack(width=22):
                        ui.Spacer()
                        ui.Image(
                            f"{EXTENSION_FOLDER_PATH}/icons/more_options.svg",
                            width=18,
                            height=18,
                            tooltip="Choose which columns to show",
                            style={"color": MUTED_LABEL_COLOR},
                            mouse_pressed_fn=lambda x, y, b, m: self._on_gain_columns_menu_pressed(b),
                        )
                        ui.Spacer()
                    ui.Spacer(width=8)

                # -- Gain table body: a single multi-select TreeView of the tunable
                #    joints.  Height is content-sized (capped at the viewport) and
                #    updated in place by sync_viewport_from_scroll_frame ->
                #    _apply_table_height so a resize never rebuilds this frame and a
                #    taller window does not open an empty band under the rows. --
                self._gains_table_row_count = len(joint_entries)
                _data_h = self._gains_table_data_height(self._gains_table_row_count)
                self._gains_table_hstack = ui.HStack(spacing=0, height=_data_h)
                with self._gains_table_hstack:
                    # Rebuildable so toggling a column (hamburger) or a name-search
                    # filter repaints the table in place without rebuilding the whole
                    # Gain Settings frame.
                    self._gain_table_frame = ui.Frame()
                    self._gain_table_frame.set_build_fn(self._build_gain_table)
                    self._gain_table_frame.rebuild()

                # -- Focused-joint detail / advanced / validation editor (retained
                #    below the table; reflects the last-selected row).
                #
                #    Sized to its content (``height=0``) rather than given a fixed
                #    height with a scrollbar of its own.  Expanding Advanced Actuator
                #    Parameters takes this section past 500 px, and a fixed 300 px
                #    window cut Max Joint Velocity in half mid-row and put the copy
                #    buttons -- the panel's only cross-backend action -- below a fold
                #    the user had to find, while the panel underneath sat blank.  The
                #    surrounding content ScrollingFrame already scrolls the page, so
                #    this section simply takes the room it needs from it. --
                with ui.ZStack(height=0):
                    ui.Rectangle(style={"background_color": FRAME_BG_COLOR})
                    self._joint_detail_frame = ui.Frame(height=0)
                    self._joint_detail_frame.set_build_fn(self._build_joint_detail_editor)
                    self._joint_detail_frame.rebuild()

                # -- Save Target row (rebuildable so the mirror toggle / target
                #    pickers and info line refresh in place) --
                self._save_target_frame = ui.Frame()
                self._save_target_frame.set_build_fn(self._build_save_target_row)
                self._save_target_frame.rebuild()

    # ------------------------------------------------------------------
    # Left column: joint list with per-joint gain-source badges
    # ------------------------------------------------------------------

    def _joint_source_availability(self, entry) -> tuple[bool, bool, bool]:
        """Return ``(has_physics_drive, has_actuator, has_mjc)`` for a joint entry.

        ``has_mjc`` reflects MuJoCo-native actuator gains authored for the joint in
        the composed stage/variant (e.g. under the ``mujoco`` Physics variant); it
        is independent of the active physics engine.
        """
        joint = entry.joint
        try:
            has_pd = gain_tuner.has_physics_drive(joint, entry.drive_axis)
        except _UI_GUARD_ERRORS:
            has_pd = False
        has_act = False
        has_mjc = False
        ctx = self._detail_gain_ctx
        if ctx is not None:
            try:
                joint_path = joint.GetPath().pathString
                has_act = joint_path in ctx.actuator_map
                has_mjc = joint_path in getattr(ctx, "mjc_map", {})
            except _UI_GUARD_ERRORS:
                has_act = False
                has_mjc = False
        return has_pd, has_act, has_mjc

    @staticmethod
    def _source_badge(has_pd: bool, has_act: bool, has_mjc: bool = False) -> tuple[str, str, str]:
        """Map source availability to ``(badge_text, badge_style, subtitle)``.

        Surfaces MuJoCo-native gains as an additional source so a joint that carries
        both ``UsdPhysics.DriveAPI`` gains and ``mjc:*`` actuator gains is not shown
        as drive-only.
        """
        if has_pd and has_act and has_mjc:
            return "Multiple", "source_newton", "PhysicsDriveAPI + Newton Actuator + MuJoCo"
        if has_pd and has_act:
            return "Both available", "source_newton", "PhysicsDriveAPI + Newton Actuator"
        if has_pd and has_mjc:
            return "Drive + MuJoCo", "source_newton", "PhysicsDriveAPI + MuJoCo-native gains"
        if has_act and has_mjc:
            return "Actuator + MuJoCo", "source_newton", "Newton Actuator + MuJoCo-native gains"
        if has_act:
            return "Actuator only", "source_newton", "Newton Actuator"
        if has_mjc:
            return "MuJoCo only", "source_newton", "MuJoCo-native gains"
        if has_pd:
            return "Drive only", "source_physics", "PhysicsDriveAPI"
        return "No gains", "source_physics", "-"

    # ------------------------------------------------------------------
    # Gain table: single multi-select TreeView + hamburger column picker
    # ------------------------------------------------------------------

    def _stiffness_unit_hint(self, rows, backend: str) -> str:
        """Return the stiffness/damping unit shown in the gain column headers.

        Uses the backend-dependent angle unit when the shown joints are all
        angular (revolute), the linear unit when all prismatic, and ``"mixed"``
        when they combine both DOF kinds (per-row tooltips still name each row's
        exact unit).
        """
        flags = {row.is_angular for row in rows}
        if flags == {True}:
            return gain_dof_unit(True, backend)
        if flags == {False}:
            return gain_dof_unit(False, backend)
        if not flags:
            return gain_dof_unit(True, backend)
        return "mixed"

    def _filter_rows_by_name(self, rows):
        """Return rows whose joint name matches the current search filter.

        Rows keep their original joint index (their position in the full tunable
        set) so selection, the detail editor, and writes stay aligned.
        """
        query = (self._name_search_query or "").strip().lower()
        if not query:
            return list(rows)
        return [row for row in rows if query in row.display_name.lower()]

    def _build_gain_table(self) -> None:
        """Build the single-TreeView gain table (multi-select, dynamic columns)."""
        entries = self._get_tunable_joint_entries()
        ctx = self._detail_gain_ctx or self._build_gain_read_context()
        backend = getattr(self._backend_ctx, "backend", "PhysX")

        # Rows are built from the full tunable set so their index maps back to the
        # detail editor / write helpers; the name filter only hides display rows.
        # The shared per-joint viewed-source map forces each row (and the "Source"
        # column) to its chosen source, re-resolving gains + editability.
        all_rows = build_gain_table_rows(
            entries, ctx, viewed_source_for=self._joint_viewed_source, engine_value_fn=self._engine_param_value
        )
        columns = resolve_visible_columns(
            all_rows, self._gain_column_overrides, self._active_backend(), self._active_solver()
        )
        unit_hint = self._stiffness_unit_hint(all_rows, backend)
        rows = self._filter_rows_by_name(all_rows)

        focused = self._selected_joint_index
        if focused is not None and not any(row.index == focused for row in rows):
            focused = None

        # Restore any multi-selection preserved across an in-place rebuild (e.g. a
        # source switch); consumed once so a robot swap does not carry stale rows.
        preselect = self._pending_table_selection
        self._pending_table_selection = None
        if preselect:
            preselect = {i for i in preselect if any(row.index == i for row in rows)}

        self._gain_table_view = GainTableView(
            rows=rows,
            columns=columns,
            backend=backend,
            unit_hint=unit_hint,
            write_cell_fn=self._write_gain_cell,
            selection_changed_fn=self._on_table_selection_changed,
            source_changed_fn=self._on_source_cell_changed,
            edit_committed_fn=self._on_table_cell_committed,
            read_row_fn=self._read_fresh_table_row,
            focused_index=focused,
            selected_indices=preselect,
        )
        # Keep the detail-panel gating in sync with the (re)built table's live
        # selection, so a rebuild (column toggle, name filter, robot swap) does not
        # leave a stale single/multi placeholder decision.
        self._detail_selection_count = self._gain_table_view.selected_count()

    def _write_gain_cell(self, row, column_key: str, value: float) -> None:
        """Persist a single edited cell (gain attr / mjc arrays or advanced attr).

        The :class:`GainTableView` calls this for every selected, editable row when
        a value is edited; read-only / blank cells are filtered out upstream by
        :func:`selection_edit_row_indices`.
        """
        spec = gain_column_spec(column_key)
        if spec is not None and spec.kind == "advanced":
            self._write_advanced_cell(row, column_key, value)
            return
        self._apply_gain_value(row, column_key, value)

    def _write_advanced_cell(self, row, column_key: str, value: float) -> None:
        """Write one advanced-parameter cell to the active backend's schema.

        A write that cannot be applied is reported, and the row rebuilt, so the cell
        stops showing a value the stage does not hold -- the same treatment the
        detail field and the copy buttons give a failure.
        """
        column = advanced_gain_column(column_key)
        if column is not None and column.param_spec is not None:
            # The cell was read through the active backend's resolver chain, so the
            # edit is authored on that backend's schema.  The other backend keeps
            # its own value: the two formulations are tuned independently.
            try:
                wrote = self._author_joint_param(row.entry.joint, column.param_spec, value, self._active_backend())
            except _UI_GUARD_ERRORS:
                wrote = False
            if not wrote:
                self._report_failed_cell_write()
            return
        _value, attr = row.advanced_cell(column_key)
        try:
            if attr is not None and attr.IsValid():
                attr.Set(float(value))
        except _UI_GUARD_ERRORS:
            pass

    def _report_failed_cell_write(self) -> None:
        """Toast a failed table-cell write and rebuild the row from the stage.

        There is no per-cell previous value to restore the way the detail field
        does, so the whole table is re-read; the cell then shows what the stage
        holds, which is what it was showing before the edit.
        """
        nm.post_notification(GT_JOINT_PARAM_WRITE_FAILED, duration=5, status=nm.NotificationStatus.WARNING)
        if self._gain_table_frame is None:
            return
        try:
            self._gain_table_frame.rebuild()
        except _UI_GUARD_ERRORS:
            pass

    def _apply_gain_value(self, row, column_key: str, value: float) -> None:
        """Write a single gain value to a row's editable cell (attr or mjc arrays)."""
        resolved = row.resolved
        if resolved.source == gain_tuner.GainSource.MUJOCO and resolved.mjc_source is not None:
            kp = value if column_key == GAIN_COLUMN_KP else resolved.kp
            kd = value if column_key == GAIN_COLUMN_KD else resolved.kd
            if column_key in (GAIN_COLUMN_KP, GAIN_COLUMN_KD):
                gain_tuner.author_mjc_gains(resolved.mjc_source, kp, kd)
            return
        attr = {
            GAIN_COLUMN_KP: resolved.kp_attr,
            GAIN_COLUMN_KD: resolved.kd_attr,
            GAIN_COLUMN_KI: resolved.ki_attr,
        }.get(column_key)
        try:
            if attr is not None and attr.IsValid():
                attr.Set(float(value))
        except _UI_GUARD_ERRORS:
            pass

    def _on_table_selection_changed(self, focused_index, selected_count: int | None = None) -> None:
        """Follow the table selection: rebuild the detail editor.

        ``focused_index`` picks *which* joint the detail panel shows; the
        ``selected_count`` (the size of the current multi-selection) decides
        *whether* the per-joint panel is shown at all — it renders only for a
        single selection, otherwise a placeholder is shown (see
        :func:`detail_panel_mode`).
        """
        self._selected_joint_index = focused_index
        if selected_count is not None:
            self._detail_selection_count = selected_count
        elif self._gain_table_view is not None:
            self._detail_selection_count = self._gain_table_view.selected_count()
        # Note: the per-joint viewed-source override (self._joint_viewed_source) is
        # NOT reset here — it persists per joint (shared with the table Source
        # column) so re-selecting a joint keeps its chosen source.
        if self._joint_detail_frame:
            try:
                self._joint_detail_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass

    def _on_name_search_changed(self, model) -> None:
        """Filter the table rows by joint name from the search field."""
        query = model.get_value_as_string()
        if hasattr(self, "_name_search_placeholder") and self._name_search_placeholder:
            self._name_search_placeholder.visible = not query
        self._name_search_query = query
        if self._gain_table_frame is not None:
            try:
                self._gain_table_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass

    # ---- Hamburger column picker (auto-select by schema + manual toggle) ----

    def _on_gain_columns_menu_pressed(self, button: int) -> None:
        """Open the column-picker popup when the hamburger button is clicked."""
        if button != 0:
            return
        self._open_gain_columns_menu()

    def _open_gain_columns_menu(self) -> None:
        """Build and show the grouped column picker, like the Joint Inspector.

        The menu is built **dynamically from the loaded articulation**: only
        categories/columns applicable to the shown joints are listed (Kp/Kd always;
        Ki only with a PID actuator; advanced columns per the authored-only /
        schema-present rules), and the read-only BACKENDS row offers only the
        backends relevant to the asset (PhysX always; MuJoCo only when a joint
        authors ``mjc:*`` gains).  A column's initial state follows the
        schema-driven auto-selection; toggling it records a manual override
        (scoped to applicable columns) and rebuilds the table.
        """
        entries = self._get_tunable_joint_entries()
        ctx = self._detail_gain_ctx or self._build_gain_read_context()
        rows = build_gain_table_rows(entries, ctx, engine_value_fn=self._engine_param_value)
        groups = grouped_columns_for_menu(
            rows, self._gain_column_overrides, self._active_backend(), self._active_solver()
        )
        mujoco_active = getattr(self._backend_ctx, "is_mujoco_solver", False)
        has_mjc = menu_has_mjc(rows, getattr(ctx, "mjc_map", None))
        backend_rows = menu_backend_rows(has_mjc, mujoco_active)

        self._gain_columns_menu = ui.Menu("Columns")
        with self._gain_columns_menu:
            ui.MenuItem("BACKENDS", enabled=False)
            for label, checked in backend_rows:
                ui.MenuItem(label, checkable=True, checked=checked, enabled=False, hide_on_click=False)
            for category, specs in groups:
                ui.Separator()
                ui.MenuItem(category.upper(), enabled=False)
                for spec, checked, _auto in specs:
                    ui.MenuItem(
                        spec.label,
                        checkable=True,
                        checked=checked,
                        hide_on_click=False,
                        checked_changed_fn=lambda m, key=spec.key: self._on_gain_column_toggled(key, m),
                    )
        self._gain_columns_menu.show()

    def _on_gain_column_toggled(self, column_key: str, model) -> None:
        """Record a manual column-visibility override and rebuild the table."""
        try:
            checked = model.get_value_as_bool()
        except _UI_GUARD_ERRORS:
            checked = bool(model)
        self._gain_column_overrides[column_key] = checked
        if self._gain_table_frame is not None:
            try:
                self._gain_table_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass

    # ------------------------------------------------------------------
    # Right column: selected-joint detail editor
    # ------------------------------------------------------------------

    @staticmethod
    def _source_full_label(source) -> str:
        """Display name for a :class:`GainSource` (viewed-source toggle + info).

        Delegates to the shared :func:`source_option_label` so the detail-panel
        viewed-source toggle and the table's "Source" column always show the same
        wording for a given source.
        """
        return source_option_label(source)

    # Per-joint viewed-source override keys used by the detail toggle.
    _VIEWED_SOURCE_KEYS = {
        gain_tuner.GainSource.PHYSICS_DRIVE: "physics",
        gain_tuner.GainSource.MUJOCO: "mujoco",
        gain_tuner.GainSource.ACTUATOR: "actuator",
    }

    @staticmethod
    def _joint_path_str(joint) -> str:
        """Best-effort USD path string for a joint prim (key into the viewed-source map)."""
        try:
            return joint.GetPath().pathString
        except _UI_GUARD_ERRORS:
            return ""

    @staticmethod
    def _viewed_source_enum(key):
        """Map a viewed-source key (``"physics"`` / ``"mujoco"`` / ``"actuator"``) to a GainSource.

        Returns None for an unset key so the resolver falls back to the source the
        active backend consumes (the default view).
        """
        if key == "physics":
            return gain_tuner.GainSource.PHYSICS_DRIVE
        if key == "mujoco":
            return gain_tuner.GainSource.MUJOCO
        if key == "actuator":
            return gain_tuner.GainSource.ACTUATOR
        return None

    def _build_joint_detail_editor(self) -> None:
        """Build the per-joint detail editor, but only for a single selection.

        The detail/info panel (gain source, viewed-source toggle, Controller
        Gains, advanced params, validation summary) is specific to one joint, so
        it is shown only when exactly one joint is selected in the table.  For an
        empty or multi-selection a short placeholder is shown instead and gains
        are edited directly in the table.  The gating is driven by the table's
        selection *count* (:func:`detail_panel_mode`), not a focused-row fallback.
        The global Save Target row lives outside this frame and stays visible.
        """
        entries = self._get_tunable_joint_entries()
        # Prefer the table's live selection count; fall back to the stored count
        # when the view is not available (e.g. during teardown / first build).
        count = self._detail_selection_count
        if self._gain_table_view is not None:
            try:
                count = self._gain_table_view.selected_count()
            except _UI_GUARD_ERRORS:
                pass
        mode = detail_panel_mode(count)

        index_ok = entries and self._selected_joint_index is not None and self._selected_joint_index < len(entries)
        if mode != DETAIL_PANEL_SINGLE or not index_ok:
            if mode == DETAIL_PANEL_MULTI:
                # Plain ASCII hyphen: the app's default font has no em/en-dash glyph
                # (it renders as a stray "?"), same reason as NA_CELL_TEXT.
                message = f"{count} joints selected - edit gains directly in the table."
            else:
                message = "Select a single joint to view its details."
            with ui.VStack(style=get_style()):
                ui.Spacer(height=24)
                ui.Label(
                    message,
                    alignment=ui.Alignment.CENTER,
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
                ui.Spacer()
            return

        entry = entries[self._selected_joint_index]
        joint = entry.joint
        drive_axis = entry.drive_axis
        base_ctx = self._detail_gain_ctx or self._build_gain_read_context()
        has_pd, has_act, has_mjc = self._joint_source_availability(entry)

        # Resolve gains for the viewed source.  The per-joint toggle forces a
        # specific source (PhysicsDrive / MuJoCo-native / Newton actuator); when
        # unset the source defaults to the one the active backend consumes.  The
        # override lives in the shared per-joint map so the table's "Source"
        # column and this toggle agree on the joint's viewed source.
        joint_path = self._joint_path_str(joint)
        resolved = gain_tuner.resolve_joint_gains(
            joint, drive_axis, base_ctx, viewed_source=self._joint_viewed_source.get(joint_path)
        )

        # Reset the live detail-field registries; the sections below repopulate them
        # so a later table edit can refresh these fields in place (table -> detail).
        self._detail_gain_models = {}
        self._detail_adv_models = []
        self._detail_nf_params = None

        with ui.VStack(style=get_style(), spacing=6, height=0):
            ui.Spacer(height=2)
            ui.Label(
                f"Joint: {entry.display_name}",
                style={"color": LABEL_COLOR, "font_size": HEADER_FONT_SIZE},
            )

            # -- Gain Source header (active source + routing, plus a viewed-source
            #    toggle when BOTH sources are authored).  Replaces the old
            #    selector + info box + Routing Preview blocks. --
            self._build_gain_source_header(resolved, has_pd, has_act, has_mjc)

            # -- Controller Gains (with an inline inactive-gains warning) --
            self._build_controller_gains_section(entry, resolved)

            # -- Advanced actuator parameters (both backends) --
            self._build_advanced_params_section(entry, resolved)

            # -- Current validation result summary (at the very bottom; empty
            #    state until a test has been run for this joint) --
            self._build_validation_summary_section(entry)

    def _on_detail_source_selected(self, which: str) -> None:
        """Switch the viewed gain source for the selected joint (comparison view).

        Records the choice in the shared per-joint map and rebuilds BOTH the
        detail panel and the table so the inline "Source" column mirrors it.
        """
        entries = self._get_tunable_joint_entries()
        if not entries or self._selected_joint_index is None or self._selected_joint_index >= len(entries):
            return
        joint_path = self._joint_path_str(entries[self._selected_joint_index].joint)
        source = self._viewed_source_enum(which)
        self._set_joint_viewed_source(joint_path, source)
        if self._joint_detail_frame:
            self._joint_detail_frame.rebuild()

    def _set_joint_viewed_source(self, joint_path: str, source) -> None:
        """Update the shared per-joint viewed-source map and rebuild the table.

        Shared by the detail panel's viewed-source toggle and the table's inline
        "Source" dropdown so both stay in sync.  A None source clears the override
        (back to the active-source default).  The table frame is rebuilt so the
        row re-resolves to the selected source's gains + editability; the current
        multi-selection is preserved across the rebuild.
        """
        if not joint_path:
            return
        if source is None or source == gain_tuner.GainSource.NONE:
            self._joint_viewed_source.pop(joint_path, None)
        else:
            self._joint_viewed_source[joint_path] = source
        if self._gain_table_view is not None:
            try:
                self._pending_table_selection = set(self._gain_table_view.selected_indices())
            except _UI_GUARD_ERRORS:
                self._pending_table_selection = None
        if self._gain_table_frame is not None:
            try:
                self._gain_table_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass

    def _on_source_cell_changed(self, row, source) -> None:
        """Handle an inline "Source" dropdown change from the table.

        Records the joint's viewed source in the shared map (keeping the detail
        panel's toggle in sync) and rebuilds the table + detail panel.
        """
        joint_path = self._joint_path_str(row.entry.joint)
        self._set_joint_viewed_source(joint_path, source)
        if self._joint_detail_frame is not None:
            try:
                self._joint_detail_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass

    def _sync_selected_row_to_table(self) -> None:
        """Push the selected joint's freshly written gains to its table row.

        The per-joint detail panel (Controller Gains, Natural-Frequency mode, and
        Advanced Actuator Parameters) writes gains straight to USD, but the table
        row's cell value models were snapshotted when the table was built, so
        without this the table cell keeps showing the pre-edit value (detail ->
        table sync was missing; table -> detail already works via reselection).

        Re-resolve just the edited (selected) joint -- reusing
        :func:`build_gain_table_rows` for a single entry so the exact same
        source/context/advanced-cell logic is applied, honoring the shared
        per-joint viewed-source map -- and hand the fresh values to the table view
        for a targeted, in-place cell update (no rebuild, so selection and scroll
        are preserved and there is no flicker).
        """
        if self._gain_table_view is None:
            return
        entries = self._get_tunable_joint_entries()
        idx = self._selected_joint_index
        if not entries or idx is None or idx >= len(entries):
            return
        entry = entries[idx]
        joint_path = self._joint_path_str(entry.joint)
        fresh = self._read_fresh_row_for_entry(entry)
        if fresh is None:
            return
        try:
            self._gain_table_view.refresh_row_cells(
                joint_path, fresh.resolved, fresh.advanced_cells, fresh.param_infos, fresh.param_displays
            )
        except _UI_GUARD_ERRORS:
            pass

    def _read_fresh_row_for_entry(self, entry):
        """Re-read one joint's table row from USD, the way the table was built.

        One entry through :func:`build_gain_table_rows` rather than a hand-rolled
        read, so the refreshed row is resolved through the same source, context,
        viewed-source map, and per-backend advanced-cell logic as the row it
        replaces -- the two disagreeing is how a cell ends up showing the other
        backend's value.

        Returns:
            The fresh :class:`~.gain_table_model.GainTableRow`, or None when the
            joint could not be read.
        """
        ctx = self._detail_gain_ctx or self._build_gain_read_context()
        try:
            rows = build_gain_table_rows(
                [entry], ctx, viewed_source_for=self._joint_viewed_source, engine_value_fn=self._engine_param_value
            )
        except _UI_GUARD_ERRORS:
            return None
        return rows[0] if rows else None

    def _read_fresh_table_row(self, row):
        """Re-read a table row the view asks about, by its own entry.

        Args:
            row: The row to re-read.

        Returns:
            The fresh :class:`~.gain_table_model.GainTableRow`, or None.
        """
        entry = getattr(row, "entry", None)
        if entry is None:
            return None
        return self._read_fresh_row_for_entry(entry)

    def _on_table_cell_committed(self, row, column_key: str, value: float) -> None:
        """Refresh the detail panel after a TABLE gain edit (table -> detail sync).

        Mirror of :meth:`_sync_selected_row_to_table`.  The detail panel snapshots
        its Controller-Gains / Natural-Frequency / advanced fields into their own
        value models when built, so a table edit (which writes USD and updates the
        table cell) otherwise leaves the detail fields stale until reselection.
        Only act when the edited row is the currently selected *single* joint --
        for a multi-selection the panel shows a placeholder, so there is nothing to
        update.
        """
        if self._gain_table_view is None:
            return
        try:
            if self._gain_table_view.selected_count() != 1:
                return
        except _UI_GUARD_ERRORS:
            return
        entries = self._get_tunable_joint_entries()
        idx = self._selected_joint_index
        if not entries or idx is None or idx >= len(entries):
            return
        if self._joint_path_str(entries[idx].joint) != self._joint_path_str(row.entry.joint):
            return
        self._refresh_detail_fields_for_selected()

    def _refresh_detail_fields_for_selected(self) -> None:
        """Push the selected joint's fresh gains into the live detail field models.

        Re-resolves the selected joint the same way the panel was built (single-row
        :func:`build_gain_table_rows` honoring the shared viewed-source map) and
        updates the registered Controller-Gains models (``kp`` / ``kd`` / ``ki``),
        the derived Natural-Frequency / Damping-Ratio models when that mode is
        active, and the advanced-param models (re-read straight from USD).  Detail
        write-backs are suspended during the update so setting the models cannot
        write to USD or re-sync the table (loop guard).  Targeted and in place: no
        panel rebuild, so there is no flicker even during a rapid table drag.
        """
        if not self._detail_gain_models and not self._detail_adv_models:
            return
        entries = self._get_tunable_joint_entries()
        idx = self._selected_joint_index
        if not entries or idx is None or idx >= len(entries):
            return
        entry = entries[idx]
        ctx = self._detail_gain_ctx or self._build_gain_read_context()
        try:
            fresh = build_gain_table_rows(
                [entry], ctx, viewed_source_for=self._joint_viewed_source, engine_value_fn=self._engine_param_value
            )[0]
        except _UI_GUARD_ERRORS:
            return
        resolved = fresh.resolved
        self._suspend_detail_writes = True
        try:
            self._set_detail_model(GAIN_COLUMN_KP, resolved.kp)
            self._set_detail_model(GAIN_COLUMN_KD, resolved.kd)
            self._set_detail_model(GAIN_COLUMN_KI, resolved.ki)
            # Natural-frequency mode: recompute nf / damping-ratio from the fresh
            # stiffness/damping using the same conversion inputs the panel built with.
            nf = self._detail_nf_params
            if nf is not None and "nf" in self._detail_gain_models and "dr" in self._detail_gain_models:
                # Indexed, not defaulted: a missing is_angular would silently apply
                # the angular per-degree scale to a linear DOF.
                use_force, m_eq, is_angular = nf["use_force"], nf["m_eq"], nf["is_angular"]
                try:
                    nf_val = gain_tuner.natural_frequency_hz_from_stiffness_position_drive(
                        resolved.kp, is_angular=is_angular, use_force_drive=use_force, m_eq=m_eq
                    )
                    dr_val = gain_tuner.damping_ratio_from_stiffness_damping_position_drive(
                        resolved.kd, resolved.kp, is_angular=is_angular, use_force_drive=use_force, m_eq=m_eq
                    )
                    self._set_detail_model("nf", nf_val)
                    self._set_detail_model("dr", dr_val)
                except _UI_GUARD_ERRORS:
                    pass
            # Advanced params are bound to USD; re-read each through the same
            # resolution its field was built with.
            for adv in self._detail_adv_models:
                try:
                    v = adv.read_value()
                    if v is not None and adv.model is not None:
                        adv.model.set_value(float(v))
                except _UI_GUARD_ERRORS:
                    pass
            # And re-derive how each renders.  A table edit authors the same
            # parameter this field shows, so the number is not the only thing that
            # went stale -- an unauthored field is now authored.
            self._refresh_detail_adv_displays()
        finally:
            self._suspend_detail_writes = False

    def _set_detail_model(self, key: str, value) -> None:
        """Set a registered detail field model to ``value`` when both exist."""
        model = self._detail_gain_models.get(key)
        if model is not None and value is not None:
            try:
                model.set_value(float(value))
            except _UI_GUARD_ERRORS:
                pass

    def _build_gain_source_header(self, resolved, has_pd: bool, has_act: bool, has_mjc: bool = False) -> None:
        """Compact 'Gain Source' header for the selected joint.

        Shows the active (consumed) source on a single line — folding in the
        routing meaning, since the active source is where trajectory commands go —
        and, only when BOTH sources are authored, a compact segmented toggle to
        switch the *viewed* source for comparison.  The segment matching the
        active source is tagged so "viewing" and "active" stay distinguishable.
        Replaces the old source selector, info box, and Routing Preview blocks.
        """
        # -- Single active-source line (also conveys command routing) --
        with ui.HStack(height=20, spacing=0):
            ui.Spacer(width=8)
            ui.Label(
                "Gain Source:",
                width=0,
                style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
            )
            ui.Spacer(width=6)
            if resolved.active_source == gain_tuner.GainSource.NONE:
                ui.Label(
                    "No editable gains authored",
                    width=0,
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
            else:
                ui.Label(
                    f"{self._source_full_label(resolved.active_source)} " "(Active, receives trajectory commands)",
                    width=0,
                    style={"color": LABEL_COLOR, "font_size": FONT_SIZE},
                )
            ui.Spacer()

        # -- MuJoCo-native gains notice --
        # The mjc:* actuator gains are authored on the composed stage/variant
        # independently of the active engine; surface that they exist alongside the
        # DriveAPI/actuator gains so the joint is not shown as drive-only.
        if has_mjc:
            with ui.HStack(height=0):
                ui.Spacer(width=8)
                ui.Label(
                    "MuJoCo-native actuator gains (mjc:gainPrm/biasPrm) are also authored for this "
                    "joint. Tuned DriveAPI gains are mirrored to MuJoCo on save (see the mirror "
                    "toggle in the Save Target row).",
                    word_wrap=True,
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
                ui.Spacer(width=8)

        # -- Viewed-source comparison toggle (whenever more than one source is
        #    authored on the joint).  Lets the user switch which source's params
        #    are shown among PhysicsDrive / MuJoCo-native / Newton actuator; only
        #    the active (consumed) source is editable, the rest are read-only for
        #    sim-to-sim inspection. --
        available = gain_tuner.available_viewed_sources(has_pd, has_mjc, has_act)
        if len(available) > 1:
            _active_style = {"Button": {"background_color": SUB_TAB_ACTIVE_BG, "border_radius": 4}}
            _inactive_style = {"Button": {"background_color": SUB_TAB_INACTIVE_BG, "border_radius": 4}}
            with ui.HStack(height=24, spacing=4):
                for src in available:
                    label = self._source_full_label(src)
                    # Tag the active (consumed) segment so "viewing" and "active"
                    # stay distinguishable.
                    if src == resolved.active_source and src != gain_tuner.GainSource.NONE:
                        label += "  (Active)"
                    showing = resolved.source == src
                    key = self._VIEWED_SOURCE_KEYS[src]
                    ui.Button(
                        label,
                        height=24,
                        clicked_fn=lambda k=key: self._on_detail_source_selected(k),
                        style=_active_style if showing else _inactive_style,
                    )

    def _field_row(
        self,
        label: str,
        model: ui.SimpleFloatModel | None,
        *,
        width: int = 120,
        unit: str | None = None,
        enabled: bool = True,
        on_change=None,
        tooltip: str = "",
        field_format: str = "",
        muted: bool = False,
        on_built: Callable[[ui.Widget], None] | None = None,
    ) -> ui.SimpleFloatModel | None:
        """Build one labeled numeric field row shared by every detail-panel field.

        A ``model`` of None renders a disabled placeholder (no backing value, used
        for pending-API parameters); otherwise a ``FloatDrag`` bound to ``model`` is
        built, an optional ``on_change`` handler is wired, and an optional trailing
        ``unit`` suffix label is shown.

        Args:
            label: The field label shown at the left of the row.
            model: The backing float model, or None for a disabled placeholder.
            width: Pixel width of the label column.
            unit: Optional trailing unit suffix (e.g. ``deg`` / ``rad``).
            enabled: Whether the ``FloatDrag`` is editable.
            on_change: Optional value-changed callback wired to ``model``.
            tooltip: Optional field tooltip.
            field_format: Optional ``printf`` format override for the drag's text,
                used to render a parameter that has no number (see
                :data:`~.gain_display.UNLIMITED_FIELD_FORMAT`).
            muted: Render the value muted, marking it as an engine default standing
                in for an unauthored parameter rather than an authored value.
            on_built: Called with the built widget.  ``field_format`` and ``muted``
                are a snapshot of what the value was worth at build time, and an
                edit can change that, so a caller that has to re-render the field
                later needs to keep hold of it.

        Returns:
            The ``model`` passed in (None for a placeholder row).
        """
        with ui.HStack(height=24):
            ui.Spacer(width=8)
            ui.Label(
                label,
                width=width,
                height=24,
                alignment=ui.Alignment.LEFT_CENTER,
                style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
            )
            if model is None:
                field = ui.FloatField(height=20, name="adv_param_field", enabled=False)
            else:
                drag_kwargs = {"format": field_format} if field_format else {}
                field = ui.FloatDrag(
                    model=model,
                    min=0.0,
                    step=0.1,
                    height=20,
                    enabled=enabled,
                    name="adv_param_field",
                    style={"color": MUTED_LABEL_COLOR} if muted else {},
                    **drag_kwargs,
                )
                if on_change is not None:
                    model.add_value_changed_fn(on_change)
            # Built rather than passed as a string so the sentence wraps inside the
            # panel instead of being clipped at its right edge.
            set_wrapped_tooltip(field, tooltip)
            if on_built is not None:
                on_built(field)
            if unit:
                ui.Spacer(width=4)
                ui.Label(
                    unit,
                    width=32,
                    height=24,
                    alignment=ui.Alignment.LEFT_CENTER,
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
            ui.Spacer(width=8)
        return model

    def _gain_field_row(
        self,
        label: str,
        value,
        attr,
        read_only: bool,
        unit: str | None = None,
        write_fn: Callable[[float, ui.SimpleFloatModel], None] | None = None,
        tooltip: str = "",
        field_format: str = "",
        muted: bool = False,
        on_built: Callable[[ui.Widget], None] | None = None,
    ) -> ui.SimpleFloatModel:
        """Build a labeled numeric gain field; writes back to ``attr`` when editable.

        When ``unit`` is given it is shown as a trailing suffix next to the field
        (e.g. the backend-dependent ``deg`` / ``rad`` unit for stiffness/damping).
        ``write_fn`` replaces the default single-attribute write for parameters
        whose target attribute is not fixed at build time, such as the per-backend
        joint params, where which schema an edit lands on depends on the engine
        running at the moment of the edit.  It is called with the new value and the
        field's own model, so a write that cannot be applied can put the model back
        rather than leaving the field showing a number the stage does not hold.

        ``value`` of None seeds the model with zero, so a caller that has no value
        must also pass a ``field_format`` that renders something other than the
        digit, or set ``read_only``.  ``muted`` marks the value as an engine
        default rather than an authored one.  ``on_built`` receives the widget, for
        a caller that has to re-render it after an edit (see
        :meth:`_apply_adv_field_display`).
        """
        model = ui.SimpleFloatModel(float(value) if value is not None else 0.0)
        on_change = None
        if not read_only and (attr is not None or write_fn is not None):

            def _on_change(m, _attr=attr) -> None:
                # Skip while the detail fields are being refreshed from a table
                # edit, so we do not write back / loop.
                if self._suspend_detail_writes:
                    return
                try:
                    if write_fn is not None:
                        write_fn(float(m.get_value_as_float()), m)
                    elif _attr and _attr.IsValid():
                        _attr.Set(float(m.get_value_as_float()))
                except (Tf.ErrorException, ValueError, TypeError) as exc:
                    carb.log_warn(f"[GainTuner] Failed to write gain attribute {label!r}: {exc}")
                # Mirror the new value into the matching table cell (detail ->
                # table sync).
                self._sync_selected_row_to_table()

            on_change = _on_change
        return self._field_row(
            label,
            model,
            width=120,
            unit=unit,
            enabled=not read_only,
            on_change=on_change,
            tooltip=tooltip,
            field_format=field_format,
            muted=muted,
            on_built=on_built,
        )

    def _build_controller_gains_section(self, entry, resolved) -> None:
        """Build the per-joint Controller Gains form with backend-correct labels.

        Field labels follow the resolved gain source (stiffness/damping for PhysX
        ``DriveAPI``; Kp/Kd/Ki for a Newton actuator).  For editable PhysX
        PhysicsDrive gains a tuning-mode toggle swaps the stiffness/damping fields
        for natural-frequency + damping-ratio fields (converted using the joint's
        effective inertia).  When the viewed source is not the one the active
        controller consumes, an inline warning is shown next to the fields.
        """
        # Editable only when the viewed source is the one the active backend
        # consumes; every other view (a non-active backend's source, or the
        # always-inspection MuJoCo-native params) is read-only.
        read_only = not gain_tuner.is_viewed_source_editable(
            getattr(resolved, "source", gain_tuner.GainSource.NONE),
            getattr(resolved, "active_source", gain_tuner.GainSource.NONE),
        )
        ui.Label(
            GT_CONTROLLER_GAINS_TITLE,
            style={"color": LABEL_COLOR, "font_size": FONT_SIZE},
        )
        # Inline inactive-gains warning: only when the viewed source is authored
        # but not the active (consumed) one.  Placed right where the user edits.
        if not resolved.is_active and resolved.source != gain_tuner.GainSource.NONE:
            self._inline_warning_row(GT_WARN_INACTIVE_GAINS)

        # Natural-frequency mode is only meaningful for editable PhysX
        # PhysicsDrive gains (not Newton kp/kd/ki, MuJoCo raw params, or mimic
        # joints).  Show the mode toggle and NF fields only in that case.
        if self._supports_natural_frequency_mode(entry, resolved):
            self._build_tuning_mode_toggle()
            if self._detail_tuning_mode == TuningMode.NATURAL_FREQUENCY:
                self._build_natural_frequency_rows(entry, resolved)
                return

        # Backend-dependent stiffness/damping unit (PhysX degrees, Newton/MuJoCo
        # radians for revolute DOFs; linear units for prismatic).  Applied to both
        # the table (column headers / tooltips) and here in the detail view.
        backend = getattr(self._backend_ctx, "backend", "PhysX")
        gain_unit = gain_dof_unit(is_angular_dof(entry.joint, getattr(entry, "drive_axis", None)), backend)

        # Editable MuJoCo-native gains (MuJoCo solver active): the tuned stiffness /
        # damping map to the ``mjc:gainPrm`` / ``mjc:biasPrm`` arrays, not to a
        # single scalar attribute, so route the writeback through author_mjc_gains.
        if resolved.source == gain_tuner.GainSource.MUJOCO and not read_only and resolved.mjc_source is not None:
            self._build_mjc_gain_fields(resolved, gain_unit)
            return

        # Default: name the fields from the resolved source's dynamic labels.  Keep
        # the models so a table edit can refresh these fields in place.
        self._detail_gain_models[GAIN_COLUMN_KP] = self._gain_field_row(
            resolved.kp_label, resolved.kp, resolved.kp_attr, read_only, unit=gain_unit
        )
        if resolved.ki is not None:
            self._detail_gain_models[GAIN_COLUMN_KI] = self._gain_field_row(
                resolved.ki_label or "Ki", resolved.ki, resolved.ki_attr, read_only
            )
        self._detail_gain_models[GAIN_COLUMN_KD] = self._gain_field_row(
            resolved.kd_label, resolved.kd, resolved.kd_attr, read_only, unit=gain_unit
        )

    def _build_mjc_gain_fields(self, resolved, unit: str | None = None) -> None:
        """Build editable MuJoCo-native stiffness/damping fields.

        Both fields recompute and re-author the joint's ``mjc:gainPrm`` /
        ``mjc:biasPrm`` arrays (plus scalar ``mjc:stiffness`` / ``mjc:damping`` when
        present) from the current field values via
        :func:`isaacsim.robot_setup.gain_tuner.author_mjc_gains`, so tuning the
        MuJoCo gains updates what the MuJoCo solver consumes.
        """
        mjc_source = resolved.mjc_source
        kp_model = self._mjc_field_row(resolved.kp_label, resolved.kp, unit)
        kd_model = self._mjc_field_row(resolved.kd_label, resolved.kd, unit)
        # Register so a table edit (MuJoCo solver active) refreshes these in place.
        self._detail_gain_models[GAIN_COLUMN_KP] = kp_model
        self._detail_gain_models[GAIN_COLUMN_KD] = kd_model

        def _write(_m=None) -> None:
            if self._suspend_detail_writes:
                return
            gain_tuner.author_mjc_gains(mjc_source, kp_model.get_value_as_float(), kd_model.get_value_as_float())
            # Mirror the new MuJoCo gains into the matching table cell.
            self._sync_selected_row_to_table()

        kp_model.add_value_changed_fn(_write)
        kd_model.add_value_changed_fn(_write)

    def _mjc_field_row(self, label: str, value, unit: str | None = None) -> ui.SimpleFloatModel:
        """Build a labeled, editable numeric field for a MuJoCo-native gain."""
        model = ui.SimpleFloatModel(float(value) if value is not None else 0.0)
        return self._field_row(label, model, width=120, unit=unit, enabled=True)

    def _supports_natural_frequency_mode(self, entry, resolved) -> bool:
        """Return True when the joint's gains support the natural-frequency toggle.

        Only editable (active) PhysX ``PhysicsDrive`` gains on a non-mimic joint
        qualify; Newton actuator gains and mimic joints keep their normal labels.
        """
        if resolved.source != gain_tuner.GainSource.PHYSICS_DRIVE or not resolved.is_active:
            return False
        if resolved.kp_attr is None and resolved.kd_attr is None:
            return False
        try:
            return not gain_tuner.is_joint_mimic(entry.joint)
        except _UI_GUARD_ERRORS:
            return False

    def _on_tuning_mode_selected(self, mode: TuningMode) -> None:
        """Switch the PhysicsDrive tuning mode and rebuild the detail editor."""
        self._detail_tuning_mode = mode
        if self._joint_detail_frame:
            self._joint_detail_frame.rebuild()

    def _on_inertia_updated(self) -> None:
        """Rebuild the detail editor when joint inertia is (re)computed.

        Natural-frequency values depend on the effective joint inertia, so refresh
        the panel once the physics tensors populate it.
        """
        if self._detail_tuning_mode == TuningMode.NATURAL_FREQUENCY and self._joint_detail_frame:
            try:
                self._joint_detail_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass

    def _build_tuning_mode_toggle(self) -> None:
        """Build the stiffness/damping vs natural-frequency segmented toggle."""
        _active_style = {"Button": {"background_color": SUB_TAB_ACTIVE_BG, "border_radius": 4}}
        _inactive_style = {"Button": {"background_color": SUB_TAB_INACTIVE_BG, "border_radius": 4}}
        is_nf = self._detail_tuning_mode == TuningMode.NATURAL_FREQUENCY
        with ui.HStack(height=24, spacing=4):
            ui.Button(
                GT_TUNING_MODE_STIFFNESS_LABEL,
                height=24,
                tooltip=GT_TUNING_MODE_TOOLTIP,
                clicked_fn=lambda: self._on_tuning_mode_selected(TuningMode.STIFFNESS),
                style=_inactive_style if is_nf else _active_style,
            )
            ui.Button(
                GT_TUNING_MODE_NATURAL_FREQUENCY_LABEL,
                height=24,
                tooltip=GT_TUNING_MODE_TOOLTIP,
                clicked_fn=lambda: self._on_tuning_mode_selected(TuningMode.NATURAL_FREQUENCY),
                style=_active_style if is_nf else _inactive_style,
            )

    def _nf_conversion_params(self, entry) -> dict:
        """Return the inputs every natural-frequency conversion for this joint needs.

        ``m_eq`` is the gain tuner's accumulated effective inertia (0.0 until the
        timeline has played once); ``use_force`` reflects the drive's ``force`` vs
        ``acceleration`` type (mass cancels for acceleration drives, matching the
        drive-math fallback); ``is_angular`` selects the stored-gain convention,
        since angular drives author both gains per degree and linear drives do not.
        """
        m_eq = 0.0
        try:
            m_eq = float(self._gains_tuner.get_joint_accumulated_inertia(entry.joint))
        except _UI_GUARD_ERRORS:
            m_eq = 0.0
        use_force = False
        try:
            type_attr = gain_tuner.get_joint_drive_type_attr(entry.joint, entry.drive_axis)
            if type_attr and type_attr.IsValid():
                use_force = type_attr.Get() == "force"
        except _UI_GUARD_ERRORS:
            use_force = False
        is_angular = is_angular_dof(entry.joint, getattr(entry, "drive_axis", None))
        return {"m_eq": m_eq, "use_force": use_force, "is_angular": is_angular}

    def _build_natural_frequency_rows(self, entry, resolved) -> None:
        """Build the natural-frequency + damping-ratio fields for PhysicsDrive gains.

        Edits are converted back to drive stiffness/damping (using the joint's
        effective inertia + drive type) and written to the same ``DriveAPI``
        attributes, so the existing save/mirror plan persists them unchanged.
        """
        params = self._nf_conversion_params(entry)
        m_eq, use_force, is_angular = params["m_eq"], params["use_force"], params["is_angular"]
        stiffness_attr = resolved.kp_attr
        damping_attr = resolved.kd_attr

        # Derive the current NF / damping ratio from the authored stiffness/damping.
        nf_value = gain_tuner.natural_frequency_hz_from_stiffness_position_drive(
            resolved.kp, is_angular=is_angular, use_force_drive=use_force, m_eq=m_eq
        )
        dr_value = gain_tuner.damping_ratio_from_stiffness_damping_position_drive(
            resolved.kd, resolved.kp, is_angular=is_angular, use_force_drive=use_force, m_eq=m_eq
        )

        # Warn when inertia is not yet available (force drives fall back to m_eq=1).
        if use_force and m_eq == 0.0:
            with ui.HStack(height=0):
                ui.Spacer(width=8)
                ui.Label(
                    GT_NATURAL_FREQUENCY_NEEDS_INERTIA,
                    word_wrap=True,
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
                ui.Spacer(width=8)

        nf_model = self._nf_field_row(NATURAL_FREQUENCY_LABEL, nf_value)
        dr_model = self._nf_field_row(DAMPING_RATIO_LABEL, dr_value)
        # Register the NF/DR models + conversion inputs so a table Stiffness/Damping
        # edit can recompute and refresh these derived fields in place.
        self._detail_gain_models["nf"] = nf_model
        self._detail_gain_models["dr"] = dr_model
        self._detail_nf_params = params

        def _write_stiffness_damping(stiffness_stored: float, damping: float) -> None:
            try:
                if stiffness_attr is not None and stiffness_attr.IsValid():
                    stiffness_attr.Set(float(stiffness_stored))
                if damping_attr is not None and damping_attr.IsValid():
                    damping_attr.Set(float(damping))
            except _UI_GUARD_ERRORS:
                pass

        def _on_nf_changed(m) -> None:
            if self._suspend_detail_writes:
                return
            # NF edit recomputes both stiffness and damping from (nf, damping ratio).
            stiffness_stored, damping = gain_tuner.stiffness_and_damping_from_natural_frequency_position_drive(
                m.get_value_as_float(),
                dr_model.get_value_as_float(),
                is_angular=is_angular,
                use_force_drive=use_force,
                m_eq=m_eq,
            )
            _write_stiffness_damping(stiffness_stored, damping)
            # The NF fields write the joint's stiffness/damping, so refresh the
            # table's Stiffness/Kp + Damping/Kd cells (detail -> table sync).
            self._sync_selected_row_to_table()

        def _on_dr_changed(m) -> None:
            if self._suspend_detail_writes:
                return
            # Damping-ratio edit recomputes damping from the current stored stiffness.
            current_stiffness = resolved.kp
            try:
                if stiffness_attr is not None and stiffness_attr.IsValid() and stiffness_attr.Get() is not None:
                    current_stiffness = float(stiffness_attr.Get())
            except _UI_GUARD_ERRORS:
                current_stiffness = resolved.kp
            damping = gain_tuner.damping_from_damping_ratio_position_drive(
                m.get_value_as_float(),
                current_stiffness,
                is_angular=is_angular,
                use_force_drive=use_force,
                m_eq=m_eq,
            )
            try:
                if damping_attr is not None and damping_attr.IsValid():
                    damping_attr.Set(float(damping))
            except _UI_GUARD_ERRORS:
                pass
            # The damping-ratio edit writes the joint's damping, so refresh the
            # table's Damping/Kd cell (detail -> table sync).
            self._sync_selected_row_to_table()

        nf_model.add_value_changed_fn(_on_nf_changed)
        dr_model.add_value_changed_fn(_on_dr_changed)

    def _nf_field_row(self, label: str, value) -> ui.SimpleFloatModel:
        """Build a labeled float field for a natural-frequency-mode parameter."""
        model = ui.SimpleFloatModel(float(value) if value is not None else 0.0)
        return self._field_row(label, model, width=160, enabled=True)

    def _compute_joint_validation_summary(self, entry) -> list[tuple[str, str]] | None:
        """Compute a standardized per-joint validation summary from the last test.

        Thin wrapper around ``_compute_dof_validation_summary`` that resolves the
        DOF index from a joint ``entry``.

        Returns:
            A list of ``(label, value_str)`` rows, or ``None`` when no results
            exist yet for this joint.
        """
        dof_index = getattr(entry, "dof_index", None)
        if dof_index is None:
            return None
        return self._compute_dof_validation_summary(dof_index)

    def _compute_dof_validation_summary(self, dof_index) -> list[tuple[str, str]] | None:
        """Compute a standardized validation summary for a single DOF.

        Shared by both the Gain Settings detail panel ("Current Test Results")
        and the Test Gains tab so both use identical computation and gating.  The
        metric set is chosen per test mode so it mirrors the mode-specific results
        tables the develop build reports:

        * **Stress Test** -> ``Max Velocity`` / ``Trigger Time`` / ``Trigger
          Velocity`` / ``Result`` (STABLE|UNSTABLE), read from the backend's
          per-DOF stress metrics (:meth:`_stress_metric_rows`).
        * **Snap to Limits** -> per-limit ``Lower/Upper Mean Error`` /
          ``Lower/Upper Max Error`` / ``Lower/Upper Settle`` / ``Result``
          (PASS|BLOCKED|FAIL), read from the backend's per-DOF snap metrics
          (:meth:`_snap_metric_rows`).
        * **Sinusoidal / Step Function** -> ``Peak Error`` / ``RMS Error`` (and
          ``Overshoot`` for step-like responses), computed from the recorded
          command-vs-observed position trajectory (:meth:`_dof_metric_rows`).

        Every mode is prefixed with a ``Test:`` row.  Rows that do not apply are
        omitted rather than shown with a placeholder, and all values use ASCII
        text (plus ``\u00b0`` for degrees) so nothing renders as a "?" glyph.

        Args:
            dof_index: Articulation DOF index to summarize.

        Returns:
            A list of ``(label, value_str)`` rows, or ``None`` when no results
            exist yet for this DOF.
        """
        try:
            if not self._gains_tuner or not self._gains_tuner.is_data_ready():
                return None
            if dof_index is None:
                return None

            # Display units: degrees for rotational DOFs, otherwise base units (m).
            is_rotational = False
            try:
                if self._gains_tuner.get_articulation().dof_types[dof_index] == DofType.Rotation:
                    is_rotational = True
            except _UI_GUARD_ERRORS:
                pass

            mode = self._last_test_mode
            if mode in (GainsTestMode.STRESS_TEST, GainsTestMode.SNAP_TO_LIMITS, GainsTestMode.DISCRETIZATION):
                # Stress, snap, and the dt sweep report a rich per-joint metric set
                # read straight from the backend's per-DOF metrics dict rather than
                # recomputed from the trajectory.
                metric = (self._gains_tuner.get_test_result_metrics() or {}).get(dof_index)
                if not metric:
                    return None
                if mode == GainsTestMode.STRESS_TEST:
                    rows = self._stress_metric_rows(metric)
                elif mode == GainsTestMode.DISCRETIZATION:
                    rows = self._discretization_metric_rows(metric, is_rotational)
                else:
                    rows = self._snap_metric_rows(metric, is_rotational)
            else:
                # Sinusoidal / step: peak/RMS (+overshoot) from the recorded
                # command-vs-observed trajectory.
                cmd_pos, _cmd_vel, obs_pos, _obs_vel, _times = self._gains_tuner.get_joint_states_from_gains_test(
                    dof_index
                )
                if cmd_pos is None or obs_pos is None or len(cmd_pos) == 0:
                    return None
                cmd = np.asarray(cmd_pos, dtype=float)
                obs = np.asarray(obs_pos, dtype=float)
                n = min(len(cmd), len(obs))
                if n == 0:
                    return None
                rows = self._dof_metric_rows(cmd[:n], obs[:n], is_rotational, mode)

            if not rows:
                return None

            return rows
        except _UI_GUARD_ERRORS:
            return None

    @staticmethod
    def _test_mode_display_name(mode) -> str | None:
        """Human-readable name for a ``GainsTestMode``, or None when unset.

        Matches the mode combo labels in ``_build_test_controls_frame`` so the
        summary's "Test:" row names the run consistently with the run controls.
        """
        return {
            GainsTestMode.SNAP_TO_LIMITS: "Snap to Limits",
            GainsTestMode.SINUSOIDAL: "Sinusoidal",
            GainsTestMode.STEP: "Step Function",
            GainsTestMode.STRESS_TEST: "Stress",
            GainsTestMode.DISCRETIZATION: "dt Sweep",
        }.get(mode)

    @staticmethod
    def _compute_test_total_duration(
        test_mode,
        num_sequences: int,
        test_duration: float,
        stress_duration: float,
        hold_duration: float,
        num_dt_steps: int = 0,
        timeout: float = 0.0,
    ) -> float:
        """Compute the progress-bar denominator (sim seconds) for a test run.

        Every mode loops per-sequence in the core, so the total is
        ``num_sequences * per_sequence_duration``, but each mode derives its
        per-sequence duration from a different field:

        * SINUSOIDAL / STEP run each sequence for ``test_duration`` seconds.
        * STRESS_TEST runs each sequence for ``stress_duration`` seconds.
        * SNAP_TO_LIMITS runtime is data-dependent, so it uses an estimate of
          ``_SNAP_CYCLES_PER_SEQUENCE * (hold_duration + _SNAP_APPROACH_ESTIMATE_S)``
          per sequence.
        * DISCRETIZATION runs ``num_dt_steps`` single-level probes (approach +
          hold), so each level is estimated at
          ``hold_duration + _DISCRETIZATION_SETTLE_TIMEOUT_FRACTION * timeout``
          (the fixed hold plus a settle allowance derived from the approach
          ``timeout``); the total ignores ``num_sequences`` because all joints run
          together at each timestep.

        Args:
            test_mode: The :class:`GainsTestMode` being run.
            num_sequences: Number of joint sequences the run partitions into.
            test_duration: Sinusoidal/step per-sequence duration field value.
            stress_duration: Stress-test per-sequence duration field value.
            hold_duration: Snap-to-limits / dt-sweep per-hold duration field value.
            num_dt_steps: Number of dt levels for the dt sweep (DISCRETIZATION only).
            timeout: dt-sweep per-level approach timeout, seconds (DISCRETIZATION only).

        Returns:
            The estimated total run duration in simulation seconds.
        """
        if test_mode == GainsTestMode.DISCRETIZATION:
            return num_dt_steps * (hold_duration + _DISCRETIZATION_SETTLE_TIMEOUT_FRACTION * timeout)
        if test_mode == GainsTestMode.STRESS_TEST:
            per_sequence = stress_duration
        elif test_mode == GainsTestMode.SNAP_TO_LIMITS:
            per_sequence = _SNAP_CYCLES_PER_SEQUENCE * (hold_duration + _SNAP_APPROACH_ESTIMATE_S)
        else:
            per_sequence = test_duration
        return num_sequences * per_sequence

    @staticmethod
    def _progress_fraction(elapsed: float, total: float, running: bool = True) -> float:
        """Compute the progress-bar fill fraction for a test run.

        The snap-to-limits and dt-sweep denominators are estimates, so ``elapsed``
        can exceed ``total``.  While the run is still going the fraction is held
        below 1.0 so a bar that has outrun its estimate does not read as finished.

        Args:
            elapsed: Simulation seconds accumulated so far.
            total: Estimated total run duration in simulation seconds.
            running: Whether the test is still running.

        Returns:
            The fill fraction in the range 0.0 to 1.0.

        Example:

        .. code-block:: python

            >>> UIBuilder._progress_fraction(45.0, 27.0)  # doctest: +NO_CHECK
            0.99
        """
        if total <= 0.0:
            return 0.0
        fraction = max(elapsed / total, 0.0)
        if running:
            return min(fraction, _MAX_RUNNING_PROGRESS)
        return min(fraction, 1.0)

    @staticmethod
    def _format_progress_time(elapsed: float, total: float) -> str:
        """Format the elapsed/estimated time readout for the progress panel.

        Once ``elapsed`` passes the estimate the readout switches from a
        ``elapsed/total`` form to an estimate-relative one, so an overrunning run
        reports its real elapsed time rather than appearing pinned at the estimate.

        Args:
            elapsed: Simulation seconds accumulated so far.
            total: Estimated total run duration in simulation seconds.

        Returns:
            The formatted time readout.

        Example:

        .. code-block:: python

            >>> UIBuilder._format_progress_time(45.0, 27.0)  # doctest: +NO_CHECK
            'Time: 45.0s (est. 27.0s)'
        """
        if elapsed > total:
            return f"Time: {elapsed:.1f}s (est. {total:.1f}s)"
        return f"Time: {elapsed:.1f}s/{total:.1f}s"

    @staticmethod
    def _compute_overshoot(cmd_scaled, obs_scaled, last_test_mode) -> float | None:
        """Compute the percent overshoot of a step-like response, or None.

        Overshoot only makes sense for the Step Function test (the command moves
        to a value and holds).  Gating is on both ``last_test_mode`` (so it never
        appears for sinusoidal/stress even if the data momentarily looks step-like)
        and a data heuristic; None is returned whenever overshoot does not apply.
        Snap-to-Limits reports its own develop-style per-limit metric set instead
        of overshoot, so it is intentionally excluded here.

        Args:
            cmd_scaled: Commanded positions in display units (already scaled).
            obs_scaled: Observed positions in display units (already scaled).
            last_test_mode: The :class:`GainsTestMode` that produced the data.

        Returns:
            The non-negative overshoot as a percentage of the step magnitude, or
            None when overshoot is not applicable.
        """
        if last_test_mode != GainsTestMode.STEP:
            return None
        try:
            cmd_s = np.asarray(cmd_scaled, dtype=float)
            obs_s = np.asarray(obs_scaled, dtype=float)
            n = min(len(cmd_s), len(obs_s))
            if n == 0:
                return None
            cmd_s, obs_s = cmd_s[:n], obs_s[:n]
            initial, final = cmd_s[0], cmd_s[-1]
            step_mag = abs(final - initial)
            cmd_range = float(np.max(cmd_s) - np.min(cmd_s))
            tail = cmd_s[max(0, int(0.8 * n)) :]
            is_step = step_mag > 1e-6 and (float(np.max(tail) - np.min(tail)) <= 0.05 * max(cmd_range, 1e-6))
            if not is_step:
                return None
            if final >= initial:
                overshoot = (float(np.max(obs_s)) - final) / step_mag * 100.0
            else:
                overshoot = (final - float(np.min(obs_s))) / step_mag * 100.0
            return max(overshoot, 0.0)
        except _UI_GUARD_ERRORS:
            return None

    @staticmethod
    def _dof_metric_rows(cmd, obs, is_rotational: bool, last_test_mode) -> list[tuple[str, str]]:
        """Assemble the sinusoidal/step per-DOF metric rows from cmd/observed arrays.

        Computes peak and RMS error with rotational-vs-linear unit scaling (rad to
        degrees for rotational DOFs), then assembles the display rows:

        * A leading "Test:" row names the run when ``last_test_mode`` is set.
        * Peak Error / RMS Error are always reported.
        * Overshoot is reported only for a step-like Step Function response.

        This path handles the trajectory-derived modes (Sinusoidal / Step).  The
        Stress and Snap-to-Limits modes report their own develop-style metric sets
        via :meth:`_stress_metric_rows` / :meth:`_snap_metric_rows`.  The "Last Run
        Time" row is intentionally excluded; the caller appends it from state.

        Args:
            cmd: Commanded positions in base units (radians / meters).
            obs: Observed positions in base units (radians / meters).
            is_rotational: True when the DOF is rotational (scale rad to degrees).
            last_test_mode: The :class:`GainsTestMode` that produced the data.

        Returns:
            A list of ``(label, value_str)`` rows (empty when no samples exist).
        """
        cmd = np.asarray(cmd, dtype=float)
        obs = np.asarray(obs, dtype=float)
        n = min(len(cmd), len(obs))
        if n == 0:
            return []
        cmd, obs = cmd[:n], obs[:n]

        # Display units: degrees for rotational DOFs, otherwise base units (m).
        unit = "m"
        scale = 1.0
        if is_rotational:
            scale = 180.0 / np.pi
            unit = "\u00b0"

        err = (obs - cmd) * scale
        peak_error = float(np.max(np.abs(err)))
        rms_error = float(np.sqrt(np.mean(np.square(err))))

        rows: list[tuple[str, str]] = []
        # Lead with a short summary of which test produced these results, so the
        # metrics below are read in the right context (matches the combo labels).
        test_name = UIBuilder._test_mode_display_name(last_test_mode)
        if test_name is not None:
            rows.append((GT_TEST_INFO_TEST_TYPE, test_name))
        rows.extend(
            [
                (GT_TEST_INFO_PEAK_ERROR, f"{peak_error:.3f} {unit}".strip()),
                (GT_TEST_INFO_RMS_ERROR, f"{rms_error:.3f} {unit}".strip()),
            ]
        )

        overshoot = UIBuilder._compute_overshoot(cmd * scale, obs * scale, last_test_mode)
        if overshoot is not None:
            rows.append((GT_TEST_INFO_OVERSHOOT, f"{overshoot:.1f} %"))

        return rows

    @staticmethod
    def _stress_metric_rows(metric: dict) -> list[tuple[str, str]]:
        """Assemble the per-DOF Stress-test metric rows from backend metrics.

        Mirrors the develop build's "Stress Test Results" table (Max Vel / Trigger
        Time / Trigger Vel / Result) in the per-joint ``(label, value)`` layout so
        the branch reports the same data.  ``max_velocity`` and ``trigger_velocity``
        are shown unscaled and unit-less exactly as develop does; ``trigger_time``
        and ``trigger_velocity`` fall back to ``N/A`` when not triggered (NaN).

        Args:
            metric: The backend per-DOF stress metrics dict (``get_test_result_metrics()[dof]``).

        Returns:
            A list of ``(label, value_str)`` rows led by the ``Test:`` row.
        """
        rows: list[tuple[str, str]] = [
            (GT_TEST_INFO_TEST_TYPE, UIBuilder._test_mode_display_name(GainsTestMode.STRESS_TEST))
        ]
        if not metric:
            return rows
        max_vel = float(metric.get("max_velocity", 0.0))
        trigger_time = metric.get("trigger_time", float("nan"))
        trigger_vel = metric.get("trigger_velocity", float("nan"))
        status = str(metric.get("status", "stable")).upper()

        rows.append((GT_TEST_INFO_MAX_VELOCITY, f"{max_vel:.2f}"))
        rows.append(
            (
                GT_TEST_INFO_TRIGGER_TIME,
                f"{float(trigger_time):.3f} s" if UIBuilder._is_finite_number(trigger_time) else "N/A",
            )
        )
        rows.append(
            (
                GT_TEST_INFO_TRIGGER_VELOCITY,
                f"{float(trigger_vel):.2f}" if UIBuilder._is_finite_number(trigger_vel) else "N/A",
            )
        )
        rows.append((GT_TEST_INFO_RESULT, status))
        return rows

    @staticmethod
    def _snap_metric_rows(metric: dict, is_rotational: bool) -> list[tuple[str, str]]:
        """Assemble the per-DOF Snap-to-Limits metric rows from backend metrics.

        Mirrors the develop build's "Snap to Limits Results" table (per-limit mean
        and max position error, per-limit settling time, and PASS/BLOCKED/FAIL
        result) in the per-joint ``(label, value)`` layout so the branch reports
        the same data.  Position errors scale rad -> degrees for rotational DOFs
        (using ``\u00b0`` to stay consistent with the peak/RMS rows); settling times
        fall back to ``N/A`` when unavailable (NaN).

        Args:
            metric: The backend per-DOF snap metrics dict (``get_test_result_metrics()[dof]``).
            is_rotational: True when the DOF is rotational (scale rad to degrees).

        Returns:
            A list of ``(label, value_str)`` rows led by the ``Test:`` row.
        """
        rows: list[tuple[str, str]] = [
            (GT_TEST_INFO_TEST_TYPE, UIBuilder._test_mode_display_name(GainsTestMode.SNAP_TO_LIMITS))
        ]
        if not metric:
            return rows
        scale = 180.0 / np.pi if is_rotational else 1.0
        unit = "\u00b0" if is_rotational else "m"

        def _err(key: str) -> str:
            return f"{float(metric.get(key, float('inf'))) * scale:.4f} {unit}".strip()

        def _settle(key: str) -> str:
            v = metric.get(key, float("nan"))
            return f"{float(v):.3f} s" if UIBuilder._is_finite_number(v) else "N/A"

        rows.append((GT_TEST_INFO_LOWER_MEAN_ERROR, _err("lower_position_error")))
        rows.append((GT_TEST_INFO_LOWER_MAX_ERROR, _err("lower_max_error")))
        rows.append((GT_TEST_INFO_LOWER_SETTLE, _settle("lower_settling_time")))
        rows.append((GT_TEST_INFO_UPPER_MEAN_ERROR, _err("upper_position_error")))
        rows.append((GT_TEST_INFO_UPPER_MAX_ERROR, _err("upper_max_error")))
        rows.append((GT_TEST_INFO_UPPER_SETTLE, _settle("upper_settling_time")))
        rows.append((GT_TEST_INFO_RESULT, str(metric.get("status", "fail")).upper()))
        return rows

    #: Error-degradation fraction at/below which the detail summary drops the row as
    #: uninformative (it is +0.0 % for well-behaved joints, adding noise to a compact
    #: summary).  The all-joints table and vs-dt charts still carry the full detail.
    _DISCRETIZATION_ERROR_DEGRAD_EPS = 1e-9

    @staticmethod
    def _dt_sweep_target_dt_text(target_dt) -> str:
        """Format a target dt value as ``<s> s (<Hz> Hz)``, or ``N/A``."""
        if UIBuilder._is_finite_number(target_dt) and target_dt > 0:
            return f"{target_dt:.5f} s ({1.0 / target_dt:.0f} Hz)"
        return "N/A"

    @staticmethod
    def _dt_sweep_cliff_text(cliff) -> str:
        """Format an accuracy-cliff dt as ``<s> s (<Hz> Hz)``, or ``N/A``."""
        if UIBuilder._is_finite_number(cliff) and cliff > 0:
            return f"{float(cliff):.5f} s ({1.0 / cliff:.0f} Hz)"
        return "N/A"

    @staticmethod
    def _dt_sweep_settle_at_target_text(level: dict) -> str:
        """Format the target-level settling time with its inline degradation percent.

        Returns ``"<t> s (+d%)"`` when both the settling time and its degradation
        are available, ``"<t> s"`` when only the time is known, or ``"N/A"``.
        """
        settle_t = level.get("settling_time")
        if not UIBuilder._is_finite_number(settle_t):
            return "N/A"
        text = f"{float(settle_t):.3f} s"
        settle_deg = level.get("settle_degradation")
        if UIBuilder._is_finite_number(settle_deg):
            text += f" ({settle_deg * 100.0:+.1f}%)"
        return text

    @staticmethod
    def _dt_sweep_ss_error_text(level: dict, scale: float, unit: str) -> str:
        """Format the target-level steady-state error in display units, or ``N/A``."""
        ss_error = level.get("steady_state_error", float("nan"))
        if not UIBuilder._is_finite_number(ss_error):
            return "N/A"
        return f"{float(ss_error) * scale:.4f} {unit}".strip()

    @staticmethod
    def _dt_sweep_result_text(status) -> str:
        """Map a dt-sweep status token to its display label (``ACCURATE`` etc.)."""
        return str(status or "did_not_settle").upper().replace("_", " ")

    @staticmethod
    def _dt_sweep_status_color(status) -> int:
        """Map a dt-sweep status token to its result color (green / amber / red)."""
        return {
            "accurate": PASS_COLOR,
            "degraded": WARNING_COLOR,
            "indeterminate": WARNING_COLOR,
            "did_not_settle": FAIL_COLOR,
        }.get(str(status), FAIL_COLOR)

    @staticmethod
    def _dt_sweep_verdict(statuses: list[str]) -> str:
        """Aggregate per-joint classifications into a scene-level verdict token.

        The worst outcome wins: any joint that did not settle makes the whole scene
        ``did_not_settle``; otherwise any degraded joint makes it ``degraded``;
        otherwise any joint whose finest-dt reference did not settle makes it
        ``indeterminate``; otherwise (every joint accurate) it is ``accurate``.

        Args:
            statuses: Per-joint status tokens (``accurate`` / ``degraded`` /
                ``indeterminate`` / ``did_not_settle``).

        Returns:
            The aggregated verdict token, defaulting to ``did_not_settle`` when the
            list is empty.
        """
        if not statuses:
            return "did_not_settle"
        if any(s == "did_not_settle" for s in statuses):
            return "did_not_settle"
        if any(s == "degraded" for s in statuses):
            return "degraded"
        if any(s == "indeterminate" for s in statuses):
            return "indeterminate"
        if all(s == "accurate" for s in statuses):
            return "accurate"
        return "degraded"

    @staticmethod
    def _dt_sweep_verdict_text(verdict: str) -> str:
        """Map a verdict token to its banner text."""
        return {
            "accurate": GT_DT_SWEEP_VERDICT_ALL_ACCURATE,
            "degraded": GT_DT_SWEEP_VERDICT_DEGRADED,
            "indeterminate": GT_DT_SWEEP_VERDICT_INDETERMINATE,
            "did_not_settle": GT_DT_SWEEP_VERDICT_DID_NOT_SETTLE,
        }.get(str(verdict), GT_DT_SWEEP_VERDICT_DID_NOT_SETTLE)

    @staticmethod
    def _discretization_table_row(
        joint_name: str, metric: dict, is_rotational: bool
    ) -> tuple[str, str, str, str, str, str]:
        """Build one all-joints dt-sweep table row from a joint's aggregated metrics.

        Args:
            joint_name: The joint's display name for the first column.
            metric: The aggregated per-DOF dt-sweep metrics dict.
            is_rotational: True when the DOF is rotational (scale rad to degrees).

        Returns:
            A ``(joint, result, settle_at_target, ss_error, cliff, status)`` tuple;
            the trailing ``status`` token drives the result cell color.
        """
        scale = 180.0 / np.pi if is_rotational else 1.0
        unit = "\u00b0" if is_rotational else "m"
        level = (metric or {}).get("target_level") or {}
        status = (metric or {}).get("status", "did_not_settle")
        return (
            joint_name,
            UIBuilder._dt_sweep_result_text(status),
            UIBuilder._dt_sweep_settle_at_target_text(level),
            UIBuilder._dt_sweep_ss_error_text(level, scale, unit),
            UIBuilder._dt_sweep_cliff_text((metric or {}).get("accuracy_cliff_dt")),
            str(status),
        )

    @staticmethod
    def _snap_table_row(
        joint_name: str, metric: dict, is_rotational: bool
    ) -> tuple[str, str, str, str, str, str, str, str, str]:
        """Build one all-joints Snap-to-Limits table row from a joint's metrics.

        Mirrors develop's snap results table columns (per-limit mean and max
        position error, per-limit settling time, and a PASS/BLOCKED/FAIL result).
        Position errors scale rad -> degrees for rotational DOFs (using ``\u00b0``
        to match the detail summary); settling times fall back to ``N/A`` when
        unavailable (NaN).

        Args:
            joint_name: The joint's display name for the first column.
            metric: The backend per-DOF snap metrics dict.
            is_rotational: True when the DOF is rotational (scale rad to degrees).

        Returns:
            A ``(joint, lower_mean, lower_max, lower_settle, upper_mean,
            upper_max, upper_settle, result, status)`` tuple; the trailing
            ``status`` token (``pass`` / ``blocked`` / ``fail``) drives the result
            cell color.
        """
        scale = 180.0 / np.pi if is_rotational else 1.0
        unit = "\u00b0" if is_rotational else "m"
        m = metric or {}
        status = str(m.get("status", "fail")).lower()

        def _err(key: str) -> str:
            val = m.get(key, float("inf"))
            return f"{float(val) * scale:.4f} {unit}".strip()

        def _settle(key: str) -> str:
            val = m.get(key, float("nan"))
            return f"{float(val):.3f} s" if UIBuilder._is_finite_number(val) else "N/A"

        return (
            joint_name,
            _err("lower_position_error"),
            _err("lower_max_error"),
            _settle("lower_settling_time"),
            _err("upper_position_error"),
            _err("upper_max_error"),
            _settle("upper_settling_time"),
            status.upper(),
            status,
        )

    @staticmethod
    def _snap_status_color(status) -> int:
        """Map a snap status token to its result color (green / amber / red)."""
        return {
            "pass": PASS_COLOR,
            "blocked": WARNING_COLOR,
            "fail": FAIL_COLOR,
        }.get(str(status).lower(), FAIL_COLOR)

    @staticmethod
    def _snap_verdict(statuses: list[str]) -> str:
        """Aggregate per-joint snap statuses into a scene-level verdict token.

        The worst outcome wins: any failed joint makes the scene ``fail``;
        otherwise any blocked joint makes it ``blocked``; otherwise (every joint
        passed) it is ``pass``.  Defaults to ``fail`` for an empty list.

        Args:
            statuses: Per-joint status tokens (``pass`` / ``blocked`` / ``fail``).

        Returns:
            The aggregated verdict token.
        """
        normalized = [str(s).lower() for s in statuses]
        if not normalized:
            return "fail"
        if any(s == "fail" for s in normalized):
            return "fail"
        if any(s == "blocked" for s in normalized):
            return "blocked"
        return "pass"

    @staticmethod
    def _snap_verdict_text(verdict: str) -> str:
        """Map a snap verdict token to its banner text."""
        return {
            "pass": GT_SNAP_VERDICT_ALL_PASSED,
            "blocked": GT_SNAP_VERDICT_SOME_BLOCKED,
            "fail": GT_SNAP_VERDICT_SOME_FAILED,
        }.get(str(verdict).lower(), GT_SNAP_VERDICT_SOME_FAILED)

    @staticmethod
    def _stress_table_row(joint_name: str, metric: dict) -> tuple[str, str, str, str, str, str]:
        """Build one all-joints Stress-test table row from a joint's metrics.

        Mirrors develop's stress results table columns (max velocity, trigger
        time, trigger velocity, and a STABLE/UNSTABLE result).  ``max_velocity``
        and ``trigger_velocity`` are shown unscaled and unit-less exactly as
        develop does; ``trigger_time`` and ``trigger_velocity`` fall back to
        ``N/A`` when not triggered (NaN).

        Args:
            joint_name: The joint's display name for the first column.
            metric: The backend per-DOF stress metrics dict.

        Returns:
            A ``(joint, max_velocity, trigger_time, trigger_velocity, result,
            status)`` tuple; the trailing ``status`` token (``stable`` /
            ``unstable``) drives the result cell color.
        """
        m = metric or {}
        status = str(m.get("status", "stable")).lower()
        max_vel = float(m.get("max_velocity", 0.0))
        trigger_time = m.get("trigger_time", float("nan"))
        trigger_vel = m.get("trigger_velocity", float("nan"))
        return (
            joint_name,
            f"{max_vel:.2f}",
            f"{float(trigger_time):.3f} s" if UIBuilder._is_finite_number(trigger_time) else "N/A",
            f"{float(trigger_vel):.2f}" if UIBuilder._is_finite_number(trigger_vel) else "N/A",
            status.upper(),
            status,
        )

    @staticmethod
    def _stress_status_color(status) -> int:
        """Map a stress status token to its result color (green / red)."""
        return {
            "stable": PASS_COLOR,
            "unstable": FAIL_COLOR,
        }.get(str(status).lower(), FAIL_COLOR)

    @staticmethod
    def _stress_verdict(statuses: list[str]) -> str:
        """Aggregate per-joint stress statuses into a scene-level verdict token.

        Any unstable joint makes the scene ``unstable``; otherwise ``stable``.

        Args:
            statuses: Per-joint status tokens (``stable`` / ``unstable``).

        Returns:
            The aggregated verdict token.
        """
        if any(str(s).lower() == "unstable" for s in statuses):
            return "unstable"
        return "stable"

    @staticmethod
    def _stress_verdict_text(verdict: str) -> str:
        """Map a stress verdict token to its banner text."""
        return {
            "stable": GT_STRESS_VERDICT_ALL_STABLE,
            "unstable": GT_STRESS_VERDICT_UNSTABLE,
        }.get(str(verdict).lower(), GT_STRESS_VERDICT_ALL_STABLE)

    @staticmethod
    def _format_dt_level_progress(level_index: int, num_levels: int, dt_value: float) -> str:
        """Format the during-run dt-sweep progress label for one timestep level.

        Args:
            level_index: Zero-based index of the level currently running.
            num_levels: Total number of dt levels in the sweep.
            dt_value: The physics timestep of the current level, in seconds.

        Returns:
            A label like ``"dt level 3/10: 120 Hz (dt=0.00833 s)"``.
        """
        k = max(1, min(level_index + 1, num_levels))
        hz = 1.0 / dt_value if dt_value and dt_value > 0 else 0.0
        return f"dt level {k}/{num_levels}: {hz:.0f} Hz (dt={dt_value:.5f} s)"

    @staticmethod
    def _discretization_metric_rows(metric: dict, is_rotational: bool) -> list[tuple[str, str]]:
        """Assemble the SHORT per-DOF dt-sweep summary rows for the detail panel.

        Kept intentionally compact (Test, Target dt, Result, Settle @ Target, SS
        Error, Accuracy Cliff) so the "Current Test Results" summary fits within the
        Gain Settings detail scroll area above the sticky Save Target row.  The
        settle degradation is folded into the ``Settle @ Target`` value; the
        standalone Error Degradation row is dropped unless it is meaningfully above
        zero (it is +0.0 % for well-behaved joints).  The all-joints results table
        and vs-dt charts in the Test Gains tab carry the full breakdown.  Steady-state
        error scales rad -> degrees for rotational DOFs.

        Args:
            metric: The aggregated per-DOF dt-sweep metrics dict
                (``get_test_result_metrics()[dof]``).
            is_rotational: True when the DOF is rotational (scale rad to degrees).

        Returns:
            A list of ``(label, value_str)`` rows led by the ``Test:`` row.
        """
        rows: list[tuple[str, str]] = [
            (GT_TEST_INFO_TEST_TYPE, UIBuilder._test_mode_display_name(GainsTestMode.DISCRETIZATION))
        ]
        if not metric:
            return rows
        scale = 180.0 / np.pi if is_rotational else 1.0
        unit = "\u00b0" if is_rotational else "m"

        target_dt = metric.get("target_dt")
        if UIBuilder._is_finite_number(target_dt) and target_dt > 0:
            rows.append((GT_TEST_INFO_TARGET_DT, UIBuilder._dt_sweep_target_dt_text(target_dt)))

        rows.append((GT_TEST_INFO_RESULT, UIBuilder._dt_sweep_result_text(metric.get("status"))))

        level = metric.get("target_level") or {}
        rows.append((GT_TEST_INFO_SETTLE_AT_TARGET, UIBuilder._dt_sweep_settle_at_target_text(level)))
        rows.append((GT_TEST_INFO_SS_ERROR, UIBuilder._dt_sweep_ss_error_text(level, scale, unit)))

        # Error degradation is uninformative (+0.0 %) for well-behaved joints, so it
        # is only surfaced when it is meaningfully above zero.
        error_deg = level.get("error_degradation")
        if UIBuilder._is_finite_number(error_deg) and error_deg > UIBuilder._DISCRETIZATION_ERROR_DEGRAD_EPS:
            rows.append((GT_TEST_INFO_ERROR_DEGRAD, f"{error_deg * 100.0:+.1f} %"))

        rows.append((GT_TEST_INFO_ACCURACY_CLIFF, UIBuilder._dt_sweep_cliff_text(metric.get("accuracy_cliff_dt"))))
        return rows

    @staticmethod
    def _is_finite_number(value) -> bool:
        """True when ``value`` is a finite real number (not None / NaN / inf)."""
        try:
            return value is not None and bool(np.isfinite(value))
        except _UI_GUARD_ERRORS:
            return False

    def _build_validation_summary_section(self, entry) -> None:
        """Render the per-joint 'Current Test Results' summary, or an empty state."""
        ui.Label(
            "Current Test Results",
            style={"color": LABEL_COLOR, "font_size": FONT_SIZE},
        )
        rows = self._compute_joint_validation_summary(entry)
        if not rows:
            with ui.HStack(height=0):
                ui.Spacer(width=8)
                ui.Label(
                    "No test results yet. Run a test in the Test Gains tab.",
                    word_wrap=True,
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
                ui.Spacer(width=8)
            return
        for label, value in rows:
            with ui.HStack(height=20):
                ui.Spacer(width=8)
                ui.Label(
                    label,
                    width=150,
                    name="test_info_label",
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
                ui.Label(value, width=0, name="test_info_value", style={"color": LABEL_COLOR, "font_size": FONT_SIZE})
                ui.Spacer()
        # Compact command-vs-observed position chart preview.
        self._build_validation_chart_preview(entry)

    def _build_validation_chart_preview(self, entry) -> None:
        """Draw a compact command-vs-observed position preview for the selected joint.

        A lightweight mini-chart (two line series sharing a y-range); the full
        interactive charts remain in the Test Gains tab.
        """
        try:
            dof_index = getattr(entry, "dof_index", None)
            if dof_index is None or not self._gains_tuner or not self._gains_tuner.is_data_ready():
                return
            cmd_pos, _cv, obs_pos, _ov, _t = self._gains_tuner.get_joint_states_from_gains_test(dof_index)
            if cmd_pos is None or obs_pos is None:
                return
            scale = 1.0
            unit = "m"
            try:
                if self._gains_tuner.get_articulation().dof_types[dof_index] == DofType.Rotation:
                    scale = 180.0 / np.pi
                    unit = "\u00b0"
            except _UI_GUARD_ERRORS:
                pass
            cmd_v = [float(v) * scale for v in np.asarray(cmd_pos, dtype=float)]
            obs_v = [float(v) * scale for v in np.asarray(obs_pos, dtype=float)]
            if len(cmd_v) < 2 or len(obs_v) < 2:
                return
            y_lo = min(min(cmd_v), min(obs_v))
            y_hi = max(max(cmd_v), max(obs_v))
            pad = (y_hi - y_lo) * 0.08 if (y_hi - y_lo) > 1e-9 else 1.0
            y_lo, y_hi = y_lo - pad, y_hi + pad
        except _UI_GUARD_ERRORS:
            return

        _PREVIEW_H = 96
        # Legend row.
        with ui.HStack(height=16, spacing=8):
            ui.Spacer(width=8)
            for _color, _text in ((CHART_CMD_COLOR, "Command"), (CHART_OBS_COLOR, "Observed")):
                with ui.VStack(width=8):
                    ui.Spacer()
                    ui.Rectangle(width=8, height=8, style={"background_color": _color, "border_radius": 4})
                    ui.Spacer()
                ui.Label(_text, width=0, style={"color": _color, "font_size": FONT_SIZE})
            ui.Spacer()
        with ui.HStack(height=_PREVIEW_H):
            ui.Spacer(width=8)
            with ui.ZStack(
                height=_PREVIEW_H,
                style={
                    "Rectangle::preview_vp": {
                        "background_color": CHART_VIEWPORT_BG,
                        "border_color": CHART_STROKE,
                        "border_width": 1,
                        "border_radius": 4,
                    }
                },
            ):
                ui.Rectangle(name="preview_vp")
                # Light horizontal gridlines.
                with ui.VStack():
                    for i in range(4):
                        ui.Rectangle(height=1, style={"background_color": CHART_GRID_COLOR})
                        if i < 3:
                            ui.Spacer(height=ui.Fraction(1))
                ui.Plot(
                    ui.Type.LINE,
                    y_lo,
                    y_hi,
                    *cmd_v,
                    # No `border_width`: on `ui.Plot` it outlines the widget rather
                    # than thickening the trace.
                    style={"color": CHART_CMD_COLOR, "background_color": 0x0},
                )
                ui.Plot(
                    ui.Type.LINE,
                    y_lo,
                    y_hi,
                    *obs_v,
                    style={"color": CHART_OBS_COLOR, "background_color": 0x0},
                )
            ui.Spacer(width=8)
        # Y-range caption for context.
        with ui.HStack(height=14):
            ui.Spacer(width=8)
            ui.Label(
                f"y: {y_lo:.1f} to {y_hi:.1f} {unit}".strip(),
                style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
            )
            ui.Spacer()

    def _build_advanced_params_section(self, entry, resolved) -> None:
        """Build the Advanced Actuator Parameters accordion for the selected joint.

        Available for both backends: PhysicsDriveAPI joints expose armature and max
        drive limits (read/written on USD); Newton actuators expose control-range /
        force-limit / saturation / anti-windup as read-only info pending the actuator
        introspection API.
        """
        self._adv_params_detail_frame = CollapsableFrame(
            GT_ADVANCED_PARAMS_TITLE,
            collapsed=True,
            enabled=True,
            show_copy_button=False,
        )
        with self._adv_params_detail_frame:
            with ui.VStack(style=get_style(), spacing=4, height=0):
                ui.Spacer(height=4)
                if resolved.source == gain_tuner.GainSource.ACTUATOR:
                    ui.Label(
                        "Control range, force limit, saturation, anti-windup",
                        style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                    )
                    ui.Spacer(height=2)
                    # Read-only until the Newton actuator introspection API lands.
                    for label in ["Max Effort", "Saturation Effort", "Velocity Limit", "Integral Max"]:
                        self._adv_readonly_field(label)
                else:
                    joint = entry.joint
                    supported = gain_tuner.backend_supported(self._active_backend())
                    if not supported:
                        # Fail closed and say so.  Guessing ``physxJoint:*`` for an
                        # engine with no known joint schema would author a value
                        # nothing reads, which looks identical to a working edit.
                        self._inline_warning_row(GT_JOINT_PARAM_BACKEND_UNSUPPORTED)
                    self._adv_joint_param_field(joint, "armature", entry=entry)
                    self._adv_drive_maxforce_field("Max Drive Force", joint, entry.drive_axis)
                    self._adv_joint_param_field(joint, "max_joint_velocity", entry=entry)
                    self._adv_joint_param_field(joint, "joint_friction", entry=entry)
                    if supported:
                        self._inline_info_row(GT_JOINT_PARAM_PER_BACKEND_NOTE)
                        replay_note = self._replay_to_apply_note()
                        if replay_note:
                            self._inline_warning_row(replay_note)
                        self._build_joint_param_copy_row(entry)
                ui.Spacer(height=4)

    @staticmethod
    def replay_to_apply_note(backend: str, playing: bool) -> str:
        """Return the "replay to apply" line, or ``""`` when it does not apply.

        Separate from the caller so the condition can be tested without a panel.
        The note is only shown while the timeline is playing: stopped, the next
        play parses the stage anyway, so an edit made then is not delayed at all.

        Args:
            backend: The active backend label.
            playing: Whether the timeline is currently playing.

        Returns:
            :data:`~.global_variables.GT_JOINT_PARAM_REPLAY_TO_APPLY` when a write
            made now would not reach the running engine, otherwise ``""``.
        """
        if not playing or gain_tuner.backend_reads_usd_while_playing(backend):
            return ""
        return GT_JOINT_PARAM_REPLAY_TO_APPLY.format(backend=gain_tuner.backend_display_label(backend))

    def _replay_to_apply_note(self) -> str:
        """Return the "replay to apply" line for the live backend and timeline."""
        try:
            playing = bool(self._timeline.is_playing())
        except _UI_GUARD_ERRORS:
            return ""
        return UIBuilder.replay_to_apply_note(self._active_backend(), playing)

    def _adv_joint_param_field(self, joint, param_key: str, entry=None) -> None:
        """Advanced field for a param the two backends author on separate schemas.

        Shows the value the ACTIVE backend resolves through its own chain, and an
        edit writes only that backend's schema -- the other backend's value is left
        alone, because the PhysX and Newton formulations need different tuning.

        An unauthored parameter shows what the engine simulates instead -- measured
        from the running engine where it can be, and the documented config default
        otherwise -- muted and with a line saying so, because zero is a value
        neither engine uses and the field is live enough to author it by accident.
        A parameter that enforces no limit renders as the word rather than a number.

        Below the field, three kinds of line can appear.  What the other backend
        holds, what the active backend never reads, an undeterminable resolver
        chain, and what an unauthored parameter is worth are *informational*:
        statements of fact about a legitimate authoring choice, in
        :data:`~.style.INFO_COLOR`.  The velocity limit the engine actually enforces
        differing from the authored one (when ``entry`` is given and a simulation is
        running) stays a warning, since that one really does mean the panel is
        showing a number the sweep will not use.

        A parameter with nothing stateable at all -- no schema on the joint, an
        undetermined resolver chain, an engine whose schema is unknown -- renders as
        a blank placeholder rather than a field.  There is no number, and a
        ``FloatDrag`` has no way to show that: seeded with zero and disabled, it
        read ``0.00``, which is the false zero this whole value object exists to
        remove, while the table blanked the identical state.
        """
        spec = gain_tuner.joint_param_spec(param_key)
        if spec is None:
            return
        backend = self._active_backend()
        solver = self._active_solver()
        resolution = self._joint_param_resolution(joint, spec, backend, solver)
        engine_value = self._engine_param_value(entry, spec)
        display = self._joint_param_display(joint, spec, resolution, engine_value)
        attr = self._joint_param_attr(joint, spec, resolution)
        read_only = attr is None or not display.editable
        model = None
        if display.blank:
            self._adv_readonly_field(spec.label, tooltip=display.note)
        else:
            registered = _DetailAdvField(
                read_value=self._joint_param_reader(joint, spec, backend, solver),
                read_rendering=self._joint_param_rendering_reader(joint, spec, entry),
            )
            model = self._gain_field_row(
                spec.label,
                display.field_value,
                attr,
                read_only=read_only,
                # The backend is resolved when the edit happens, not captured here: this
                # field can outlive an engine switch, and a closure over the build-time
                # backend would quietly author the schema the engine no longer reads.
                write_fn=(
                    None
                    if read_only
                    else (lambda v, m: self._author_joint_param_live(joint, spec, v, m, display.field_value))
                ),
                tooltip=display.note,
                field_format=display.field_format,
                muted=not display.authored,
                on_built=lambda widget: setattr(registered, "field", widget),
            )
        # Built whether or not they have anything to say, so an edit that answers
        # the question a sentence was asking can take the sentence away -- and one
        # that introduces a divergence can put a sentence there.  A hidden row
        # takes up no height at all, so this changes nothing on screen.
        explanations = self._joint_param_explanations(joint, spec, resolution, display, entry)
        explain_rows = [self._inline_info_row(explanations[0]), self._inline_info_row(explanations[1])]
        if len(explanations) > 2:
            explain_rows.append(self._inline_warning_row(explanations[2]))
        if read_only or model is None:
            return
        registered.model = model
        registered.explain_rows = tuple(explain_rows)
        self._detail_adv_models.append(registered)

    def _joint_param_explanations(self, joint, spec, resolution, display, entry) -> tuple[str, ...]:
        """Return the sentences that belong under one advanced field.

        Ordered and fixed-length per parameter, because the rows they fill are
        built once and refreshed by position: what the field is worth, what the
        resolver chain does with it, and -- for the velocity limit alone -- the
        limit the running engine enforces instead.  Any of the three can be ``""``,
        which is a row with nothing to say rather than a missing row.

        Returns:
            Two sentences, or three for Max Joint Velocity with an ``entry`` to
            measure the engine through.
        """
        # The display is passed on so the two informational lines under one field
        # cannot both open with "nothing is authored" -- the first states it, so
        # ``joint_param_info_text`` drops its own version.
        info_text = joint_param_info_text(resolution, display) if resolution is not None else ""
        texts = (display.note, info_text)
        if spec.key == "max_joint_velocity" and entry is not None:
            return (*texts, self._max_velocity_engine_warning(entry, self._resolved_usd_velocity(display)))
        return texts

    def _joint_param_rendering_reader(self, joint, spec, entry) -> Callable[[], _AdvFieldRendering]:
        """Return a reader that re-derives everything one advanced field renders.

        Resolves the backend and solver at read time rather than capturing them,
        for the same reason the write path does: the field outlives an engine
        switch, and the marking belongs to whichever engine is running when it is
        drawn, not to the one that was running when it was built.

        Returns:
            A callable giving a fresh :class:`_AdvFieldRendering` -- the field's
            display and the sentences under it, from one resolve, so the number and
            the words cannot disagree.
        """

        def read() -> _AdvFieldRendering:
            backend = self._active_backend()
            solver = self._active_solver()
            resolution = self._joint_param_resolution(joint, spec, backend, solver)
            display = self._joint_param_display(joint, spec, resolution, self._engine_param_value(entry, spec))
            return _AdvFieldRendering(
                display=display,
                explanations=self._joint_param_explanations(joint, spec, resolution, display, entry),
            )

        return read

    @staticmethod
    def _apply_adv_field_display(field, display) -> None:
        """Re-render one advanced field from a freshly derived display.

        ``format``, ``style`` and the tooltip are all read/write on
        ``ui.FloatDrag``, and ``omni.ui`` re-emits a widget from its current
        properties on every frame, so assigning them repaints the field without
        rebuilding it.  Rebuilding is not an option here: the caller is the
        value-changed handler, which fires on every frame of a drag, and destroying
        the widget mid-drag would take the drag with it.

        Args:
            field: The built drag widget.
            display: The freshly derived
                :class:`~.gain_display.AdvancedParamDisplay`.
        """
        try:
            field.format = display.field_format
            field.style = {} if display.authored else {"color": MUTED_LABEL_COLOR}
            set_wrapped_tooltip(field, display.note)
        except _UI_GUARD_ERRORS:
            pass

    def _refresh_detail_adv_displays(self) -> None:
        """Re-derive every registered advanced field's rendering from the stage.

        The number is refreshed separately, through
        :attr:`_DetailAdvField.read_value`; this is the other half, and it covers
        both the field and the sentences under it.  An unauthored parameter's field
        is built with ``"(default)"`` in its ``format``, a muted colour, and a line
        beneath saying the engine is deciding the value; a successful write turns it
        into an authored value.  Without this, the field the user just dragged Max
        Joint Velocity into keeps reading ``unlimited (default)`` over a line
        reading "Nothing is authored here, so Newton leaves this unlimited" --
        asserting the joint is unclamped over a stage that now clamps it, in the
        number and then again in words.

        Every registered field is refreshed rather than only the edited one: they
        are three per joint, each one a resolve of one parameter, and the edited
        field is not the only one an edit can change -- authoring ``newton:armature``
        moves what the fallback line under the *other* fields is describing.
        """
        for entry in self._detail_adv_models:
            if entry.field is None or entry.read_rendering is None:
                continue
            try:
                rendering = entry.read_rendering()
            except _UI_GUARD_ERRORS:
                continue
            if rendering is None:
                continue
            self._apply_adv_field_display(entry.field, rendering.display)
            self._apply_adv_field_explanations(entry, rendering.explanations)

    @staticmethod
    def _apply_adv_field_explanations(entry, explanations: tuple[str, ...]) -> None:
        """Write the freshly derived sentences into one field's inline rows.

        Positional, against rows built in the same order, so a sentence cannot
        land under the wrong field; a row with no sentence left hides itself, which
        is the case the whole refresh exists for.
        """
        for row, text in zip(entry.explain_rows, explanations):
            try:
                row.set_text(text)
            except _UI_GUARD_ERRORS:
                continue

    @staticmethod
    def _resolved_usd_velocity(display) -> float | None:
        """Return the velocity limit the USD side resolves to, for engine comparison.

        ``math.inf`` for a limit that is unlimited, whether authored as ``inf`` or
        left unauthored -- the two mean the same thing to the engine, and passing
        None for the unauthored case is what let a wrong number go unreported.
        None only when the chain resolved nothing at all, which really is
        unknowable and stays quiet.
        """
        if display.unlimited:
            return math.inf
        return display.value

    def _engine_param_value(self, entry, spec) -> float | None:
        """Return what the running engine reports for a per-backend joint param.

        Measured truth, preferred over the documented default for an unauthored
        parameter.  Only queried while the timeline is playing: the accessors fall
        back to reading USD when there is no physics view, and USD is the very
        thing that has nothing authored, so a pre-play answer would restate the
        absence as a measurement.

        Returns:
            The engine's value, or None when there is no engine to ask -- no
            articulation, a stopped timeline, an entry with no DOF index, or a
            parameter the backend does not expose (joint friction, which has no
            implemented Newton tensor accessor).
        """
        dof_index = getattr(entry, "dof_index", None)
        if dof_index is None or self._gains_tuner is None:
            return None
        try:
            if not self._timeline.is_playing():
                return None
            if spec.key == "armature":
                return self._gains_tuner.get_dof_engine_armature(dof_index)
            if spec.key == "max_joint_velocity":
                return self._gains_tuner.get_dof_effective_max_velocity(dof_index)
        except _UI_GUARD_ERRORS:
            return None
        return None

    @staticmethod
    def _inline_warning_row(text: str) -> _InlineTextRow:
        """Render an indented, wrapping warning line under a detail field.

        Same presentation as the inactive-gains warning in the Controller Gains
        section, so every "what you see is not what the simulation uses" message
        in the detail panel reads as one idea.

        Returns:
            The row, for a caller that has to keep it in step with an edit (see
            :class:`_InlineTextRow`).  An empty ``text`` builds the row hidden
            rather than not at all, so it is there to be filled in later.
        """
        return UIBuilder._inline_text_row(text, WARNING_COLOR)

    @staticmethod
    def _inline_info_row(text: str) -> _InlineTextRow:
        """Render an indented, wrapping informational line under a detail field.

        Same geometry as :meth:`_inline_warning_row` but in
        :data:`~.style.INFO_COLOR`, because per-backend joint values are a
        deliberate authoring choice and nothing here asks the user to act.

        Returns:
            The row, for a caller that has to keep it in step with an edit.
        """
        return UIBuilder._inline_text_row(text, INFO_COLOR)

    @staticmethod
    def _inline_text_row(text: str, color: int) -> _InlineTextRow:
        """Build the one row geometry every inline explanation line shares."""
        with ui.HStack(height=0) as row:
            ui.Spacer(width=8)
            label = ui.Label(
                text,
                word_wrap=True,
                style={"color": color, "font_size": FONT_SIZE},
            )
            ui.Spacer(width=8)
        row.visible = bool(text)
        return _InlineTextRow(row=row, label=label)

    def _active_backend(self) -> str:
        """Return the active backend label the resolution and wording are phrased from.

        Read live rather than from :attr:`_backend_ctx`, because the engine can
        change without any event this panel can subscribe to, and the cached
        context is only refilled on stage load and play.  Authoring against a
        stale backend is the failure this redesign exists to remove: an edit made
        while the cache still said PhysX would land on ``physxJoint:*``, which
        Newton never reads for friction, so the user would tune a value that does
        nothing and is told nothing.

        Returns:
            The live backend label, or ``""`` when the engine could not be
            determined.  Empty is not PhysX: callers that author a joint schema
            check :func:`~isaacsim.robot_setup.gain_tuner.backend_supported` and
            disable the edit rather than guessing.
        """
        try:
            return BackendContext.active_backend_label()
        except _UI_GUARD_ERRORS:
            return getattr(self._backend_ctx, "backend", "")

    def _sync_backend_if_engine_changed(self) -> bool:
        """Re-resolve everything backend-derived when the physics engine changed.

        Detection is a string comparison against the simulation manager's cached
        engine name, cheap enough for the render tick; the real work runs only on
        an actual change.  Polling is the only option available: there is no
        engine-switch event to subscribe to (see :meth:`_on_engine_changed`).

        An engine that could not be *read* is not an engine that *changed*, and the
        two have to be told apart because :meth:`_on_engine_changed` discards the
        user's recorded test results.  The manager query fails transiently and
        reports ``""``, which against a cached ``"PhysX"`` looked like a switch --
        so a momentary failure destroyed the results, and the recovery destroyed
        them again.  Either label being empty therefore takes the new label and
        stops there.  The cost is that a real switch masked by a failed read
        (PhysX -> unreadable -> Newton) is not handled as one; the write path does
        not depend on this, because every write re-reads the engine through
        :meth:`_active_backend` and refuses an unsupported one.

        Returns:
            True when an engine change was detected and handled.
        """
        live = self._active_backend()
        cached = getattr(self._backend_ctx, "backend", "") or ""
        if live == cached:
            return False
        if not live or not cached:
            # Adopt the label so the header and the fields stop describing an
            # engine that is no longer the answer, but leave the articulation and
            # the recorded results alone.  Non-destructive and self-limiting: the
            # next tick compares equal.
            self._refresh_backend_from_app()
            return False
        self._on_engine_changed(live)
        return True

    def _on_engine_changed(self, backend: str) -> None:
        """Drop every piece of state that belonged to the previous physics engine.

        Switching engines invalidates the physics tensor views behind the bound
        articulation without nulling our handle, so the articulation is re-acquired
        rather than reused; a ``get_dof_*`` call through the old view would be
        reaching into an invalidated view.  Recorded test results are discarded
        instead of being carried over, because a chart that mixes samples from two
        solvers in one trace is worse than one the user has to re-record: the two
        solvers are precisely what the user is comparing.
        """
        carb.log_warn(
            f"[GainTuner] Physics engine changed to {backend}; re-reading the backend, "
            "re-acquiring the articulation, and discarding results recorded under the previous engine."
        )
        self._refresh_backend_from_app()
        self._reacquire_articulation_for_engine_switch()
        # Save-target defaults are backend-derived, so a stale context can route a
        # save to the wrong overlay layer.
        try:
            self._resolve_save_targets()
        except _UI_GUARD_ERRORS:
            pass
        if self._charts_frame is not None:
            try:
                self._charts_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass

    def _reacquire_articulation_for_engine_switch(self) -> None:
        """Release the invalidated articulation and re-bind the same robot."""
        tuner = self._gains_tuner
        if tuner is None:
            return
        path = getattr(tuner, "_robot_prim_path", None)
        try:
            tuner.invalidate_physics_views()
        except _UI_GUARD_ERRORS:
            pass
        self._make_plot_on_next_frame = False
        if not path:
            return
        # Re-bind against the new engine.  Under Newton the tensor articulation is
        # built lazily, so this may report no DOFs for a frame or two; the existing
        # deferred-repopulate path picks it up once the view has cooked.
        try:
            self._on_articulation_selection(path, force=True)
        except _UI_GUARD_ERRORS:
            self._awaiting_articulation_dofs = True

    def _active_solver(self) -> str:
        """Return the running Newton solver token, "" when unknown or under PhysX.

        The Newton resolver chain differs by solver (the MuJoCo solver inserts the
        ``mjc:*`` resolver between the Newton and PhysX ones), so this selects which
        chain the advanced fields resolve through.
        """
        return getattr(self._backend_ctx, "solver_type", "")

    @staticmethod
    def _joint_param_resolution(joint, spec, backend: str, solver: str):
        """Return the joint param's :class:`JointParamResolution`, or None on error."""
        try:
            return gain_tuner.resolve_joint_param(joint, spec, backend, solver)
        except _UI_GUARD_ERRORS:
            return None

    @staticmethod
    def _joint_param_attr(joint, spec, resolution) -> object | None:
        """Return the attribute whose presence makes the detail field editable.

        Whichever of the two schema halves exists; it gates editability only -- the
        write itself goes through
        :func:`~isaacsim.robot_setup.gain_tuner.joint_schema_attrs.author_joint_param`
        so it lands on the active backend's schema, creating that half if needed.
        """
        if resolution is None:
            return None
        try:
            newton_attr, physx_attr = gain_tuner.joint_param_attrs(joint, spec)
        except _UI_GUARD_ERRORS:
            return None
        return newton_attr if newton_attr is not None else physx_attr

    @staticmethod
    def _joint_param_display(joint, spec, resolution, engine_value: float | None = None):
        """Return how the detail field should render, degrading to a blank field."""
        if resolution is None:
            return AdvancedParamDisplay(None, False, False, False, "")
        try:
            return advanced_param_display(resolution, joint, spec, engine_value)
        except _UI_GUARD_ERRORS:
            return AdvancedParamDisplay(None, False, False, False, "")

    def _max_velocity_engine_warning(self, entry, usd_value) -> str:
        """Return the engine-vs-USD velocity-limit warning, or "" when they agree.

        The velocity sweeps scale their commands by
        :meth:`~isaacsim.robot_setup.gain_tuner.GainTuner.get_dof_effective_max_velocity`,
        so a joint whose authored limit differs is excited at a speed this panel
        does not show.  Degrades to no warning when there is no articulation or the
        timeline is not playing, which is also the state in which the sweep cannot
        run: the accessor falls back to reading USD without a physics view, and
        comparing USD against USD can only ever agree.
        """
        dof_index = getattr(entry, "dof_index", None)
        if dof_index is None or self._gains_tuner is None:
            return ""
        try:
            if not self._timeline.is_playing():
                return ""
            engine_value = self._gains_tuner.get_dof_effective_max_velocity(dof_index)
        except _UI_GUARD_ERRORS:
            return ""
        if gain_tuner.max_velocity_agrees(usd_value, engine_value):
            return ""
        return max_velocity_engine_text(usd_value, engine_value, is_angular_dof(entry.joint, entry.drive_axis))

    def _build_joint_param_copy_row(self, entry) -> None:
        """Build the opt-in buttons that copy these values to the other backend.

        Not a remedy for anything: the two backends hold their own values on
        purpose.  The buttons exist because retyping a value cannot copy it -- the
        field's model only fires on an actual change -- so seeding the other backend
        from what is shown here would otherwise mean editing the value away and
        back.  Each writes the displayed values as-is, and the tooltips say that
        doing so changes what the other backend simulates.

        Both labels name the backend being written.  It is resolved here, at build
        time, from the live inactive backend, and the panel rebuilds on an engine
        switch, so the label never names the backend a click would not write.
        """
        target = gain_tuner.other_backend(self._active_backend())
        if target is None:
            # No opposite backend to copy to; the section already says why.
            return
        target_name = gain_tuner.backend_display_label(target)
        with ui.HStack(height=24, spacing=6):
            ui.Spacer(width=8)
            copy_one = ui.Button(
                GT_COPY_JOINT_PARAMS_BUTTON.format(backend=target_name),
                width=140,
                height=20,
                clicked_fn=lambda e=entry: self._on_copy_joint_params([e]),
            )
            copy_all = ui.Button(
                GT_COPY_ALL_JOINT_PARAMS_BUTTON.format(backend=target_name),
                width=140,
                height=20,
                clicked_fn=self._on_copy_all_joint_params,
            )
            ui.Spacer()
        # These two are the longest tooltips in the panel, so they are both wrapped
        # and anchored below their own button rather than at the mouse: a tooltip
        # that hides the button it is explaining is worse than none.
        for button, text in (
            (copy_one, GT_COPY_JOINT_PARAMS_TOOLTIP),
            (copy_all, GT_COPY_ALL_JOINT_PARAMS_TOOLTIP),
        ):
            set_wrapped_tooltip(button, text.format(backend=target_name), offset_y=_COPY_TOOLTIP_OFFSET_Y)

    def _on_copy_joint_params(self, entries: list) -> None:
        """Copy the advanced params of ``entries`` to the other backend's schema.

        Writes only where the other backend does not already hold the value, as a
        single undoable edit, and reports what happened as a toast.  A selection
        the other backend already matches writes nothing and says so.
        """
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        # Copy reads the active backend and writes the other one, so a stale backend
        # would send the value in exactly the wrong direction.
        self._sync_backend_if_engine_changed()
        backend = self._active_backend()
        solver = self._active_solver()
        targets = joint_param_copy_targets(entries, backend, solver)
        result = copy_joint_params_to_other_backend(stage, targets, backend, solver)
        nm.post_notification(
            copy_summary_text(result),
            duration=5,
            status=nm.NotificationStatus.WARNING if result.unwritable_attrs else nm.NotificationStatus.INFO,
        )
        if result.wrote_nothing:
            return
        # The active backend keeps resolving what the panel already showed, so the
        # only visible change is the per-backend notes: rebuild both views so those
        # refresh, preserving the table selection the way a source switch does.
        if self._gain_table_view is not None:
            try:
                self._pending_table_selection = set(self._gain_table_view.selected_indices())
            except _UI_GUARD_ERRORS:
                self._pending_table_selection = None
        for frame in (self._gain_table_frame, self._joint_detail_frame):
            if frame is None:
                continue
            try:
                frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass

    def _on_copy_all_joint_params(self) -> None:
        """Copy every tunable joint's advanced params to the other backend."""
        self._on_copy_joint_params(self._get_tunable_joint_entries())

    @staticmethod
    def _joint_param_reader(joint, spec, backend: str, solver: str) -> Callable[[], float | None]:
        """Return a reader that re-resolves a param through the active backend's chain.

        Refreshing a field off its raw attributes instead would show whichever half
        happened to be read last rather than the one the backend consumes -- and on
        a joint whose inactive half is applied but unauthored, snap the field to
        that schema's zero while the active backend still resolves its own value.
        """
        return lambda: advanced_cell_value(gain_tuner.resolve_joint_param(joint, spec, backend, solver), joint, spec)

    def _author_joint_param_live(self, joint, spec, value: float, model=None, previous: float | None = None) -> None:
        """Author a joint param on whichever backend is active at this moment.

        Every write goes through here rather than through a backend captured when
        the widget was built, so an edit made after an engine switch lands on the
        schema the running engine actually reads.  The switch is picked up first,
        so the rest of the panel is not left describing the previous engine.

        A successful write re-renders the advanced fields, because it changes what
        they are: the value the user just authored was, until this call, an
        unauthored engine default, rendered muted and suffixed ``"(default)"``.
        Leaving that in place left the field claiming provenance for a number the
        user had typed -- and for Max Joint Velocity, claiming the joint was
        unclamped over a stage that now clamps it.

        Args:
            joint: The joint prim being edited.
            spec: The parameter being authored.
            value: The value typed or dragged into the field.
            model: The field's model, put back to ``previous`` when the write fails.
            previous: The value the field held before the edit.
        """
        try:
            self._sync_backend_if_engine_changed()
        except _UI_GUARD_ERRORS:
            pass
        if self._author_joint_param(joint, spec, value, self._active_backend()):
            self._refresh_detail_adv_displays()
            return
        self._revert_failed_param_edit(model, previous)

    def _revert_failed_param_edit(self, model, previous: float | None) -> None:
        """Put a field back and say so after a write that could not be applied.

        A field left showing the typed number after a failed write is the silent
        no-op this redesign exists to remove: the panel would be the only place
        that value exists.  Copy already toasts its unwritable attributes; a direct
        edit has to do the same, or the two paths disagree about whether a failure
        is worth telling the user about.
        """
        nm.post_notification(GT_JOINT_PARAM_WRITE_FAILED, duration=5, status=nm.NotificationStatus.WARNING)
        if model is None or previous is None:
            return
        # The revert is itself a value change, so suppress the write-back it would
        # otherwise trigger -- which would fail again, and toast again.
        was_suspended = self._suspend_detail_writes
        self._suspend_detail_writes = True
        try:
            model.set_value(float(previous))
        except _UI_GUARD_ERRORS:
            pass
        finally:
            self._suspend_detail_writes = was_suspended

    @staticmethod
    def _author_joint_param(joint, spec, value: float, backend: str) -> bool:
        """Author a joint param on the active backend's schema only.

        The other backend's value is left untouched: the two solvers are tuned
        independently.  A failed write is either an unsupported engine, whose joint
        schema is unknown, or a missing ``omni.usd.schema.newton``; both would
        otherwise look like a silent no-op.

        Returns:
            True when the value was authored.
        """
        if gain_tuner.author_joint_param(joint, spec, value, backend) is not None:
            return True
        schema = gain_tuner.backend_write_schema(backend)
        attr_name = "no known schema" if schema is None else spec.attr_for_schema(schema)
        try:
            joint_path = str(joint.GetPath())
        except _UI_GUARD_ERRORS:
            joint_path = "<unknown joint>"
        carb.log_warn(
            f"[GainTuner] {spec.label} on {joint_path} could not be written to {attr_name}; "
            "the value shown is not what the stage holds. "
            "Check that the running engine is PhysX or Newton and that "
            "omni.usd.schema.newton is enabled."
        )
        return False

    def _adv_drive_maxforce_field(self, label: str, joint, drive_axis) -> None:
        """Advanced field bound to the joint drive's maxForce (max drive limit)."""
        attr = None
        try:
            axis = drive_axis
            if not axis:
                if joint.IsA(UsdPhysics.RevoluteJoint):
                    axis = "angular"
                elif joint.IsA(UsdPhysics.PrismaticJoint):
                    axis = "linear"
            if axis:
                api = UsdPhysics.DriveAPI(joint, axis)
                if api:
                    attr = api.GetMaxForceAttr()
        except _UI_GUARD_ERRORS:
            attr = None
        valid = bool(attr and attr.IsValid())
        value = 0.0
        if valid:
            v = attr.Get()
            value = float(v) if v is not None else 0.0
        model = self._gain_field_row(label, value, attr if valid else None, read_only=not valid)
        if valid:
            # Register so a table edit to Max Drive Force refreshes it in place.  No
            # ``read_rendering``: a single attribute is either authored or absent, so
            # this field's rendering has nothing to go stale.
            self._detail_adv_models.append(_DetailAdvField(read_value=attr.Get, model=model))

    def _adv_readonly_field(self, label: str, tooltip: str = "") -> None:
        """Blank, disabled advanced field placeholder with an optional explanation.

        Used for the Newton actuator parameters that have no introspection API yet,
        and for a per-backend joint param with nothing stateable -- an undetermined
        resolver chain, or an engine whose joint schema is unknown.  Renders as an
        empty field rather than as a disabled zero, which is a number no engine is
        using and reads exactly like an authored one.

        Args:
            label: The field label.
            tooltip: Optional sentence saying why the field is blank.
        """
        self._field_row(label, None, width=160, tooltip=tooltip)

    def _resolve_save_targets(self) -> None:
        """Resolve DriveAPI save-target layers + mjc/Newton mirror state for the robot.

        Populates ``self._backend_ctx`` with the candidate DriveAPI save-target
        layers (neutral ``physics.usda`` default), the selected identifier, and an
        informational message about the MuJoCo / Newton targets that will be
        mirrored / written on save.  Fully defensive: falls back to an unresolved
        state on any error so the row still renders.
        """
        try:
            stage = omni.usd.get_context().get_stage()
            entries = self._gains_tuner.get_joint_entries()
            root = self._gains_tuner.get_articulation_root()
            targets = gain_tuner.resolve_gain_write_targets(
                entries,
                stage,
                root,
                mirror_enabled=self._mirror_writeback_enabled,
                mirror_drive_to_mjc=self._mirror_drive_to_mjc,
                mjc_target_identifier=self._selected_mjc_target_identifier,
                newton_target_identifier=self._selected_newton_target_identifier,
            )
        except _UI_GUARD_ERRORS:
            targets = None
        if targets is None:
            self._backend_ctx.save_target = ""
            self._backend_ctx.save_target_identifier = ""
            self._backend_ctx.save_target_candidates = []
            self._backend_ctx.mirror_info = ""
            self._backend_ctx.has_mjc_sources = False
            self._backend_ctx.has_newton_sources = False
        else:
            self._backend_ctx.apply_save_targets(targets)
        # Keep the user's current selections if they are still valid candidates.
        candidate_ids = [identifier for identifier, _ in self._backend_ctx.save_target_candidates]
        if self._selected_save_target_identifier not in candidate_ids:
            self._selected_save_target_identifier = self._backend_ctx.save_target_identifier or None
        # Mirror-target selections fall back to the resolved defaults when unset or
        # no longer valid (the picker offers the same Physics/*.usda candidates).
        if self._selected_mjc_target_identifier not in candidate_ids:
            self._selected_mjc_target_identifier = self._backend_ctx.mjc_target_identifier or None
        newton_ids = candidate_ids + (
            [self._backend_ctx.newton_target_identifier] if self._backend_ctx.newton_target_identifier else []
        )
        if self._selected_newton_target_identifier not in newton_ids:
            self._selected_newton_target_identifier = self._backend_ctx.newton_target_identifier or None

    def _on_save_target_changed(self, model) -> None:
        """Update the selected DriveAPI save-target layer when the combo changes."""
        try:
            idx = model.get_item_value_model().as_int
            candidates = self._backend_ctx.save_target_candidates
            if 0 <= idx < len(candidates):
                self._selected_save_target_identifier = candidates[idx][0]
        except _UI_GUARD_ERRORS:
            pass

    def _on_mirror_toggle_changed(self, model) -> None:
        """Enable/disable the opt-in DriveAPI->MuJoCo mirror and refresh the row."""
        try:
            self._mirror_drive_to_mjc = bool(model.get_value_as_bool())
        except _UI_GUARD_ERRORS:
            self._mirror_drive_to_mjc = False
        if self._save_target_frame is not None:
            self._save_target_frame.rebuild()

    def _on_mjc_target_changed(self, model) -> None:
        """Update the selected mjc mirror target layer when its combo changes."""
        try:
            idx = model.get_item_value_model().as_int
            candidates = self._backend_ctx.save_target_candidates
            if 0 <= idx < len(candidates):
                self._selected_mjc_target_identifier = candidates[idx][0]
        except _UI_GUARD_ERRORS:
            pass
        if self._save_target_frame is not None:
            self._save_target_frame.rebuild()

    def _on_newton_target_changed(self, model) -> None:
        """Update the selected Newton writeback target layer when its combo changes."""
        try:
            idx = model.get_item_value_model().as_int
            candidates = self._newton_target_candidates()
            if 0 <= idx < len(candidates):
                self._selected_newton_target_identifier = candidates[idx][0]
        except _UI_GUARD_ERRORS:
            pass
        if self._save_target_frame is not None:
            self._save_target_frame.rebuild()

    def _newton_target_candidates(self) -> list[tuple[str, str]]:
        """Return ``(identifier, display_name)`` options for the Newton target picker.

        Offers the actuator's own defining layer (the default) plus the
        Physics/*.usda candidates as alternatives, de-duplicated.
        """
        ctx = self._backend_ctx
        candidates: list[tuple[str, str]] = []
        seen: set[str] = set()
        default_id = ctx.newton_target_identifier
        if default_id:
            candidates.append((default_id, os.path.basename(default_id)))
            seen.add(default_id)
        for identifier, name in ctx.save_target_candidates:
            if identifier not in seen:
                candidates.append((identifier, name))
                seen.add(identifier)
        return candidates

    def _build_save_target_row(self, *args) -> None:
        """Builds the inline Save Target row shown below the split pane.

        The first line always shows the DriveAPI save-target (default
        ``physics.usda``) and the Save button.  When the robot has MuJoCo-native
        gains and the MuJoCo solver is not active, an opt-in "Mirror tuned DriveAPI
        gains to MuJoCo" checkbox (OFF by default) is shown; when it also has
        Newton actuators the Newton writeback target picker is shown.  An info line
        reflects the current state.
        """
        self._resolve_save_targets()
        ctx = self._backend_ctx
        candidates = ctx.save_target_candidates

        # The opt-in DriveAPI->MuJoCo mirror checkbox only applies when DriveAPI is
        # the active/tuned source: hide it while the MuJoCo solver is running (then
        # mjc:* is the active source, edited/saved directly).  The Newton actuator
        # writeback stays on by default and shows its target picker when present.
        show_mjc_toggle = ctx.has_mjc_sources and not ctx.is_mujoco_solver
        mjc_mirror_on = self._mirror_drive_to_mjc
        show_mjc_picker = show_mjc_toggle and mjc_mirror_on and len(candidates) > 1
        show_newton_picker = ctx.has_newton_sources
        # Height: base row + optional toggle + optional pickers + optional info.
        row_height = 30
        if show_mjc_toggle:
            row_height += 24
        if show_mjc_picker:
            row_height += 24
        if show_newton_picker:
            row_height += 24
        if ctx.has_mirror_info:
            row_height += 16

        with ui.ZStack(height=row_height):
            ui.Rectangle(style={"background_color": SAVE_ROW_BG})
            with ui.VStack():
                ui.Spacer()
                with ui.HStack(height=30):
                    ui.Spacer(width=8)
                    with ui.VStack(width=0):
                        ui.Spacer()
                        ui.Label(
                            GT_SAVE_TARGET_LABEL,
                            width=0,
                            style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                        )
                        ui.Spacer()
                    ui.Spacer(width=6)
                    with ui.VStack():
                        ui.Spacer()
                        if not ctx.save_target_resolved:
                            self._save_target_label_widget = ui.Label(
                                GT_WARN_SAVE_TARGET_UNRESOLVED,
                                name="save_target_warning",
                            )
                        elif len(candidates) > 1:
                            names = [name for _, name in candidates]
                            selected_idx = self._selected_save_target_combo_index(candidates)
                            self._save_target_combo = ui.ComboBox(selected_idx, *names, height=20)
                            self._save_target_combo.model.add_item_changed_fn(
                                lambda m, _: self._on_save_target_changed(m)
                            )
                            self._save_target_label_widget = self._save_target_combo
                        else:
                            self._save_target_label_widget = ui.Label(
                                ctx.save_target,
                                name="save_target_value",
                                elided_text=True,
                                tooltip=ctx.save_target,
                            )
                        ui.Spacer()
                    ui.Spacer(width=8)
                    with ui.VStack(width=0):
                        ui.Spacer()
                        ui.Button(
                            GT_SAVE_BUTTON_LABEL,
                            height=22,
                            width=60,
                            clicked_fn=self._on_save_gains_to_physics_layer,
                            style={"Button": {"background_color": TAB_ACTIVE_BG, "border_radius": 2}},
                        )
                        ui.Spacer()
                    ui.Spacer(width=8)
                # -- Opt-in DriveAPI->MuJoCo mirror checkbox (OFF by default) --
                if show_mjc_toggle:
                    with ui.HStack(height=22):
                        ui.Spacer(width=8)
                        with ui.VStack(width=0):
                            ui.Spacer()
                            self._mirror_toggle_checkbox = ui.CheckBox(width=0)
                            self._mirror_toggle_checkbox.model.set_value(mjc_mirror_on)
                            self._mirror_toggle_checkbox.model.add_value_changed_fn(self._on_mirror_toggle_changed)
                            ui.Spacer()
                        ui.Spacer(width=6)
                        ui.Label(
                            GT_MIRROR_TOGGLE_LABEL,
                            elided_text=True,
                            tooltip=GT_MIRROR_TOGGLE_TOOLTIP,
                            style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                        )
                        ui.Spacer(width=8)
                # -- MuJoCo mirror target picker --
                if show_mjc_picker:
                    self._mjc_target_combo = self._build_mirror_target_picker(
                        GT_MJC_TARGET_LABEL,
                        candidates,
                        self._selected_mjc_target_identifier or ctx.mjc_target_identifier,
                        self._on_mjc_target_changed,
                    )
                # -- Newton actuator writeback target picker --
                if show_newton_picker:
                    newton_candidates = self._newton_target_candidates()
                    if len(newton_candidates) > 1:
                        self._newton_target_combo = self._build_mirror_target_picker(
                            GT_NEWTON_TARGET_LABEL,
                            newton_candidates,
                            self._selected_newton_target_identifier or ctx.newton_target_identifier,
                            self._on_newton_target_changed,
                        )
                if ctx.has_mirror_info:
                    with ui.HStack(height=16):
                        ui.Spacer(width=8)
                        ui.Label(
                            ctx.mirror_info,
                            name="save_target_info",
                            elided_text=True,
                            tooltip=ctx.mirror_info,
                            style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                        )
                        ui.Spacer(width=8)
                ui.Spacer()

    def _build_mirror_target_picker(self, label, candidates, selected_identifier, on_changed) -> ui.ComboBox:
        """Build a labeled combo row for a mirror-writeback target layer.

        Args:
            label: Left-hand label text (e.g. ``"MuJoCo target:"``).
            candidates: ``(identifier, display_name)`` options.
            selected_identifier: Identifier to preselect.
            on_changed: Callback invoked with the combo model on change.

        Returns:
            The created ``ui.ComboBox`` widget.
        """
        with ui.HStack(height=22):
            ui.Spacer(width=20)
            with ui.VStack(width=0):
                ui.Spacer()
                ui.Label(
                    label,
                    width=0,
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
                ui.Spacer()
            ui.Spacer(width=6)
            with ui.VStack():
                ui.Spacer()
                selected_idx = 0
                for idx, (identifier, _name) in enumerate(candidates):
                    if identifier == selected_identifier:
                        selected_idx = idx
                        break
                combo = ui.ComboBox(selected_idx, *[name for _, name in candidates], height=20)
                combo.model.add_item_changed_fn(lambda m, _: on_changed(m))
                ui.Spacer()
            ui.Spacer(width=8)
        return combo

    def _selected_save_target_combo_index(self, candidates) -> int:
        """Return the combo index of the currently selected save target (default 0)."""
        selected = self._selected_save_target_identifier or self._backend_ctx.save_target_identifier
        for idx, (identifier, _name) in enumerate(candidates):
            if identifier == selected:
                return idx
        return 0

    def _build_advanced_params_frame(self) -> None:
        """Builds the Advanced Actuator Parameters section (placeholder fields)."""
        with self._advanced_params_frame:
            with ui.VStack(style=get_style(), spacing=4, height=0):
                ui.Spacer(height=4)
                for label in [
                    "Armature",
                    "Friction",
                    "Max Force",
                    "Max Velocity",
                    "Effort Limit",
                    "Velocity Limit",
                ]:
                    with ui.HStack(height=24):
                        ui.Spacer(width=8)
                        ui.Label(
                            label,
                            width=140,
                            height=24,
                            alignment=ui.Alignment.LEFT_CENTER,
                            style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                        )
                        ui.FloatField(
                            height=20,
                            name="adv_param_field",
                            enabled=False,
                        )
                        ui.Spacer(width=8)
                ui.Spacer(height=4)

    def _build_test_in_progress_panel(self) -> None:
        """Builds the full-panel overlay shown while a gains test is running (Figma node 2163:5972)."""
        # Progress fill: #2378CA → ABGR 0xFFCA7823
        _FILL_COLOR = 0xFFCA7823
        # Background track: #1F2124 → already TAB_INACTIVE_BG
        _TRACK_COLOR = TAB_INACTIVE_BG
        _BAR_H = 22
        _BAR_R = 11  # pill: radius = height / 2

        with ui.ZStack(height=0):
            ui.Rectangle(style={"background_color": HEADER_BG_COLOR})
            with ui.VStack(spacing=0, height=0):
                ui.Spacer(height=12)
                # ── Content block ──────────────────────────────────────────
                with ui.VStack(spacing=6, height=0):
                    with ui.HStack(height=20):
                        ui.Spacer(width=16)
                        ui.Label(
                            "Test in Progress",
                            style={"color": LABEL_COLOR, "font_size": 14},
                        )
                        ui.Spacer()
                    with ui.HStack(height=16):
                        ui.Spacer(width=16)
                        _progress_label = ui.Label(
                            "Sequence: 1/1   Time: 0.0s/0.0s",
                            style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                        )
                        self._test_progress_labels.append(_progress_label)
                        ui.Spacer()
                    # Progress bar with a single centered percentage label.
                    # We draw a custom fill (track + fill Rectangle) instead of
                    # ui.ProgressBar so there is no built-in percentage text that
                    # follows the fill edge — only the centered label remains.
                    with ui.HStack(height=_BAR_H):
                        ui.Spacer(width=16)
                        with ui.ZStack(height=_BAR_H):
                            ui.Rectangle(style={"background_color": _TRACK_COLOR, "border_radius": _BAR_R})
                            with ui.HStack():
                                _progress_fill = ui.Rectangle(
                                    width=ui.Fraction(0.0001),
                                    style={"background_color": _FILL_COLOR, "border_radius": _BAR_R},
                                )
                                self._test_progress_fills.append(_progress_fill)
                                _progress_gap = ui.Spacer(width=ui.Fraction(1))
                                self._test_progress_gaps.append(_progress_gap)
                            _progress_pct_label = ui.Label(
                                "0%",
                                alignment=ui.Alignment.CENTER,
                                style={"color": 0xFFD8D8D8, "font_size": FONT_SIZE},
                            )
                            self._test_progress_pct_labels.append(_progress_pct_label)
                        ui.Spacer(width=16)
                ui.Spacer(height=12)
                # ── Cancel Test button ──────────────────────────────────────
                with ui.HStack(height=22):
                    ui.Spacer(width=16)
                    ui.Button(
                        "Cancel Test",
                        height=22,
                        clicked_fn=self._on_cancel_gains_test,
                        style={"Button": {"background_color": TAB_ACTIVE_BG, "border_radius": 2}},
                    )
                    ui.Spacer(width=16)
                ui.Spacer(height=8)

    def _build_test_controls_frame(self) -> None:
        """Builds the Test Gains Settings section: duration, mode, table, and run button."""
        if not self._gains_tuner.initialized:
            if self._articulation_menu_model and self._articulation_menu_model.has_item():
                self._gains_tuner.initialize()
            else:
                with self._test_controls_frame:
                    ui.Label(
                        "Start Simulation to run Tests",
                        style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                    )
                return

        with self._test_controls_frame:
            with ui.VStack(style=get_style(), spacing=4, height=0):
                ui.Spacer(height=4)
                # Duration + Mode row
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    ui.Label(GT_TEST_MODE_LABEL, width=0, height=24, alignment=ui.Alignment.LEFT_CENTER)
                    ui.Spacer(width=8)
                    # Test mode selection.  Combo index -> GainsTestMode is mapped
                    # via `self._test_mode_combo_modes`; default index 0 = Snap to
                    # Limits.
                    self._test_mode_combo = ui.ComboBox(
                        0, "Snap to Limits", "Sinusoidal", "Step Function", "Stress", "dt Sweep", width=140, height=20
                    )
                    self._test_mode = GainsTestMode.SNAP_TO_LIMITS
                    # Sync backend/table mode defensively — these can raise before the
                    # gains tuner is initialized and must not abort the frame build
                    # (which would drop the Run Test button below).
                    try:
                        self._gains_tuner.test_mode = int(GainsTestMode.SNAP_TO_LIMITS)
                    except _UI_GUARD_ERRORS:
                        pass
                    try:
                        if self._test_inline_widget:
                            self._test_inline_widget.switch_mode(GainsTestMode.SNAP_TO_LIMITS)
                    except _UI_GUARD_ERRORS:
                        pass
                    self._test_mode_combo.model.add_item_changed_fn(
                        lambda m, _: self._switch_test_mode(m.get_item_value_model())
                    )
                    ui.Spacer(width=16)
                    # Test duration only applies to Sinusoidal / Step; hidden for
                    # Snap to Limits and Stress (which carry their own duration).
                    self._test_duration_frame = ui.Frame(width=0, visible=False)
                    with self._test_duration_frame:
                        with ui.HStack(width=0):
                            ui.Label(GT_TEST_DURATION_LABEL, width=0, height=24, alignment=ui.Alignment.LEFT_CENTER)
                            ui.Spacer(width=8)
                            self._test_duration_field = ui.FloatField(
                                value=5.0,
                                height=20,
                                width=80,
                                step=0.1,
                                min_value=0.0,
                                max_value=10.0,
                            )
                            self._test_duration_field.model.set_value(5)
                            self._gains_tuner.set_test_duration(5)
                            self._test_duration_field.model.add_value_changed_fn(
                                lambda m: self._on_test_duration_changed(m)
                            )
                    ui.Spacer(width=ui.Fraction(1))

                # Snap-to-limits settings (hold duration, tolerance, overrides).
                self._build_snap_settings()
                # Stress-test settings (sub-mode, duration, seed, thresholds, overrides).
                self._build_stress_test_settings()
                # dt physics sweep settings (dt range, level count, probe + threshold params).
                self._build_discretization_settings()

                ui.Spacer(height=4)
                # Per-joint test configuration table (which joints to test and how).
                ui.Label(
                    "Joints to test",
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
                if self._gains_tuner.get_articulation():
                    with ui.HStack(height=220):
                        self._test_inline_widget = TestJointWidget(
                            self._gains_tuner,
                            no_name_column=False,
                            hidden_columns={col for col, vis in self._test_column_visible.items() if not vis},
                        )
                    # Apply the current (default: Step) test mode to the table columns.
                    try:
                        self._test_inline_widget.switch_mode(self._test_mode)
                    except _UI_GUARD_ERRORS:
                        pass
                else:
                    with ui.VStack(height=40):
                        ui.Spacer()
                        ui.Label(
                            "Start simulation to configure test parameters",
                            alignment=ui.Alignment.CENTER,
                            style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                        )
                        ui.Spacer()

                ui.Spacer(height=2)
                # Run Test button
                self._test_play_icon = f"{EXTENSION_FOLDER_PATH}/icons/ico_waypoint_play.svg"
                with ui.ZStack(height=28):
                    self._test_button_ui = ui.Button(
                        " ",
                        height=28,
                        clicked_fn=self._on_test_button_clicked,
                        style={"Button": {"background_color": TAB_ACTIVE_BG, "border_radius": 2}},
                    )
                    with ui.HStack(height=28):
                        ui.Spacer()
                        with ui.VStack(width=0):
                            ui.Spacer()
                            self._test_button_icon = ui.Image(
                                self._test_play_icon,
                                width=14,
                                height=14,
                                style={"Image": {"image_url": self._test_play_icon}},
                            )
                            ui.Spacer()
                        ui.Spacer(width=6)
                        with ui.VStack(width=0):
                            ui.Spacer()
                            self._test_button_label = ui.Label(GT_RUN_TEST_BUTTON, width=0, height=0)
                            ui.Spacer()
                        ui.Spacer()
                self._test_button_is_running = False
                self._test_physics_sub = None
                ui.Spacer(height=4)

    def _build_snap_settings(self) -> None:
        """Builds the Snap-to-Limits settings row (hold duration, tolerance, overrides).

        Visible only when the Snap to Limits test mode is selected.
        """
        self._snap_settings_frame = ui.Frame(visible=self._test_mode == GainsTestMode.SNAP_TO_LIMITS)
        with self._snap_settings_frame:
            with ui.VStack(height=0, spacing=4):
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    ui.Label("Hold Duration (s)", width=0, height=24, alignment=ui.Alignment.LEFT_CENTER)
                    ui.Spacer(width=8)
                    self._hold_duration_field = ui.FloatField(
                        height=20, width=80, step=0.1, min_value=0.1, max_value=5.0
                    )
                    self._hold_duration_field.model.set_value(1.0)
                    ui.Spacer(width=16)
                    ui.Label("Tolerance (rad/m)", width=0, height=24, alignment=ui.Alignment.LEFT_CENTER)
                    ui.Spacer(width=8)
                    self._tolerance_field = ui.FloatField(
                        height=20, width=80, step=0.001, min_value=0.001, max_value=1.0
                    )
                    self._tolerance_field.model.set_value(0.01)
                    ui.Spacer(width=ui.Fraction(1))
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    self._disable_self_collisions_cb = ui.CheckBox(width=20, height=20)
                    self._disable_self_collisions_cb.model.set_value(False)
                    ui.Spacer(width=6)
                    ui.Label(
                        "Disable Self-Collisions During Test",
                        width=0,
                        height=24,
                        alignment=ui.Alignment.LEFT_CENTER,
                    )
                    ui.Spacer(width=ui.Fraction(1))
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    self._disable_velocity_limits_cb = ui.CheckBox(width=20, height=20)
                    self._disable_velocity_limits_cb.model.set_value(False)
                    ui.Spacer(width=6)
                    ui.Label(
                        "Disable Velocity Limits During Test",
                        width=0,
                        height=24,
                        alignment=ui.Alignment.LEFT_CENTER,
                    )
                    ui.Spacer(width=ui.Fraction(1))

    def _build_stress_test_settings(self) -> None:
        """Builds the Stress-test settings rows (sub-mode, duration, seed, thresholds, overrides).

        Visible only when the Stress test mode is selected.
        """
        self._stress_test_settings_frame = ui.Frame(visible=self._test_mode == GainsTestMode.STRESS_TEST)
        with self._stress_test_settings_frame:
            with ui.VStack(height=0, spacing=4):
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    ui.Label("Sub-mode", width=0, height=24, alignment=ui.Alignment.LEFT_CENTER)
                    ui.Spacer(width=8)
                    self._stress_test_submode_combo = ui.ComboBox(0, "Random Walk", "Adversarial", width=140, height=20)
                    self._stress_test_submode_combo.model.add_item_changed_fn(
                        lambda m, _: self._on_stress_test_submode_changed(m)
                    )
                    ui.Spacer(width=16)
                    ui.Label("Duration (s)", width=0, height=24, alignment=ui.Alignment.LEFT_CENTER)
                    ui.Spacer(width=8)
                    self._stress_test_duration_field = ui.FloatField(
                        height=20, width=80, step=1.0, min_value=0.1, max_value=600.0
                    )
                    self._stress_test_duration_field.model.set_value(10.0)
                    ui.Spacer(width=16)
                    ui.Label("Seed", width=0, height=24, alignment=ui.Alignment.LEFT_CENTER)
                    ui.Spacer(width=8)
                    self._stress_test_seed_field = ui.IntField(height=20, width=80)
                    self._stress_test_seed_field.model.set_value(42)
                    ui.Spacer(width=ui.Fraction(1))
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    ui.Label("Vel Threshold (rad/s)", width=0, height=24, alignment=ui.Alignment.LEFT_CENTER)
                    ui.Spacer(width=8)
                    self._stress_test_vel_threshold_field = ui.FloatField(height=20, width=80, step=1.0, min_value=0.1)
                    self._stress_test_vel_threshold_field.model.set_value(100.0)
                    ui.Spacer(width=16)
                    # Sigma applies to Random Walk; Snap Interval applies to Adversarial.
                    self._stress_test_sigma_frame = ui.Frame(visible=True, width=0)
                    with self._stress_test_sigma_frame:
                        with ui.HStack(width=0):
                            ui.Label("Sigma (% range)", width=0, height=24, alignment=ui.Alignment.LEFT_CENTER)
                            ui.Spacer(width=8)
                            self._stress_test_sigma_field = ui.FloatField(
                                height=20, width=80, step=0.1, min_value=0.01, max_value=100.0
                            )
                            self._stress_test_sigma_field.model.set_value(1.0)
                    self._stress_test_snap_interval_frame = ui.Frame(visible=False, width=0)
                    with self._stress_test_snap_interval_frame:
                        with ui.HStack(width=0):
                            ui.Label("Snap Interval (steps)", width=0, height=24, alignment=ui.Alignment.LEFT_CENTER)
                            ui.Spacer(width=8)
                            self._stress_test_snap_interval_field = ui.IntField(height=20, width=80)
                            self._stress_test_snap_interval_field.model.set_value(10)
                    ui.Spacer(width=ui.Fraction(1))
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    self._stress_test_disable_self_collisions_cb = ui.CheckBox(width=20, height=20)
                    self._stress_test_disable_self_collisions_cb.model.set_value(False)
                    ui.Spacer(width=6)
                    ui.Label(
                        "Disable Self-Collisions During Test",
                        width=0,
                        height=24,
                        alignment=ui.Alignment.LEFT_CENTER,
                    )
                    ui.Spacer(width=ui.Fraction(1))
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    self._stress_test_disable_velocity_limits_cb = ui.CheckBox(width=20, height=20)
                    self._stress_test_disable_velocity_limits_cb.model.set_value(False)
                    ui.Spacer(width=6)
                    ui.Label(
                        "Disable Velocity Limits During Test",
                        width=0,
                        height=24,
                        alignment=ui.Alignment.LEFT_CENTER,
                    )
                    ui.Spacer(width=ui.Fraction(1))

    def _build_discretization_settings(self) -> None:
        """Builds the dt-sweep settings rows (dt range, level count, probe + thresholds).

        Visible only when the dt Sweep test mode is selected.  The sweep runs a
        single-timestep accuracy probe at each of ``Steps`` timesteps between
        ``dt Max`` and ``dt Min`` and classifies each joint at the ``Target dt``.
        """
        self._discretization_settings_frame = ui.Frame(visible=self._test_mode == GainsTestMode.DISCRETIZATION)
        with self._discretization_settings_frame:
            with ui.VStack(height=0, spacing=4):
                # -- Timestep range: which physics timesteps to sweep. --
                self._build_dt_sweep_group_header(GT_DT_SWEEP_GROUP_RANGE)
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    self._dt_sweep_field_label("dt Max (s)", GT_DT_SWEEP_TIP_DT_MAX)
                    ui.Spacer(width=8)
                    self._discretization_dt_max_field = ui.FloatField(
                        height=20, width=80, step=0.001, min_value=0.0001, max_value=1.0, tooltip=GT_DT_SWEEP_TIP_DT_MAX
                    )
                    self._discretization_dt_max_field.model.set_value(1.0 / 30.0)
                    ui.Spacer(width=16)
                    self._dt_sweep_field_label("dt Min (s)", GT_DT_SWEEP_TIP_DT_MIN)
                    ui.Spacer(width=8)
                    self._discretization_dt_min_field = ui.FloatField(
                        height=20,
                        width=80,
                        step=0.0001,
                        min_value=0.00001,
                        max_value=1.0,
                        tooltip=GT_DT_SWEEP_TIP_DT_MIN,
                    )
                    self._discretization_dt_min_field.model.set_value(1.0 / 600.0)
                    ui.Spacer(width=16)
                    self._dt_sweep_field_label("Steps", GT_DT_SWEEP_TIP_STEPS)
                    ui.Spacer(width=8)
                    self._discretization_dt_steps_field = ui.IntField(
                        height=20, width=60, tooltip=GT_DT_SWEEP_TIP_STEPS
                    )
                    self._discretization_dt_steps_field.model.set_value(10)
                    ui.Spacer(width=ui.Fraction(1))
                # -- Probe: how each single-timestep accuracy probe is run. --
                self._build_dt_sweep_group_header(GT_DT_SWEEP_GROUP_PROBE)
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    self._dt_sweep_field_label("Target dt (s)", GT_DT_SWEEP_TIP_TARGET_DT)
                    ui.Spacer(width=8)
                    self._discretization_target_dt_field = ui.FloatField(
                        height=20,
                        width=80,
                        step=0.001,
                        min_value=0.00001,
                        max_value=1.0,
                        tooltip=GT_DT_SWEEP_TIP_TARGET_DT,
                    )
                    self._discretization_target_dt_field.model.set_value(1.0 / 120.0)
                    ui.Spacer(width=16)
                    self._dt_sweep_field_label("Timeout (s)", GT_DT_SWEEP_TIP_TIMEOUT)
                    ui.Spacer(width=8)
                    self._discretization_timeout_field = ui.FloatField(
                        height=20, width=80, step=0.5, min_value=0.5, max_value=60.0, tooltip=GT_DT_SWEEP_TIP_TIMEOUT
                    )
                    self._discretization_timeout_field.model.set_value(10.0)
                    ui.Spacer(width=16)
                    self._dt_sweep_field_label("Hold Duration (s)", GT_DT_SWEEP_TIP_HOLD)
                    ui.Spacer(width=8)
                    self._discretization_hold_duration_field = ui.FloatField(
                        height=20, width=80, step=0.1, min_value=0.1, max_value=5.0, tooltip=GT_DT_SWEEP_TIP_HOLD
                    )
                    self._discretization_hold_duration_field.model.set_value(1.0)
                    ui.Spacer(width=ui.Fraction(1))
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    self._dt_sweep_field_label("Tolerance (rad/m)", GT_DT_SWEEP_TIP_TOLERANCE)
                    ui.Spacer(width=8)
                    self._discretization_tolerance_field = ui.FloatField(
                        height=20,
                        width=80,
                        step=0.001,
                        min_value=0.001,
                        max_value=1.0,
                        tooltip=GT_DT_SWEEP_TIP_TOLERANCE,
                    )
                    self._discretization_tolerance_field.model.set_value(0.01)
                    ui.Spacer(width=ui.Fraction(1))
                # -- Accuracy thresholds: when a joint counts as degraded. --
                self._build_dt_sweep_group_header(GT_DT_SWEEP_GROUP_THRESHOLDS)
                with ui.HStack(height=24):
                    ui.Spacer(width=8)
                    self._dt_sweep_field_label("Settle Degrad (%)", GT_DT_SWEEP_TIP_SETTLE_DEGRAD)
                    ui.Spacer(width=8)
                    self._discretization_settle_degrad_field = ui.FloatField(
                        height=20,
                        width=80,
                        step=1.0,
                        min_value=0.0,
                        max_value=1000.0,
                        tooltip=GT_DT_SWEEP_TIP_SETTLE_DEGRAD,
                    )
                    self._discretization_settle_degrad_field.model.set_value(20.0)
                    ui.Spacer(width=16)
                    self._dt_sweep_field_label("Error Degrad (%)", GT_DT_SWEEP_TIP_ERROR_DEGRAD)
                    ui.Spacer(width=8)
                    self._discretization_error_degrad_field = ui.FloatField(
                        height=20,
                        width=80,
                        step=1.0,
                        min_value=0.0,
                        max_value=1000.0,
                        tooltip=GT_DT_SWEEP_TIP_ERROR_DEGRAD,
                    )
                    self._discretization_error_degrad_field.model.set_value(10.0)
                    ui.Spacer(width=ui.Fraction(1))

    def _build_dt_sweep_group_header(self, text: str) -> None:
        """Render a small muted section header inside the dt-sweep options panel."""
        with ui.HStack(height=18):
            ui.Spacer(width=8)
            ui.Label(text, width=0, style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE})
            ui.Spacer()

    def _dt_sweep_field_label(self, text: str, tooltip: str) -> None:
        """Render a dt-sweep option field label carrying its explanatory tooltip."""
        ui.Label(text, width=0, height=24, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)

    def _on_stress_test_submode_changed(self, model) -> None:
        """Toggle the sigma / snap-interval fields based on the stress-test sub-mode.

        Args:
            model: Combo-box model containing the selected stress-test sub-mode.
        """
        is_random_walk = model.get_item_value_model().get_value_as_int() == 0
        if self._stress_test_sigma_frame is not None:
            self._stress_test_sigma_frame.visible = is_random_walk
        if self._stress_test_snap_interval_frame is not None:
            self._stress_test_snap_interval_frame.visible = not is_random_walk

    def _build_test_info_frame(self) -> None:
        """Build the Test Information panel from the last test's real per-joint metrics.

        Renders one block per joint currently selected in the Test Gains color
        picker (falling back to the plotted joints, or the first DOF when nothing
        is selected), each labelled with its joint name.  Every block reuses
        ``_compute_dof_validation_summary`` so the values and gating match the
        per-joint Gain Settings summary exactly.  Shows a clear empty state until
        a test has produced data.

        This is a ``CollapsableFrame`` build function; it is invoked only on
        explicit ``rebuild()`` (test completion / color-joint selection change),
        never per render or physics step.
        """
        with self._test_info_frame:
            with ui.VStack(style=get_style(), spacing=0, height=0):
                ui.Spacer(height=4)

                if not self._gains_tuner or not self._gains_tuner.is_data_ready():
                    self._build_test_info_empty_state()
                    ui.Spacer(height=4)
                    return

                # The dt sweep / snap / stress modes report an all-joints results
                # table + scene-level verdict banner (consistent column layouts)
                # rather than per-selected-joint metric blocks.
                if self._last_test_mode == GainsTestMode.DISCRETIZATION:
                    self._build_dt_sweep_results()
                    ui.Spacer(height=4)
                    return
                if self._last_test_mode == GainsTestMode.SNAP_TO_LIMITS:
                    self._build_snap_results()
                    ui.Spacer(height=4)
                    return
                if self._last_test_mode == GainsTestMode.STRESS_TEST:
                    self._build_stress_results()
                    ui.Spacer(height=4)
                    return

                # Prefer the color-picker selection; fall back to the first
                # tunable (non-mimic) DOF so the panel is never empty once data
                # exists and never defaults to a mimic joint.
                if self._plotting_indices:
                    dof_indices = list(self._plotting_indices)
                else:
                    tunable = self._get_tunable_joint_entries()
                    dof_indices = [tunable[0].dof_index] if tunable else []

                try:
                    dof_names = self._gains_tuner.get_articulation().dof_names
                except _UI_GUARD_ERRORS:
                    dof_names = None

                rendered_any = False
                for dof_index in dof_indices:
                    rows = self._compute_dof_validation_summary(dof_index)
                    if not rows:
                        continue

                    # Separate consecutive per-joint blocks with a small gap.
                    if rendered_any:
                        ui.Spacer(height=8)
                    rendered_any = True

                    joint_label = None
                    if dof_names is not None and 0 <= dof_index < len(dof_names):
                        joint_label = dof_names[dof_index]
                    if joint_label:
                        with ui.HStack(height=20):
                            ui.Spacer(width=8)
                            ui.Label(
                                joint_label,
                                width=0,
                                name="test_info_label",
                                style={"color": LABEL_COLOR, "font_size": FONT_SIZE},
                            )

                    for label, value in rows:
                        with ui.HStack(height=22):
                            ui.Spacer(width=8)
                            ui.Label(
                                label,
                                width=180,
                                name="test_info_label",
                                style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                            )
                            ui.Label(
                                value,
                                width=0,
                                name="test_info_value",
                                style={"color": LABEL_COLOR, "font_size": FONT_SIZE},
                            )

                if not rendered_any:
                    self._build_test_info_empty_state()
                ui.Spacer(height=4)

    def _build_test_info_empty_state(self) -> None:
        """Render the Test Information empty state (no validation results yet)."""
        with ui.HStack(height=0):
            ui.Spacer(width=8)
            ui.Label(
                "No validation results yet. Run a test in the Test Gains tab.",
                word_wrap=True,
                style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
            )
            ui.Spacer(width=8)

    def _build_verdict_banner(self, text: str, color: int) -> None:
        """Render a shared scene-level verdict banner (bordered, colored label).

        Used by the dt Sweep / Snap / Stress all-joints results tables so the
        overall verdict is presented consistently across modes.

        Args:
            text: The verdict text (e.g. ``ALL JOINTS STABLE``).
            color: The banner border and text color for the verdict.
        """
        with ui.HStack(height=26):
            ui.Spacer(width=8)
            with ui.ZStack():
                ui.Rectangle(
                    style={
                        "background_color": FRAME_BG_COLOR,
                        "border_color": color,
                        "border_width": 1,
                        "border_radius": 4,
                    }
                )
                with ui.HStack():
                    ui.Spacer(width=8)
                    ui.Label(
                        text,
                        style={"color": color, "font_size": FONT_SIZE},
                        alignment=ui.Alignment.LEFT_CENTER,
                    )
                    ui.Spacer()
            ui.Spacer(width=8)

    def _build_results_table_header(self, columns: list[tuple[str, int, str | None]]) -> None:
        """Render a shared all-joints results-table header row.

        Column widths use ``ui.Fraction`` so the layout stays consistent across
        the dt Sweep / Snap / Stress tables and scales with the panel width.

        Args:
            columns: Ordered ``(label, fraction, tooltip)`` column specs;
                ``tooltip`` may be ``None`` for columns without one.
        """
        with ui.HStack(height=22):
            ui.Spacer(width=8)
            for label, fraction, tooltip in columns:
                kwargs = {"tooltip": tooltip} if tooltip else {}
                ui.Label(
                    label,
                    width=ui.Fraction(fraction),
                    name="test_info_label",
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                    **kwargs,
                )
            ui.Spacer(width=8)

    def _build_results_table_row(self, cells: list[tuple[str, int, int | None]]) -> None:
        """Render one shared all-joints results-table row.

        Args:
            cells: Ordered ``(text, fraction, color)`` cell specs; ``color`` may
                be ``None`` to use the default label color (the result cell passes
                its status color).
        """
        with ui.HStack(height=22):
            ui.Spacer(width=8)
            for text, fraction, color in cells:
                ui.Label(
                    text,
                    width=ui.Fraction(fraction),
                    name="test_info_value",
                    style={"color": color if color is not None else LABEL_COLOR, "font_size": FONT_SIZE},
                )
            ui.Spacer(width=8)

    def _build_snap_results(self) -> None:
        """Render the Snap-to-Limits results (verdict banner + all-joints table).

        Delegates to :class:`SnapResultsPanel`, which shares the spec-driven
        results-panel builder with the Stress and dt-Sweep modes.  Rendered inside
        the Test Information ``CollapsableFrame`` build function, so it is produced
        only on an explicit rebuild (never per render / physics step).
        """
        SnapResultsPanel(self).build_results_panel()

    def _build_stress_results(self) -> None:
        """Render the Stress-test results (mode/seed context, verdict banner + table).

        Delegates to :class:`StressResultsPanel`.  See :meth:`_build_snap_results`
        for the shared rendering contract.
        """
        StressResultsPanel(self).build_results_panel()

    def _build_dt_sweep_results(self) -> None:
        """Render the dt Sweep results (target-dt context, verdict banner, table, charts).

        Delegates to :class:`DtSweepResultsPanel`.  See :meth:`_build_snap_results`
        for the shared rendering contract.
        """
        DtSweepResultsPanel(self).build_results_panel()

    @staticmethod
    def _dt_sweep_chart_series(metric: dict, key: str, scale: float) -> tuple[list[float], list[float]]:
        """Extract ascending-dt (x) and per-level metric (y) arrays for a vs-dt chart.

        Only non-skipped levels with a finite metric value are included, and the
        pairs are sorted by dt ascending (fine -> coarse) so the chart widget's
        left-to-right x-axis and windowing behave.  ``x`` values are returned in
        milliseconds for readable axis labels.

        Args:
            metric: The aggregated per-DOF dt-sweep metrics dict.
            key: The per-level metric key to plot (``settling_time`` or
                ``steady_state_error``).
            scale: Multiplier applied to the y value (rad -> deg for rotational SS
                error; 1.0 for settle time).

        Returns:
            A ``(x_ms, y)`` tuple of matched, dt-ascending sample lists.
        """
        pairs: list[tuple[float, float]] = []
        for level in (metric or {}).get("dt_sweep", []) or []:
            if level.get("skipped"):
                continue
            dt = level.get("dt")
            val = level.get(key)
            if not UIBuilder._is_finite_number(dt) or dt <= 0:
                continue
            if not UIBuilder._is_finite_number(val):
                continue
            pairs.append((float(dt) * 1000.0, float(val) * scale))
        pairs.sort(key=lambda p: p[0])
        return [p[0] for p in pairs], [p[1] for p in pairs]

    def _build_dt_sweep_vs_dt_charts(self, dof_metrics, joint_name_fn, is_rotational_fn, target_dt) -> None:
        """Render settle-time and steady-state-error versus dt charts for selected joints.

        Reuses :class:`JointGraphWidget` (the chart widget used by the Position /
        Effort charts) with one series per color-picker-selected joint.  Falls back
        to the first tested joint when nothing is selected.  dt is plotted on the x
        axis (milliseconds, ascending = fine to coarse) with a caption naming the
        target dt; settle time (seconds) and steady-state error (deg or m) are on
        the respective y axes.

        Args:
            dof_metrics: Ordered ``(dof_index, metric)`` pairs for tested joints.
            joint_name_fn: Callable mapping a DOF index to its display name.
            is_rotational_fn: Callable reporting whether a DOF is rotational.
            target_dt: The sweep's target dt (seconds), used in the caption.
        """
        metrics_by_dof = dict(dof_metrics)
        # Prefer the color-picker selection; fall back to the first tested joint.
        selected = [d for d in (self._plotting_indices or []) if d in metrics_by_dof]
        if not selected and dof_metrics:
            selected = [dof_metrics[0][0]]
        if not selected:
            return

        def _chart(title: str, key: str, y_unit_for: "callable", rotational_scale: bool) -> None:
            x_list: list[list[float]] = []
            y_list: list[list[float]] = []
            plotted: list[int] = []
            legends: list[str] = []
            for dof in selected:
                scale = (180.0 / np.pi) if (rotational_scale and is_rotational_fn(dof)) else 1.0
                xs, ys = self._dt_sweep_chart_series(metrics_by_dof[dof], key, scale)
                if len(xs) < 2:
                    continue
                x_list.append(xs)
                y_list.append(ys)
                plotted.append(dof)
                legends.append(joint_name_fn(dof))
            if not y_list:
                return
            # ``selected`` already drops joints the sweep has no metrics for, and a
            # joint with fewer than two dt levels is dropped again here, so the
            # colours have to follow the DOF indices that survived both.
            colors = self.series_colors(self._plotting_group_colors, plotted)
            # y unit follows the first plotted joint (deg when any rotational).
            y_unit = y_unit_for(selected[0])
            ui.Spacer(height=6)
            with ui.HStack(height=18):
                ui.Spacer(width=8)
                ui.Label(title, style={"color": LABEL_COLOR, "font_size": FONT_SIZE})
                ui.Spacer()
            JointGraphWidget(
                x_data=x_list,
                y_data=y_list,
                data_colors=colors if colors else None,
                header_count=max(1, len(y_list)),
                legends=legends,
                y_unit=y_unit,
                x_unit="ms",
            )

        # Settle time vs dt (seconds on y).
        _chart(GT_DT_SWEEP_SETTLE_CHART_TITLE, "settling_time", lambda _dof: "s", rotational_scale=False)
        # Steady-state error vs dt (deg for rotational, m otherwise).
        _chart(
            GT_DT_SWEEP_SS_ERROR_CHART_TITLE,
            "steady_state_error",
            lambda dof: "\u00b0" if is_rotational_fn(dof) else "m",
            rotational_scale=True,
        )
        # Caption naming the target dt (in ms to match the x axis).
        if UIBuilder._is_finite_number(target_dt) and target_dt > 0:
            with ui.HStack(height=16):
                ui.Spacer(width=8)
                ui.Label(
                    f"x: dt in ms (fine -> coarse); target dt = {float(target_dt) * 1000.0:.3f} ms",
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
                ui.Spacer()

    def _on_save_gains_to_physics_layer(self, *args) -> None:
        """Saves tuned gain values to the selected DriveAPI save-target layer.

        Builds a per-layer save plan that routes DriveAPI stiffness/damping/type to
        the layer selected in the save-target dropdown.  MuJoCo ``mjc:*`` values are
        written only when the user opts in via the "Mirror tuned DriveAPI gains to
        MuJoCo" checkbox (OFF by default) or when the MuJoCo solver is active (mjc
        is then the active source, saved directly); otherwise mjc prims are never
        touched.  Tuned Newton actuator gains are written back to their own layer
        by default.  Presents the writable target layers in the stage save dialog;
        edits are authored and saved only when the user confirms.

        Args:
            *args: Variable arguments passed from the UI callback.
        """
        stage = omni.usd.get_context().get_stage()
        entries = self._gains_tuner.get_joint_entries()
        root = self._gains_tuner.get_articulation_root()
        drive_target = self._selected_save_target_identifier or self._backend_ctx.save_target_identifier or None

        plan = gain_tuner.build_gain_save_plan(
            entries,
            stage,
            drive_target,
            mirror_enabled=self._mirror_writeback_enabled,
            mirror_drive_to_mjc=self._mirror_drive_to_mjc,
            mjc_target_identifier=self._selected_mjc_target_identifier,
            newton_target_identifier=self._selected_newton_target_identifier,
            articulation_root_path=root,
            mujoco_solver_active=self._backend_ctx.is_mujoco_solver,
        )

        savable_layers = []
        has_unsavable = False
        for identifier in plan.keys():
            layer = gain_tuner.find_layer_by_save_identifier(identifier)
            if layer is not None and gain_tuner.is_layer_savable(layer):
                savable_layers.append(identifier)
            else:
                has_unsavable = True

        def on_save(save_plan, selected_layers, comment="") -> None:
            # Author only the confirmed layers, then persist them.
            confirmed = {identifier: save_plan[identifier] for identifier in selected_layers if identifier in save_plan}
            gain_tuner.apply_gain_save_plan(confirmed, save=True)

        if self._save_stage_prompt:
            self._save_stage_prompt.destroy()
        self._save_stage_prompt = StageSaveDialog(
            enable_dont_save=True,
            on_save_fn=partial(on_save, plan),
        )
        if savable_layers:
            self._save_stage_prompt.show(savable_layers)

        if has_unsavable or not savable_layers:
            self._post_no_permission_toast()

    def _on_save_file(self, *args) -> None:
        """Handles saving the USD stage file.

        Identifies writable layers in the stage and shows the stage save dialog. Displays a warning if no
        layers have edit permissions.

        Args:
            *args: Variable arguments passed from the UI callback.
        """
        stage = omni.usd.get_context().get_stage()
        decision = gain_tuner.plan_stage_layer_save(stage.GetLayerStack())

        if self._save_stage_prompt:
            self._save_stage_prompt.destroy()
        self._save_stage_prompt = StageSaveDialog(
            enable_dont_save=True,
        )
        if decision.show_dialog:
            self._save_stage_prompt.show(decision.writable_layer_identifiers)

        if decision.post_no_permission_toast:
            self._post_no_permission_toast()

    def _post_no_permission_toast(self) -> None:
        """Post a toast notification for the no-permission / no-physics-layer error."""
        nm.post_notification(
            "Physics Layer not found or No edit permission.\nCtrl-S (File > Save) to save the changes to new file.",
            duration=5,
            status=nm.NotificationStatus.WARNING,
        )

    def _on_test_duration_changed(self, model) -> None:
        """Handles changes to the test duration field.

        Updates the gains tuner's test duration when the user modifies the duration field value.

        Args:
            model: The float field model containing the new duration value.
        """
        self._gains_tuner.test_duration = model.get_value_as_float()

    def _on_select_all(self) -> None:
        """Selects all joints in the test gains table."""
        if self._test_inline_widget:
            self._test_inline_widget.select_all()

    def _on_clear_all(self) -> None:
        """Clears all joint selections in the test gains table."""
        if self._test_inline_widget:
            self._test_inline_widget.clear_all()

    def _on_test_button_clicked(self) -> None:
        """Toggle between Run Test / Cancel Test states. Starts the timeline if not already playing."""
        if not self._test_button_is_running:
            self._test_button_is_running = True
            self._test_button_label.text = GT_CANCEL_TEST_BUTTON
            self._test_button_icon.visible = False
            if not self._timeline.is_playing():
                self._timeline.play()
                generation = self._test_run_generation
                self._start_test_task = asyncio.ensure_future(self._start_test_after_timeline(generation))
            else:
                self._start_test_now()
        else:
            self._on_cancel_gains_test()

    @staticmethod
    def _should_start_deferred_test(scheduled_generation: int, current_generation: int, is_running: bool) -> bool:
        """Return True when a deferred test-start should proceed.

        The start is abandoned when a cancel/reset has bumped the run generation
        since the start was scheduled, or when the test is no longer in the running
        state (the user cancelled during the timeline warm-up window).

        Args:
            scheduled_generation: Run generation captured when the start was scheduled.
            current_generation: The builder's current run generation.
            is_running: Whether the test button is still in its running state.

        Returns:
            True only when the generation is unchanged and the test is still running.
        """
        return scheduled_generation == current_generation and bool(is_running)

    def _cancel_pending_start(self) -> None:
        """Cancel any pending deferred test-start and invalidate its run generation.

        Safe to call on a partially constructed builder (e.g. from a teardown path
        that runs before ``__init__`` finished), so the generation/task handles are
        read defensively.
        """
        self._test_run_generation = getattr(self, "_test_run_generation", 0) + 1
        task = getattr(self, "_start_test_task", None)
        if task is not None:
            task.cancel()
        self._start_test_task = None

    async def _start_test_after_timeline(self, generation: int) -> None:
        """Wait for the timeline/physics to initialize, then start the test.

        Args:
            generation: Run generation captured when this deferred start was
                scheduled; if a cancel/reset bumps it during the wait, the start is
                abandoned so a cancelled test cannot install a physics subscription.
        """
        try:
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
        except asyncio.CancelledError:
            return
        finally:
            self._start_test_task = None
        if not self._should_start_deferred_test(generation, self._test_run_generation, self._test_button_is_running):
            return
        self._start_test_now()

    def _start_test_now(self) -> None:
        """Run the gains test and subscribe to physics steps."""
        self._on_run_gains_test(self._test_mode)
        self._test_physics_sub = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
            pre_step=False, order=0, on_update=self._update_gains_test
        )

    def _reset_test_button(self) -> None:
        """Reset the Run Test button to its initial state."""
        self._test_button_is_running = False
        self._test_physics_sub = None
        if hasattr(self, "_test_button_label") and self._test_button_label:
            self._test_button_label.text = GT_RUN_TEST_BUTTON
        if hasattr(self, "_test_button_icon") and self._test_button_icon:
            self._test_button_icon.visible = True
        # Hide test panels, restore accordions
        for panel in self._test_in_progress_panels:
            panel.visible = False
        if self._accordions_container:
            self._accordions_container.visible = True

    def _on_cancel_gains_test(self) -> None:
        """Cancels the running gains test, stops the timeline, and resets the test UI state."""
        # A dt sweep is driven by its own async loop; request cancellation so it can
        # skip the remaining timestep levels and restore the physics dt in its
        # cleanup rather than being torn down mid-level here.
        if self._discretization_sweep_active:
            self._discretization_cancel_requested = True
        # Abort a deferred start still waiting on timeline warm-up so it can't
        # start the test (and subscribe to physics) after this cancel.
        self._cancel_pending_start()
        self._test_running = False
        self._reset_test_button()
        self._gains_tuner.stop_test()
        self._restore_self_collision_override()
        if self._timeline.is_playing():
            self._timeline.stop()

    def _switch_test_mode(self, switch) -> None:
        """Switches the test mode between the available gain-testing options.

        Maps the combo selection index to a ``GainsTestMode`` member, updates the
        backend and table mode, and toggles the per-mode settings frames.  This is
        event-driven (fires only on combo change), never per render/physics step.

        Args:
            switch: The value model containing the selected combo index.
        """
        idx = switch.get_value_as_int()
        mode = (
            self._test_mode_combo_modes[idx]
            if 0 <= idx < len(self._test_mode_combo_modes)
            else GainsTestMode.SINUSOIDAL
        )
        self._test_mode = mode
        self._gains_tuner.test_mode = int(mode)
        if self._test_inline_widget:
            self._test_inline_widget.switch_mode(int(mode))

        is_snap = mode == GainsTestMode.SNAP_TO_LIMITS
        is_stress = mode == GainsTestMode.STRESS_TEST
        is_discretization = mode == GainsTestMode.DISCRETIZATION
        if self._snap_settings_frame is not None:
            self._snap_settings_frame.visible = is_snap
        if self._stress_test_settings_frame is not None:
            self._stress_test_settings_frame.visible = is_stress
        if self._discretization_settings_frame is not None:
            self._discretization_settings_frame.visible = is_discretization
        if self._test_duration_frame is not None:
            self._test_duration_frame.visible = not is_snap and not is_stress and not is_discretization

        # Switching modes invalidates the previous run's results (each mode reports a
        # different metric set), so clear them and refresh the results panel to its
        # empty state rather than leaving stale rows from the prior mode lingering.
        self._last_test_mode = None
        self._gains_tuner.clear_test_results()
        if self._test_info_frame is not None:
            try:
                self._test_info_frame.rebuild()
            except (AttributeError, RuntimeError):
                pass

    # def _switch_radian_degree(self, switch):
    #     radian_degree_mode = switch.get_value_as_int()
    #     if self._test_table_widget:
    #         self._test_table_widget.switch_radian_degree(radian_degree_mode)

    def _build_charts_frame(self) -> None:
        """Builds the Test Gains page: color picker left, overall results summary + Position + Effort charts right.

        The overall test-results summary (verdict banner + all-joints table) is
        placed at the TOP, above the Position and Effort charts, for all modes so
        the scene-level outcome is read first.
        """
        if not self._gains_tuner.initialized:
            return
        with self._charts_frame:
            with ui.HStack(style=get_style(), spacing=4, height=0):
                self._color_joint_widget = ColorJointWidget(
                    self._gains_tuner, selected_changed_fn=self._on_color_joint_selection
                )
                if not self._gains_tuner.is_data_ready():
                    # No completed run to chart.  Say so instead of leaving the
                    # page blank next to the joint picker: results are only
                    # recorded when a run finishes, so a run that was stopped
                    # early (or never started) has nothing to plot.
                    self._test_info_frame = None
                    self._position_frame = None
                    self._effort_frame = None
                    self._build_chart_empty_state(
                        "No test results yet. Press Play, then Run Test, and let the run finish. "
                        "Stopping a test early discards its data."
                    )
                    return
                with ui.VStack(style=get_style(), spacing=0, height=0):
                    # Overall results summary first (above the Position/Effort charts).
                    self._test_info_frame = CollapsableFrame(
                        GT_TEST_INFO_TITLE,
                        collapsed=False,
                        enabled=True,
                        build_fn=self._build_test_info_frame,
                        show_copy_button=False,
                    )

                    self._position_frame = CollapsableFrame(
                        GT_POSITION_CHART_TITLE, collapsed=False, show_copy_button=False
                    )
                    self._position_frame.set_build_fn(self._build_position_plot)

                    if self._measured_forces_available():
                        self._effort_frame = CollapsableFrame(
                            GT_EFFORT_CHART_TITLE, collapsed=False, show_copy_button=False
                        )
                        self._effort_frame.set_build_fn(self._build_effort_plot)
                    else:
                        # The active backend (e.g. Newton) has no measured-joint-force
                        # API, so omit the Effort chart rather than show an empty or
                        # misleading panel.
                        self._effort_frame = None

    @staticmethod
    def _build_chart_empty_state(message: str) -> None:
        """Build a muted placeholder used where a chart would otherwise be blank.

        Args:
            message: Explanatory text telling the user why there is no chart.
        """
        with ui.VStack(style=get_style(), height=0, spacing=0):
            ui.Spacer(height=4)
            with ui.HStack(height=0):
                ui.Spacer(width=8)
                ui.Label(
                    message,
                    style={"color": MUTED_LABEL_COLOR, "font_size": FONT_SIZE},
                )
            ui.Spacer(height=60)

    def _on_color_joint_selection(self, selection) -> None:
        """Handles joint selection changes for plotting and updates the chart colors.

        The colours are kept per DOF index, not as a flat list, because a chart's
        series are the joints that produced a trajectory rather than the joints that
        were selected.  A flat list can only be indexed by position in the
        selection, which stops meaning anything the moment one joint is skipped.

        Args:
            selection: List of selected joint items with associated colors and indices.
        """
        self._plotting_indices = [item.joint_index for item in selection]
        self._plotting_group_colors = {item.joint_index: tuple(item.colors) for item in selection}
        if self._position_frame:
            self._position_frame.rebuild()
        if self._effort_frame:
            self._effort_frame.rebuild()
        # Refresh the Test Information text so it tracks the selected joint(s).
        if getattr(self, "_test_info_frame", None):
            try:
                self._test_info_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass

    @staticmethod
    def series_colors(group_colors: dict, plotted_indices: list, groups: int = 1) -> list:
        """Return one chart colour per series, aligned with the plotted series.

        ``JointGraphWidget.data_colors`` is per *series*, and a chart's series are
        the joints that produced data -- not the joints the user selected.  Every
        chart here filters its selection (a joint with no recorded trajectory, no
        effort samples, or too few dt levels is dropped), and the flat colour list
        the selection produced can only be indexed by position in that selection,
        so one dropped joint shifts every later trace onto another joint's colour.
        Looking the colour up by DOF index instead keeps them together no matter
        what was dropped.

        Args:
            group_colors: Map of DOF index -> that joint's colour swatches, darkest
                first (``ColorJointItem.colors``).
            plotted_indices: The DOF indices actually plotted, in series order.
            groups: How many series each joint contributes, concatenated in
                ``y_data`` -- 2 for a chart plotting a commanded and an observed
                trace per joint.  Group ``n`` takes the joint's ``n``-th swatch, so
                the observed trace is a lighter shade of its command's colour.

        Returns:
            ``groups * len(plotted_indices)`` colours, ordered to match
            ``y_data`` built as ``group_0_series + group_1_series + ...``.  A joint
            with no swatches (never selected) contributes
            :data:`~.style.LABEL_COLOR`, the widget's own fallback.

        Example:

        .. code-block:: python

            >>> UIBuilder.series_colors({3: (0xAA, 0xBB), 7: (0xCC, 0xDD)}, [7], groups=2)
            [204, 221]
        """
        colors: list = []
        for group in range(max(1, groups)):
            for dof_index in plotted_indices:
                swatches = group_colors.get(dof_index) or ()
                if not swatches:
                    colors.append(LABEL_COLOR)
                    continue
                colors.append(swatches[min(group, len(swatches) - 1)])
        return colors

    def _dof_display_names(self, joint_indices: list) -> list[str]:
        """Return the articulation's display name for each DOF index.

        Args:
            joint_indices: DOF indices to name, in presentation order.

        Returns:
            One name per index, falling back to ``"DOF <i>"`` when the
            articulation cannot be queried.
        """
        try:
            dof_names = self._gains_tuner.get_articulation().dof_names
        except _UI_GUARD_ERRORS:
            dof_names = None
        names = []
        for dof_index in joint_indices:
            if dof_names is not None and 0 <= dof_index < len(dof_names):
                names.append(str(dof_names[dof_index]))
            else:
                names.append(f"DOF {dof_index}")
        return names

    def _build_position_plot(self) -> None:
        """Builds the position plot chart with command and observed position data from the gains test."""
        if len(self._plotting_indices) == 0:
            self._build_chart_empty_state("Select one or more joints to plot.")
            return
        joint_indices = self._plotting_indices
        cmd_pos_list = []
        obs_pos_list = []
        cmd_vel_list = []
        obs_vel_list = []
        cmd_times_list = []
        # Joints that returned a trajectory, with the scale each was plotted at.
        # A skipped joint must not shift the per-joint divergence bounds out of
        # alignment with the series they apply to, nor the names off their series.
        plotted_indices: list[int] = []
        plotted_scale: list[float] = []
        scale = [1.0] * len(joint_indices)
        for i, joint_index in enumerate(joint_indices):
            if self._gains_tuner.get_articulation().dof_types[joint_index] == DofType.Rotation:
                scale[i] = 180.0 / np.pi
        for i, joint_index in enumerate(joint_indices):
            cmd_pos, cmd_vel, obs_pos, obs_vel, cmd_times = self._gains_tuner.get_joint_states_from_gains_test(
                joint_index
            )
            if cmd_pos is None:
                continue
            cmd_pos_list.append(cmd_pos * scale[i])
            obs_pos_list.append(obs_pos * scale[i])
            cmd_vel_list.append(cmd_vel * scale[i])
            obs_vel_list.append(obs_vel * scale[i])
            cmd_times_list.append(cmd_times)
            plotted_indices.append(joint_index)
            plotted_scale.append(scale[i])

        # Every selected joint came back without a trajectory (for example an
        # un-driven joint that the run never commanded), so there is nothing to
        # draw.  An empty chart looks broken; name the reason instead.
        if not cmd_pos_list:
            self._build_chart_empty_state("No trajectory recorded for the selected joint(s).")
            return

        # The commanded trajectories lead `y_data`, so the observed responses that
        # follow them get the bounds; a command is bounded by construction and is
        # never clipped.
        limits = [None] * len(cmd_pos_list) + self._position_divergence_limits(
            plotted_indices, cmd_pos_list, plotted_scale
        )

        names = self._dof_display_names(plotted_indices)
        JointGraphWidget(
            x_data=cmd_times_list + cmd_times_list,
            y_data=cmd_pos_list + obs_pos_list,
            # Per series, not per joint: the command half and the observed half are
            # both plotted, and a skipped joint is absent from both.
            data_colors=self.series_colors(self._plotting_group_colors, plotted_indices, groups=2),
            header_count=2,
            legends=["Command Joint", "Observed Joint"],
            y_unit="\u00b0",
            x_unit="s",
            series_labels=[f"Command ({n})" for n in names] + [f"Observed ({n})" for n in names],
            joint_labels=names,
            reference_count=len(cmd_pos_list),
            divergence_limits=limits,
        )

    def _position_divergence_limits(
        self, joint_indices: list[int], cmd_pos_list: list[np.ndarray], scale: list[float]
    ) -> list[float | None]:
        """Return the per-joint magnitude bound for each plotted observed response.

        Each bound comes from that joint's own travel limits, in the same scaled
        units the chart plots, so it is independent of the test mode, of the
        commanded amplitude, and of which other joints happen to be selected.
        Joints whose limits do not bound anything (a continuous revolute) fall back
        to their own commanded peak.
        """
        lower, upper = None, None
        try:
            articulation = self._gains_tuner.get_articulation()
            lower, upper = (np.array(lim.list()) for lim in articulation.get_dof_limits())
        except (AttributeError, TypeError, ValueError, IndexError):
            # Without limits every joint falls back to its commanded peak below.
            pass

        limits: list[float | None] = []
        for i, joint_index in enumerate(joint_indices):
            lo = hi = None
            if lower is not None and upper is not None and joint_index < len(lower):
                lo = float(lower[joint_index]) * scale[i]
                hi = float(upper[joint_index]) * scale[i]
            reference = cmd_pos_list[i] if i < len(cmd_pos_list) else None
            limits.append(gain_tuner.divergence_limit(lo, hi, reference))
        return limits

    def _build_velocity_plot(self) -> None:
        """Builds the velocity plot chart with command and observed velocity data from the gains test."""
        if len(self._plotting_indices) == 0:
            return
        joint_indices = self._plotting_indices
        cmd_pos_list = []
        cmd_vel_list = []
        obs_pos_list = []
        obs_vel_list = []
        cmd_times_list = []
        plotted_indices: list[int] = []
        for joint_index in joint_indices:
            cmd_pos, cmd_vel, obs_pos, obs_vel, cmd_times = self._gains_tuner.get_joint_states_from_gains_test(
                joint_index
            )
            if cmd_pos is None:
                continue
            cmd_pos_list.append(cmd_pos)
            cmd_vel_list.append(cmd_vel)
            obs_pos_list.append(obs_pos)
            obs_vel_list.append(obs_vel)
            cmd_times_list.append(cmd_times)
            plotted_indices.append(joint_index)
        series = self.series_colors(self._plotting_group_colors, plotted_indices, groups=2)
        plot = CustomXYPlot(
            show_legend=True,
            x_data=cmd_times_list + cmd_times_list,
            y_data=cmd_vel_list + obs_vel_list,
            data_colors=series,
            header_count=2,
        )
        plot.set_data_colors(series)

    def _build_effort_plot(self) -> None:
        """Builds the Effort (Nm) chart from per-joint effort captured during the test.

        Effort (torque for rotational joints, force for prismatic) is sampled each
        physics step on the frontend and plotted here — one trace per selected
        joint.          This is a temporary path until the backend exposes effort history.
        """
        if len(self._plotting_indices) == 0:
            self._build_chart_empty_state("Select one or more joints to plot.")
            return

        # No effort samples yet (e.g. before a test has run) — show a hint.
        if not self._test_effort_history or not self._test_effort_times:
            self._build_chart_empty_state("Run a test to view effort data.")
            return

        joint_indices = self._plotting_indices
        times = np.asarray(self._test_effort_times, dtype=float)
        hist = np.asarray(self._test_effort_history, dtype=float)  # shape (steps, num_dof)

        x_list = []
        y_list = []
        plotted_indices = []
        for joint_index in joint_indices:
            if hist.ndim != 2 or joint_index >= hist.shape[1]:
                continue
            y_list.append(hist[:, joint_index])
            x_list.append(times)
            plotted_indices.append(joint_index)

        if not y_list:
            self._build_chart_empty_state("No effort recorded for the selected joint(s).")
            return

        # One trace per joint, so one colour per joint that produced effort samples
        # -- looked up by DOF index, since a joint outside the effort history's DOF
        # range is dropped from the series.
        colors = self.series_colors(self._plotting_group_colors, plotted_indices)
        effort_names = self._dof_display_names(plotted_indices)
        # Effort carries no commanded reference and no authored bound to calibrate
        # against, so only non-finite samples can be identified here.  A joint that
        # diverges to a large but finite effort still stretches this chart's range.
        JointGraphWidget(
            x_data=x_list,
            y_data=y_list,
            data_colors=colors,
            header_count=1,
            legends=["Effort"],
            y_unit="Nm",
            x_unit="s",
            series_labels=[f"Effort ({n})" for n in effort_names],
            joint_labels=effort_names,
        )

    def _reset_robot_ui_state(self) -> None:
        """Reset all per-robot test/selection/plot/validation UI state to defaults.

        Called on a genuine robot change so nothing carries over from the previously
        loaded robot: the selected-joint index, viewed-source override, tuning mode,
        test mode + progress accumulators, captured plot/effort data, last-run
        validation summary, save-target overrides, and the Charts tab (hidden and
        disabled until a test is run for the new robot).  The core gain tuner is
        reset separately by the caller (``_gains_tuner.reset()``).
        """
        # Test mode / run state.
        self._test_running = False
        self._test_mode = GainsTestMode.SNAP_TO_LIMITS
        self._last_test_mode = None
        # Selection / detail-view state.
        self._selected_joint_index = None
        self._detail_selection_count = 0
        self._pending_table_selection = None
        # Per-joint viewed-source overrides are cleared on a robot swap so the new
        # robot's rows start from each joint's active-source default.
        self._joint_viewed_source = {}
        self._detail_tuning_mode = TuningMode.STIFFNESS
        self._detail_gain_ctx = None
        # Gain-table column overrides + search reset so the new robot starts from
        # the schema-driven auto-selection with no name filter.
        self._gain_column_overrides = {}
        self._name_search_query = ""
        # Captured plot / effort data.
        self._plotting_indices = []
        self._plotting_group_colors = {}
        self._test_effort_history = []
        self._test_effort_times = []
        self._dt_sweep_level_dts = []
        self._dt_sweep_level_index = 0
        self._make_plot_on_next_frame = False
        # Test progress accumulators.
        self._test_start_time = 0.0
        self._test_total_duration = 0.0
        self._test_num_sequences = 0
        self._test_seq_duration = 0.0
        self._test_elapsed_sim = 0.0
        # Save-target overrides revert to the new robot's resolved defaults.
        self._selected_save_target_identifier = None
        self._selected_mjc_target_identifier = None
        self._selected_newton_target_identifier = None
        self._mirror_writeback_enabled = True
        self._mirror_drive_to_mjc = False
        # The Test Gains tab hosts the test *run* controls, so it must stay enabled:
        # disabling it until a test runs would make the Run Test button unreachable
        # (the only way to run a test is from this tab), a deadlock that leaves the
        # tab permanently unclickable.  A robot swap just resets the active tab back
        # to Gain Settings; the Test Gains tab itself remains clickable.
        self._nav_show_charts = False
        if self._charts_button:
            self._charts_button.enabled = True
        # Reflect the reset test mode / controls in the (defensively-guarded) widgets.
        if self._test_inline_widget:
            try:
                self._test_inline_widget.switch_mode(self._test_mode)
            except _UI_GUARD_ERRORS:
                pass
        if self._test_controls_frame:
            try:
                self._test_controls_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass
        self._update_nav_button_styles()
        self._apply_nav_panel_visibility()

    def _selected_robot_signature(self, robot_path: str):
        """Fingerprint the selected robot's joint set (articulation identity).

        Changes when a different robot is swapped in at ``robot_path`` (different
        joints), which triggers a full re-setup + reset.  Returns None when the prim
        is missing/invalid.  Read only from USD, so it is cheap and does not require
        the physics articulation to be initialized.
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return None
        prim = stage.GetPrimAtPath(robot_path)
        if not prim or not prim.IsValid():
            return None
        names = []
        try:
            for p in pxr.Usd.PrimRange(prim):
                if p.IsA(UsdPhysics.Joint):
                    names.append(p.GetName())
        except _UI_GUARD_ERRORS:
            return None
        return (len(names), tuple(names))

    def _gain_source_signature_for(self, robot_path: str):
        """Fingerprint the authored gain sources under the selected robot.

        Counts joints with a ``UsdPhysics.DriveAPI`` drive, Newton actuator prims,
        and MuJoCo actuator gains (``MjcActuator`` prims or ``mjc:gainPrm`` authored
        on a joint).  A Physics-variant switch that composes mjc:* / actuator prims
        in or out changes this signature, triggering a lightweight gain-view
        repaint (without discarding the user's selection or test results).
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return None
        prim = stage.GetPrimAtPath(robot_path)
        if not prim or not prim.IsValid():
            return None
        pd = act = mjc = 0
        try:
            for p in pxr.Usd.PrimRange(prim):
                if p.GetTypeName() == "MjcActuator":
                    mjc += 1
                    continue
                schemas = p.GetAppliedSchemas()
                if "NewtonPDControlAPI" in schemas or "NewtonPIDControlAPI" in schemas:
                    act += 1
                if p.IsA(UsdPhysics.Joint):
                    if p.HasAPI(UsdPhysics.DriveAPI):
                        pd += 1
                    mjc_gain = p.GetAttribute("mjc:gainPrm")
                    if mjc_gain and mjc_gain.IsValid() and mjc_gain.HasAuthoredValue():
                        mjc += 1
        except _UI_GUARD_ERRORS:
            return None
        return (pd, act, mjc)

    def _maybe_refresh_on_stage_change(self) -> None:
        """Re-populate / repaint the gain views when the selected robot changed.

        Handles two authoring-time changes that the ASSETS_LOADED idempotency
        guards (``_robot_menu_items`` / ``_backend_applied_signature``) would
        otherwise suppress:

        * a robot swapped for a different one at the SAME prim path (menu identity
          unchanged): re-run joint-entry setup and fully reset the UI, and
        * a Physics-variant switch that composes mjc:* / actuator prims in or out
          (same joints): repaint the gain views only, preserving selection/results.

        Gated to a stopped timeline: robot swaps and variant switches are
        authoring-time actions, while ASSETS_LOADED fires continuously during play.
        Running this only while stopped preserves the flicker guard (no per-frame
        re-setup) rather than removing it.
        """
        if not self._timeline.is_stopped():
            return
        model = self._articulation_menu_model
        if not model or not model.has_item():
            return
        selected = model.get_current_string()
        if not selected or selected == self._NO_ROBOT_PLACEHOLDER:
            return
        content_sig = self._selected_robot_signature(selected)
        if content_sig != self._robot_content_signature:
            # A different articulation now occupies the selected path: force a full
            # re-setup + reset even though the combo string is unchanged.
            self._on_articulation_selection(selected, force=True)
            return
        source_sig = self._gain_source_signature_for(selected)
        if source_sig != self._gain_source_signature:
            self._gain_source_signature = source_sig
            if self._gains_tuning_frame is not None and not self._test_running:
                try:
                    self._gains_tuning_frame.rebuild()
                except _UI_GUARD_ERRORS:
                    pass

    def _invalidate_articulation(self) -> None:
        """This function handles the event that the existing articulation becomes invalid and there is
        not a new articulation to select.  It is called explicitly in the code when the timeline is
        stopped and when the DropDown menu finds no articulations on the stage.
        """
        self._selected_path_lacks_schema = None
        self._awaiting_articulation_dofs = False
        self._gains_tuner.reset()
        self._reset_robot_ui_state()
        self._robot_content_signature = None
        self._gain_source_signature = None
        if self._gains_tuning_frame:
            self._gains_tuning_frame.rebuild()

    def _on_articulation_selection(self, articulation_path, force: bool = False) -> None:
        """Handles articulation selection changes and updates the gains tuner with the new robot.

        Args:
            articulation_path: USD path to the selected articulation prim.
            force: Re-run setup + reset even when ``articulation_path`` matches the
                currently bound robot.  Used when a robot is swapped for a different
                one at the same prim path so the stale articulation is rebuilt.
        """
        if articulation_path is None:
            self._invalidate_articulation()
            self._apply_schema_visibility()
            return

        stage = omni.usd.get_context().get_stage()
        if not stage:
            self._invalidate_articulation()
            self._apply_schema_visibility()
            return
        prim = stage.GetPrimAtPath(articulation_path)
        if not prim.IsValid():
            self._invalidate_articulation()
            self._apply_schema_visibility()
            return

        if not prim.HasAPI(Classes.ROBOT_API.value):
            self._selected_path_lacks_schema = articulation_path
            self._gains_tuner.reset()
            self._reset_robot_ui_state()
            self._robot_content_signature = None
            self._gain_source_signature = None
            self._reset_ui_next_frame = True
            self._apply_schema_visibility()
            return

        self._selected_path_lacks_schema = None
        if force or self._gains_tuner.get_robot_prim_path() != articulation_path:
            self._gains_tuner.reset()
            self._gains_tuner.setup(articulation_path)
            # The Newton backend constructs the physics-tensor articulation lazily,
            # so setup() can enumerate zero joints on the first try (num_dofs == 0
            # for the first frame(s)).  If so, arrange a re-setup from
            # on_render_step once the DOF view cooks, so the joints still populate
            # ("no joints found" regression under Newton).  A normal (PhysX) robot
            # enumerates its joints immediately, so the flag stays clear.
            self._arm_deferred_joint_wait()
            # Fully reset per-robot test/selection/plot state so nothing carries
            # over from the previously loaded robot (fixes stale selection/results
            # after a robot switch).
            self._reset_robot_ui_state()
            # Cache the new robot's content + gain-source signatures so the
            # stopped-time stage-change refresh only re-fires on a genuine
            # subsequent change (not on the repeated ASSETS_LOADED events).
            self._robot_content_signature = self._selected_robot_signature(articulation_path)
            self._gain_source_signature = self._gain_source_signature_for(articulation_path)
            if self._gains_tuning_frame:
                self._gains_tuning_frame.rebuild()

            self._reset_ui_next_frame = True
            self._apply_schema_visibility()

    def _set_test_progress_fraction(self, progress: float) -> None:
        """Set every custom progress-bar fill to ``progress`` (0.0–1.0)."""
        progress = max(0.0, min(progress, 1.0))
        for fill in self._test_progress_fills:
            fill.width = ui.Fraction(max(progress, 1e-4))
        for gap in self._test_progress_gaps:
            gap.width = ui.Fraction(max(1.0 - progress, 1e-4))

    def _set_test_progress_complete(self) -> None:
        """Fill the progress bars and read 100%, now that the run has finished.

        While a run is going the fill is capped below 1.0 so an estimate that has
        been overrun does not read as finished; completion is the one point that
        cap has to be lifted, or the panel's last state is a run stuck at 99%.
        """
        self._set_test_progress_fraction(self._progress_fraction(1.0, 1.0, running=False))
        for pct_label in self._test_progress_pct_labels:
            pct_label.text = "100%"

    def _measured_forces_available(self) -> bool:
        """True when the active physics backend exposes measured joint forces.

        The Effort chart is built from ``get_dof_projected_joint_forces()``, which
        the Newton backend does not implement.  The only physically faithful effort
        source is that measured force (an analytical PD estimate ignores max-force
        clamping and the implicit solver's dynamics coupling), so effort capture and
        the Effort chart are skipped entirely when Newton is active.
        """
        return getattr(self._backend_ctx, "backend", "PhysX") != "NewtonAPI"

    def _capture_step_effort(self) -> None:
        """Record the current step's per-DOF effort for the Effort chart.

        Uses the *measured* projected joint force (actual torque in the joint's
        motion direction).  Backends without that API (e.g. Newton) are skipped
        outright via :meth:`_measured_forces_available`, so no effort is captured
        and the Effort chart is omitted.  As a defensive fallback, if the API still
        raises it is probed at most once per run (guarded by
        ``self._projected_forces_unavailable``) so a backend gap cannot flood the
        log with a per-step exception.
        """
        if not self._measured_forces_available():
            return
        try:
            art = self._gains_tuner.get_articulation()
            if art is None:
                return
            efforts = None
            if not self._projected_forces_unavailable:
                try:
                    efforts = art.get_dof_projected_joint_forces().numpy()[0]
                except Exception:  # noqa: BLE001 - backend may not implement this API
                    self._projected_forces_unavailable = True
                    carb.log_warn(
                        "Gain Tuner: the active physics backend does not provide measured "
                        "joint forces; the Effort chart is unavailable for this run."
                    )
            if efforts is not None:
                self._test_effort_history.append(np.asarray(efforts, dtype=float).copy())
                self._test_effort_times.append(self._test_elapsed_sim)
        except _UI_GUARD_ERRORS:
            pass

    def _update_gains_test(self, step: float, context) -> None:
        """Updates the running gains test and handles test completion.

        Args:
            step: Physics step size.
            context: Physics step context.
        """
        if not self._test_running:
            return

        # Advance progress by SIMULATION time (accumulated physics steps), not
        # wall-clock time.  The test runs on sim time which is typically slower
        # than real-time, so a wall-clock bar finishes early (e.g. hits 100% while
        # the test is still on the 5th of 6 sequences).
        self._test_elapsed_sim += step
        if self._test_total_duration > 0 and self._test_progress_fills:
            elapsed = self._test_elapsed_sim
            progress = self._progress_fraction(elapsed, self._test_total_duration)
            self._set_test_progress_fraction(progress)
            pct_text = f"{int(progress * 100)}%"
            for pct_label in self._test_progress_pct_labels:
                pct_label.text = pct_text
            if self._test_progress_labels and self._test_seq_duration > 0:
                time_text = self._format_progress_time(elapsed, self._test_total_duration)
                if self._discretization_sweep_active:
                    # dt sweep: name the current timestep level (its Hz + dt), which
                    # the sweep orchestrator tracks, rather than a generic sequence.
                    dt_levels = getattr(self, "_dt_sweep_level_dts", [])
                    level_index = getattr(self, "_dt_sweep_level_index", 0)
                    num_levels = len(dt_levels) if dt_levels else self._test_num_sequences
                    dt_value = float(dt_levels[level_index]) if 0 <= level_index < len(dt_levels) else 0.0
                    seq_text = f"{self._format_dt_level_progress(level_index, num_levels, dt_value)}   {time_text}"
                else:
                    cur_seq = min(int(elapsed / self._test_seq_duration) + 1, self._test_num_sequences)
                    seq_text = f"Sequence: {cur_seq}/{self._test_num_sequences}   {time_text}"
                for seq_label in self._test_progress_labels:
                    seq_label.text = seq_text

        done = self._gains_tuner.update_gains_test(step)

        # Record this step's per-DOF effort for the Effort chart.
        self._capture_step_effort()

        if done:
            # During a dt sweep each timestep level is one physics-driven probe.
            # The sweep's async orchestrator owns the whole run's completion UI, so
            # a level finishing only ends that level; do not reset the button, plot,
            # or navigate here.
            if self._discretization_sweep_active:
                self._test_running = False
                return
            self._set_test_progress_complete()
            self._reset_ui_next_frame = True
            # Restore any self-collision override applied for this run.
            self._restore_self_collision_override()
            if self._gains_tuner.is_data_ready():
                self._make_plot_on_next_frame = True
                self._test_running = False
                self._reset_test_button()
                # Refresh the per-joint validation summary shown in the Gain
                # Settings detail panel with the freshly produced results.
                if self._joint_detail_frame:
                    try:
                        self._joint_detail_frame.rebuild()
                    except _UI_GUARD_ERRORS:
                        pass
                if self._charts_button:
                    self._charts_button.enabled = True
                if self._test_info_frame:
                    # Populate the Test Information text with the freshly produced
                    # results, then expand the panel so they are visible.
                    try:
                        self._test_info_frame.rebuild()
                    except _UI_GUARD_ERRORS:
                        pass
                    self._test_info_frame.collapsed = False
                self._on_nav_charts()
                if self._timeline.is_playing():
                    self._timeline.stop()

    @staticmethod
    def _read_float(field, default: float) -> float:
        """Read a float from a value field, falling back to ``default`` when absent.

        Args:
            field: A field wrapper exposing ``.model.get_value_as_float()``, or None.
            default: Value returned when ``field`` is falsy (not yet built).

        Returns:
            The field's current float value, or ``default``.
        """
        return field.model.get_value_as_float() if field else default

    @staticmethod
    def _read_int(field, default: int) -> int:
        """Read an int from a value field, falling back to ``default`` when absent.

        Args:
            field: A field wrapper exposing ``.model.get_value_as_int()``, or None.
            default: Value returned when ``field`` is falsy (not yet built).

        Returns:
            The field's current int value, or ``default``.
        """
        return field.model.get_value_as_int() if field else default

    @staticmethod
    def _read_bool(field, default: bool) -> bool:
        """Read a bool from a checkbox field, falling back to ``default`` when absent.

        Args:
            field: A field wrapper exposing ``.model.get_value_as_bool()``, or None.
            default: Value returned when ``field`` is falsy (not yet built).

        Returns:
            The field's current bool value, or ``default``.
        """
        return field.model.get_value_as_bool() if field else default

    @staticmethod
    def _joint_index_sequences(joint_params, sequence_ids) -> list[dict]:
        """Build the per-sequence joint-index partition shared by several test modes.

        Args:
            joint_params: The tested per-joint parameter items (each with
                ``joint_index`` and ``sequence``).
            sequence_ids: The distinct sequence group ids to partition by.

        Returns:
            One ``{"joint_indices": np.ndarray}`` dict per sequence id.
        """
        return [
            {
                "joint_indices": np.array([p.joint_index for p in joint_params if p.sequence == i], dtype=np.int32),
            }
            for i in sequence_ids
        ]

    def _on_run_gains_test(self, gains_test_mode) -> None:
        """Starts the gains test with the configured parameters from the test table.

        Args:
            gains_test_mode: The test mode to run for the gains test.
        """
        # Remember which test produced the results so the per-joint summary can
        # gate metrics (settling time is only meaningful for snap-to-limits).
        self._last_test_mode = gains_test_mode
        if not self._gains_tuner.initialized:
            self._gains_tuner.initialize()
        # The "Gains Settings" panel stays expanded; it is hidden (not collapsed)
        # while a test runs via _accordions_container.visible, so it is already
        # expanded when the user returns to the Gain Settings page.

        test_mode = self._test_inline_widget.mode if self._test_inline_widget else GainsTestMode.SINUSOIDAL

        joint_params = [
            p
            for p in (self._test_inline_widget.model.get_item_children() if self._test_inline_widget else [])
            if p.test
        ]

        duration = (
            self._test_duration_field.model.get_value_as_float()
            if hasattr(self, "_test_duration_field") and self._test_duration_field
            else 5.0
        )

        # Base dictionary shared by every test mode.
        test_params = {
            "test_mode": test_mode,
            "test_duration": duration,
            "joint_indices": [param.joint_index for param in joint_params],
        }

        # Distinct sequence groups (used by all modes to partition joints).
        sequence_ids = {p.sequence for p in joint_params}

        if test_mode == GainsTestMode.SNAP_TO_LIMITS:
            # Snap-to-limits carries its own hold duration / tolerance and only
            # needs the joint-index partition in each sequence.
            test_params["hold_duration"] = self._read_float(self._hold_duration_field, 1.0)
            test_params["tolerance"] = self._read_float(self._tolerance_field, 0.01)
            test_params["disable_self_collisions"] = self._read_bool(self._disable_self_collisions_cb, False)
            test_params["disable_velocity_limits"] = self._read_bool(self._disable_velocity_limits_cb, False)
            test_params["sequence"] = self._joint_index_sequences(joint_params, sequence_ids)
        elif test_mode == GainsTestMode.STRESS_TEST:
            # Stress test carries its own duration and RNG/threshold parameters.
            # The submode combo exposes its value through the item-value model, so it
            # is read directly rather than via the scalar-field helpers.
            test_params["stress_test_mode"] = (
                self._stress_test_submode_combo.model.get_item_value_model().get_value_as_int()
                if self._stress_test_submode_combo
                else 0
            )
            test_params["duration"] = self._read_float(self._stress_test_duration_field, 10.0)
            test_params["velocity_threshold"] = self._read_float(self._stress_test_vel_threshold_field, 100.0)
            # UI shows sigma as a percentage of range; the backend expects a fraction.
            test_params["sigma"] = self._read_float(self._stress_test_sigma_field, 1.0) / 100.0
            test_params["snap_interval"] = self._read_int(self._stress_test_snap_interval_field, 10)
            test_params["seed"] = self._read_int(self._stress_test_seed_field, 42)
            test_params["disable_self_collisions"] = self._read_bool(
                self._stress_test_disable_self_collisions_cb, False
            )
            test_params["disable_velocity_limits"] = self._read_bool(
                self._stress_test_disable_velocity_limits_cb, False
            )
            test_params["sequence"] = self._joint_index_sequences(joint_params, sequence_ids)
        elif test_mode == GainsTestMode.DISCRETIZATION:
            # dt physics sweep: dt range / level count and the per-level probe +
            # threshold parameters.  The sweep is orchestrated by its own async
            # loop (it changes the physics dt between levels), so it is dispatched
            # here and returns early rather than starting a single physics-driven run.
            test_params["dt_max"] = self._read_float(self._discretization_dt_max_field, 1.0 / 30.0)
            test_params["dt_min"] = self._read_float(self._discretization_dt_min_field, 1.0 / 600.0)
            test_params["num_dt_steps"] = self._read_int(self._discretization_dt_steps_field, 10)
            test_params["timeout"] = self._read_float(self._discretization_timeout_field, 10.0)
            test_params["hold_duration"] = self._read_float(self._discretization_hold_duration_field, 1.0)
            test_params["tolerance"] = self._read_float(self._discretization_tolerance_field, 0.01)
            test_params["target_dt"] = self._read_float(self._discretization_target_dt_field, 1.0 / 120.0)
            # UI shows degradation thresholds as percentages; the core expects fractions.
            test_params["settle_degrad_threshold"] = (
                self._read_float(self._discretization_settle_degrad_field, 20.0) / 100.0
            )
            test_params["error_degrad_threshold"] = (
                self._read_float(self._discretization_error_degrad_field, 10.0) / 100.0
            )
            test_params["sequence"] = self._joint_index_sequences(joint_params, sequence_ids)
            # Reject reversed/equal/non-positive/non-finite dt bounds before starting:
            # np.logspace would otherwise run fine->coarse while aggregation assumes
            # coarse->fine, silently producing a wrong accuracy cliff / verdict.
            dt_error = self._validate_dt_sweep_params(test_params)
            if dt_error:
                nm.post_notification(
                    f"dt sweep not started: {dt_error}",
                    duration=6,
                    status=nm.NotificationStatus.WARNING,
                )
                self._on_cancel_gains_test()
                return
            asyncio.ensure_future(self._run_discretization_sweep(test_params))
            return
        else:
            # Sinusoidal / Step: full per-joint signal parameters.
            test_params["sequence"] = [
                {
                    "joint_indices": np.array(
                        [param.joint_index for param in joint_params if param.sequence == i], dtype=np.int32
                    ),
                    "joint_amplitudes": np.array(
                        [param.amplitude * 0.005 for param in joint_params if param.sequence == i],
                        dtype=np.float32,
                    ),
                    "joint_offsets": np.array(
                        [param.offset / param.values_scale for param in joint_params if param.sequence == i],
                        dtype=np.float32,
                    ),
                    "joint_periods": np.array(
                        [param.period for param in joint_params if param.sequence == i], dtype=np.float32
                    ),
                    "joint_phases": np.array(
                        [param.phase for param in joint_params if param.sequence == i], dtype=np.float32
                    ),
                    "joint_step_max": np.array(
                        [param.step_max / param.values_scale for param in joint_params if param.sequence == i],
                        dtype=np.float32,
                    ),
                    "joint_step_min": np.array(
                        [param.step_min / param.values_scale for param in joint_params if param.sequence == i],
                        dtype=np.float32,
                    ),
                    "joint_user_provided": [param.user_provided for param in joint_params if param.sequence == i],
                }
                for i in sequence_ids
            ]

        # Progress-bar basis (simulation seconds).  Every mode loops per-sequence in
        # the core, so total = num_sequences * per_sequence_duration.  Compute the
        # per-sequence duration via the shared `_compute_test_total_duration` helper
        # (num_sequences=1 -> per-sequence value); `_begin_gains_test` then scales it
        # by the actual sequence count.  Using the helper keeps the run path and its
        # unit tests on one implementation.  Each mode derives its per-sequence
        # duration from a different field (see `_compute_test_total_duration`).
        progress_seq_duration = self._compute_test_total_duration(
            test_mode,
            1,
            duration,
            test_params.get("duration", 0.0),
            test_params.get("hold_duration", 0.0),
        )

        # An override from an earlier run survives anything that ended the run
        # without going through cancel or completion, such as the toolbar stop.
        # Clear it here so a test never inherits the previous run's collision
        # state, least of all one launched with the option switched off.
        self._restore_self_collision_override()

        # Both engines read their self-collision property when the scene is
        # cooked, so the option needs a timeline stop/play recook before the test
        # starts.  Dispatch that asynchronously; the test begins once it lands.
        if test_params.get("disable_self_collisions") and test_mode in (
            GainsTestMode.SNAP_TO_LIMITS,
            GainsTestMode.STRESS_TEST,
        ):
            asyncio.ensure_future(self._apply_override_and_run(test_params, progress_seq_duration))
            return

        self._begin_gains_test(test_params, progress_seq_duration)

    def _begin_gains_test(self, test_params, duration) -> None:
        """Initialize the backend test run and set up the in-progress UI.

        Args:
            test_params: Fully built test parameter dictionary.
            duration: Mode-appropriate per-sequence duration (sim seconds) used to
                drive the progress bar. Exact for sinusoidal/step/stress; an estimate
                for snap-to-limits, whose runtime is data-dependent.
        """
        self._gains_tuner.initialize_gains_test(test_params)
        self._test_running = True

        # ---- progress tracking ----
        self._test_num_sequences = len(test_params["sequence"])
        self._test_seq_duration = duration
        self._test_total_duration = self._test_num_sequences * duration
        self._test_start_time = time.time()
        self._test_elapsed_sim = 0.0  # accumulated simulation time (drives progress)
        self._test_effort_history = []
        self._test_effort_times = []
        self._projected_forces_unavailable = False
        self._set_test_progress_fraction(0.0)
        for pct_label in self._test_progress_pct_labels:
            pct_label.text = "0%"
        _init_seq_text = f"Sequence: 1/{self._test_num_sequences}   " f"Time: 0.0s/{self._test_total_duration:.1f}s"
        for seq_label in self._test_progress_labels:
            seq_label.text = _init_seq_text
        # Hide accordions, show test panels (on both the Gain Settings and Test Gains pages)
        if self._accordions_container:
            self._accordions_container.visible = False
        for panel in self._test_in_progress_panels:
            panel.visible = True

    def _warn_self_collision_override_unavailable(self, reason: str) -> None:
        """Tell the user the self-collision override will not take effect.

        Args:
            reason: Why the override cannot be applied, used verbatim in the message.
        """
        message = f"Self-collisions stay enabled for this test: {reason}."
        carb.log_warn(f"GainTuner: {message}")
        nm.post_notification(message, duration=6, status=nm.NotificationStatus.WARNING)

    def _self_collision_override_unsupported_reason(self) -> str | None:
        """Return why the active backend cannot honour the override, or None.

        PhysX and Newton each define a self-collision property and both are
        authored, so both are supported. Any other engine is unsupported, as is
        an engine that could not be read.

        Returns:
            A reason to surface, or None when the backend honours the override.
        """
        backend = self._active_backend()
        if gain_tuner.backend_supported(backend):
            return None
        engine = f"is {gain_tuner.backend_display_label(backend)}" if backend else "could not be read"
        return f"disabling self-collisions is supported under PhysX and Newton and the active physics engine {engine}"

    def _restore_self_collision_override(self) -> None:
        """Drop the self-collision override for this test run.

        Safe to call when no override was applied. Both engines read their
        self-collision property when the scene is cooked, so the restored value
        is picked up on the next timeline play; no recook is forced here, to
        avoid replaying the timeline after a test completes or is cancelled.
        """
        self._gains_tuner.restore_self_collision_override()

    async def _apply_override_and_run(self, test_params, duration) -> None:
        """Disable self-collisions (cook-time), recook via timeline restart, then run.

        Both engines read their self-collision property when physics loads the
        scene, so a timeline stop/play cycle forces that reload before the test
        starts.  The test still runs when the override cannot be applied, but the
        user is told so the run is not mistaken for a self-collision-free one.

        Args:
            test_params: Fully built test parameter dictionary.
            duration: Mode-appropriate per-sequence duration (sim seconds) used to
                drive the progress bar. Exact for sinusoidal/step/stress; an estimate
                for snap-to-limits, whose runtime is data-dependent.
        """
        app = omni.kit.app.get_app()

        # Check the engine before authoring anything: on a backend that ignores the
        # property, writing it and paying the recook only delays a test that runs
        # with self-collisions on regardless.
        unsupported = self._self_collision_override_unsupported_reason()
        if unsupported:
            self._warn_self_collision_override_unavailable(unsupported)
        else:
            reason = self._gains_tuner.apply_self_collision_override()
            if reason:
                self._warn_self_collision_override_unavailable(reason)

        self._restarting_for_override = True
        cancelled = False
        try:
            self._timeline.stop()
            for _ in range(3):
                await app.next_update_async()
            self._timeline.play()
            for _ in range(3):
                await app.next_update_async()
            # The user may have cancelled while the timeline was recooking.
            if not self._test_button_is_running:
                cancelled = True
        finally:
            self._restarting_for_override = False

        if cancelled:
            self._restore_self_collision_override()
            return

        started = False
        try:
            self._gains_tuner.initialize()
            await app.next_update_async()
            self._begin_gains_test(test_params, duration)
            started = True
        finally:
            # Nothing downstream drops the override unless the run actually began.
            if not started:
                self._restore_self_collision_override()

    def _rebased_effort_snapshot(self) -> tuple[list, list]:
        """Copy the current per-step effort capture, rebasing its times to start at 0.

        The sweep accumulates ``_test_effort_times`` on the cumulative sim clock, so
        a mid-sweep level would otherwise start at a large x offset.  Rebasing keeps
        each retained level's Effort chart starting at t=0 like a normal single run.

        Returns:
            A ``(history, times)`` tuple copied from the live effort buffers.
        """
        history = list(self._test_effort_history)
        times = list(self._test_effort_times)
        if times:
            t0 = times[0]
            times = [t - t0 for t in times]
        return history, times

    @staticmethod
    def _validate_dt_sweep_params(test_params: dict) -> str | None:
        """Validate dt-sweep bounds and target dt before launching the sweep.

        Args:
            test_params: The built dt-sweep parameter dict (``dt_max`` / ``dt_min`` /
                ``num_dt_steps`` / ``target_dt``).

        Returns:
            A user-facing error message when the bounds are reversed, equal,
            non-positive, or non-finite (or the target dt is invalid), otherwise
            None.
        """
        num_steps = int(test_params.get("num_dt_steps", 0))
        dt_max = float(test_params.get("dt_max", 0.0))
        dt_min = float(test_params.get("dt_min", 0.0))
        target_dt = float(test_params.get("target_dt", 0.0))
        try:
            if num_steps >= 2:
                gain_tuner.validate_dt_bounds(dt_max, dt_min)
            elif not math.isfinite(dt_max) or dt_max <= 0.0:
                raise ValueError(f"dt Max must be a finite positive number (got {dt_max!r}).")
            if not math.isfinite(target_dt) or target_dt <= 0.0:
                raise ValueError(f"Target dt must be a finite positive number (got {target_dt!r}).")
        except ValueError as exc:
            return str(exc)
        return None

    async def _run_discretization_sweep(self, test_params) -> None:
        """Orchestrate the dt physics sweep across logarithmically-spaced timesteps.

        Runs the single-timestep accuracy probe (:class:`DiscretizationSweepTest`,
        registered under ``GainsTestMode.DISCRETIZATION``) once per timestep from
        coarse to fine, restarting the timeline and setting the physics dt between
        levels.  Each level is driven to completion by the physics-step callback
        (:meth:`_update_gains_test`), which yields per-level control back here.
        After the sweep the per-level results are aggregated into per-joint accuracy
        metrics (:func:`aggregate_dt_sweep`) and the normal completion UI is shown.

        The physics dt is always restored to its pre-sweep value, even on cancel or
        error.

        Args:
            test_params: Fully built dt-sweep parameter dictionary (dt range, level
                count, probe parameters, degradation thresholds, joint sequences).
        """
        from isaacsim.core.simulation_manager import SimulationManager

        app = omni.kit.app.get_app()

        dt_max = test_params["dt_max"]
        dt_min = test_params["dt_min"]
        num_dt_steps = int(test_params["num_dt_steps"])
        target_dt = test_params["target_dt"]
        tolerance = test_params.get("tolerance", 0.01)
        hold_duration = test_params.get("hold_duration", 1.0)
        timeout = test_params.get("timeout", 10.0)
        settle_threshold = test_params.get("settle_degrad_threshold", 0.20)
        error_threshold = test_params.get("error_degrad_threshold", 0.10)
        joint_indices = list(test_params.get("joint_indices", []))

        dt_levels = gain_tuner.dt_sweep_levels(dt_max, dt_min, num_dt_steps)
        num_dt_steps = len(dt_levels)
        if num_dt_steps == 0 or not joint_indices:
            self._reset_test_button()
            return

        # dt get/set intentionally goes through SimulationManager rather than the
        # newer `PhysicsScene.get_dt()`/`set_dt()` wrappers. The manager methods
        # delegate to those same wrappers internally, but additionally (a) resolve
        # and fan out across every registered physics scene, (b) auto-create a
        # default scene when none is registered yet, and (c) guard against changing
        # dt while the timeline is playing (`set_physics_dt` raises RuntimeError).
        # The raw `PhysicsScene` wrappers expose none of that orchestration, so the
        # sweep keeps the manager entry point to preserve behavior and safety.
        original_dt = SimulationManager.get_physics_dt()
        self._discretization_sweep_active = True
        self._discretization_cancel_requested = False

        # Progress accounting.  Reuse the shared per-step progress mechanism: the
        # denominator is the per-level estimate times the level count (see
        # `_compute_test_total_duration`), and each level counts as one "sequence".
        self._test_num_sequences = num_dt_steps
        self._test_seq_duration = hold_duration + _DISCRETIZATION_SETTLE_TIMEOUT_FRACTION * timeout
        self._test_total_duration = self._compute_test_total_duration(
            GainsTestMode.DISCRETIZATION, 1, 0.0, 0.0, hold_duration, num_dt_steps=num_dt_steps, timeout=timeout
        )
        # Level tracking so the during-run progress label can name the current
        # timestep (its Hz + dt) instead of a generic sequence.
        self._dt_sweep_level_dts = [float(dt) for dt in dt_levels]
        self._dt_sweep_level_index = 0
        self._test_start_time = time.time()
        self._test_elapsed_sim = 0.0
        self._test_effort_history = []
        self._test_effort_times = []
        self._set_test_progress_fraction(0.0)
        for pct_label in self._test_progress_pct_labels:
            pct_label.text = "0%"
        if self._accordions_container:
            self._accordions_container.visible = False
        for panel in self._test_in_progress_panels:
            panel.visible = True

        per_joint_sweep: dict[int, list[dict]] = {idx: [] for idx in joint_indices}
        # Per-level snapshots of the recorded trajectory and effort so the retained
        # Position / Effort charts can show the TARGET dt level (not the last/finest
        # level that would otherwise overwrite the tuner's plot arrays).
        level_trajectories: dict[int, tuple] = {}
        level_efforts: dict[int, tuple[list, list]] = {}

        # Suppress the render-step frame rebuilds while the sweep repeatedly stops and
        # plays the timeline (a rebuild would reset the test controls mid-sweep).
        self._restarting_for_override = True
        # Probe projected-force availability once for the whole sweep (see
        # _update_gains_test) so a backend that lacks it warns once, not per level.
        self._projected_forces_unavailable = False
        try:
            for level_index, dt_val in enumerate(dt_levels):
                dt_val = float(dt_val)
                self._dt_sweep_level_index = level_index
                if self._discretization_cancel_requested:
                    for idx in joint_indices:
                        per_joint_sweep[idx].append(
                            {
                                "dt": dt_val,
                                "settling_time": None,
                                "steady_state_error": float("nan"),
                                "settled": False,
                                "skipped": True,
                            }
                        )
                    continue

                self._timeline.stop()
                for _ in range(3):
                    await app.next_update_async()

                SimulationManager.set_physics_dt(dt_val)
                await app.next_update_async()

                self._timeline.play()
                for _ in range(3):
                    await app.next_update_async()

                self._gains_tuner.initialize()
                await app.next_update_async()

                level_params = dict(test_params)
                level_params["test_mode"] = GainsTestMode.DISCRETIZATION
                self._gains_tuner.initialize_gains_test(level_params)
                # Reset per-level effort capture so each level's Effort chart data is
                # isolated (the physics-step callback appends into these).
                self._test_effort_history = []
                self._test_effort_times = []
                self._test_running = True

                while self._test_running:
                    if self._discretization_cancel_requested:
                        self._gains_tuner.stop_test()
                        self._test_running = False
                        break
                    await app.next_update_async()

                level_metrics = self._gains_tuner.get_test_result_metrics() or {}
                for idx in joint_indices:
                    m = level_metrics.get(idx, {})
                    per_joint_sweep[idx].append(
                        {
                            "dt": dt_val,
                            "settling_time": m.get("settling_time"),
                            "steady_state_error": m.get("steady_state_error", float("nan")),
                            "settled": m.get("settled", False),
                            "skipped": False,
                        }
                    )
                # Snapshot this level's recorded trajectory + effort so the target
                # level can be restored for the Position / Effort charts after the
                # sweep (later levels would otherwise overwrite the tuner's arrays).
                level_trajectories[level_index] = self._gains_tuner.snapshot_recorded_trajectory()
                level_efforts[level_index] = self._rebased_effort_snapshot()
        finally:
            self._timeline.stop()
            for _ in range(3):
                await app.next_update_async()
            SimulationManager.set_physics_dt(original_dt)
            await app.next_update_async()
            self._restarting_for_override = False
            self._discretization_sweep_active = False
            cancelled = self._discretization_cancel_requested
            self._discretization_cancel_requested = False
            self._test_running = False

        if cancelled:
            # The cancel handler already reset the button and stopped the timeline.
            return

        combined_metrics = gain_tuner.aggregate_dt_sweep(
            per_joint_sweep, target_dt, settle_threshold, error_threshold, tolerance
        )

        # Retain the TARGET dt level's trajectory + effort for the Position / Effort
        # charts (each level overwrote the tuner's plot arrays, leaving the finest
        # level; restore the level that matches the user's target dt instead).  The
        # aggregated metrics and the target level's trajectory are installed through
        # the tuner's public ingest API rather than by poking its private arrays.
        target_index = gain_tuner.select_target_level_index(dt_levels, target_dt)
        self._gains_tuner.ingest_sweep_results(combined_metrics, level_trajectories.get(target_index))
        if target_index in level_efforts:
            self._test_effort_history, self._test_effort_times = level_efforts[target_index]

        self._last_test_mode = GainsTestMode.DISCRETIZATION

        # Completion UI (mirrors the single-run done-branch in `_update_gains_test`).
        self._set_test_progress_complete()
        self._reset_test_button()
        self._reset_ui_next_frame = True
        self._make_plot_on_next_frame = True
        if self._joint_detail_frame:
            try:
                self._joint_detail_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass
        if self._charts_button:
            self._charts_button.enabled = True
        if self._test_info_frame:
            try:
                self._test_info_frame.rebuild()
            except _UI_GUARD_ERRORS:
                pass
            self._test_info_frame.collapsed = False
        self._on_nav_charts()
