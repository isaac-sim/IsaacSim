# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides UI widgets for configuring joint test parameters in the gains tuner interface."""

from enum import IntEnum
from functools import partial

import numpy as np
import omni.ui as ui
import pxr
from omni.physics.tensors import DofType

from ..gains_tuner import GainsTestMode
from .base_table_widget import ITEM_HEIGHT, TableItem, TableItemDelegate, TableModel, TableWidget
from .cell_widget import CellLabelField
from .joint_table_widget import is_joint_mimic


class ColumnIndex(IntEnum):
    """Enumeration defining column indices for the test joint table widget.

    This enum maps column positions to their corresponding data types in the joint testing interface,
    including joint identification, test parameters, and motion characteristics for gain tuning operations.
    """

    JOINT = 0
    """Column index for the joint name display."""
    TEST = 1
    """Column index for the test enable/disable checkbox."""
    SEQUENCE = 2
    """Column index for the test sequence number."""
    AMPLITUDE = 3
    """Column index for the sinusoidal test amplitude value."""
    OFFSET = 4
    """Column index for the position offset value."""
    PERIOD = 5
    """Column index for the test period duration."""
    PHASE = 6
    """Column index for the test phase shift value."""
    STEP_MAX = 7
    """Column index for the maximum step value in step tests."""
    STEP_MIN = 8
    """Column index for the minimum step value in step tests."""
    USER_PROVIDED = 9  # User provided gains is still not implemented.
    """Column index for user-provided test data source."""


class TestMode(IntEnum):
    """Enumeration defining the different test modes available for joint testing.

    This enumeration specifies the types of test signals that can be applied to joints during gain tuning.
    Each mode represents a different mathematical function used to generate reference trajectories for testing
    joint control performance.
    """

    SINUSOIDAL = 0
    """Sinusoidal test mode for joint movement patterns."""
    STEP = 1
    """Step test mode for joint movement patterns."""
    USER_PROVIDED = 2
    """User-provided test mode for custom joint movement patterns."""
    SNAP_TO_LIMITS = 3
    """Snap-to-limits test mode that commands joints to their lower and upper limits."""
    STRESS_TEST = 4
    """Stress test that bombards joints with extreme random commands."""


class TestJointItem(TableItem):
    """Represents a joint item in the test configuration table for gain tuning.

    This class encapsulates the configuration parameters for testing individual joints during gain tuning
    operations. It manages test parameters like amplitude, period, phase, and step values, while handling
    the conversion between radians and degrees for rotational joints. The item provides properties for
    adjusting test parameters and maintains references to UI models for interactive editing.

    Args:
        name: Display name of the joint.
        joint_index: Index of the joint in the articulation.
        test: Whether this joint should be included in testing.
        sequence: Order sequence for testing this joint.
        dof_type: Type of degree of freedom (rotation or translation).
        amplitude: Amplitude value for sinusoidal test patterns.
        offset: Offset value applied to the joint during testing.
        period: Period duration for test patterns in seconds.
        phase: Phase shift for test patterns in seconds.
        step_max: Maximum step value for step test patterns.
        step_min: Minimum step value for step test patterns.
        user_provided: User-provided data source for custom test patterns.
        model: Parent model that manages this joint item.
        value_changed_fn: Callback function triggered when values change.
    """

    def __init__(
        self,
        name: str,
        joint_index: int,
        test: bool,
        sequence: int,
        dof_type: object,
        amplitude: float,
        offset: float,
        period: float,
        phase: float,
        step_max: float,
        step_min: float,
        user_provided: str,
        model: object,
        value_changed_fn: callable = None,
    ) -> None:
        super().__init__(joint_index, value_changed_fn)
        self.model = model
        self.dof_type = dof_type
        self.values_scale = 1.0
        if dof_type == DofType.Rotation:
            self.values_scale = 180.0 / np.pi
        if np.isinf(step_max) or step_max > 1e10:
            step_max = float("inf")
        else:
            step_max = step_max * self.values_scale
        if np.isinf(step_min) or step_min < -1e10:
            step_min = float("-inf")
        else:
            step_min = step_min * self.values_scale
        self.model_cols = [
            ui.SimpleStringModel(name),
            ui.SimpleBoolModel(test),
            ui.SimpleIntModel(joint_index + 1),
            ui.SimpleFloatModel(amplitude),
            ui.SimpleFloatModel(offset * self.values_scale),
            ui.SimpleFloatModel(period),
            ui.SimpleFloatModel(phase),
            ui.SimpleFloatModel(step_max),
            ui.SimpleFloatModel(step_min),
            ui.SimpleStringModel(user_provided),
        ]
        self.joint_index = joint_index
        for i in range(1, 9):
            self.model_cols[i].add_value_changed_fn(partial(self._on_value_changed, adjusted_col_id=i))

        # Add Config update callbacs
        self.model_cols[ColumnIndex.SEQUENCE].add_value_changed_fn(
            partial(
                self.on_update_sequence,
            )
        )
        self.model_cols[ColumnIndex.AMPLITUDE].add_value_changed_fn(
            partial(
                self.on_update_amplitude,
            )
        )
        self.model_cols[ColumnIndex.PERIOD].add_value_changed_fn(
            partial(
                self.on_update_period,
            )
        )
        self.model_cols[ColumnIndex.OFFSET].add_value_changed_fn(
            partial(
                self.on_update_offset,
            )
        )
        self.model_cols[ColumnIndex.PHASE].add_value_changed_fn(
            partial(
                self.on_update_phase,
            )
        )
        self.model_cols[ColumnIndex.STEP_MAX].add_value_changed_fn(
            partial(
                self.on_update_step_max,
            )
        )
        self.model_cols[ColumnIndex.STEP_MIN].add_value_changed_fn(
            partial(
                self.on_update_step_min,
            )
        )
        self.model_cols[ColumnIndex.USER_PROVIDED].add_value_changed_fn(
            partial(
                self.on_update_user_provided,
            )
        )
        self.model_cols[ColumnIndex.TEST].add_value_changed_fn(
            partial(
                self.on_update_test,
            )
        )
        self.model_cols[ColumnIndex.SEQUENCE].add_value_changed_fn(
            partial(
                self.on_update_sequence,
            )
        )
        self.value_field = {}
        self.mode = GainsTestMode.SINUSOIDAL

    def on_update_position(self, model: object, *args: object) -> None:
        """Updates the joint position value in the model.

        Args:
            model: The model containing the position value.
            *args: Additional arguments passed to the callback.
        """
        self.model.joint_positions[self.joint_index] = model.get_value_as_float()

    def on_update_velocity(self, model: object, *args: object) -> None:
        """Updates the maximum velocity value in the model.

        Args:
            model: The model containing the velocity value.
            *args: Additional arguments passed to the callback.
        """
        self.model.v_max[self.joint_index] = model.get_value_as_float()

    def on_update_step_max(self, model: object, *args: object) -> None:
        """Updates the maximum step value for the joint test.

        Args:
            model: The model containing the step maximum value.
            *args: Additional arguments passed to the callback.
        """

    def on_update_step_min(self, model: object, *args: object) -> None:
        """Updates the minimum step value for the joint test.

        Args:
            model: The model containing the step minimum value.
            *args: Additional arguments passed to the callback.
        """

    def on_update_sequence(self, model: object, *args: object) -> None:
        """Updates the test sequence value for the joint.

        Args:
            model: The model containing the sequence value.
            *args: Additional arguments passed to the callback.
        """

    def on_update_amplitude(self, model: object, *args: object) -> None:
        """Updates the amplitude value for the joint test.

        Args:
            model: The model containing the amplitude value.
            *args: Additional arguments passed to the callback.
        """

    def on_update_period(self, model: object, *args: object) -> None:
        """Updates the period value for the joint test.

        Args:
            model: The model containing the period value.
            *args: Additional arguments passed to the callback.
        """

    def on_update_offset(self, model: object, *args: object) -> None:
        """Updates the offset value for the joint test.

        Args:
            model: The model containing the offset value.
            *args: Additional arguments passed to the callback.
        """

    def on_update_phase(self, model: object, *args: object) -> None:
        """Updates the phase value for the joint test.

        Args:
            model: The model containing the phase value.
            *args: Additional arguments passed to the callback.
        """

    def on_update_test(self, model: object, *args: object) -> None:
        """Updates the test enable/disable state for the joint.

        Args:
            model: The model containing the test state value.
            *args: Additional arguments passed to the callback.
        """

    def on_update_user_provided(self, model: object, *args: object) -> None:
        """Handles updates to the user-provided data source parameter.

        Args:
            model: The model that triggered the update.
            *args: Variable length argument list for additional parameters.
        """

    @property
    def test(self) -> bool:
        """Whether this joint is enabled for testing.

        Returns:
            True if the joint is enabled for testing.
        """
        return self.model_cols[ColumnIndex.TEST].get_value_as_bool()

    @test.setter
    def test(self, value: bool) -> None:
        self.model_cols[ColumnIndex.TEST].set_value(value)
        self.model._item_changed(self)

    @property
    def amplitude(self) -> float:
        """Amplitude value for the sinusoidal test motion.

        Returns:
            The amplitude value as a float.
        """
        return self.model_cols[ColumnIndex.AMPLITUDE].get_value_as_float()

    @amplitude.setter
    def amplitude(self, value: float) -> None:
        self.model_cols[ColumnIndex.AMPLITUDE].set_value(value)

    @property
    def step_max(self) -> float:
        """Maximum step value for the test motion.

        Returns:
            The maximum step value as a float.
        """
        return self.model_cols[ColumnIndex.STEP_MAX].get_value_as_float()

    @step_max.setter
    def step_max(self, value: float) -> None:
        self.model_cols[ColumnIndex.STEP_MAX].set_value(value)

    @property
    def step_min(self) -> float:
        """Minimum step value for the test motion.

        Returns:
            The minimum step value as a float.
        """
        return self.model_cols[ColumnIndex.STEP_MIN].get_value_as_float()

    @step_min.setter
    def step_min(self, value: float) -> None:
        self.model_cols[ColumnIndex.STEP_MIN].set_value(value)

    @property
    def period(self) -> float:
        """Period duration for the test motion in seconds.

        Returns:
            The period value as a float.
        """
        return self.model_cols[ColumnIndex.PERIOD].get_value_as_float()

    @period.setter
    def period(self, value: float) -> None:
        self.model_cols[ColumnIndex.PERIOD].set_value(value)

    @property
    def phase(self) -> float:
        """Phase offset for the test motion in seconds.

        Returns:
            The phase value as a float.
        """
        return self.model_cols[ColumnIndex.PHASE].get_value_as_float()

    @phase.setter
    def phase(self, value: float) -> None:
        self.model_cols[ColumnIndex.PHASE].set_value(value)

    @property
    def offset(self) -> float:
        """Offset value for the test motion position.

        Returns:
            The offset value as a float.
        """
        return self.model_cols[ColumnIndex.OFFSET].get_value_as_float()

    @offset.setter
    def offset(self, value: float) -> None:
        self.model_cols[ColumnIndex.OFFSET].set_value(value)

    @property
    def sequence(self) -> int:
        """Sequence order for the joint test execution.

        Returns:
            The sequence number as an integer.
        """
        return self.model_cols[ColumnIndex.SEQUENCE].get_value_as_int()

    @sequence.setter
    def sequence(self, value: int) -> None:
        self.model_cols[ColumnIndex.SEQUENCE].set_value(value)

    @property
    def user_provided(self) -> str:
        """User-provided data source for the test motion.

        Returns:
            The user-provided data source as a string.
        """
        return self.model_cols[ColumnIndex.USER_PROVIDED].get_value_as_string()

    @user_provided.setter
    def user_provided(self, value: str) -> None:
        self.model_cols[ColumnIndex.USER_PROVIDED].set_value(value)

    def get_item_value(self, col_id: int = 0) -> str | bool | float:
        """Retrieves the value for the specified column.

        Args:
            col_id: Column index to get the value from.

        Returns:
            The value from the column as a string for column 0, bool for column 1, or float for other columns.
        """
        if col_id == 0:
            return self.model_cols[col_id].get_value_as_string()
        elif col_id == 1:
            return self.model_cols[col_id].get_value_as_bool()
        return self.model_cols[col_id].get_value_as_float()

    def set_item_value(self, col_id: int, value: object) -> None:
        """Sets the value for the specified column.

        Args:
            col_id: Column index to set the value for.
            value: The value to set in the column.
        """
        self.model_cols[col_id].set_value(value)

    def get_value_model(self, col_id: int = 0) -> object:
        """Gets the UI model for the specified column.

        Args:
            col_id: Column index to get the model from.

        Returns:
            The UI model for the specified column.
        """
        return self.model_cols[col_id]


class TestJointItemDelegate(TableItemDelegate):
    """A delegate class that handles the rendering and interaction of test joint configuration items in a table view.

    This delegate is responsible for creating and managing UI widgets for each column of the test joint table,
    including joint names, test checkboxes, sequence numbers, amplitude controls, offset fields, period settings,
    phase adjustments, step limits, and data source specifications. It provides column headers with tooltips
    and handles different test modes (sinusoidal, step, user-provided) by dynamically showing or hiding
    relevant columns based on the current mode.

    The delegate creates interactive widgets such as checkboxes for enabling/disabling joint tests, drag fields
    for numeric parameters with appropriate units and limits, and string fields for custom data sources.
    It supports bulk editing operations and maintains proper styling through the Omni UI framework.

    Args:
        model: The TestJointModel instance that provides the data and structure for the table view.
    """

    header_tooltip = {
        ColumnIndex.JOINT: "Joint",
        ColumnIndex.TEST: "Test",
        ColumnIndex.SEQUENCE: "Sequence",
        ColumnIndex.AMPLITUDE: "Amplitude",
        ColumnIndex.OFFSET: "Offset",
        ColumnIndex.PERIOD: "Period",
        ColumnIndex.PHASE: "Phase",
        ColumnIndex.STEP_MAX: "Step Max",
        ColumnIndex.STEP_MIN: "Step Min",
        ColumnIndex.USER_PROVIDED: "Data Source",
    }
    """Dictionary mapping column indices to their tooltip text displayed in the table header."""
    header = {
        ColumnIndex.JOINT: "Joint",
        ColumnIndex.TEST: "Test",
        ColumnIndex.SEQUENCE: "Sequence",
        ColumnIndex.AMPLITUDE: "Amplitude",
        ColumnIndex.OFFSET: "Offset",
        ColumnIndex.PERIOD: "Period",
        ColumnIndex.PHASE: "Phase",
        ColumnIndex.STEP_MAX: "Step Max",
        ColumnIndex.STEP_MIN: "Step Min",
        ColumnIndex.USER_PROVIDED: "Data Source",
    }
    """Dictionary mapping column indices to their display text shown in the table header."""

    def __init__(self, model: object) -> None:

        self.column_headers = {}
        super().__init__(model)

    def init_model(self) -> None:
        """Initializes the model with default test mode settings."""
        self.mode = TestMode.SINUSOIDAL

    @property
    def mode(self) -> int:
        """Current test mode of the delegate.

        Returns:
            The test mode from the underlying model.
        """
        return self._model.mode

    @mode.setter
    def mode(self, mode: int) -> None:
        self._model.mode = mode

    def build_branch(
        self, model: object, item: object = None, column_id: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        """Builds branch widgets for tree view items.

        Args:
            model: The model containing the data.
            item: The item to build the branch for.
            column_id: Column identifier for the branch.
            level: Hierarchy level of the item.
            expanded: Whether the branch is expanded.
        """

    def model_col_id(self, column_id: int) -> int | None:
        """Maps display column ID to model column ID based on current mode.

        Args:
            column_id: The display column identifier.

        Returns:
            The corresponding model column ID, or None if invalid.
        """
        if column_id < len(self._model.column_id_map[self.mode]):
            return self._model.column_id_map[self.mode][column_id]
        return None

    def build_header(self, column_id: int = 0) -> None:
        """Builds header widgets for table columns.

        Args:
            column_id: Column identifier for the header.
        """
        if self.model_col_id(column_id) is None:
            return
        model_col_id = self.model_col_id(column_id)
        with ui.ZStack(style_type_name_override="TreeView"):
            ui.Rectangle(
                name="Header",
                style_type_name_override="TreeView",
            )
            alignment = ui.Alignment.CENTER
            offset = 10
            if model_col_id in [ColumnIndex.JOINT, ColumnIndex.SEQUENCE]:
                alignment = ui.Alignment.LEFT
            if model_col_id in [ColumnIndex.AMPLITUDE, ColumnIndex.PERIOD, ColumnIndex.PHASE, ColumnIndex.OFFSET]:
                offset = 0
            with ui.HStack():
                ui.Spacer(width=offset)
                if model_col_id not in [ColumnIndex.JOINT, ColumnIndex.SEQUENCE, ColumnIndex.TEST]:
                    ui.Spacer()
                with ui.VStack():
                    ui.Spacer(height=ui.Pixel(3))
                    ui.Label(
                        TestJointItemDelegate.header[model_col_id],
                        tooltip=TestJointItemDelegate.header_tooltip[model_col_id],
                        name="header",
                        style_type_name_override="TreeView",
                        # elided_text=True,
                        alignment=alignment,
                    )
                ui.Spacer()
                self.build_sort_button(column_id)

    def update_defaults(self) -> None:
        """Updates default values for all value fields in the model items."""
        for item in self.__model.get_item_children():
            for i in [1, 2, 3, 4, 5]:
                field = item.value_field.get(i)
                if field:
                    field.update_default_value()

    def build_widget(
        self, model: object, item: object = None, index: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        """Builds UI widgets for table cells based on column type and test mode.

        Args:
            model: The model containing the data.
            item: The item to build the widget for.
            index: Column index for the widget.
            level: Hierarchy level of the item.
            expanded: Whether the item is expanded.
        """
        if self.model_col_id(index) is None:
            return
        if item:
            item.mode = self.mode
            model_col_id = self.model_col_id(index)
            if model_col_id == ColumnIndex.JOINT:
                with ui.ZStack(height=ITEM_HEIGHT, style_type_name_override="TreeView"):
                    ui.Rectangle(name="treeview_first_item")
                    with ui.VStack():
                        ui.Spacer()
                        with ui.HStack(height=0):
                            ui.Label(
                                str(model.get_item_value(item, model_col_id)),
                                tooltip=model.get_item_value(item, model_col_id),
                                name="treeview_item",
                                elided_text=True,
                                height=0,
                            )
                            ui.Spacer(width=1)
                        ui.Spacer()

            if model_col_id == ColumnIndex.TEST:
                with ui.ZStack(height=ITEM_HEIGHT, style_type_name_override="TreeView"):
                    ui.Rectangle(name="treeview_first_item")
                    with ui.VStack():
                        ui.Spacer()
                        with ui.HStack(height=0):
                            ui.Spacer()
                            check_box = ui.CheckBox(width=10, height=0)
                            check_box.model.set_value(item.test)

                            def on_click(value_model: object) -> None:
                                item.test = value_model.get_value_as_bool()

                            check_box.model.add_value_changed_fn(on_click)
                            ui.Spacer()
                        ui.Spacer()
            if model_col_id == ColumnIndex.SEQUENCE:
                with ui.ZStack(height=ITEM_HEIGHT):
                    ui.Rectangle(name="treeview_item")
                    with ui.VStack():
                        ui.Spacer()
                        item.value_field[model_col_id] = CellLabelField(
                            model.get_item_value_model(item, model_col_id), ui.IntDrag, "%d"
                        )
                        model.get_item_value_model(item, model_col_id)
                        item.value_field[model_col_id].field.min = 1
                        item.value_field[model_col_id].field.max = len(self.get_children())
                        ui.Spacer()
            elif model_col_id in [
                ColumnIndex.AMPLITUDE,
                ColumnIndex.PERIOD,
                ColumnIndex.PHASE,
                ColumnIndex.OFFSET,
                ColumnIndex.STEP_MAX,
                ColumnIndex.STEP_MIN,
            ]:
                min_value = 0
                max_value = 100.0
                if model_col_id in [ColumnIndex.STEP_MAX, ColumnIndex.STEP_MIN, ColumnIndex.OFFSET]:
                    min_value = item.step_min
                    max_value = item.step_max

                unit = "deg"

                if model_col_id in [ColumnIndex.AMPLITUDE]:
                    unit = "%"
                elif model_col_id == ColumnIndex.PERIOD:
                    unit = "s"
                elif model_col_id == ColumnIndex.PHASE:
                    unit = "s"

                with ui.ZStack(height=ITEM_HEIGHT):
                    ui.Rectangle(name="treeview_item")
                    with ui.VStack():
                        ui.Spacer()
                        item.value_field[model_col_id] = CellLabelField(
                            model.get_item_value_model(item, model_col_id), ui.FloatDrag, "%.2f"
                        )
                        item.value_field[model_col_id].field.min = min_value
                        item.value_field[model_col_id].field.max = max_value
                        ui.Spacer()
                    if unit:
                        with ui.VStack():
                            ui.Spacer()
                            with ui.HStack():
                                ui.Spacer()
                                ui.Label(unit, width=0, name="unit")
                                ui.Spacer(width=10)
                            ui.Spacer()
            elif model_col_id == ColumnIndex.USER_PROVIDED:
                with ui.ZStack(height=ITEM_HEIGHT):
                    ui.Rectangle(name="treeview_item")
                    with ui.VStack():
                        ui.Spacer()
                        item.value_field[index] = CellLabelField(
                            model.get_item_value_model(item, model_col_id), ui.StringField, "%s"
                        )


class TestJointModel(TableModel):
    """Model for managing joint test configurations in the gains tuner.

    This model provides a data structure for storing and managing test parameters for individual joints during
    gains tuning operations. It supports multiple test modes including sinusoidal, step, and user-provided data
    modes, with appropriate parameter sets for each mode.

    The model automatically filters joints to include only testable joints (excluding fixed joints and mimic
    joints) and initializes test parameters with appropriate defaults based on joint limits and DOF types.
    Rotational joints have their values scaled to degrees for user interface purposes.

    Args:
        gains_tuner: The gains tuner instance that provides access to articulation and joint information.
        value_changed_fn: Callback function invoked when joint test parameters are modified.
        **kwargs: Additional keyword arguments passed to the parent TableModel class.
    """

    def __init__(self, gains_tuner: object, value_changed_fn: callable, **kwargs: object) -> None:
        self.column_id_map = {
            TestMode.SINUSOIDAL: [
                ColumnIndex.JOINT,
                ColumnIndex.TEST,
                ColumnIndex.SEQUENCE,
                ColumnIndex.AMPLITUDE,
                ColumnIndex.OFFSET,
                ColumnIndex.PERIOD,
                ColumnIndex.PHASE,
            ],
            TestMode.STEP: [
                ColumnIndex.JOINT,
                ColumnIndex.TEST,
                ColumnIndex.SEQUENCE,
                ColumnIndex.STEP_MIN,
                ColumnIndex.STEP_MAX,
                ColumnIndex.PERIOD,
                ColumnIndex.PHASE,
            ],
            TestMode.USER_PROVIDED: [ColumnIndex.JOINT, ColumnIndex.TEST, ColumnIndex.USER_PROVIDED],
            TestMode.SNAP_TO_LIMITS: [ColumnIndex.JOINT, ColumnIndex.TEST, ColumnIndex.SEQUENCE],
            TestMode.STRESS_TEST: [ColumnIndex.JOINT, ColumnIndex.TEST, ColumnIndex.SEQUENCE],
        }
        super().__init__(value_changed_fn)
        self._articulation = gains_tuner.get_articulation()
        self._gains_tuner = gains_tuner
        self.lower_joint_limits, self.upper_joint_limits = [i[0].list() for i in self._articulation.get_dof_limits()]

        self.joint_indices = [
            joint_index
            for joint_index in self._gains_tuner.get_all_joint_indices()
            if self.is_joint_testable(joint_index)
        ]

        self._children = [
            TestJointItem(
                name=self._gains_tuner._joint_entries[i].display_name,
                joint_index=joint_index,
                test=True,
                sequence=i,
                dof_type=self._gains_tuner.get_dof_type(joint_index),
                amplitude=100.0,
                offset=0.0,
                period=1.0,
                phase=0.0,
                step_max=self.upper_joint_limits[joint_index],
                step_min=self.lower_joint_limits[joint_index],
                user_provided="",
                model=self,
                value_changed_fn=self._on_joint_changed,
            )
            for i, joint_index in enumerate(self.joint_indices)
        ]

    def is_joint_testable(self, joint_index: int) -> bool:
        """Determines if a joint at the given index can be tested.

        Args:
            joint_index: Index of the joint to check.

        Returns:
            True if the joint is testable (not a fixed joint or mimic joint), False otherwise.
        """
        joint = self._gains_tuner._joint_entries[joint_index].joint
        return not (pxr.UsdPhysics.FixedJoint(joint) or is_joint_mimic(joint))

    @property
    def mode(self) -> GainsTestMode:
        """Current test mode for the joints.

        Returns:
            The test mode enumeration value.
        """
        return self._mode

    @mode.setter
    def mode(self, mode: int) -> None:
        self._mode = mode
        self._item_changed(None)

    def init_model(self) -> None:
        """Initializes the model by setting the test mode to sinusoidal."""
        self.mode = GainsTestMode.SINUSOIDAL

    def get_item_value_model_count(self, item: object) -> int:
        """The number of columns based on the current test mode.

        Args:
            item: The table item to get column count for.

        Returns:
            Number of columns for the current test mode.
        """
        if self.mode in self.column_id_map:
            return len(self.column_id_map[self.mode])
        return 1


class TestJointWidget(TableWidget):
    """A widget for configuring joint test parameters in the gains tuner interface.

    Provides a table-based interface for setting up joint testing configurations including test modes
    (sinusoidal, step, user-provided), amplitudes, periods, phases, and other parameters. The widget
    allows users to select which joints to test and configure their specific test parameters through
    an interactive table view.

    Supports bulk editing functionality where changes to one selected joint can be applied to multiple
    joints simultaneously. The widget automatically filters out non-testable joints (fixed joints and
    mimic joints) and provides appropriate input controls based on the joint's degrees of freedom type.

    Args:
        gains_tuner: The gains tuner instance that manages the articulation and joint configurations.
        value_changed_fn: Callback function invoked when joint test parameters are modified.
    """

    def __init__(self, gains_tuner: object, value_changed_fn: callable = None) -> None:
        self.column_widths = [
            ui.Fraction(1),
            ui.Pixel(50),
            ui.Pixel(50),
            ui.Pixel(110),
            ui.Pixel(110),
            ui.Pixel(110),
            ui.Pixel(110),
            ui.Pixel(110),
            ui.Pixel(110),
            ui.Fraction(1),
        ]
        self._last_multi_selection = []
        if gains_tuner.get_articulation():
            model = TestJointModel(gains_tuner, self._on_value_changed)
            delegate = TestJointItemDelegate(model)
            mode = GainsTestMode.SINUSOIDAL
            super().__init__(value_changed_fn, model, delegate, mode)
            self._enable_bulk_edit = True

    def switch_mode(self, mode: object) -> None:
        """Switches the test mode and updates the tree view column widths.

        Args:
            mode: The test mode to switch to.
        """
        super().switch_mode(mode)
        if self.list:
            self.list.column_widths = [self.column_widths[i] for i in self.model.column_id_map[self.model.mode]]

    # def switch_radian_degree(self, radian_degree_mode):
    #     # TODO: Implement this
    #     carb.log_error("switch_radian_degree is not implemented")

    def _on_value_changed(self, joint_item: object, col_id: int = 1, adjusted_col_id: int = None) -> None:
        """Handles value changes in joint test parameters.

        When bulk edit is enabled, propagates the change to all selected items.
        If the TreeView selection was reduced to a single item by a widget click
        but a prior multi-selection included the edited item, that multi-selection
        is used instead so bulk edits are not lost.

        Args:
            joint_item: The joint item that had its value changed.
            col_id: The column ID in the current view.
            adjusted_col_id: The actual column ID if different from col_id.
        """
        if adjusted_col_id is None:
            value_col_id = self.model.column_id_map[self.model.mode][col_id]
        else:
            value_col_id = adjusted_col_id
        if self._enable_bulk_edit:
            selection = list(self.list.selection)
            if len(selection) <= 1 and len(self._last_multi_selection) > 1 and joint_item in self._last_multi_selection:
                selection = self._last_multi_selection
            elif joint_item not in selection:
                self.list.selection = [joint_item]
                selection = [joint_item]
            self.set_bulk_edit(False)
            for item in selection:
                if item is not joint_item:
                    item.set_item_value(value_col_id, joint_item.get_item_value(value_col_id))
                    self.model._item_changed(item)
            self.set_bulk_edit(True)
            self._last_multi_selection = []
        if self._value_changed_fn:
            self._value_changed_fn(joint_item.joint)

    def build_tree_view(self) -> None:
        """Builds the tree view widget for joint test parameters."""
        self._last_multi_selection = []
        self.list = ui.TreeView(
            self.model,
            delegate=self.delegate,
            alignment=ui.Alignment.LEFT_TOP,
            column_widths=[self.column_widths[i] for i in self.model.column_id_map[self.model.mode]],
            min_column_widths=[50, 30, 30, 80, 80, 80, 80, 80, 80, 310],
            columns_resizable=True,
            header_visible=True,
            width=ui.Fraction(1),
            height=ui.Fraction(1),
        )
        self.list.set_selection_changed_fn(self._on_tree_selection_changed)

    def _on_tree_selection_changed(self, selection: list) -> None:
        """Tracks multi-selections so bulk edit survives widget click focus changes.

        Args:
            selection: Current tree view selection.
        """
        if len(selection) > 1:
            self._last_multi_selection = list(selection)

    def select_all(self) -> None:
        """Selects all joints for testing by setting their test property to True."""
        for item in self.model.get_item_children():
            item.test = True

    def clear_all(self) -> None:
        """Deselects all joints from testing by setting their test property to False."""
        for item in self.model.get_item_children():
            item.test = False
