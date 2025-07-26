# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from enum import Enum, IntEnum
from functools import partial

import carb
import numpy as np
import omni.physics.tensors as physics_tensors
import omni.ui as ui
import pxr

from ..gains_tuner import GainsTestMode
from .base_table_widget import ITEM_HEIGHT, TableItem, TableItemDelegate, TableModel, TableWidget
from .cell_widget import CellLabelField
from .joint_table_widget import is_joint_mimic
from .style import get_style


class ColumnIndex(IntEnum):
    JOINT = 0
    TEST = 1
    SEQUENCE = 2
    AMPLITUDE = 3
    OFFSET = 4
    PERIOD = 5
    PHASE = 6
    STEP_MAX = 7
    STEP_MIN = 8
    USER_PROVIDED = 9  # User provided gains is still not implemented.


class TestMode(IntEnum):
    SINUSOIDAL = 0
    STEP = 1
    USER_PROVIDED = 2


class TestJointItem(TableItem):

    def __init__(
        self,
        name,
        joint_index,
        test,
        sequence,
        dof_type,
        amplitude,
        offset,
        period,
        phase,
        step_max,
        step_min,
        user_provided,
        model,
        value_changed_fn=None,
    ):
        super().__init__(joint_index, value_changed_fn)
        self.model = model
        self.dof_type = dof_type
        self.values_scale = 1.0
        if dof_type == physics_tensors.DofType.Rotation:
            self.values_scale = 180.0 / np.pi
        self.model_cols = [
            ui.SimpleStringModel(name),
            ui.SimpleBoolModel(test),
            ui.SimpleIntModel(joint_index + 1),
            ui.SimpleFloatModel(amplitude),
            ui.SimpleFloatModel(offset * self.values_scale),
            ui.SimpleFloatModel(period),
            ui.SimpleFloatModel(phase),
            ui.SimpleFloatModel(step_max * self.values_scale),
            ui.SimpleFloatModel(step_min * self.values_scale),
            ui.SimpleStringModel(user_provided),
        ]
        self.joint_index = joint_index
        for i in range(1, 8):
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

    def on_update_position(self, model, *args):
        self.model.joint_positions[self.joint_index] = model.get_value_as_float()

    def on_update_velocity(self, model, *args):
        self.model.v_max[self.joint_index] = model.get_value_as_float()

    def on_update_step_max(self, model, *args):
        pass

    def on_update_step_min(self, model, *args):
        pass

    def on_update_sequence(self, model, *args):
        pass

    def on_update_amplitude(self, model, *args):
        pass

    def on_update_period(self, model, *args):
        pass

    def on_update_offset(self, model, *args):
        pass

    def on_update_phase(self, model, *args):
        pass

    def on_update_test(self, model, *args):
        pass

    def on_update_user_provided(self, model, *args):
        pass

    @property
    def test(self):
        return self.model_cols[ColumnIndex.TEST].get_value_as_bool()

    @test.setter
    def test(self, value: bool):
        self.model_cols[ColumnIndex.TEST].set_value(value)
        self.model._item_changed(self)

    @property
    def amplitude(self):
        return self.model_cols[ColumnIndex.AMPLITUDE].get_value_as_float()

    @amplitude.setter
    def amplitude(self, value: float):
        self.model_cols[ColumnIndex.AMPLITUDE].set_value(value)

    @property
    def step_max(self):
        return self.model_cols[ColumnIndex.STEP_MAX].get_value_as_float()

    @step_max.setter
    def step_max(self, value: float):
        self.model_cols[ColumnIndex.STEP_MAX].set_value(value)

    @property
    def step_min(self):
        return self.model_cols[ColumnIndex.STEP_MIN].get_value_as_float()

    @step_min.setter
    def step_min(self, value: float):
        self.model_cols[ColumnIndex.STEP_MIN].set_value(value)

    @property
    def period(self):
        return self.model_cols[ColumnIndex.PERIOD].get_value_as_float()

    @period.setter
    def period(self, value: float):
        self.model_cols[ColumnIndex.PERIOD].set_value(value)

    @property
    def phase(self):
        return self.model_cols[ColumnIndex.PHASE].get_value_as_float()

    @phase.setter
    def phase(self, value: float):
        self.model_cols[ColumnIndex.PHASE].set_value(value)

    @property
    def offset(self):
        return self.model_cols[ColumnIndex.OFFSET].get_value_as_float()

    @offset.setter
    def offset(self, value: float):
        self.model_cols[ColumnIndex.OFFSET].set_value(value)

    @property
    def sequence(self):
        return self.model_cols[ColumnIndex.SEQUENCE].get_value_as_int()

    @sequence.setter
    def sequence(self, value: int):
        self.model_cols[ColumnIndex.SEQUENCE].set_value(value)

    @property
    def user_provided(self):
        return self.model_cols[ColumnIndex.USER_PROVIDED].get_value_as_string()

    @user_provided.setter
    def user_provided(self, value: str):
        self.model_cols[ColumnIndex.USER_PROVIDED].set_value(value)

    def get_item_value(self, col_id=0):
        if col_id == 0:
            return self.model_cols[col_id].get_value_as_string()
        elif col_id == 1:
            return self.model_cols[col_id].get_value_as_bool()
        return self.model_cols[col_id].get_value_as_float()

    def set_item_value(self, col_id, value):
        self.model_cols[col_id].set_value(value)

    def get_value_model(self, col_id=0):
        return self.model_cols[col_id]


class TestJointItemDelegate(TableItemDelegate):
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

    def __init__(self, model):

        self.column_headers = {}
        super().__init__(model)

    def init_model(self):
        self.mode = TestMode.SINUSOIDAL

    @property
    def mode(self):
        return self._model.mode

    @mode.setter
    def mode(self, mode):
        self._model.mode = mode

    def build_branch(self, model, item=None, column_id=0, level=0, expanded=False):
        pass

    def model_col_id(self, column_id):
        if column_id < len(self._model.column_id_map[self.mode]):
            return self._model.column_id_map[self.mode][column_id]
        return None

    def build_header(self, column_id=0):
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

    def update_defaults(self):
        for item in self.__model.get_item_children():
            for i in [1, 2, 3, 4, 5]:
                field = item.value_field.get(i)
                if field:
                    field.update_default_value()

    def build_widget(self, model, item=None, index=0, level=0, expanded=False):
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

                            def on_click(value_model):
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

    def __init__(self, gains_tuner, value_changed_fn, **kwargs):
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
                name=self._gains_tuner._joint_names[joint_index],
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

    def is_joint_testable(self, joint_index):
        joint_name = self._articulation.dof_names[joint_index]
        joint = self._gains_tuner._joint_map[joint_name]
        return not (pxr.UsdPhysics.FixedJoint(joint) or is_joint_mimic(joint))

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        self._mode = mode
        self._item_changed(None)

    def init_model(self):
        self.mode = GainsTestMode.SINUSOIDAL

    def get_item_value_model_count(self, item):
        """The number of columns"""
        if self.mode == GainsTestMode.SINUSOIDAL:
            return len(self.column_id_map[self.mode])
        if self.mode == GainsTestMode.STEP:
            return len(self.column_id_map[self.mode])
        if self.mode == GainsTestMode.USER_PROVIDED:
            return len(self.column_id_map[self.mode])
        return 1


class TestJointWidget(TableWidget):
    def __init__(self, gains_tuner, value_changed_fn=None):
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
        if gains_tuner.get_articulation():
            model = TestJointModel(gains_tuner, self._on_value_changed)
            delegate = TestJointItemDelegate(model)
            mode = GainsTestMode.SINUSOIDAL
            super().__init__(value_changed_fn, model, delegate, mode)
            self._enable_bulk_edit = True

    def switch_mode(self, mode):
        super().switch_mode(mode)
        if self.list:
            self.list.column_widths = [self.column_widths[i] for i in self.model.column_id_map[self.model.mode]]

    # def switch_radian_degree(self, radian_degree_mode):
    #     # TODO: Implement this
    #     carb.log_error("switch_radian_degree is not implemented")

    def _on_value_changed(self, joint_item, col_id=1, adjusted_col_id=None):
        if adjusted_col_id is None:
            value_col_id = self.model.column_id_map[self.model.mode][col_id]
        else:
            value_col_id = adjusted_col_id
        if self._enable_bulk_edit:
            if joint_item not in self.list.selection:
                self.list.selection = [joint_item]
            self.set_bulk_edit(False)
            for item in self.list.selection:
                if item is not joint_item:
                    item.set_item_value(value_col_id, joint_item.get_item_value(value_col_id))
                    self.model._item_changed(item)
            self.set_bulk_edit(True)
        if self._value_changed_fn:
            self._value_changed_fn(joint_item.joint)

    def build_tree_view(self):

        self.list = ui.TreeView(
            self.model,
            delegate=self.delegate,
            alignment=ui.Alignment.LEFT_TOP,
            column_widths=[self.column_widths[i] for i in self.model.column_id_map[self.model.mode]],
            min_column_widths=[50, 30, 30, 80, 80, 80, 80, 80, 80, 310],
            columns_resizable=True,
            header_visible=True,
            height=ui.Fraction(1),
        )

    def select_all(self):
        for item in self.model.get_item_children():
            item.test = True

    def clear_all(self):
        for item in self.model.get_item_children():
            item.test = False
