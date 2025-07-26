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
from ctypes import alignment
from enum import Enum, IntEnum, auto
from functools import partial
from re import I

import omni.ui as ui
import pxr

from .base_table_widget import TableWidget
from .resetable_widget import ResetableLabelField
from .style import get_style

ITEM_HEIGHT = 28


class WidgetColumns(IntEnum):
    NAME = 0
    DRIVE_MODE = 1
    DRIVE_TYPE = 2
    STIFFNESS = 3
    DAMPING = 4
    NATURAL_FREQUENCY = 5
    DAMPING_RATIO = 6


class SearchableItemSortPolicy(IntEnum):
    """Sort policy for stage items."""

    DEFAULT = 0
    """The default sort policy."""

    A_TO_Z = 1
    """Sort by name from A to Z."""

    Z_TO_A = 2
    """Sort by name from Z to A."""


class JointSettingMode(IntEnum):
    """The mode of setting joint parameters."""

    STIFFNESS = 0
    """Set the joint parameters by stiffness."""

    NATURAL_FREQUENCY = 1
    """Set the joint parameters by natural Frequency."""


class JointDriveType(IntEnum):
    """The mode of setting joint parameters."""

    ACCELERATION = 0
    """Set the joint parameters by acceleration."""

    FORCE = 1
    """Set the joint parameters by force."""

    MIMIC = 2
    """Set the joint parameters by velocity."""

    @classmethod
    def from_token(cls, token):
        if token.lower() == "acceleration":
            return cls.ACCELERATION
        elif token.lower() == "force":
            return cls.FORCE
        elif token.lower() == "":
            return cls.MIMIC
        else:
            raise ValueError(f"Invalid joint drive type: {token}")

    def to_token(self):
        if self == self.ACCELERATION:
            return "acceleration"
        elif self == self.FORCE:
            return "force"
        else:
            raise ValueError(f"Invalid joint drive type: {self}")


class JointDriveMode(Enum):
    """The mode of setting joint parameters."""

    NONE = 0
    """Set the joint parameters by stiffness."""

    POSITION = 1
    """Set the joint parameters by stiffness."""

    VELOCITY = 2
    """Set the joint parameters by stiffness."""

    MIMIC = 0
    """Set the joint parameters by stiffness."""


class ComboListModel(ui.AbstractItemModel):
    class ComboListItem(ui.AbstractItem):
        def __init__(self, item):
            super().__init__()
            self.model = ui.SimpleStringModel(item)

    def __init__(self, item_list, default_index):
        super().__init__()
        self._default_index = default_index.value
        self._current_index = ui.SimpleIntModel(default_index.value)
        self._current_index.add_value_changed_fn(lambda a: self._item_changed(None))
        self._item_list = item_list
        self._items = []
        if item_list:
            for item in item_list:
                self._items.append(ComboListModel.ComboListItem(item))

    def get_item_children(self, item):
        return self._items

    def set_items(self, items):
        self._items = []
        for item in items:
            self._items.append(ComboListModel.ComboListItem(item))

    def get_item_list(self):
        return self._item_list

    def get_item_value_model(self, item=None, column_id=-1):
        if item is None:
            return self._current_index
        return item.model

    def get_current_index(self):
        return self._current_index.get_value_as_int()

    def set_current_index(self, index):
        if index < len(self._items):
            self._current_index.set_value(index)

    def get_value_as_string(self):
        if self._current_index.get_value_as_int() < len(self._items):
            return self._items[self._current_index.get_value_as_int()].model.get_value_as_string()
        return ""

    def is_default(self):
        return self.get_current_index() == self._default_index


def is_joint_mimic(joint):
    """Check if a joint has mimic joint API applied.

    Args:
        joint: The joint to check

    Returns:
        bool: True if joint has MimicJointAPI applied, False otherwise
    """
    return len([a for a in joint.GetAppliedSchemas() if "MimicJointAPI" in a]) > 0


def get_mimic_natural_frequency_attr(joint):
    """Get the natural frequency attribute for a mimic joint.

    Args:
        joint: The joint to get the attribute from

    Returns:
        Attribute: The natural frequency attribute if joint is mimic, None otherwise
    """
    if is_joint_mimic(joint):
        mimic_axis = [a for a in joint.GetAppliedSchemas() if "MimicJointAPI" in a][-1].split(":")[-1]
        attr = joint.GetAttribute(f"physxMimicJoint:{mimic_axis}:naturalFrequency")
        return attr
    return None


def get_mimic_damping_ratio_attr(joint):
    """Get the damping ratio attribute for a mimic joint.

    Args:
        joint: The joint to get the attribute from

    Returns:
        Attribute: The damping ratio attribute if joint is mimic, None otherwise
    """
    if is_joint_mimic(joint):
        mimic_axis = [a for a in joint.GetAppliedSchemas() if "MimicJointAPI" in a][-1].split(":")[-1]
        attr = joint.GetAttribute(f"physxMimicJoint:{mimic_axis}:dampingRatio")
        return attr
    return None


def get_stiffness_attr(joint):
    """Get the stiffness attribute for a joint.

    Args:
        joint: The joint to get the attribute from

    Returns:
        Attribute: The stiffness attribute if valid joint type, None otherwise
    """
    if joint.IsA(pxr.UsdPhysics.RevoluteJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "angular")
        return driveAPI.GetStiffnessAttr()
    elif joint.IsA(pxr.UsdPhysics.PrismaticJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "linear")
        return driveAPI.GetStiffnessAttr()
    return None


def get_joint_drive_type_attr(joint):
    """Get the drive type attribute for a joint.

    Args:
        joint: The joint to get the attribute from

    Returns:
        Attribute: The drive type attribute if valid joint type, None otherwise
    """
    driveAPI = None
    if joint.IsA(pxr.UsdPhysics.RevoluteJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "angular")
    elif joint.IsA(pxr.UsdPhysics.PrismaticJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "linear")
    if driveAPI:
        return driveAPI.GetTypeAttr()
    return None


def get_damping_attr(joint):
    """Get the damping attribute for a joint.

    Args:
        joint: The joint to get the attribute from

    Returns:
        Attribute: The damping attribute if valid joint type, None otherwise
    """
    if is_joint_mimic(joint):
        mimic_axis = [a for a in joint.GetAppliedSchemas() if "MimicJointAPI" in a][-1].split(":")[-1]
        attr = joint.GetAttribute(f"physxMimicJoint:{mimic_axis}:damping")
        return attr
    if joint.IsA(pxr.UsdPhysics.RevoluteJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "angular")
        return driveAPI.GetDampingAttr()
    if joint.IsA(pxr.UsdPhysics.PrismaticJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "linear")
        return driveAPI.GetDampingAttr()
    return None


def get_joint_drive_mode(joint):
    """Get the drive mode for a joint.

    Args:
        joint: The joint to get the mode from

    Returns:
        int: The drive mode value
    """
    if is_joint_mimic(joint):
        return 3
    stiffness = get_stiffness_attr(joint)
    damping = get_damping_attr(joint)
    if stiffness:
        if stiffness.Get() > 0:
            return 1
        else:
            if damping:
                if damping.Get() > 0:
                    return 2
        return 0


class JointItem(ui.AbstractItem):
    target_type = [
        "None",
        "Position",
        "Velocity",
    ]
    joint_drive_type = [
        "Acceleration",
        "Force",
    ]
    target_type_with_mimic = ["Mimic"]
    joint_drive_type_with_mimic = [""]

    def __init__(self, joint, inertia, value_changed_fn=None):
        super().__init__()

        self.joint = joint
        self.inertia = inertia
        self._value_changed_fn = value_changed_fn
        target_type = JointItem.target_type
        joint_drive_type = JointItem.joint_drive_type
        stiffness = get_stiffness_attr(joint)
        damping = get_damping_attr(joint)
        jointdrive = None

        if is_joint_mimic(joint):
            drive_mode = JointDriveMode.MIMIC
            joint_drive_type = JointItem.joint_drive_type_with_mimic
            target_type = JointItem.target_type_with_mimic
        else:
            if stiffness:
                drive_mode = JointDriveMode.POSITION if stiffness.Get() > 0 else JointDriveMode.VELOCITY
            elif damping:
                drive_mode = JointDriveMode.VELOCITY
            else:
                drive_mode = JointDriveMode.NONE
            jointdrive = get_joint_drive_type_attr(joint)
        if drive_mode == JointDriveMode.POSITION:
            stiffness = stiffness.Get()
            damping = damping.Get()
        elif drive_mode == JointDriveMode.VELOCITY:
            stiffness = 0
            damping = damping.Get()
        else:
            stiffness = 0
            damping = 0
        if stiffness is None:
            stiffness = 0
        if damping is None:
            damping = 0
        self.model_cols = [None] * 7
        self.model_cols[WidgetColumns.NAME] = ui.SimpleStringModel(joint.GetName())
        self.model_cols[WidgetColumns.DRIVE_MODE] = ComboListModel(target_type, drive_mode)
        self.model_cols[WidgetColumns.DRIVE_TYPE] = ComboListModel(
            joint_drive_type, JointDriveType.from_token(jointdrive.Get() if jointdrive else "")
        )
        self.model_cols[WidgetColumns.STIFFNESS] = ui.SimpleFloatModel(stiffness)
        self.model_cols[WidgetColumns.DAMPING] = ui.SimpleFloatModel(damping)
        self.model_cols[WidgetColumns.NATURAL_FREQUENCY] = ui.SimpleFloatModel(self.compute_natural_frequency())
        self.model_cols[WidgetColumns.DAMPING_RATIO] = ui.SimpleFloatModel(self.compute_damping_ratio())

        # Add Model Update UI callbacks
        self.model_cols[WidgetColumns.DRIVE_TYPE].get_item_value_model().add_value_changed_fn(
            partial(self._on_value_changed, WidgetColumns.DRIVE_TYPE)
        )
        self.model_cols[WidgetColumns.DRIVE_MODE].get_item_value_model().add_value_changed_fn(
            partial(self._on_value_changed, WidgetColumns.DRIVE_MODE)
        )

        self.model_cols[WidgetColumns.STIFFNESS].add_value_changed_fn(
            partial(self._on_value_changed, WidgetColumns.STIFFNESS)
        )
        self.model_cols[WidgetColumns.DAMPING].add_value_changed_fn(
            partial(self._on_value_changed, WidgetColumns.DAMPING)
        )
        self.model_cols[WidgetColumns.NATURAL_FREQUENCY].add_value_changed_fn(
            partial(self._on_value_changed, WidgetColumns.NATURAL_FREQUENCY)
        )
        self.model_cols[WidgetColumns.DAMPING_RATIO].add_value_changed_fn(
            partial(self._on_value_changed, WidgetColumns.DAMPING_RATIO)
        )

        # Add Config update callbacs
        self.model_cols[WidgetColumns.STIFFNESS].add_value_changed_fn(
            partial(
                self.on_update_stiffness,
            )
        )
        self.model_cols[WidgetColumns.DAMPING].add_value_changed_fn(
            partial(
                self.on_update_damping,
            )
        )
        self.model_cols[WidgetColumns.NATURAL_FREQUENCY].add_value_changed_fn(partial(self.on_update_natural_frequency))
        self.model_cols[WidgetColumns.DAMPING_RATIO].add_value_changed_fn(
            partial(
                self.on_update_damping_ratio,
            )
        )
        self.value_field = {}
        self.mode = JointSettingMode.STIFFNESS

    def on_update_drive_type(self, model, *args):
        drive_type = model.get_value_as_int()
        self.drive_type = JointDriveType(drive_type)
        if self.drive_mode == JointDriveMode.VELOCITY:
            self.stiffness = 0
        if self.mode == JointSettingMode.NATURAL_FREQUENCY:
            self.compute_drive_stiffness()

    def on_update_stiffness(self, model, *args):
        attr = get_stiffness_attr(self.joint)
        if attr:
            if self.drive_mode == JointDriveMode.POSITION:
                attr.Set(model.get_value_as_float())
            elif self.drive_mode == JointDriveMode.VELOCITY:
                attr.Set(0.0)
            self.natural_frequency = self.compute_natural_frequency()

    def on_update_damping(self, model, *args):
        if self.drive_mode in [JointDriveMode.POSITION, JointDriveMode.VELOCITY, JointDriveMode.MIMIC]:
            attr = get_damping_attr(self.joint)
            if attr:
                attr.Set(model.get_value_as_float())
        new_damping_ratio = self.compute_damping_ratio()
        if abs(self.damping_ratio - new_damping_ratio) > 0.0001:
            self.damping_ratio = new_damping_ratio

    def on_update_natural_frequency(self, model, *args):
        if self.drive_mode in [JointDriveMode.MIMIC]:
            attr = get_mimic_natural_frequency_attr(self.joint)
            if attr:
                attr.Set(model.get_value_as_float())
        else:
            self.compute_drive_stiffness()

    def on_update_damping_ratio(self, model, *args):
        if self.mode == JointSettingMode.NATURAL_FREQUENCY:
            if self.drive_mode == JointDriveMode.MIMIC:
                attr = get_mimic_damping_ratio_attr(self.joint)
                if attr:
                    attr.Set(model.get_value_as_float())
        m_eq = 1
        if self.drive_type == JointDriveType.FORCE:
            m_eq = self.inertia
        if self.mode == JointSettingMode.NATURAL_FREQUENCY:
            self.damping = 2 * m_eq * self.natural_frequency * self.damping_ratio

    def compute_damping_ratio(self):
        if self.drive_mode == JointDriveMode.MIMIC:
            mimic_attr = get_mimic_damping_ratio_attr(self.joint)
            if mimic_attr:
                return mimic_attr.Get()
            else:
                return 0
        m_eq = 1
        if self.drive_type == JointDriveType.FORCE:
            m_eq = self.inertia
        if self.natural_frequency > 0:
            damping_ratio = self.damping / (2 * m_eq * self.natural_frequency)
            return damping_ratio
        return 0

    @property
    def natural_frequency(self):
        return self.model_cols[WidgetColumns.NATURAL_FREQUENCY].get_value_as_float()

    @natural_frequency.setter
    def natural_frequency(self, value: float):
        self.model_cols[WidgetColumns.NATURAL_FREQUENCY].set_value(value)

    @property
    def damping_ratio(self):
        return self.model_cols[WidgetColumns.DAMPING_RATIO].get_value_as_float()

    @damping_ratio.setter
    def damping_ratio(self, value: float):
        self.model_cols[WidgetColumns.DAMPING_RATIO].set_value(value)

    @property
    def stiffness(self):
        return self.model_cols[WidgetColumns.STIFFNESS].get_value_as_float()

    @stiffness.setter
    def stiffness(self, value: float):
        if self.drive_mode != JointDriveMode.MIMIC:
            self.model_cols[WidgetColumns.STIFFNESS].set_value(value)

    @property
    def damping(self):
        return self.model_cols[WidgetColumns.DAMPING].get_value_as_float()

    @damping.setter
    def damping(self, value: float):
        self.model_cols[WidgetColumns.DAMPING].set_value(value)

    @property
    def drive_mode(self):
        return JointDriveMode(self.model_cols[WidgetColumns.DRIVE_MODE].get_item_value_model().get_value_as_int())

    @drive_mode.setter
    def drive_mode(self, value: JointDriveMode):
        self.set_item_target_type(value.value)
        self.on_update_stiffness(self.model_cols[WidgetColumns.STIFFNESS], None)
        self.on_update_damping(self.model_cols[WidgetColumns.DAMPING], None)

    @property
    def drive_type(self):
        return JointDriveType(self.model_cols[WidgetColumns.DRIVE_TYPE].get_item_value_model().get_value_as_int())

    @drive_type.setter
    def drive_type(self, value: JointDriveType):
        drive_type_attr = get_joint_drive_type_attr(self.joint)
        if drive_type_attr:
            drive_type_attr.Set(value.to_token())
            if self.mode == JointSettingMode.NATURAL_FREQUENCY:
                self.compute_drive_stiffness()

    def set_item_target_type(self, value):
        if is_joint_mimic(self.joint):
            value = 1
        self.model_cols[WidgetColumns.DRIVE_MODE].set_current_index(value)

    def _get_item_target_type(self):
        if is_joint_mimic(self.joint):
            return 1
        else:
            return self.model_cols[WidgetColumns.DRIVE_MODE].get_item_value_model().get_value_as_int()

    def _on_value_changed(self, col_id=1, _=None):
        if col_id == WidgetColumns.DRIVE_MODE:
            if is_joint_mimic(self.joint):
                self.drive_mode = JointDriveMode.MIMIC
            else:
                self.drive_mode = JointDriveMode(
                    self.model_cols[WidgetColumns.DRIVE_MODE].get_item_value_model().get_value_as_int()
                )
        elif col_id == WidgetColumns.DRIVE_TYPE:
            self.drive_type = JointDriveType(
                self.model_cols[WidgetColumns.DRIVE_TYPE].get_item_value_model().get_value_as_int()
            )

        if self._value_changed_fn:
            self._value_changed_fn(self, col_id)

    def compute_natural_frequency(self):
        if self.drive_mode == JointDriveMode.MIMIC:
            mimic_attr = get_mimic_natural_frequency_attr(self.joint)
            if mimic_attr:
                return mimic_attr.Get()
            else:
                return 0
        else:
            m_eq = 1
            if self.drive_type == JointDriveType.FORCE:
                m_eq = self.inertia
            return (self.stiffness / m_eq) ** (0.5)

    def compute_drive_stiffness(self):
        m_eq = 1
        if self.drive_type == JointDriveType.FORCE:
            m_eq = self.inertia
        stiffness = m_eq * self.natural_frequency**2
        value_changed = self.stiffness != stiffness
        if value_changed and self.mode == JointSettingMode.NATURAL_FREQUENCY:
            self.stiffness = stiffness
            # print(self.joint.drive.target_type)
            if self.drive_mode == JointDriveMode.POSITION:
                self.damping = 2 * m_eq * self.natural_frequency * self.damping_ratio
                # damping_attr = get_damping_attr(self.joint)
                # if damping_attr:
                #     damping_attr.Set(self.damping)
            elif self.drive_mode == JointDriveMode.VELOCITY:
                self.stiffness = 0

    def get_item_value(self, col_id=0):
        if col_id in [WidgetColumns.NAME, WidgetColumns.DRIVE_MODE, WidgetColumns.DRIVE_TYPE]:
            return self.model_cols[col_id].get_value_as_string()
        elif col_id in [
            WidgetColumns.STIFFNESS,
            WidgetColumns.DAMPING,
            WidgetColumns.NATURAL_FREQUENCY,
            WidgetColumns.DAMPING_RATIO,
        ]:
            if self.mode == JointSettingMode.STIFFNESS:
                return self.model_cols[col_id].get_value_as_float()
            else:
                return self.model_cols[col_id].get_value_as_float()

    def set_item_value(self, col_id, value):
        if col_id == 1:
            self.model_cols[col_id].set_current_index(value)
        elif col_id in [
            WidgetColumns.STIFFNESS,
            WidgetColumns.DAMPING,
            WidgetColumns.NATURAL_FREQUENCY,
            WidgetColumns.DAMPING_RATIO,
        ]:
            if self.mode == JointSettingMode.STIFFNESS:
                if self.drive_mode != JointDriveMode.MIMIC:
                    self.model_cols[col_id].set_value(value)
            else:
                self.model_cols[col_id].set_value(value)

    def get_value_model(self, col_id=0):
        if col_id in [WidgetColumns.STIFFNESS, WidgetColumns.DAMPING]:
            if self.mode == JointSettingMode.STIFFNESS:
                return self.model_cols[col_id]
            else:
                return self.model_cols[col_id]
        else:
            return self.model_cols[col_id]


class JointItemDelegate(ui.AbstractItemDelegate):
    # TODO: the name is too long for "Natural Frequency", "Damping Ratio"
    header_tooltip = ["Name", "Mode", "Type", "Stiffness", "Damping", "Nat. Freq.", "Damping R."]
    header = ["Name", "Mode", "Type", "Stiffness", "Damping", "Nat. Freq.", "Damping R."]

    def __init__(self, model):
        super().__init__()
        self.__model = model
        self.__name_sort_options_menu = None
        self.__items_sort_policy = [SearchableItemSortPolicy.DEFAULT] * self.__model.get_item_value_model_count(None)
        self.__mode = JointSettingMode.STIFFNESS
        self.column_headers = {}

    def set_mode(self, mode):
        self.__mode = mode
        self.update_mimic()

    def build_branch(self, model, item=None, column_id=0, level=0, expanded=False):
        pass

    def build_header(self, column_id=0):
        with ui.ZStack(style_type_name_override="TreeView"):
            ui.Rectangle(
                name="Header",
                style_type_name_override="TreeView",
            )
            if column_id in [WidgetColumns.NAME, WidgetColumns.DRIVE_MODE, WidgetColumns.DRIVE_TYPE]:
                alignment = ui.Alignment.CENTER
                with ui.HStack():
                    with ui.VStack():
                        ui.Spacer(height=ui.Pixel(3))
                        ui.Label(
                            JointItemDelegate.header[column_id],
                            tooltip=JointItemDelegate.header_tooltip[column_id],
                            name="Header",
                            style_type_name_override="TreeView",
                            elided_text=True,
                            alignment=alignment,
                        )
                    ui.Image(
                        name="sort",
                        height=19,
                        width=19,
                        mouse_pressed_fn=lambda x, y, b, a, column_id=column_id: self.sort_button_pressed_fn(
                            b, column_id
                        ),
                    )
            elif column_id in [WidgetColumns.STIFFNESS, WidgetColumns.DAMPING]:
                alignment = ui.Alignment.CENTER
                self.column_headers[column_id] = ui.HStack()
                with self.column_headers[column_id]:
                    with ui.VStack():
                        ui.Spacer(height=ui.Pixel(3))
                        if self.__mode == JointSettingMode.STIFFNESS:
                            text = JointItemDelegate.header[column_id]
                        else:
                            text = JointItemDelegate.header_tooltip[column_id + 2]
                        ui.Label(
                            text,
                            tooltip=text,
                            elided_text=True,
                            name="header",
                            style_type_name_override="TreeView",
                            alignment=alignment,
                        )
                    ui.Image(
                        name="sort",
                        height=19,
                        width=19,
                        mouse_pressed_fn=lambda x, y, b, a, column_id=column_id: self.sort_button_pressed_fn(
                            b, column_id
                        ),
                    )

    def update_mimic(self):
        pass
        # for item in self.__model.get_item_children():
        #     if 1 in item.value_field:
        #         if not item.config.parse_mimic:
        #             item.value_field[1].model.set_items(JointItem.target_type)
        #             item.value_field[1].model._default_index = 1
        #             item.value_field[1].model.set_current_index(1)
        #         else:
        #             if item.joint.mimic.joint != "":
        #                 item.value_field[1].model.set_items(JointItem.target_type_with_mimic)
        #                 item.value_field[1].model._default_index = 3
        #                 item.value_field[1].model.set_current_index(3)
        #         self.__on_target_change(item, item.value_field[1].model.get_value_as_string())

    def update_defaults(self):
        for item in self.__model.get_item_children():
            for i in [
                WidgetColumns.STIFFNESS,
                WidgetColumns.DAMPING,
                WidgetColumns.NATURAL_FREQUENCY,
                WidgetColumns.DAMPING_RATIO,
            ]:
                field = item.value_field.get(i)
                if field:
                    field.update_default_value()

    def build_widget(self, model, item=None, index=0, level=0, expanded=False):
        if item:
            drive_mode = self.__model.get_item_value_model(item, 1).get_current_index()
            # item.mode = self.__mode
            if index == WidgetColumns.NAME:
                with ui.ZStack(height=ITEM_HEIGHT, style_type_name_override="TreeView"):
                    ui.Rectangle(name="treeview_first_item")
                    with ui.VStack():
                        ui.Spacer()
                        with ui.HStack(height=0):
                            ui.Label(
                                str(model.get_item_value(item, index)),
                                tooltip=model.get_item_value(item, index),
                                name="treeview_item",
                                elided_text=True,
                                height=0,
                            )
                            ui.Spacer(width=1)
                        ui.Spacer()
            elif index in [WidgetColumns.DRIVE_MODE, WidgetColumns.DRIVE_TYPE]:
                with ui.ZStack(height=ITEM_HEIGHT, style_type_name_override="TreeView"):
                    ui.Rectangle(name="treeview_item")
                    with ui.HStack():
                        ui.Spacer(width=2)
                        with ui.VStack():
                            ui.Spacer(height=6)
                            with ui.ZStack():
                                item.value_field[index] = ui.ComboBox(
                                    model.get_item_value_model(item, index), name="treeview_item", height=0
                                )
                                if index == WidgetColumns.DRIVE_MODE:
                                    item.value_field[index].model.add_item_changed_fn(
                                        lambda m, i: self.__on_target_change(item, m.get_value_as_string())
                                    )
                                else:
                                    item.value_field[index].model.add_item_changed_fn(
                                        lambda m, i: self.on_joint_mode_changed(item, m.get_current_index())
                                    )
                                if model.get_item_value(item, WidgetColumns.DRIVE_MODE) == "Mimic":
                                    item.value_field[index].enabled = False
                                    # item.value_field[index].visible = False

                                with ui.HStack():
                                    ui.Spacer()
                                    ui.Rectangle(name="mask", width=15)
                                if model.get_item_value(item, WidgetColumns.DRIVE_MODE) != "Mimic":
                                    with ui.HStack():
                                        ui.Spacer()
                                        with ui.VStack(width=0):
                                            ui.Spacer()
                                            ui.Triangle(
                                                name="mask", height=5, width=7, alignment=ui.Alignment.CENTER_BOTTOM
                                            )
                                            ui.Spacer()
                                        ui.Spacer(width=2)
                            ui.Spacer(height=2)
                        ui.Spacer(width=2)

            elif index in [WidgetColumns.STIFFNESS, WidgetColumns.DAMPING]:
                with ui.ZStack(height=ITEM_HEIGHT):
                    ui.Rectangle(name="treeview_item")
                    with ui.VStack():
                        ui.Spacer()
                        if self.__mode == JointSettingMode.STIFFNESS:
                            index_offset = 0
                        else:
                            index_offset = 2
                        item.value_field[index + index_offset] = ResetableLabelField(
                            model.get_item_value_model(item, index + index_offset), ui.FloatDrag, ".2f"
                        )
                        item.value_field[index + index_offset].field.min = 0.0
                        self.__on_target_change(item, model.get_item_value(item, WidgetColumns.DRIVE_MODE))
                        ui.Spacer()
            self.update_mimic()

    def sort_button_pressed_fn(self, b, column_id):
        if b != 0:
            return

        def on_sort_policy_changed(policy, value):
            if self.__items_sort_policy[column_id] != policy:
                self.__items_sort_policy[column_id] = policy
                self.__model.sort_by_name(policy, column_id)

        items_sort_policy = self.__items_sort_policy[column_id]
        self.__name_sort_options_menu = ui.Menu("Sort Options")
        with self.__name_sort_options_menu:
            ui.MenuItem("Sort By", enabled=False)
            ui.Separator()
            ui.MenuItem(
                "Ascending",
                checkable=True,
                checked=items_sort_policy == SearchableItemSortPolicy.A_TO_Z,
                checked_changed_fn=partial(on_sort_policy_changed, SearchableItemSortPolicy.A_TO_Z),
                hide_on_click=False,
            )
            ui.MenuItem(
                "Descending",
                checkable=True,
                checked=items_sort_policy == SearchableItemSortPolicy.Z_TO_A,
                checked_changed_fn=partial(on_sort_policy_changed, SearchableItemSortPolicy.Z_TO_A),
                hide_on_click=False,
            )
        self.__name_sort_options_menu.show()

    def on_joint_mode_changed(self, item, mode):
        item.drive_mode = mode

    def __on_target_change(self, item, current_target: str):
        # None: disables all
        # Position: enables all
        # Velocity: enables stiffness (2) and natural frequency (4)
        # Mimic: disables drive mode, type, stiffness, damping
        for field in item.value_field.values():
            field.enabled = True
            field.visible = True
        if current_target == "Mimic":
            for i in [
                WidgetColumns.DRIVE_MODE,
                WidgetColumns.DRIVE_TYPE,
                WidgetColumns.STIFFNESS,
                WidgetColumns.DAMPING,
            ]:
                if field := item.value_field.get(i):
                    field.enabled = False
                    if i in [
                        WidgetColumns.DRIVE_MODE,
                        WidgetColumns.DRIVE_TYPE,
                        WidgetColumns.STIFFNESS,
                        WidgetColumns.DAMPING,
                    ]:
                        field.visible = False
        if current_target == "None":
            for i in [
                WidgetColumns.STIFFNESS,
                WidgetColumns.DAMPING,
                WidgetColumns.NATURAL_FREQUENCY,
                WidgetColumns.DAMPING_RATIO,
            ]:
                if field := item.value_field.get(i):
                    field.enabled = False
                    field.visible = False
        # elif current_target == "Position":
        # for field in item.value_field.values():
        #     field.enabled = True
        #     field.visible = True
        elif current_target == "Velocity":
            if field := item.value_field.get(WidgetColumns.STIFFNESS):
                field.enabled = False
                field.visible = False
            # if field := item.value_field.get(WidgetColumns.DAMPING):
            #     field.enabled = True
            #     field.visible = True
            # if field := item.value_field.get(WidgetColumns.DAMPING_RATIO):
            #     field.enabled = False
            #     field.visible = False


class JointListModel(ui.AbstractItemModel):
    def __init__(self, joints_list, inertias, value_changed_fn, **kwargs):
        super().__init__()
        self._children = [JointItem(j, inertias[j], self._on_joint_changed) for j in joints_list]
        self._joint_changed_fn = value_changed_fn
        self._items_sort_func = None
        self._items_sort_reversed = False
        self._mode = JointSettingMode.STIFFNESS

    def _on_joint_changed(self, joint, col_id):
        if self._joint_changed_fn:
            self._joint_changed_fn(joint, col_id)

    def get_item_children(self, item=None):
        """Returns all the children when the widget asks it."""
        if item is not None:
            return []
        else:
            children = self._children
            if self._items_sort_func:
                children = sorted(children, key=self._items_sort_func, reverse=self._items_sort_reversed)

            return children

    def get_item_value(self, item, column_id):
        if item:
            return item.get_item_value(column_id)

    def set_item_value(self, item, column_id, value):
        if item:
            item.set_item_value(column_id, value)

    def get_item_value_model_count(self, item):
        """The number of columns"""
        return 5

    def get_item_value_model(self, item, column_id):
        """
        Return value model.
        It's the object that tracks the specific value.
        """
        if item:
            if isinstance(item, JointItem):
                return item.get_value_model(column_id)

    def sort_by_name(self, policy, column_id):
        if policy == SearchableItemSortPolicy.Z_TO_A:
            self._items_sort_reversed = True
        else:
            self._items_sort_reversed = False
        if column_id in [WidgetColumns.NAME, WidgetColumns.DRIVE_MODE, WidgetColumns.DRIVE_TYPE]:
            self._items_sort_func = (
                lambda item: self.get_item_value_model(item, column_id).get_value_as_string().lower()
            )
        if column_id in [WidgetColumns.STIFFNESS, WidgetColumns.DAMPING]:
            if self._mode == JointSettingMode.STIFFNESS:
                self._items_sort_func = lambda item: self.get_item_value_model(item, column_id).get_value_as_float()
            else:
                self._items_sort_func = lambda item: self.get_item_value_model(item, column_id).get_value_as_int()
        self._item_changed(None)

    def set_mode(self, mode):
        if self._mode != mode:
            self._mode = mode
            for item in self.get_item_children():
                item.mode = mode
                self._item_changed(item)
            self._item_changed(None)

    def set_drive_type(self, drive_type):
        for item in self._children:
            item.drive_type = drive_type
            item.compute_drive_stiffness()
            self._item_changed(item)
        self._item_changed(None)


class JointWidget(TableWidget):
    def __init__(self, joints, inertias, value_changed_fn=None):
        self.joints = joints
        self.inertias = inertias
        if not inertias:
            return
        self.model = JointListModel(joints, inertias, self._on_value_changed)
        self.delegate = JointItemDelegate(self.model)
        self._enable_bulk_edit = True
        self._value_changed_fn = value_changed_fn
        self.model._mode = JointSettingMode.STIFFNESS
        self.drive_type = JointDriveType.ACCELERATION
        super().__init__(value_changed_fn, self.model, self.delegate)

    def update_mimic(self):
        self.delegate.update_mimic()

    def build_tree_view(self):
        self.list = ui.TreeView(
            self.model,
            delegate=self.delegate,
            alignment=ui.Alignment.CENTER_TOP,
            column_widths=[ui.Fraction(1), ui.Pixel(65), ui.Pixel(65), ui.Pixel(100), ui.Pixel(100)],
            # TODO: uncomment this when we could set the default option width
            min_column_widths=[80, 65, 65, 100, 70],
            columns_resizable=True,
            header_visible=True,
            resizeable_on_columns_resized=True,
            # height=ui.Fraction(1),
        )

    def switch_mode(self, switch: JointSettingMode):
        super().switch_mode(switch)
        self.update_mimic()

    def switch_drive_type(self, drive_type: JointDriveType):
        self.set_bulk_edit(False)
        drive_type = JointDriveType.ACCELERATION
        if drive_type == JointDriveType.FORCE:
            drive_type = JointDriveType.FORCE
        self.model.set_drive_type(drive_type)
        if self.model._mode == JointSettingMode.STIFFNESS:
            self.delegate.update_defaults()
        self.drive_type = drive_type
        self.set_bulk_edit(True)

    def _on_value_changed(self, joint_item, col_id=1):
        if self._enable_bulk_edit:
            if joint_item not in self.list.selection:
                self.list.selection = [joint_item]
            self.set_bulk_edit(False)
            for item in self.list.selection:
                if item is not joint_item:
                    if col_id == WidgetColumns.DRIVE_MODE:
                        item.set_item_value(col_id, joint_item.drive_mode)
                    if (
                        col_id in [WidgetColumns.STIFFNESS, WidgetColumns.DAMPING]
                        and self.model._mode == JointSettingMode.STIFFNESS
                    ) or (
                        col_id in [WidgetColumns.NATURAL_FREQUENCY, WidgetColumns.DAMPING_RATIO]
                        and self.model._mode == JointSettingMode.NATURAL_FREQUENCY
                    ):
                        if item.get_item_value(col_id) != joint_item.get_item_value(col_id):
                            item.set_item_value(col_id, joint_item.get_item_value(col_id))

            self.set_bulk_edit(True)
        if self._value_changed_fn:
            self._value_changed_fn(joint_item.joint)
