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

"""Provides a widget for tuning joint drive parameters including stiffness, damping, natural frequency, and damping ratio in Isaac Sim."""

from __future__ import annotations

from enum import IntEnum
from functools import partial
from types import SimpleNamespace

import omni.ui as ui
import pxr

from ..gain_tuner_drive_math import (
    damping_from_damping_ratio_revolute_position,
    damping_ratio_from_stiffness_damping_revolute_position,
    natural_frequency_hz_from_stiffness_revolute_position,
    stiffness_stored_and_damping_from_natural_frequency_revolute_position,
)
from .base_table_widget import TableWidget
from .resetable_widget import ResetableLabelField

ITEM_HEIGHT = 28
_CELL_BG_ACTIVE = "treeview_item"
_CELL_BG_INAPPLICABLE = "treeview_item_inapplicable"


def _set_gain_cell_field_state(item: object, column_id: int, *, applicable: bool) -> None:
    """Show an editable cell, or a blank lighter-gray cell when the parameter does not apply.

    Args:
        item: Joint table item containing value fields and cell backgrounds.
        column_id: Column identifier for the cell to update.
        applicable: True when the value applies to the current joint configuration.
    """
    field = item.value_field.get(column_id)
    backgrounds = getattr(item, "_cell_backgrounds", {})
    rect = backgrounds.get(column_id)
    if applicable:
        if field is not None:
            field.visible = True
            field.enabled = True
        if rect is not None:
            rect.visible = True
            rect.name = _CELL_BG_ACTIVE
    else:
        if field is not None:
            field.visible = False
            field.enabled = False
        if rect is not None:
            rect.visible = True
            rect.name = _CELL_BG_INAPPLICABLE


def _tunable_value_column_ids(item: object) -> tuple[int, int]:
    """Return the two numeric column ids used for the current table tuning mode.

    Args:
        item: Joint table item whose tuning mode selects the columns.

    Returns:
        Pair of column identifiers for the editable gain values.
    """
    if item.mode == JointSettingMode.STIFFNESS:
        return WidgetColumns.STIFFNESS, WidgetColumns.DAMPING
    return WidgetColumns.NATURAL_FREQUENCY, WidgetColumns.DAMPING_RATIO


class WidgetColumns(IntEnum):
    """Enumeration defining column identifiers for joint parameter widget layouts.

    Provides integer constants for identifying specific columns in joint configuration tables,
    enabling consistent column indexing across joint parameter editing interfaces.
    """

    NAME = 0
    """Column index for the joint name display."""
    DRIVE_MODE = 1
    """Column index for the joint drive mode selection."""
    DRIVE_TYPE = 2
    """Column index for the joint drive type selection."""
    STIFFNESS = 3
    """Column index for the joint stiffness parameter."""
    DAMPING = 4
    """Column index for the joint damping parameter."""
    NATURAL_FREQUENCY = 5
    """Column index for the joint natural frequency parameter."""
    DAMPING_RATIO = 6
    """Column index for the joint damping ratio parameter."""


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
    def from_token(cls, token: str) -> "JointDriveType":
        """Creates a JointDriveType from a string token.

        Args:
            token: The string token to convert. Valid values are "acceleration", "force", or "" for MIMIC.

        Returns:
            The corresponding JointDriveType enum value.

        Raises:
            ValueError: If the token is not a valid joint drive type.
        """
        if token.lower() == "acceleration":
            return cls.ACCELERATION
        elif token.lower() == "force":
            return cls.FORCE
        elif token.lower() == "":
            return cls.MIMIC
        else:
            raise ValueError(f"Invalid joint drive type: {token}")

    def to_token(self) -> str:
        """Converts the JointDriveType to its string token representation.

        Returns:
            The string token for the drive type ("acceleration" or "force").

        Raises:
            ValueError: If the drive type is not valid for token conversion.
        """
        if self == self.ACCELERATION:
            return "acceleration"
        elif self == self.FORCE:
            return "force"
        else:
            raise ValueError(f"Invalid joint drive type: {self}")


class JointDriveMode(IntEnum):
    """Drive mode for a joint row in the gain tuner table."""

    NONE = 0
    """No position or velocity drive is active."""

    POSITION = 1
    """Position drive (non-zero stiffness)."""

    VELOCITY = 2
    """Velocity-only drive (zero stiffness, non-zero damping)."""

    MIMIC = 3
    """Mimic joint (parameters from PhysX MimicJointAPI)."""


class ComboListModel(ui.AbstractItemModel):
    """A combo box model that manages a list of selectable items with a default selection.

    Inherits from Omni UI's AbstractItemModel to provide data for combo box UI elements. Maintains a list of
    selectable items and tracks the currently selected item index.

    Args:
        item_list: List of items to display in the combo box.
        default_index: Default item index to select initially.
    """

    class ComboListItem(ui.AbstractItem):
        """Represents an individual item within a ComboListModel for UI dropdown selections.

        This class wraps a string value in a UI model structure that can be used in combo boxes and similar
        selection widgets. It maintains the item's display value through a SimpleStringModel.

        Args:
            item: The string value to be displayed in the combo box item.
        """

        def __init__(self, item: str) -> None:
            super().__init__()
            self.model = ui.SimpleStringModel(item)

    def __init__(self, item_list: list, default_index: object) -> None:
        super().__init__()
        self._default_index = default_index.value
        self._current_index = ui.SimpleIntModel(default_index.value)
        self._current_index.add_value_changed_fn(lambda a: self._item_changed(None))
        self._item_list = item_list
        self._items = []
        if item_list:
            for item in item_list:
                self._items.append(ComboListModel.ComboListItem(item))

    def get_item_children(self, item: object) -> list["ComboListModel.ComboListItem"]:
        """Returns the child items for the combo box list.

        Args:
            item: The parent item to get children for.

        Returns:
            List of combo box items.
        """
        return self._items

    def set_items(self, items: list) -> None:
        """Sets the items in the combo box list.

        Args:
            items: List of items to set in the combo box.
        """
        self._items = []
        for item in items:
            self._items.append(ComboListModel.ComboListItem(item))

    def get_item_list(self) -> list:
        """Returns the original item list passed to the model.

        Returns:
            The original list of items.
        """
        return self._item_list

    def get_item_value_model(
        self, item: object = None, column_id: int = -1
    ) -> ui.SimpleIntModel | ui.SimpleStringModel:
        """Returns the value model for the specified item or current selection.

        Args:
            item: The item to get the value model for.
            column_id: The column identifier.

        Returns:
            The value model for the item or current index model if item is None.
        """
        if item is None:
            return self._current_index
        return item.model

    def get_current_index(self) -> int:
        """Returns the index of the currently selected item.

        Returns:
            The current selected index as an integer.
        """
        return self._current_index.get_value_as_int()

    def set_current_index(self, index: int) -> None:
        """Sets the currently selected item by index.

        Args:
            index: The index to set as current selection.
        """
        if index < len(self._items):
            self._current_index.set_value(index)

    def get_value_as_string(self) -> str:
        """Returns the string representation of the currently selected item.

        Returns:
            The string value of the current selection, or empty string if no valid selection.
        """
        if self._current_index.get_value_as_int() < len(self._items):
            return self._items[self._current_index.get_value_as_int()].model.get_value_as_string()
        return ""

    def is_default(self) -> bool:
        """Checks if the current selection matches the default index.

        Returns:
            True if current index equals the default index, False otherwise.
        """
        return self.get_current_index() == self._default_index


def is_joint_mimic(joint: object) -> bool:
    """Check if a joint has mimic joint API applied.

    Args:
        joint: The joint to check

    Returns:
        True if joint has MimicJointAPI applied, False otherwise
    """
    return len([a for a in joint.GetAppliedSchemas() if "MimicJointAPI" in a]) > 0


def get_mimic_natural_frequency_attr(joint: object) -> pxr.Usd.Attribute | None:
    """Get the natural frequency attribute for a mimic joint.

    Args:
        joint: The joint to get the attribute from

    Returns:
        The natural frequency attribute if joint is mimic, None otherwise
    """
    if is_joint_mimic(joint):
        mimic_axis = [a for a in joint.GetAppliedSchemas() if "MimicJointAPI" in a][-1].split(":")[-1]
        attr = joint.GetAttribute(f"physxMimicJoint:{mimic_axis}:naturalFrequency")
        return attr
    return None


def get_mimic_damping_ratio_attr(joint: object) -> pxr.Usd.Attribute | None:
    """Get the damping ratio attribute for a mimic joint.

    Args:
        joint: The joint to get the attribute from

    Returns:
        The damping ratio attribute if joint is mimic, None otherwise
    """
    if is_joint_mimic(joint):
        mimic_axis = [a for a in joint.GetAppliedSchemas() if "MimicJointAPI" in a][-1].split(":")[-1]
        attr = joint.GetAttribute(f"physxMimicJoint:{mimic_axis}:dampingRatio")
        return attr
    return None


def get_stiffness_attr(joint: object, drive_axis: object = None) -> pxr.Usd.Attribute | None:
    """Get the stiffness attribute for a joint.

    Args:
        joint: The joint to get the attribute from
        drive_axis: Optional D6 drive axis token

    Returns:
        The stiffness attribute if valid joint type, None otherwise
    """
    if drive_axis:
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, drive_axis)
        if driveAPI:
            return driveAPI.GetStiffnessAttr()
        return None
    if joint.IsA(pxr.UsdPhysics.RevoluteJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "angular")
        return driveAPI.GetStiffnessAttr()
    elif joint.IsA(pxr.UsdPhysics.PrismaticJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "linear")
        return driveAPI.GetStiffnessAttr()
    return None


def get_joint_drive_type_attr(joint: object, drive_axis: object = None) -> pxr.Usd.Attribute | None:
    """Get the drive type attribute for a joint.

    Args:
        joint: The joint to get the attribute from
        drive_axis: Optional D6 drive axis token

    Returns:
        The drive type attribute if valid joint type, None otherwise
    """
    if drive_axis:
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, drive_axis)
        if driveAPI:
            return driveAPI.GetTypeAttr()
        return None
    driveAPI = None
    if joint.IsA(pxr.UsdPhysics.RevoluteJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "angular")
    elif joint.IsA(pxr.UsdPhysics.PrismaticJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "linear")
    if driveAPI:
        return driveAPI.GetTypeAttr()
    return None


def get_damping_attr(joint: object, drive_axis: object = None) -> pxr.Usd.Attribute | None:
    """Get the damping attribute for a joint.

    Args:
        joint: The joint to get the attribute from
        drive_axis: Optional D6 drive axis token

    Returns:
        The damping attribute if valid joint type, None otherwise
    """
    if is_joint_mimic(joint):
        mimic_axis = [a for a in joint.GetAppliedSchemas() if "MimicJointAPI" in a][-1].split(":")[-1]
        attr = joint.GetAttribute(f"physxMimicJoint:{mimic_axis}:damping")
        return attr
    if drive_axis:
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, drive_axis)
        if driveAPI:
            return driveAPI.GetDampingAttr()
        return None
    if joint.IsA(pxr.UsdPhysics.RevoluteJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "angular")
        return driveAPI.GetDampingAttr()
    if joint.IsA(pxr.UsdPhysics.PrismaticJoint):
        driveAPI = pxr.UsdPhysics.DriveAPI(joint, "linear")
        return driveAPI.GetDampingAttr()
    return None


def get_joint_drive_mode(joint: object) -> int:
    """Get the drive mode for a joint.

    Args:
        joint: The joint to get the mode from

    Returns:
        The drive mode value
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
    """Represents a joint configuration item in the gain tuner interface.

    This class encapsulates a joint's drive parameters and provides an interface for configuring stiffness, damping,
    natural frequency, and damping ratio. It manages the relationship between these parameters and handles updates
    to the underlying USD joint attributes. The class supports different joint drive modes (position, velocity,
    mimic) and drive types (acceleration, force).

    The class automatically computes natural frequency and damping ratio from stiffness and damping values, and
    vice versa when operating in natural frequency mode. It provides UI models for each parameter that can be
    connected to interface widgets.

    Args:
        joint_entry: Joint entry containing the joint prim and display information.
        inertia_provider: Provider for joint inertia values, either as a callable or dictionary.
        value_changed_fn: Callback function invoked when joint parameter values change.
    """

    target_type = [
        "None",
        "Position",
        "Velocity",
    ]
    """List of target types for standard joints: None, Position, and Velocity."""
    joint_drive_type = [
        "Acceleration",
        "Force",
    ]
    """List of joint drive types for standard joints: Acceleration and Force."""
    target_type_with_mimic = ["Mimic"]
    """List of target types for mimic joints: Mimic."""
    joint_drive_type_with_mimic = [""]
    """List of joint drive types for mimic joints: empty string."""

    def __init__(self, joint_entry: object, inertia_provider: object, value_changed_fn: callable = None) -> None:
        super().__init__()

        self.entry = joint_entry
        self.joint = joint_entry.joint
        self.drive_axis = joint_entry.drive_axis
        self._inertia_provider = inertia_provider
        self._value_changed_fn = value_changed_fn
        target_type = JointItem.target_type
        joint_drive_type = JointItem.joint_drive_type
        stiffness = get_stiffness_attr(self.joint, self.drive_axis)
        damping = get_damping_attr(self.joint, self.drive_axis)
        jointdrive = None
        self.updating_damping_ratio = False

        if is_joint_mimic(self.joint):
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
            jointdrive = get_joint_drive_type_attr(self.joint, self.drive_axis)
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
        self.model_cols[WidgetColumns.NAME] = ui.SimpleStringModel(joint_entry.display_name)
        drive_mode_combo_index = 0 if is_joint_mimic(self.joint) else drive_mode.value
        self.model_cols[WidgetColumns.DRIVE_MODE] = ComboListModel(
            target_type, SimpleNamespace(value=drive_mode_combo_index)
        )
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
        self._cell_backgrounds = {}
        self.mode = JointSettingMode.STIFFNESS

    def _get_inertia(self) -> float:
        """Retrieves the inertia value for the joint.

        Returns:
            The inertia value as a float, or 0.0 if unavailable or an error occurs.
        """
        try:
            if callable(self._inertia_provider):
                value = self._inertia_provider(self.joint)
            elif isinstance(self._inertia_provider, dict):
                value = self._inertia_provider.get(self.joint, 0.0)
            else:
                value = 0.0
        except Exception:
            value = 0.0
        return value if value is not None else 0.0

    def on_update_drive_type(self, model: object, *args: object) -> None:
        """Handles updates to the joint drive type.

        Args:
            model: The model containing the drive type value.
            *args: Additional arguments passed to the callback.
        """
        drive_type = model.get_value_as_int()
        self.drive_type = JointDriveType(drive_type)
        if self.drive_mode == JointDriveMode.VELOCITY:
            self.stiffness = 0
        if self.mode == JointSettingMode.NATURAL_FREQUENCY:
            self.compute_drive_stiffness()

    def on_update_stiffness(self, model: object, *args: object) -> None:
        """Handles updates to the joint stiffness parameter.

        Args:
            model: The model containing the stiffness value.
            *args: Additional arguments passed to the callback.
        """
        attr = get_stiffness_attr(self.joint, self.drive_axis)
        if attr:
            if self.drive_mode == JointDriveMode.POSITION:
                attr.Set(model.get_value_as_float())
            elif self.drive_mode == JointDriveMode.VELOCITY:
                attr.Set(0.0)
            self.natural_frequency = self.compute_natural_frequency()

    def on_update_damping(self, model: object, *args: object) -> None:
        """Handles updates to the joint damping parameter.

        Args:
            model: The model containing the damping value.
            *args: Additional arguments passed to the callback.
        """
        if self.drive_mode in [JointDriveMode.POSITION, JointDriveMode.VELOCITY, JointDriveMode.MIMIC]:
            attr = get_damping_attr(self.joint, self.drive_axis)
            if attr:
                attr.Set(model.get_value_as_float())
        if not self.updating_damping_ratio:
            new_damping_ratio = self.compute_damping_ratio()
            self.damping_ratio = new_damping_ratio

    def on_update_natural_frequency(self, model: object, *args: object) -> None:
        """Handles updates to the joint natural frequency parameter.

        Args:
            model: The model containing the natural frequency value.
            *args: Additional arguments passed to the callback.
        """
        if self.drive_mode in [JointDriveMode.MIMIC]:
            attr = get_mimic_natural_frequency_attr(self.joint)
            if attr:
                attr.Set(model.get_value_as_float())
        else:
            self.compute_drive_stiffness()

    def on_update_damping_ratio(self, model: object, *args: object) -> None:
        """Handles updates to the joint damping ratio parameter.

        Args:
            model: The model containing the damping ratio value.
            *args: Additional arguments passed to the callback.
        """
        if self.mode == JointSettingMode.NATURAL_FREQUENCY:
            if self.drive_mode == JointDriveMode.MIMIC:
                attr = get_mimic_damping_ratio_attr(self.joint)
                if attr:
                    attr.Set(model.get_value_as_float())
        if self.mode == JointSettingMode.NATURAL_FREQUENCY:
            self.updating_damping_ratio = True
            self.damping = damping_from_damping_ratio_revolute_position(
                self.damping_ratio,
                self.stiffness,
                use_force_drive=(self.drive_type == JointDriveType.FORCE),
                m_eq=self._get_inertia(),
            )
            self.updating_damping_ratio = False

    def compute_damping_ratio(self) -> float:
        """Computes the damping ratio for the joint based on current parameters.

        Returns:
            The calculated damping ratio as a float.
        """
        if self.drive_mode == JointDriveMode.MIMIC:
            mimic_attr = get_mimic_damping_ratio_attr(self.joint)
            if mimic_attr:
                return mimic_attr.Get()
            else:
                return 0
        if self.natural_frequency > 0:
            return damping_ratio_from_stiffness_damping_revolute_position(
                self.damping,
                self.stiffness,
                use_force_drive=(self.drive_type == JointDriveType.FORCE),
                m_eq=self._get_inertia(),
            )
        return 0

    @property
    def natural_frequency(self) -> float:
        """Natural frequency value of the joint.

        Returns:
            The natural frequency as a float.
        """
        return self.model_cols[WidgetColumns.NATURAL_FREQUENCY].get_value_as_float()

    @natural_frequency.setter
    def natural_frequency(self, value: float) -> None:
        self.model_cols[WidgetColumns.NATURAL_FREQUENCY].set_value(value)

    @property
    def damping_ratio(self) -> float:
        """Damping ratio value of the joint.

        Returns:
            The damping ratio as a float.
        """
        return self.model_cols[WidgetColumns.DAMPING_RATIO].get_value_as_float()

    @damping_ratio.setter
    def damping_ratio(self, value: float) -> None:
        self.model_cols[WidgetColumns.DAMPING_RATIO].set_value(value)

    @property
    def stiffness(self) -> float:
        """Stiffness value of the joint.

        Returns:
            The stiffness as a float.
        """
        return self.model_cols[WidgetColumns.STIFFNESS].get_value_as_float()

    @stiffness.setter
    def stiffness(self, value: float) -> None:
        if self.drive_mode != JointDriveMode.MIMIC:
            self.model_cols[WidgetColumns.STIFFNESS].set_value(value)

    @property
    def damping(self) -> float:
        """Damping value of the joint.

        Returns:
            Current damping value of the joint.
        """
        return self.model_cols[WidgetColumns.DAMPING].get_value_as_float()

    @damping.setter
    def damping(self, value: float) -> None:
        self.model_cols[WidgetColumns.DAMPING].set_value(value)

    @property
    def drive_mode(self) -> JointDriveMode:
        """Drive mode of the joint.

        Returns:
            Current drive mode of the joint.
        """
        if is_joint_mimic(self.joint):
            return JointDriveMode.MIMIC
        return JointDriveMode(self.model_cols[WidgetColumns.DRIVE_MODE].get_item_value_model().get_value_as_int())

    @drive_mode.setter
    def drive_mode(self, value: JointDriveMode) -> None:
        self.set_item_target_type(value.value)
        self.on_update_stiffness(self.model_cols[WidgetColumns.STIFFNESS], None)
        self.on_update_damping(self.model_cols[WidgetColumns.DAMPING], None)

    @property
    def drive_type(self) -> JointDriveType:
        """Drive type of the joint.

        Returns:
            Current drive type of the joint.
        """
        return JointDriveType(self.model_cols[WidgetColumns.DRIVE_TYPE].get_item_value_model().get_value_as_int())

    @drive_type.setter
    def drive_type(self, value: JointDriveType) -> None:
        drive_type_attr = get_joint_drive_type_attr(self.joint, self.drive_axis)
        if drive_type_attr:
            drive_type_attr.Set(value.to_token())
            if self.mode == JointSettingMode.NATURAL_FREQUENCY:
                self.compute_drive_stiffness()

    def set_item_target_type(self, value: int) -> None:
        """Sets the target type for the joint item.

        Args:
            value: Target type value to set.
        """
        if is_joint_mimic(self.joint):
            value = 0
        self.model_cols[WidgetColumns.DRIVE_MODE].set_current_index(value)

    def _get_item_target_type(self) -> int:
        """Gets the target type for the joint item.

        Returns:
            Target type value of the joint item.
        """
        if is_joint_mimic(self.joint):
            return 0
        return self.model_cols[WidgetColumns.DRIVE_MODE].get_item_value_model().get_value_as_int()

    def _on_value_changed(self, col_id: int = 1, _: object = None) -> None:
        """Handles value changes in the joint item UI columns.

        Args:
            col_id: Column ID that changed.
        """
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

    def compute_natural_frequency(self) -> float:
        """Computes the natural frequency of the joint.

        Returns:
            Natural frequency value based on current joint parameters.
        """
        if self.drive_mode == JointDriveMode.MIMIC:
            mimic_attr = get_mimic_natural_frequency_attr(self.joint)
            if mimic_attr:
                return mimic_attr.Get()
            else:
                return 0
        else:
            return natural_frequency_hz_from_stiffness_revolute_position(
                self.stiffness,
                use_force_drive=(self.drive_type == JointDriveType.FORCE),
                m_eq=self._get_inertia(),
            )

    def compute_drive_stiffness(self) -> None:
        """Computes and updates the drive stiffness based on natural frequency mode."""
        stiffness_deg, damping_val = stiffness_stored_and_damping_from_natural_frequency_revolute_position(
            self.natural_frequency,
            self.damping_ratio,
            use_force_drive=(self.drive_type == JointDriveType.FORCE),
            m_eq=self._get_inertia(),
        )
        value_changed = self.stiffness != stiffness_deg
        if value_changed and self.mode == JointSettingMode.NATURAL_FREQUENCY:
            self.stiffness = stiffness_deg
            # print(self.joint.drive.target_type)
            if self.drive_mode == JointDriveMode.POSITION:
                self.updating_damping_ratio = True
                self.damping = damping_val
                self.updating_damping_ratio = False
                # damping_attr = get_damping_attr(self.joint)
                # if damping_attr:
                #     damping_attr.Set(self.damping)
            elif self.drive_mode == JointDriveMode.VELOCITY:
                self.stiffness = 0

    def get_item_value(self, col_id: int = 0) -> str | float:
        """Gets the value for a specific column in the joint item.

        Args:
            col_id: Column ID to get the value from.

        Returns:
            Value from the specified column.
        """
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

    def set_item_value(self, col_id: int, value: object) -> None:
        """Sets the value for a specific column in the joint item.

        Args:
            col_id: Column ID to set the value for.
            value: Value to set in the specified column.
        """
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

    def get_value_model(self, col_id: int = 0) -> ui.AbstractValueModel:
        """Returns the value model for the specified column.

        Args:
            col_id: Column identifier to get the value model for.

        Returns:
            The value model for the specified column.
        """
        if col_id in [WidgetColumns.STIFFNESS, WidgetColumns.DAMPING]:
            if self.mode == JointSettingMode.STIFFNESS:
                return self.model_cols[col_id]
            else:
                return self.model_cols[col_id]
        else:
            return self.model_cols[col_id]


class JointItemDelegate(ui.AbstractItemDelegate):
    """A UI delegate for rendering joint parameter items in a table view.

    This delegate handles the display and interaction of joint parameters including drive modes, stiffness,
    damping, natural frequency, and damping ratios. It provides sorting capabilities and adapts the UI
    based on joint types (standard or mimic joints) and parameter setting modes.

    Args:
        model: The joint list model containing joint items to be displayed.
    """

    # TODO: the name is too long for "Natural Frequency", "Damping Ratio"
    header_tooltip = ["Name", "Mode", "Type", "Stiffness", "Damping", "Nat. Freq.", "Damping R."]
    """Tooltip text for each column header in the joint table widget."""
    header = ["Name", "Mode", "Type", "Stiffness", "Damping", "Nat. Freq.", "Damping R."]
    """Display text for each column header in the joint table widget."""

    def __init__(self, model: object) -> None:
        super().__init__()
        self.__model = model
        self.__name_sort_options_menu = None
        self.__items_sort_policy = [SearchableItemSortPolicy.DEFAULT] * self.__model.get_item_value_model_count(None)
        self.__mode = JointSettingMode.STIFFNESS
        self.column_headers = {}

    def set_mode(self, mode: object) -> None:
        """Sets the joint setting mode for the delegate.

        Args:
            mode: The joint setting mode to use (stiffness or natural frequency).
        """
        self.__mode = mode
        self.update_mimic()
        for item in self.__model.get_item_children():
            self.__on_target_change(item, self.__model.get_item_value(item, WidgetColumns.DRIVE_MODE))

    def build_branch(
        self, model: object, item: object = None, column_id: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        """Builds the branch widget for tree view items.

        Args:
            model: The item model containing the data.
            item: The item to build the branch for.
            column_id: The column identifier.
            level: The nesting level of the item.
            expanded: Whether the branch is expanded.
        """

    def build_header(self, column_id: int = 0) -> None:
        """Builds the header widget for the specified column.

        Creates column headers with labels and sort buttons. Headers display different text based on
        the current joint setting mode (stiffness vs natural frequency).

        Args:
            column_id: The column identifier to build the header for.
        """
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

    def update_mimic(self) -> None:
        """Updates mimic joint configuration for all items in the model."""
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

    def update_defaults(self) -> None:
        """Updates default values for all joint parameter fields.

        Refreshes the default values displayed in stiffness, damping, natural frequency,
        and damping ratio fields for all joint items.
        """
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

    def build_widget(
        self, model: object, item: object = None, index: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        """Builds the widget for a specific cell in the joint table.

        Creates different UI elements based on the column type: labels for names,
        combo boxes for drive modes/types, and resetable field widgets for numeric parameters.

        Args:
            model: The item model containing the data.
            item: The joint item to build the widget for.
            index: The column index being built.
            level: The nesting level of the item.
            expanded: Whether the item is expanded.
        """
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
                    item._cell_backgrounds[index] = ui.Rectangle(name=_CELL_BG_ACTIVE)
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
                    if self.__mode == JointSettingMode.STIFFNESS:
                        index_offset = 0
                    else:
                        index_offset = 2
                    model_col_id = index + index_offset
                    item._cell_backgrounds[model_col_id] = ui.Rectangle(name=_CELL_BG_ACTIVE)
                    with ui.VStack():
                        ui.Spacer()
                        item.value_field[model_col_id] = ResetableLabelField(
                            model.get_item_value_model(item, model_col_id), ui.FloatDrag, ".2f"
                        )
                        item.value_field[model_col_id].field.min = 0.0
                        self.__on_target_change(item, model.get_item_value(item, WidgetColumns.DRIVE_MODE))
                        ui.Spacer()
            self.update_mimic()

    def sort_button_pressed_fn(self, b: int, column_id: int) -> None:
        """Handles sort button press events for column headers.

        Shows a context menu with sorting options (ascending/descending) and applies
        the selected sort policy to the specified column.

        Args:
            b: The mouse button that was pressed.
            column_id: The column identifier to sort.
        """
        if b != 0:
            return

        def on_sort_policy_changed(policy: object, value: object) -> None:
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

    def on_joint_mode_changed(self, item: object, mode: object) -> None:
        """Handles changes to the joint drive mode.

        Updates the joint item's drive mode when the user selects a different mode
        from the combo box.

        Args:
            item: The joint item whose mode is changing.
            mode: The new drive mode value.
        """
        item.drive_mode = mode

    def __on_target_change(self, item: object, current_target: str) -> None:
        """Handles changes to the target mode of a joint item.

        Updates the visibility and enabled state of UI fields based on the selected target mode.
        Different modes (None, Position, Velocity, Mimic) control which parameters can be edited.

        Args:
            item: The joint item being modified.
            current_target: The new target mode selected.
        """
        # None: disables all gain parameters
        # Position: enables all
        # Velocity: no stiffness; NF/DR are not meaningful (k=0) — tune damping in Stiffness mode
        # Mimic: disables drive mode, type, stiffness, damping
        for col_id in list(item.value_field.keys()):
            _set_gain_cell_field_state(item, col_id, applicable=True)
        tunable_a, tunable_b = _tunable_value_column_ids(item)
        if current_target == "Mimic":
            for i in [WidgetColumns.DRIVE_MODE, WidgetColumns.DRIVE_TYPE]:
                _set_gain_cell_field_state(item, i, applicable=False)
            if item.mode == JointSettingMode.STIFFNESS:
                _set_gain_cell_field_state(item, tunable_a, applicable=False)
                _set_gain_cell_field_state(item, tunable_b, applicable=False)
        elif current_target == "None":
            _set_gain_cell_field_state(item, tunable_a, applicable=False)
            _set_gain_cell_field_state(item, tunable_b, applicable=False)
        elif current_target == "Velocity":
            if item.mode == JointSettingMode.STIFFNESS:
                _set_gain_cell_field_state(item, WidgetColumns.STIFFNESS, applicable=False)
            else:
                _set_gain_cell_field_state(item, tunable_a, applicable=False)
                _set_gain_cell_field_state(item, tunable_b, applicable=False)


class JointListModel(ui.AbstractItemModel):
    """A data model for managing joint parameters in a table view interface.

    This model provides functionality for displaying and editing joint drive properties such as stiffness, damping,
    natural frequency, and damping ratio. It supports different joint setting modes (stiffness-based or natural
    frequency-based) and handles sorting operations on joint data.

    Args:
        joints_list: List of joint entries to be managed by the model.
        inertia_provider: Function or dictionary that provides inertia values for joints.
            If callable, should accept a joint and return its inertia value.
            If dict, should map joints to their inertia values.
            If None, defaults to returning 0.0 for all joints.
        value_changed_fn: Callback function invoked when joint values are modified.
            Should accept joint and column_id parameters.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(
        self, joints_list: list, inertia_provider: object, value_changed_fn: callable, **kwargs: object
    ) -> None:
        super().__init__()
        self._inertia_provider = inertia_provider or (lambda *_: 0.0)
        self._children = [JointItem(entry, self._inertia_provider, self._on_joint_changed) for entry in joints_list]
        self._joint_changed_fn = value_changed_fn
        self._items_sort_func = None
        self._items_sort_reversed = False
        self._mode = JointSettingMode.STIFFNESS

    def _on_joint_changed(self, joint: object, col_id: int) -> None:
        """Handles joint value changes and forwards them to the registered callback.

        Args:
            joint: The joint item that was changed.
            col_id: The column ID that was modified.
        """
        if self._joint_changed_fn:
            self._joint_changed_fn(joint, col_id)

    def get_item_children(self, item: object = None) -> list[JointItem]:
        """Returns all the children when the widget asks it.

        Args:
            item: The parent item to get children for.

        Returns:
            List of child items, sorted if a sort function is set.
        """
        if item is not None:
            return []
        else:
            children = self._children
            if self._items_sort_func:
                children = sorted(children, key=self._items_sort_func, reverse=self._items_sort_reversed)

            return children

    def get_item_value(self, item: object, column_id: int) -> str | float:
        """Gets the display value for a joint item at the specified column.

        Args:
            item: The joint item to get the value from.
            column_id: The column ID to retrieve the value for.

        Returns:
            The display value for the specified column.
        """
        if item:
            return item.get_item_value(column_id)

    def set_item_value(self, item: object, column_id: int, value: object) -> None:
        """Sets the value for a joint item at the specified column.

        Args:
            item: The joint item to set the value for.
            column_id: The column ID to set the value for.
            value: The new value to set.
        """
        if item:
            item.set_item_value(column_id, value)

    def get_item_value_model_count(self, item: object) -> int:
        """The number of columns.

        Args:
            item: The item to get the column count for.

        Returns:
            Number of columns in the model.
        """
        return 5

    def get_item_value_model(self, item: object, column_id: int) -> ui.AbstractValueModel:
        """Return value model.

            It's the object that tracks the specific value.

        Args:
            item: The joint item to get the model from.
            column_id: The column ID to get the model for.

        Returns:
            The value model for the specified column.
        """
        if item:
            if isinstance(item, JointItem):
                return item.get_value_model(column_id)

    def sort_by_name(self, policy: object, column_id: int) -> None:
        """Sorts the joint items based on the specified policy and column.

        Args:
            policy: The sort policy to apply.
            column_id: The column ID to sort by.
        """
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

    def set_mode(self, mode: object) -> None:
        """Sets the joint setting mode and updates all joint items accordingly.

        Args:
            mode: The joint setting mode to apply.
        """
        if self._mode != mode:
            for item in self.get_item_children():
                # Always derive natural frequency/damping ratio from current stiffness/damping.
                item.natural_frequency = item.compute_natural_frequency()
                item.damping_ratio = item.compute_damping_ratio()
            self._mode = mode
            for item in self.get_item_children():
                item.mode = mode
                self._item_changed(item)
            self._item_changed(None)

    def set_drive_type(self, drive_type: object) -> None:
        """Sets the drive type for all joint items and recalculates drive stiffness.

        Args:
            drive_type: The joint drive type to apply to all items.
        """
        for item in self._children:
            item.drive_type = drive_type
            item.compute_drive_stiffness()
            self._item_changed(item)
        self._item_changed(None)

    def refresh_inertia_derived_columns(self) -> None:
        """Recompute natural frequency and damping ratio after joint inertia is available."""
        for item in self._children:
            item.natural_frequency = item.compute_natural_frequency()
            item.damping_ratio = item.compute_damping_ratio()
        self._item_changed(None)


class JointWidget(TableWidget):
    """A widget for tuning joint drive parameters in Isaac Sim.

    Provides an interactive table interface for adjusting joint stiffness, damping, natural frequency, and damping ratio
    parameters. Supports bulk editing across selected joints and switching between stiffness-based and natural
    frequency-based parameter modes.

    Args:
        joint_entries: List of joint entries containing joint prims and display information.
        inertia_provider: Function or dictionary that provides inertia values for joints. If callable, receives a joint
            and returns its inertia value. If dict, maps joints to inertia values.
        value_changed_fn: Optional callback function invoked when joint parameter values change. Receives the modified
            joint prim as argument.
    """

    def __init__(self, joint_entries: list, inertia_provider: object, value_changed_fn: callable = None) -> None:
        self.joints = joint_entries or []
        self.inertia_provider = inertia_provider or (lambda *_: 0.0)
        self.model = JointListModel(self.joints, self.inertia_provider, self._on_value_changed)
        self.delegate = JointItemDelegate(self.model)
        self._enable_bulk_edit = True
        self._value_changed_fn = value_changed_fn
        self.model._mode = JointSettingMode.STIFFNESS
        self.drive_type = JointDriveType.ACCELERATION
        super().__init__(value_changed_fn, self.model, self.delegate)

    def update_mimic(self) -> None:
        """Updates the mimic joint settings in the delegate."""
        self.delegate.update_mimic()

    def refresh_inertia_derived_columns(self) -> None:
        """Refresh NF/DR columns after accumulated joint inertia is computed."""
        self.model.refresh_inertia_derived_columns()

    def build_tree_view(self) -> None:
        """Builds the tree view widget for displaying joint parameters.

        Creates a TreeView with columns for joint name, drive mode, drive type, stiffness, and damping parameters.
        Configures column widths, resizing behavior, and header visibility.
        """
        self.list = ui.TreeView(
            self.model,
            delegate=self.delegate,
            alignment=ui.Alignment.CENTER_TOP,
            column_widths=[
                ui.Fraction(1),
                ui.Pixel(65),
                ui.Pixel(65),
                ui.Pixel(100),
                ui.Pixel(100),
            ],
            min_column_widths=[80, 65, 65, 100, 70],
            columns_resizable=True,
            header_visible=True,
            resizeable_on_columns_resized=True,
            width=ui.Fraction(1),
            height=ui.Fraction(1),
        )

    def switch_mode(self, switch: JointSettingMode) -> None:
        """Switches between stiffness and natural frequency parameter modes.

        Args:
            switch: The joint setting mode to switch to.
        """
        super().switch_mode(switch)
        self.update_mimic()

    def switch_drive_type(self, drive_type: JointDriveType) -> None:
        """Switches the drive type for all joints in the widget.

        Disables bulk editing, updates the drive type for all joints in the model, updates default values
        if in stiffness mode, and re-enables bulk editing.

        Args:
            drive_type: The joint drive type to switch to.
        """
        self.set_bulk_edit(False)
        drive_type = JointDriveType.ACCELERATION
        if drive_type == JointDriveType.FORCE:
            drive_type = JointDriveType.FORCE
        self.model.set_drive_type(drive_type)
        if self.model._mode == JointSettingMode.STIFFNESS:
            self.delegate.update_defaults()
        self.drive_type = drive_type
        self.set_bulk_edit(True)

    def _on_value_changed(self, joint_item: object, col_id: int = 1) -> None:
        """Handles value changes in joint parameters.

        When bulk edit is enabled, applies the changed value to all selected joints. Temporarily disables
        bulk edit during the update process to prevent recursive updates.

        Args:
            joint_item: The joint item that had its value changed.
            col_id: The column ID indicating which parameter was changed.
        """
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
