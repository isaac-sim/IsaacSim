# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Property widgets for array-type USD attributes."""

import omni
import omni.ui as ui
from omni.kit.property.usd.usd_attribute_model import UsdAttributeModel
from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidget, UsdPropertyUiEntry
from omni.kit.property.usd.usd_property_widget_builder import UsdPropertiesWidgetBuilder
from omni.kit.property.usd.widgets import ICON_PATH
from omni.kit.window.property.templates import HORIZONTAL_SPACING, LABEL_HEIGHT
from pxr import Sdf, Tf, Usd

REMOVE_BUTTON_STYLE = style = {"image_url": str(ICON_PATH.joinpath("remove.svg")), "margin": 0, "padding": 0}
ADD_BUTTON_STYLE = style = {"image_url": str(ICON_PATH.joinpath("plus.svg")), "margin": 1, "padding": 0}


class _BaseMultiField:
    """Base class for creating multi-field UI components with dynamic add/remove functionality.

    This class provides a foundation for building UI widgets that can display and edit array-like data
    structures. It creates a multi-field input widget (like MultiIntField or MultiFloatField) with an
    associated button for adding or removing items. The widget automatically handles value updates and
    notifies the underlying model when changes occur.

    The class supports both editing existing items in a list model and creating new items to be added.
    When editing an existing item, changes are automatically synchronized with the model. When creating
    a new item, the specified default value is used to initialize the field.

    Args:
        model: The data model that manages the list of items.
        index: Index of the item being edited, or None when creating a new item.
        count: Number of components in the multi-field (e.g., 3 for a Vec3 field).
        frame: UI frame that will be rebuilt when items are added or removed.
        button_style: Style dictionary for the action button (add or remove).
        callback: Name of the method to call when the button is clicked ('remove' or 'append').
        field_type: UI field class to instantiate (e.g., ui.MultiIntField, ui.MultiFloatField).
        default: Default value to use when creating new items or initializing empty fields.
    """

    def __init__(
        self,
        model: object,
        index: object,
        count: int,
        frame: object,
        button_style: dict,
        callback: str,
        field_type: type,
        default: object,
    ) -> None:
        self._model = model
        self._index = index
        self._frame = frame
        self._count = count

        item = (
            self._model.get_item(self._index)
            if index is not None
            else [default for i in range(count)] if count > 1 else default
        )
        self._args = [item[i] for i in range(count)] if count != 1 else [item]
        self._field = field_type(*self._args, h_spacing=5).model
        ui.Spacer(width=2)
        ui.Button(style=button_style, width=16, clicked_fn=getattr(self, callback))
        if self._index is not None:
            self._field.add_item_changed_fn(
                lambda m, n, list_model=self._model, field=self._field, index=self._index: list_model.set_item(
                    index, self.get_tuple()
                )
            )

    def get_field_value(self, index: int) -> object:
        """Return the value of the field at the given index.

        Args:
            index: The field index to retrieve.

        Raises:
            NotImplementedError: Subclasses must implement this method.
        """
        raise NotImplementedError

    def remove(self) -> None:
        """Removes the field item from the model and rebuilds the frame."""
        self._model.remove_item(self._index)
        self._frame.rebuild()

    def append(self) -> None:
        """Adds the current field value as a new item to the model and rebuilds the frame."""
        self._model.add_item(self.get_tuple())
        self._frame.rebuild()

    def get_tuple(self) -> object:
        """Current field value as a tuple or single value.

        Returns:
            A tuple of field values if count is greater than 1, otherwise a single field value.
        """
        return (
            tuple([self.get_field_value(i) for i in range(self._count)])
            if self._count != 1
            else self.get_field_value(0)
        )


class _CustomMultiIntField(_BaseMultiField):
    """A multi-integer input field widget for USD array property editing with add/remove functionality.

    This class extends _BaseMultiField to provide a specialized integer input widget used in array property
    editing interfaces. It creates a multi-integer field UI component that allows users to input multiple
    integer values simultaneously, with the ability to add new entries or remove existing ones from USD
    array attributes.

    The widget integrates with the USD property system to handle array modifications and automatically
    rebuild the interface when changes occur. It's specifically designed for integer-based USD array
    attributes like int[], int2[], int3[], and int4[].

    Args:
        model: The USD array attribute model that manages the underlying data.
        index: The index of the array item being edited, or None for new items.
        count: The number of integer fields to display in the multi-field widget.
        frame: The UI frame that contains this widget and handles rebuilding.
        button_style: The style dictionary for the add/remove button appearance.
        callback: The callback method name to execute when the button is clicked.
    """

    def __init__(
        self, model: object, index: object, count: int, frame: object, button_style: dict, callback: str
    ) -> None:
        super().__init__(model, index, count, frame, button_style, callback, ui.MultiIntField, 0)

    def get_field_value(self, index: int) -> int:
        """Return the integer value of the field at the given index.

        Args:
            index: The field index to retrieve.

        Returns:
            The integer value at the given index.
        """
        return self._field.get_item_value_model(self._field.get_item_children()[index]).get_value_as_int()


class _CustomStringField:
    """A single-string input field widget for USD ``string[]``/``token[]`` array editing.

    Unlike :class:`_BaseMultiField`, this class does not extend the multi-component base
    because :class:`omni.ui.StringField` is a single-value field with a different
    constructor signature and a value-model API (``add_value_changed_fn`` and
    ``get_value_as_string``) rather than the item-model API used by
    :class:`omni.ui.MultiIntField` and :class:`omni.ui.MultiFloatField`.

    Args:
        model: The USD array attribute model that manages the underlying data.
        index: The index of the array item being edited, or ``None`` for new items.
        frame: The UI frame that contains this widget and handles rebuilding.
        button_style: The style dictionary for the add/remove button appearance.
        callback: The callback method name to execute when the button is clicked
            (``"remove"`` or ``"append"``).
        default: Default value used when creating new items. Defaults to ``""``.
    """

    def __init__(
        self,
        model: object,
        index: object,
        frame: object,
        button_style: dict,
        callback: str,
        default: str = "",
    ) -> None:
        self._model = model
        self._index = index
        self._frame = frame

        initial = self._model.get_item(self._index) if index is not None else default
        self._field = ui.StringField().model
        self._field.set_value("" if initial is None else str(initial))

        ui.Spacer(width=2)
        ui.Button(style=button_style, width=16, clicked_fn=getattr(self, callback))

        if self._index is not None:
            self._field.add_value_changed_fn(
                lambda m, list_model=self._model, index=self._index: list_model.set_item(index, m.get_value_as_string())
            )

    def get_value(self) -> str:
        """Return the current string value of the field."""
        return self._field.get_value_as_string()

    def remove(self) -> None:
        """Remove this item from the model and rebuild the frame."""
        self._model.remove_item(self._index)
        self._frame.rebuild()

    def append(self) -> None:
        """Append the current field value as a new item to the model and rebuild the frame."""
        self._model.add_item(self.get_value())
        self._frame.rebuild()


class _CustomMultiFloatField(_BaseMultiField):
    """Custom multi-float field widget for editing array elements in USD property interfaces.

    This widget creates an interactive UI field that allows users to edit floating-point values within USD arrays.
    It displays multiple float input fields based on the specified count and provides a button for adding or removing
    array elements. The widget automatically updates the underlying data model when values are changed and rebuilds
    the parent frame when structural changes occur.

    Args:
        model: The data model that manages the array values and provides methods for getting and setting items.
        index: The index of the current item in the array. If None, creates a new item with default values.
        count: The number of float fields to display (e.g., 1 for scalar, 2 for Vec2f, 3 for Vec3f, 4 for Vec4f).
        frame: The parent UI frame that will be rebuilt when array structure changes.
        button_style: Style dictionary for the action button, typically containing icon and styling properties.
        callback: Name of the callback method to invoke when the button is clicked (e.g., 'remove' or 'append').
    """

    def __init__(
        self, model: object, index: object, count: int, frame: object, button_style: dict, callback: str
    ) -> None:
        super().__init__(model, index, count, frame, button_style, callback, ui.MultiFloatField, 0.0)

    def get_field_value(self, index: int) -> float:
        """Return the float value of the field at the given index.

        Args:
            index: The field index to retrieve.

        Returns:
            The float value at the given index.
        """
        return self._field.get_item_value_model(self._field.get_item_children()[index]).get_value_as_float()


class _ArrayBaseItem:
    """Base item class for creating UI elements for array attribute values.

    This class serves as a foundation for building interactive UI components that handle individual
    elements within USD array attributes. It creates appropriate input fields based on the array type
    and manages the interaction between the UI and the underlying data model.

    The class supports various USD array types including integer arrays, float arrays, vector
    arrays of different dimensions (Vec2, Vec3, Vec4), and string/token arrays. It automatically
    determines the appropriate UI field type and count based on the provided type information.

    Args:
        type_name: The USD type information specifying the array element type.
        model: The data model that manages the array values and provides access to individual elements.
        index: The position of this item within the array. If None, creates a template item for adding
            new elements.
        frame: The UI frame container that holds this item and manages rebuilding when the array
            structure changes.
    """

    def __init__(self, type_name: object, model: object, index: object = None, frame: object = None) -> None:
        self._value = None
        self._type_name = type_name
        self._model = model
        self._index = index
        self._frame = frame

    def create_list_item(self, button_style: dict, callback: str) -> None:
        """Creates a list item UI element based on the array type.

        Creates an appropriate input field (int, float, or string, single or multi-field) with an action button
        based on the array type name. The UI is built within a horizontal stack and includes the appropriate field
        type for editing array elements.

        Args:
            button_style: Style configuration for the action button.
            callback: Callback method name to invoke when the button is clicked.
        """
        with ui.HStack(height=0):
            if self._type_name == Tf.Type.FindByName("VtArray<int>"):
                _CustomMultiIntField(self._model, self._index, 1, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<float>"):
                _CustomMultiFloatField(self._model, self._index, 1, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec2i>"):
                _CustomMultiIntField(self._model, self._index, 2, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec2f>"):
                _CustomMultiFloatField(self._model, self._index, 2, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec3i>"):
                _CustomMultiIntField(self._model, self._index, 3, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec3f>"):
                _CustomMultiFloatField(self._model, self._index, 3, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec4i>"):
                _CustomMultiIntField(self._model, self._index, 4, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<GfVec4f>"):
                _CustomMultiFloatField(self._model, self._index, 4, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<string>"):
                _CustomStringField(self._model, self._index, self._frame, button_style, callback)
            elif self._type_name == Tf.Type.FindByName("VtArray<TfToken>"):
                _CustomStringField(self._model, self._index, self._frame, button_style, callback)


class _ArrayWidgetBuilder:
    """A widget builder for editing USD array attributes in the property panel.

    Creates a UI widget that displays array attribute information including the attribute name, type,
    and current length, along with an "Edit" button that opens a dedicated editing window. The editing
    window allows users to modify, add, and remove individual array elements with appropriate input
    fields based on the array's data type.

    Supports various USD array types including int[], float[], vector arrays (int2[], float2[],
    int3[], float3[], int4[], float4[]), and string[]/token[].

    Args:
        stage: The USD stage containing the attribute.
        attr_name: Name of the array attribute to edit.
        type_name: USD type information for the array attribute.
        model: The UsdArrayAttributeModel that manages the attribute data.
    """

    def __init__(self, stage: object, attr_name: str, type_name: object, model: object) -> None:
        self._model = model
        self._stage = stage
        self._attr_name = attr_name
        self._type_name = type_name
        label_kwargs = {"name": "label", "word_wrap": True, "height": LABEL_HEIGHT}
        ui.Label(attr_name, **label_kwargs)
        ui.Label(type_name.typeName, **label_kwargs)
        self._length_label = ui.Label(str(self._model.get_length()), **label_kwargs)
        ui.Button("Edit", clicked_fn=self._create_edit_window)

    def _create_edit_window(self) -> None:
        """Creates and displays the array editing window.

        Opens a new window with controls for editing array elements, including scrollable list of
        existing items with remove buttons and an interface for adding new items.
        """
        self._window = omni.ui.Window(
            f"Editing: {self._attr_name} {self._type_name.typeName}", width=600, height=400, visible=True
        )

        with self._window.frame:
            with ui.VStack():
                frame = ui.Frame()

                def rebuild() -> None:
                    with ui.ScrollingFrame():
                        with ui.VStack():
                            for i in range(self._model.get_length()):
                                mod = _ArrayBaseItem(self._type_name, self._model, i, frame)
                                mod.create_list_item(REMOVE_BUTTON_STYLE, "remove")
                    self._length_label.text = str(self._model.get_length())

                frame.set_build_fn(rebuild)
                ui.Spacer(height=1)
                ui.Label("Add new item:", height=0)
                ui.Spacer(height=2)
                mod = _ArrayBaseItem(self._type_name, self._model, None, frame)
                mod.create_list_item(ADD_BUTTON_STYLE, "append")
                ui.Spacer(height=10)


class _UsdArrayAttributeModel(UsdAttributeModel):
    """A model for managing USD array attribute properties.

    This class extends the base UsdAttributeModel to provide specialized functionality for handling array-type USD
    attributes. It enables manipulation of array elements including retrieval, modification, addition, and removal
    of items within USD array attributes. The model supports various array types including integer arrays, float
    arrays, and vector arrays with different dimensions.

    The model provides methods to access individual array elements, modify specific indices, and perform array
    operations such as appending new items or removing existing ones. It maintains synchronization with the
    underlying USD attribute data and automatically updates the attribute value when changes are made to the array
    contents.
    """

    def get_length(self) -> int:
        """Number of items in the USD array attribute.

        Returns:
            The number of items in the array.
        """
        self._update_value()
        return len(self._value)

    def get_value(self) -> object:
        """Current value of the USD array attribute.

        Returns:
            The current value of the array attribute.
        """
        return self._value

    def get_item(self, index: int) -> object:
        """Retrieves an item from the USD array attribute at the specified index.

        Args:
            index: The index of the item to retrieve.

        Returns:
            The item at the specified index.
        """
        return self._value[index]

    def set_item(self, index: int, item: object) -> None:
        """Updates an item in the USD array attribute at the specified index.

        Args:
            index: The index of the item to update.
            item: The new value to set at the specified index.
        """
        new_list = list(self._value)
        new_list[index] = item
        self.set_value(new_list)

    def add_item(self, item: object) -> None:
        """Appends a new item to the end of the USD array attribute.

        Args:
            item: The item to append to the array.
        """
        new_list = list(self._value)
        new_list.append(item)
        self.set_value(new_list)

    def remove_item(self, index: int) -> None:
        """Removes an item from the USD array attribute at the specified index.

        Args:
            index: The index of the item to remove.
        """
        new_list = list(self._value)
        del new_list[index]
        self.set_value(new_list)


class ArrayPropertiesWidget(UsdPropertiesWidget):
    """A specialized properties widget for editing USD array-based attributes in Isaac Sim.

    This widget extends UsdPropertiesWidget to provide editing functionality for USD attributes that contain
    arrays of various types including integers, floats, and multi-dimensional vectors. It creates an interactive
    interface allowing users to view, add, remove, and modify individual array elements through dedicated edit
    windows.

    The widget supports array attributes of the following types:
    - ``int[]``: Arrays of integers
    - ``float[]``: Arrays of floats
    - ``int2[]``, ``float2[]``: Arrays of 2D vectors
    - ``int3[]``, ``float3[]``: Arrays of 3D vectors
    - ``int4[]``, ``float4[]``: Arrays of 4D vectors
    - ``string[]``, ``token[]``: Arrays of strings or tokens

    When an array attribute is detected, the widget displays the attribute name, type, and current array length,
    along with an "Edit" button. Clicking this button opens a dedicated editing window where users can:
    - View all existing array elements in a scrollable list
    - Remove individual elements using remove buttons
    - Add new elements using input fields and an add button
    - See real-time updates to the array length

    The editing interface automatically adapts to the array element type, providing appropriate input fields
    (single values for scalars, multi-field inputs for vectors) and handles type conversion between the UI
    and USD attribute values.
    """

    def on_new_payload(self, payload: list) -> bool:
        """See ``PropertyWidget.on_new_payload``.

        Args:
            payload: The new prim selection payload.

        Returns:
            Whether the widget should be visible for this payload.
        """
        if not super().on_new_payload(payload):
            return False

        if len(self._payload) == 0:
            return False

        for prim_path in self._payload:
            prim = self._get_prim(prim_path)
            if not prim:
                return False

        return True

    def _filter_props_to_build(self, props: list) -> list:
        """Filters properties to include only array-based USD attributes.

        Args:
            props: List of USD properties to filter.

        Returns:
            List of properties that are array-based attributes with supported types.
        """
        # simple widget that handles array based properties
        return [
            prop
            for prop in props
            if isinstance(prop, Usd.Attribute)
            and any(
                prop.GetTypeName() == item
                for item in [
                    "int[]",
                    "float[]",
                    "int2[]",
                    "float2[]",
                    "int3[]",
                    "float3[]",
                    "int4[]",
                    "float4[]",
                    "string[]",
                    "token[]",
                ]
            )
        ]

    def build_property_item(self, stage: Usd.Stage, ui_prop: UsdPropertyUiEntry, prim_paths: list[Sdf.Path]) -> None:
        """Build the UI for a single array property.

        Args:
            stage: The USD stage.
            ui_prop: The property UI entry.
            prim_paths: The prim paths being inspected.
        """
        if ui_prop.prim_paths:
            prim_paths = ui_prop.prim_paths

        if ui_prop.property_type == Usd.Attribute:
            type_name = UsdPropertiesWidgetBuilder._get_type_name(ui_prop.metadata)
            tf_type = type_name.type

            self._array_builder(stage, ui_prop.prop_name, tf_type, ui_prop.metadata, prim_paths)

    @classmethod
    def _array_builder(
        cls,
        stage: object,
        attr_name: str,
        type_name: object,
        metadata: dict,
        prim_paths: list[Sdf.Path],
        additional_label_kwargs: object = None,
        additional_widget_kwargs: object = None,
    ) -> _UsdArrayAttributeModel:
        """Creates an array widget builder for USD array attributes.

        Args:
            stage: The USD stage containing the attributes.
            attr_name: Name of the attribute to build.
            type_name: Type information for the attribute.
            metadata: Metadata associated with the attribute.
            prim_paths: List of USD prim paths for the attributes.
            additional_label_kwargs: Additional keyword arguments for label styling.
            additional_widget_kwargs: Additional keyword arguments for widget styling.

        Returns:
            The UsdArrayAttributeModel instance created for the attribute.
        """
        with ui.HStack(spacing=HORIZONTAL_SPACING):
            model = _UsdArrayAttributeModel(
                stage, [path.AppendProperty(attr_name) for path in prim_paths], False, metadata
            )
            _ArrayWidgetBuilder(stage, attr_name, type_name, model)

            return model
