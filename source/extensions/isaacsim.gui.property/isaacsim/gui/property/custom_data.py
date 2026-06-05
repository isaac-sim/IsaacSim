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

"""Property widget for viewing and editing prim custom data as JSON."""

import numpy as np
import omni.ui as ui
from omni.kit.property.usd.usd_property_widget import UsdPropertyUiEntry
from omni.kit.window.property.templates import (
    SimplePropertyWidget,
)
from pxr import Sdf, Usd


def iterate_custom_data(custom_data: dict) -> None:
    """Recursively convert numpy arrays in custom data to plain Python lists.

    Args:
        custom_data: The custom data dictionary to convert in-place.
    """
    for key, value in custom_data.items():
        if isinstance(value, dict):
            iterate_custom_data(value)
        else:
            custom_data[key] = np.array(custom_data[key]).tolist()


class CustomDataWidget(SimplePropertyWidget):
    """Property widget for displaying and editing prim custom data as JSON."""

    def _get_prim(self, prim_path: object) -> Usd.Prim | None:
        """Gets the prim at the specified path from the current stage.

        Args:
            prim_path: Path to the prim.

        Returns:
            The prim at the given path, or None if the path is invalid or stage is unavailable.
        """
        if prim_path:
            stage = self._payload.get_stage()
            if stage:
                return stage.GetPrimAtPath(prim_path)
        return None

    def on_new_payload(self, payload: list) -> bool:
        """See ``PropertyWidget.on_new_payload``.

        Args:
            payload: The new prim selection payload.

        Returns:
            Whether the widget should be visible for this payload.
        """
        if not super().on_new_payload(payload):
            return False

        if len(self._payload) != 1:
            return False
        prim_path = self._payload.get_paths()[0]
        self._prim = self._get_prim(prim_path)
        if not self._prim:
            return False

        return True

    def build_items(self) -> None:
        """Build the JSON editor UI for custom data."""
        import json

        def dupe_checking_hook(pairs: list[tuple[str, object]]) -> dict[str, object]:
            result = {}
            for key, val in pairs:
                if key in result:
                    raise KeyError(f"Duplicate key specified: {key}")
                result[key] = val
            return result

        decoder = json.JSONDecoder(object_pairs_hook=dupe_checking_hook)

        data = ui.StringField(height=250, multiline=True).model
        ui.Label("Status:")
        error = ui.StringField(multiline=False).model
        error.set_value("Valid, changes saved")

        def validate(t: ui.AbstractValueModel) -> None:
            try:
                decoder.decode(t.get_value_as_string())
            except ValueError as e:
                error.set_value(str(e))
                return
            except KeyError as e:
                error.set_value(str(e))
                return
            error.set_value("Valid, changes saved")
            self._prim.SetCustomData(decoder.decode(t.get_value_as_string()))

        custom_data = self._prim.GetCustomData()

        iterate_custom_data(custom_data)

        j = json.dumps(custom_data, sort_keys=False, indent=4)

        data.set_value(j)
        data.add_value_changed_fn(lambda m: validate(m))

    def build_property_item(self, stage: Usd.Stage, ui_prop: UsdPropertyUiEntry, prim_paths: list[Sdf.Path]) -> None:
        """Build the UI for a single property item.

        Args:
            stage: The USD stage.
            ui_prop: The property UI entry.
            prim_paths: The prim paths being inspected.
        """
        if ui_prop.prim_paths:
            prim_paths = ui_prop.prim_paths
