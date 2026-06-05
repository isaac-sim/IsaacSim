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

"""Base class for asset transformer actions."""

__all__ = ["AssetTransformerAction"]


import carb
import omni.ui as ui


class AssetTransformerAction:
    """Base class for asset transformer actions.

    Subclass this to create custom actions. Override ``build_ui`` to provide
    custom configuration UI that will be shown when the action row is expanded.

    Args:
        action_name: Human-readable name for this action.
    """

    def __init__(self, action_name: str) -> None:
        self.__name = action_name
        self.__model = ui.SimpleBoolModel()

        self.__model.set_value(True)

    def run(self) -> bool:
        """Execute the action.

        Returns:
            True if the action completed successfully.

        Example:

        .. code-block:: python

            action = AssetTransformerAction("my_action")
            success = action.run()
        """
        carb.log_warn(f"Action [{self.__name}] STUB")
        return True

    def build_ui(self) -> None:
        """Build custom UI for this action's configuration.

        This method is called when the action row is expanded in the TreeView.
        Override this method in subclasses to provide action-specific input
        controls.  The UI is built within the context of a VStack, so widgets
        will be arranged vertically by default.
        """
        # Default implementation shows a placeholder message
        ui.Label(
            f"No configuration options for '{self.__name}'",
            name="placeholder_config",
        )

    @property
    def name(self) -> str:
        """Human-readable name of this action.

        Returns:
            The action name.
        """
        return self.__name

    @property
    def model(self) -> ui.AbstractValueModel:
        """Boolean value model tracking the enabled state.

        Returns:
            The value model for the enabled state.
        """
        return self.__model

    @property
    def enabled(self) -> bool:
        """Whether this action is enabled.

        Returns:
            True if the action is enabled, False otherwise.
        """
        return self.__model.get_value_as_bool()

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self.__model.set_value(value)
