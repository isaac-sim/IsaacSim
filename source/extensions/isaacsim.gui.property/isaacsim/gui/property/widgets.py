# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Extension entry point that registers Isaac property widgets and the Joint Inspector menu."""

import omni.ext
import omni.kit.actions.core
import omni.kit.menu.utils
import omni.usd
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, refresh_menu_items, remove_menu_items
from omni.kit.property.usd.usd_property_widget import MultiSchemaPropertiesWidget
from pxr import Usd

from .array_widget import ArrayPropertiesWidget
from .custom_data import CustomDataWidget
from .joint_inspector import WINDOW_TITLE as JOINT_INSPECTOR_TITLE
from .joint_inspector import JointInspectorWindowManager
from .motion_planning_schema import MotionPlanningAPIWidget
from .name_override import NameOverrideWidget
from .namespace import NamespaceWidget
from .robot_schema import AttachmentPointAPIWidget, JointAPIWidget, LinkAPIWidget, RobotAPIWidget, SiteAPIWidget

_ISAAC_API_SCHEMA_NAMES = (
    "IsaacRobotAPI",
    "IsaacLinkAPI",
    "IsaacJointAPI",
    "IsaacSiteAPI",
    "IsaacMotionPlanningAPI",
    "IsaacAttachmentPointAPI",
)

_JOINT_INSPECTOR_ACTION_ID = f"open_joint_inspector:{JOINT_INSPECTOR_TITLE}"
_JOINT_INSPECTOR_ACTION_DESCRIPTION = f"Open the {JOINT_INSPECTOR_TITLE} window"


class IsaacPropertyWidgets(omni.ext.IExt):
    """Extension that provides custom property widgets and the Joint Inspector window.

    Registers the following property panel widgets for Isaac Sim robotics workflows:

    - ``Array Properties`` -- editor for array-valued USD attributes.
    - ``Prim Custom Data`` -- JSON editor for prim custom metadata.
    - ``Name Override`` -- Isaac name-override attribute for base-name lookups.
    - ``Namespace`` -- Isaac namespace attribute for hierarchical grouping.
    - ``Robot Schema`` -- authoring surface for ``IsaacRobotAPI`` prims.
    - ``Robot Link`` -- authoring surface for ``IsaacLinkAPI`` prims.
    - ``Robot Joint`` -- authoring surface for ``IsaacJointAPI`` prims.
    - ``Robot Site`` -- authoring surface for ``IsaacSiteAPI`` prims.
    - ``Motion Planning`` -- authoring surface for ``IsaacMotionPlanningAPI`` prims.

    Also mounts a ``Tools > Robotics > Joint Inspector`` menu entry that opens a standalone,
    multi-window Joint Inspector tool with a searchable robot dropdown.
    """

    def __init__(self) -> None:
        super().__init__()
        self._registered = False
        self._robot_api_widget = None
        self._link_api_widget = None
        self._joint_api_widget = None
        self._site_api_widget = None
        self._attachment_point_api_widget = None
        self._motion_planning_api_widget = None
        self._isaac_array_widget: ArrayPropertiesWidget | None = None
        self._isaac_custom_data_widget: CustomDataWidget | None = None
        self._isaac_name_override = None
        self._isaac_namespace = None
        self._isaac_api_schema_anchor: MultiSchemaPropertiesWidget | None = None
        self._joint_inspector_manager: JointInspectorWindowManager | None = None
        self._joint_inspector_action_registered: bool = False
        self._joint_inspector_menu_items: list = []
        self._ext_id: str = ""

    def on_startup(self, ext_id: str) -> None:
        """Register all Isaac property widgets and the Joint Inspector menu entry.

        Args:
            ext_id: The extension identifier.
        """
        self._ext_id = ext_id
        self._register_widget()
        self._register_joint_inspector_menu()

    def on_shutdown(self) -> None:
        """Unregister all Isaac property widgets and tear down the Joint Inspector menu."""
        self._unregister_joint_inspector_menu()
        self._unregister_widget()

    def _register_widget(self) -> None:
        """Register Isaac property widgets with the property window."""
        # Register Isaac API schemas with Kit's MultiSchemaPropertiesWidget so
        # the generic "Extra Properties" panel does not duplicate attributes
        # that are already owned by our widgets. The supported entry point is
        # the `api_schemas=` constructor argument, which Kit adds to its
        # (private) known-schemas set; we retain the anchor instance so the
        # destructor -- which removes the schemas from that set -- does not
        # fire until we explicitly release it at shutdown.
        self._isaac_api_schema_anchor = MultiSchemaPropertiesWidget(
            title="IsaacApiSchemaAnchor",
            schema=Usd.Typed,
            schema_subclasses=[],
            api_schemas=list(_ISAAC_API_SCHEMA_NAMES),
        )

        import omni.kit.window.property as p

        w = p.get_window()
        self._isaac_array_widget = ArrayPropertiesWidget(title="Array Properties", collapsed=True)
        w.register_widget("prim", "isaac_array", self._isaac_array_widget, False)
        self._isaac_custom_data_widget = CustomDataWidget(title="Prim Custom Data", collapsed=True)
        w.register_widget("prim", "isaac_custom_data", self._isaac_custom_data_widget, False)
        self._isaac_name_override = NameOverrideWidget(title="Name Override", collapsed=False)
        w.register_widget("prim", "isaac_name_override", self._isaac_name_override, False)
        self._isaac_namespace = NamespaceWidget(title="Namespace", collapsed=False)
        w.register_widget("prim", "isaac_namespace", self._isaac_namespace, False)
        self._robot_api_widget = RobotAPIWidget(title="Robot Schema", collapsed=False)
        w.register_widget("prim", "isaac_robot_api", self._robot_api_widget, True)
        self._link_api_widget = LinkAPIWidget(title="Robot Link", collapsed=False)
        w.register_widget("prim", "isaac_link_api", self._link_api_widget, True)
        self._joint_api_widget = JointAPIWidget(title="Robot Joint", collapsed=False)
        w.register_widget("prim", "isaac_joint_api", self._joint_api_widget, True)
        self._site_api_widget = SiteAPIWidget(title="Robot Site", collapsed=False)
        w.register_widget("prim", "isaac_site_api", self._site_api_widget, True)
        self._attachment_point_api_widget = AttachmentPointAPIWidget(title="Attachment Point", collapsed=False)
        w.register_widget("prim", "isaac_attachment_point_api", self._attachment_point_api_widget, True)
        self._motion_planning_api_widget = MotionPlanningAPIWidget(title="Motion Planning", collapsed=False)
        w.register_widget("prim", "isaac_motion_planning_api", self._motion_planning_api_widget, False)
        self._registered = True

    def _unregister_widget(self) -> None:
        """Unregister Isaac property widgets and release the schema anchor."""
        import omni.kit.window.property as p

        w = p.get_window()
        if w:
            w.unregister_widget("prim", "isaac_array")
            w.unregister_widget("prim", "isaac_custom_data")
            w.unregister_widget("prim", "isaac_name_override")
            w.unregister_widget("prim", "isaac_namespace")
            w.unregister_widget("prim", "isaac_robot_api")
            w.unregister_widget("prim", "isaac_link_api")
            w.unregister_widget("prim", "isaac_joint_api")
            w.unregister_widget("prim", "isaac_site_api")
            w.unregister_widget("prim", "isaac_attachment_point_api")
            w.unregister_widget("prim", "isaac_motion_planning_api")
            # ArrayPropertiesWidget and CustomDataWidget are plain UsdPropertiesWidget
            # subclasses with no destroy() override; unregister_widget releases them.
            self._isaac_array_widget = None
            self._isaac_custom_data_widget = None
            if self._isaac_name_override:
                self._isaac_name_override.destroy()
                self._isaac_name_override = None
            if self._isaac_namespace:
                self._isaac_namespace.destroy()
                self._isaac_namespace = None
            if self._robot_api_widget:
                self._robot_api_widget.destroy()
            if self._link_api_widget:
                self._link_api_widget.destroy()
            if self._joint_api_widget:
                self._joint_api_widget.destroy()
            if self._site_api_widget:
                self._site_api_widget.destroy()
            if self._attachment_point_api_widget:
                self._attachment_point_api_widget.destroy()
            if self._motion_planning_api_widget:
                self._motion_planning_api_widget.destroy()
            self._registered = False

        # Drop the anchor AFTER unregistering widgets so Kit's `__del__` on the
        # anchor (which calls `difference_update` on the known schemas set) is
        # the final step, leaving the set in its pre-startup state.
        self._isaac_api_schema_anchor = None

    def _register_joint_inspector_menu(self) -> None:
        """Add the ``Tools/Robotics/Joint Inspector`` menu entry and bind the manager."""
        self._joint_inspector_manager = JointInspectorWindowManager(
            on_primary_visibility_changed=lambda _v: refresh_menu_items("Tools")
        )

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            self._ext_id,
            _JOINT_INSPECTOR_ACTION_ID,
            self._joint_inspector_manager.open_or_focus_primary,
            description=_JOINT_INSPECTOR_ACTION_DESCRIPTION,
        )
        self._joint_inspector_action_registered = True

        leaf = MenuItemDescription(
            name=JOINT_INSPECTOR_TITLE,
            onclick_action=(self._ext_id, _JOINT_INSPECTOR_ACTION_ID),
            ticked=True,
            ticked_fn=self._joint_inspector_manager.is_primary_visible,
        )
        robotics_submenu = MenuItemDescription(name="Robotics", sub_menu=[leaf])
        self._joint_inspector_menu_items = [robotics_submenu]
        add_menu_items(self._joint_inspector_menu_items, "Tools")

    def _unregister_joint_inspector_menu(self) -> None:
        """Remove the menu entry and destroy any open Joint Inspector windows."""
        if self._joint_inspector_menu_items:
            remove_menu_items(self._joint_inspector_menu_items, "Tools")
            self._joint_inspector_menu_items = []

        if self._joint_inspector_action_registered:
            action_registry = omni.kit.actions.core.get_action_registry()
            action_registry.deregister_action(self._ext_id, _JOINT_INSPECTOR_ACTION_ID)
            self._joint_inspector_action_registered = False

        if self._joint_inspector_manager:
            self._joint_inspector_manager.shutdown()
            self._joint_inspector_manager = None
