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

"""Utility classes and functions for Newton physics UI property builders."""

import carb
import omni.ui as ui
from omni.kit.property.usd.usd_attribute_model import UsdAttributeModel
from omni.kit.property.usd.usd_property_widget_builder import UsdPropertiesWidgetBuilder

try:
    from newton._src.usd.schema_resolver import PrimType
except ImportError:
    from enum import IntEnum

    class PrimType(IntEnum):  # Stub matching newton.PrimType values
        """Stub enum matching newton.PrimType values for use when Newton is not available."""

        SCENE = 0
        JOINT = 1
        SHAPE = 2
        BODY = 3
        MATERIAL = 4
        ACTUATOR = 5
        ARTICULATION = 6


_RESOLVERS = None
_RESOLVER_INDEX = None
_RESOLVER_NAMES = None


def make_hide_cb(own_resolver_name: str, prim_type: "PrimType", key: str | list, default: object = None):
    """Create a disable callback for a property covered by both Newton and Mjc resolvers.

    Resolvers are ordered by priority (Newton first, Mjc second). The *preferred* resolver is
    the first one that has authored a value for any of the keys. If nothing is authored by
    anyone, the first resolver (Newton) is preferred by default.

    The callback returns a 3-tuple ``(disabled, resolver_display_name, usd_attr_name)``:

    - ``disabled``: ``False`` if own resolver is preferred, ``True`` otherwise.
    - ``resolver_display_name``: human-readable name of the preferred resolver (e.g.
      ``"MuJoCo"``), or ``""`` if own is preferred.
    - ``usd_attr_name``: USD attribute name authored by the preferred resolver (e.g.
      ``"newton:gravityEnabled"``), or the name from the preferred resolver's mapping if
      nothing is authored. Empty string if own is preferred.

    Args:
        own_resolver_name: ``"newton"`` or ``"mjc"`` — identifies which resolver owns
            this property (matched against ``SchemaResolver.name``).
        prim_type: :class:`PrimType` enum value for the prim category (SCENE, JOINT, …).
        key: Resolver key string (e.g. ``"armature"``), or a list of key strings when
            multiple keys map to the same USD attribute (e.g. ``"mjc:solref"``).
            The property is considered authored if *any* of the keys has a value.
        default: Unused; kept for call-site documentation of the simulation default.

    Returns:
        A callback with signature ``(stage, prim_paths) -> tuple[bool, str, str]``.
    """
    keys = key if isinstance(key, list) else [key]

    def _resolver_authored_attr(r: object, prim: object) -> str | None:
        """Returns the USD attribute name of the first authored key, or None.

        Args:
            r: The schema resolver to query for authored values.
            prim: The USD prim to check for authored attributes.

        Returns:
            The USD attribute name of the first authored key, or None.
        """
        for k in keys:
            if r.get_value(prim, prim_type, k) is not None:
                spec = r.mapping.get(prim_type, {}).get(k)
                return spec.name if spec else k
        return None

    def _cb(stage, prim_paths):
        global _RESOLVERS, _RESOLVER_INDEX, _RESOLVER_NAMES

        if not prim_paths or stage is None:
            return False, "", ""

        if _RESOLVERS is None:
            try:
                from newton._src.usd.schemas import SchemaResolverMjc, SchemaResolverNewton

                _RESOLVERS = [SchemaResolverNewton(), SchemaResolverMjc()]
                _RESOLVER_NAMES = ["Newton", "MuJoCo"]
                _RESOLVER_INDEX = {r.name: idx for idx, r in enumerate(_RESOLVERS)}
            except ImportError:
                _RESOLVERS = None
        if _RESOLVERS is None:
            return False, "", ""

        own_idx = _RESOLVER_INDEX.get(own_resolver_name, -1)
        if own_idx == -1:
            carb.log_warn(f"Own resolver {own_resolver_name} not found for {prim_type} {key}")
            return False, "", ""

        prim = stage.GetPrimAtPath(str(prim_paths[0]))

        # Find preferred resolver: first with an authored value, or first overall if nothing is authored.
        # authored_attr is the corresponding USD attribute name; falls back to the mapping name when nothing is authored.
        authored_attrs = [_resolver_authored_attr(r, prim) for r in _RESOLVERS]
        authored_idx = next((i for i, a in enumerate(authored_attrs) if a is not None), None)
        preferred_idx = authored_idx if authored_idx is not None else 0
        if authored_idx is not None:
            authored_attr = authored_attrs[preferred_idx]
        else:
            spec = _RESOLVERS[0].mapping.get(prim_type, {}).get(keys[0])
            authored_attr = spec.name if spec else keys[0]

        if preferred_idx == own_idx:
            return False, "", ""
        return True, _RESOLVER_NAMES[preferred_idx], authored_attr or ""

    return _cb


class DisableByCallbackBuilder(UsdPropertiesWidgetBuilder):
    """Widget builder that disable a property based on a callback result."""

    def __new__(cls, stage, prop, prim_paths, label_kwargs, widget_kwargs, disable_callback):
        """Create a new widget with an overlay that disables based on a callback."""

        def _tooltip(resolver_name, attr_name):
            if not resolver_name:
                return ""
            return f"Controlled by {attr_name}" if attr_name else f"Controlled by {resolver_name}"

        disabled, resolver_name, attr_name = disable_callback(stage, prim_paths)
        with ui.ZStack():
            model = cls.build(
                stage, prop.prop_name, prop.metadata, prop.property_type, prim_paths, label_kwargs, widget_kwargs
            )
            overlay = ui.Rectangle(
                width=ui.Fraction(1),
                height=ui.Fraction(1),
                style={"background_color": ui.color(0, 0, 0, 64), "border_radius": 4},
                visible=disabled,
                tooltip=_tooltip(resolver_name, attr_name),
            )

        def _refresh(*_):
            disabled, resolver_name, attr_name = disable_callback(stage, prim_paths)
            overlay.visible = disabled
            overlay.set_tooltip(_tooltip(resolver_name, attr_name))

        model._remove_if_default = True
        if not hasattr(model, "_newton_disable_subs"):
            model._newton_disable_subs = []
        model._newton_disable_subs.append(model.subscribe_value_changed_fn(_refresh))
        return model


class HideByCallbackBuilder(UsdPropertiesWidgetBuilder):
    """Widget builder that hide a property based on a callback result."""

    def __new__(cls, stage, prop, prim_paths, label_kwargs, widget_kwargs, hide_callback):
        """Create a new widget that is hidden when the callback returns True."""
        hidden = hide_callback(stage, prim_paths)
        if not hidden:
            model = cls.build(
                stage, prop.prop_name, prop.metadata, prop.property_type, prim_paths, label_kwargs, widget_kwargs
            )
            return model
        else:
            return UsdAttributeModel(
                stage, [path.AppendProperty(prop.prop_name) for path in prim_paths], False, prop.metadata
            )
