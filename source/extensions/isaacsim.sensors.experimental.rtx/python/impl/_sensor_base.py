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

"""Base classes for RTX sensor authoring and runtime."""

from __future__ import annotations

import difflib
import pathlib
import weakref
from abc import ABC, abstractmethod
from collections.abc import Iterable
from typing import Any, Literal, Self

import carb
import carb.eventdispatcher
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.hydratexture
import omni.replicator.core as rep
import warp as wp
from isaacsim.core.experimental.prims import XformPrim
from pxr import Sdf, Usd, UsdRender

from ._common import ANNOTATOR_SPEC, WRITER_SPEC
from .spg import SPGNode, _author_spg, _endpoint_is_port

ANNOTATOR = Literal[
    "generic-model-output",
    "stable-id-map",
]


def _config_aliases(config_path: str) -> tuple[str, str, str, str, str]:
    """Return the five accepted aliases for a registry entry.

    The aliases are, in order:

    1. The full Isaac Sim asset path (e.g. ``/Isaac/Sensors/SICK/picoScan100/SICK_picoScan100.usd``).
    2. The USD file stem (e.g. ``SICK_picoScan100``).
    3. The stem with underscores replaced by spaces (e.g. ``SICK picoScan100``).
    4. The vendor-stripped stem (e.g. ``picoScan100``).
    5. The vendor-stripped stem with underscores replaced by spaces (e.g. ``picoScan 100``).

    The vendor is taken from the fourth path component (``/Isaac/Sensors/<Vendor>/...``);
    when the stem starts with ``<Vendor>_`` that prefix is stripped to produce the
    vendor-stripped form. The vendor-stripped form falls back to the bare stem when
    the path does not follow the conventional layout.

    Args:
        config_path: Registry asset path.

    Returns:
        Accepted aliases for the registry entry.
    """
    _p = pathlib.Path(config_path)
    _vendor = _p.parts[3] if len(_p.parts) > 3 else ""
    _stem = _p.stem
    _stem_no_vendor = _stem[len(_vendor) + 1 :] if _vendor and _stem.startswith(_vendor + "_") else _stem
    return (
        config_path,
        _stem,
        _stem.replace("_", " "),
        _stem_no_vendor,
        _stem_no_vendor.replace("_", " "),
    )


def _resolve_config_path(config: str, registry: Iterable[str], *, sensor_type: str) -> str:
    """Resolve a user-supplied config name to a registry asset path.

    Each entry in *registry* is matched against the five aliases produced by
    :func:`_config_aliases`. On a miss, a :class:`ValueError` is raised whose message:

    * lists the short (vendor-stripped) config names rather than the full asset paths,
    * includes a "Did you mean..." suggestion derived from :func:`difflib.get_close_matches`
      across all aliases, and
    * points the reader to ``SUPPORTED_<TYPE>_CONFIGS`` for the full asset paths and
      per-config variant sets.

    Args:
        config: The user-supplied config name.
        registry: Iterable of registry asset paths (typically the keys of one of
            the ``SUPPORTED_*_CONFIGS`` mappings).
        sensor_type: Capitalized sensor type (e.g. ``"Lidar"``, ``"Radar"``, ``"Acoustic"``)
            used in the error message and the ``SUPPORTED_*_CONFIGS`` reference.

    Returns:
        The matching registry asset path.

    Raises:
        ValueError: If no registry entry matches *config*.
    """
    short_names: list[str] = []
    unique_aliases: list[str] = []
    seen_aliases: set[str] = set()
    for config_path in registry:
        aliases = _config_aliases(config_path)
        if config in aliases:
            return config_path
        short_names.append(aliases[3])
        # Dedup while preserving registry order so "Did you mean..." returns
        # distinct suggestions instead of repeating identical aliases produced
        # by entries whose stem and vendor-stripped stem coincide (e.g. ``OS1``).
        for alias in aliases:
            if alias not in seen_aliases:
                seen_aliases.add(alias)
                unique_aliases.append(alias)

    parts = [f"{sensor_type} config '{config}' not found."]
    suggestions = difflib.get_close_matches(config, unique_aliases, n=3, cutoff=0.6)
    if suggestions:
        parts.append(f"Did you mean: {', '.join(repr(s) for s in suggestions)}?")
    if short_names:
        parts.append(f"Available configs: {', '.join(sorted(set(short_names)))}.")
    parts.append(
        f"See SUPPORTED_{sensor_type.upper()}_CONFIGS for the full asset paths " "and per-config variant sets."
    )
    raise ValueError(" ".join(parts))


class SensorAuthoring(XformPrim, ABC):
    """Base class for RTX sensor authoring (USD prim creation and wrapping).

    Handles path resolution, prim type validation, schema and attribute application,
    tick rate configuration, and USD reference loading. Subclasses define the
    sensor-specific prim type, schema, and creation logic.

    Wrapping a prim of the expected type applies :attr:`_SCHEMA` when the prim does not
    already have it, since the sensor is inoperable without it.

    Do not instantiate this class directly; use a concrete subclass such as
    :class:`Lidar`, :class:`Radar`, :class:`Acoustic`, or :class:`RtxCamera`.

    Subclasses must define:
        _PRIM_TYPE: str — USD prim type name (e.g. ``"OmniLidar"``)
        _SCHEMA: str — API schema name (e.g. ``"OmniSensorGenericLidarCoreAPI"``)
        _VALID_AUX_OUTPUT_LEVELS: tuple[str, ...] — valid ``aux_output_level`` values
        _create_prim(path, attributes) -> str — create a new prim and return its actual path

    Subclasses may override:
        _ALLOW_MULTIPLE_PRIMS: bool — whether the wrapper may create or wrap more than one prim
        (e.g. :class:`RtxCamera`, which batches several cameras)

    Args:
        path: USD path, list of paths, or regex for the sensor prim(s) to create or wrap.
            Multiple prims are only accepted by subclasses that set ``_ALLOW_MULTIPLE_PRIMS``.
        aux_output_level: Auxiliary output level for the GenericModelOutput RenderVar.
        tick_rate: Sensor tick rate to apply to the prim, or None to preserve authored values.
        schemas: API schemas to apply to the prim.
        attributes: Attributes to set on the prim.
        positions: World positions forwarded to the base transform wrapper.
        translations: Local translations forwarded to the base transform wrapper.
        orientations: World orientations forwarded to the base transform wrapper.
        scales: Scales forwarded to the base transform wrapper.
        reset_xform_op_properties: Whether to reset the xformOp stack.
    """

    _PRIM_TYPE: str
    _SCHEMA: str
    _VALID_AUX_OUTPUT_LEVELS: tuple[str, ...] = ("NONE",)
    # batched authoring classes (e.g. `RtxCamera`) opt out of the single-prim restriction
    _ALLOW_MULTIPLE_PRIMS: bool = False

    def __init__(
        self,
        path: str | list[str],
        *,
        aux_output_level: str = "NONE",
        tick_rate: float | None = None,
        schemas: list[str] | None = None,
        attributes: dict[str, Any] | None = None,
        # XformPrim
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
    ) -> None:
        # validate aux_output_level
        if aux_output_level not in self._VALID_AUX_OUTPUT_LEVELS:
            raise ValueError(
                f"Invalid aux_output_level '{aux_output_level}' for {type(self).__name__}. "
                f"Valid values: {self._VALID_AUX_OUTPUT_LEVELS}"
            )
        self._aux_output_level = aux_output_level
        self._asset_root_path: str | None = None
        existent_paths, nonexistent_paths = XformPrim.resolve_paths(path)
        resolved_count = len(existent_paths) + len(nonexistent_paths)
        if resolved_count > 1 and not self._ALLOW_MULTIPLE_PRIMS:
            raise ValueError(
                f"Only one {self._PRIM_TYPE} prim is supported, "
                f"but the provided argument refers to {resolved_count} prims: "
                f"{existent_paths + nonexistent_paths}",
            )
        authored_tick_rates: dict[str, Any] = {}
        # wrap existing prims
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                prim = prim_utils.get_prim_at_path(path)
                type_name = prim.GetPrimTypeInfo().GetTypeName()
                if type_name != self._PRIM_TYPE:
                    raise ValueError(f"Prim at {path} is not an '{self._PRIM_TYPE}' prim but a '{type_name}' prim")
            for path in existent_paths:
                prim = prim_utils.get_prim_at_path(path)
                attribute = prim.GetAttribute("omni:sensor:tickRate")
                if attribute.IsValid() and attribute.HasAuthoredValue():
                    authored_tick_rates[path] = attribute.Get()
                # The sensor is inoperable without its schema, so apply it rather than reject the prim.
                if self._SCHEMA not in prim.GetAppliedSchemas():
                    prim.ApplyAPI(self._SCHEMA)
            # apply additional schemas before setting attributes
            if schemas is not None:
                for path in existent_paths:
                    self._apply_schemas(path, schemas)
            if attributes is not None:
                for path in existent_paths:
                    self._apply_attributes(path, attributes)
        # create new prims
        else:
            paths = []
            for path in nonexistent_paths:
                actual_path = self._create_prim(path, attributes)
                paths.append(actual_path)
            # apply additional schemas after prim creation
            if schemas is not None:
                for p in paths:
                    self._apply_schemas(p, schemas)
            # apply attributes after all schemas are present
            if attributes is not None:
                for p in paths:
                    self._apply_attributes(p, attributes)
        # resolve tick rate: attributes dict takes precedence over tick_rate parameter.
        # ``tick_rate=None`` (the default) means "do not modify the prim attribute",
        # so any value already authored on the prim (e.g. from a USD asset) is preserved.
        if attributes is not None and "omni:sensor:tickRate" in attributes:
            if tick_rate is not None:
                carb.log_warn(
                    "Both 'tick_rate' parameter and 'omni:sensor:tickRate' attribute were provided. "
                    "Using the value from 'attributes'."
                )
            tick_rate = attributes["omni:sensor:tickRate"]
        if tick_rate is not None:
            for p in paths:
                prim = prim_utils.get_prim_at_path(p)
                if not prim.HasAttribute("omni:sensor:tickRate"):
                    continue
                attribute = prim.GetAttribute("omni:sensor:tickRate")
                # An authored value is a deliberate opinion (from a sensor asset or an earlier wrapper),
                # unlike the `OmniSensorAPI` schema default. Capture it before applying `attributes`,
                # which may contain its own `omni:sensor:tickRate` override.
                authored = authored_tick_rates.get(p)
                if authored is not None and float(authored) != float(tick_rate):
                    carb.log_warn(
                        f"{type(self).__name__} at '{p}': overwriting the authored "
                        f"'omni:sensor:tickRate' ({authored} Hz) with {tick_rate} Hz. The tick rate is stored "
                        "on the shared prim, so other sensors may observe the new value. Active render products "
                        "may retain a previously latched rate until they are recreated. Pass 'tick_rate=None' "
                        "to preserve the authored value."
                    )
                attribute.Set(tick_rate)
        # set aux_output_level as channels attribute on the sensor prim so the
        # Replicator pipeline propagates it to the GenericModelOutput RenderVar
        for p in paths:
            prim = prim_utils.get_prim_at_path(p)
            prim.CreateAttribute(
                "_replicator:rendervar:GenericModelOutput:channels", Sdf.ValueTypeNames.StringArray, True
            ).Set([self._aux_output_level])
        # initialize base class
        super().__init__(
            paths,
            resolve_paths=False,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
        )

    @abstractmethod
    def _create_prim(self, path: str, attributes: dict[str, Any] | None) -> str:
        """Create a new sensor prim.

        Args:
            path: USD path for the new prim.
            attributes: Attributes to set on the prim.

        Returns:
            Actual prim path after creation.
        """

    @staticmethod
    def _apply_schemas(path: str, schemas: list[str]) -> None:
        """Apply API schemas to a prim.

        Each entry can be a plain schema name (e.g. ``"OmniLensDistortionOpenCvFisheyeAPI"``)
        or a multi-instance schema with a colon-separated instance name
        (e.g. ``"OmniSensorGenericLidarCoreEmitterStateAPI:s002"``).

        Args:
            path: USD prim path.
            schemas: API schemas to apply.
        """
        prim = prim_utils.get_prim_at_path(path)
        for schema in schemas:
            if ":" in schema:
                schema_name, instance_name = schema.rsplit(":", 1)
                prim.ApplyAPI(schema_name, instance_name)
            else:
                if not prim.HasAPI(schema):
                    prim.ApplyAPI(schema)

    @staticmethod
    def _apply_attributes(path: str, attributes: dict[str, Any]) -> None:
        """Apply attributes to an existing prim.

        Iterate over the key-value pairs in ``attributes`` and set each on the
        prim at the given path. If an attribute does not exist on the prim, a
        warning is logged and the attribute is skipped.

        Args:
            path: USD prim path.
            attributes: Mapping of attribute names to values.
        """
        prim = prim_utils.get_prim_at_path(path)
        for attribute, value in attributes.items():
            if prim.HasAttribute(attribute):
                prim.GetAttribute(attribute).Set(value)
            else:
                carb.log_warn(
                    f"Prim (at path '{prim_utils.get_prim_path(prim)}') " f"does not have attribute '{attribute}'"
                )

    @classmethod
    def _create_from_usd(
        cls,
        *,
        path: str,
        usd_path: str | None = None,
        variant: str | dict[str, str] | None = None,
        variant_set_name: str = "sensor",
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
        **kwargs: Any,
    ) -> Self:
        """Add a USD reference to the stage and wrap the sensor prim within it.

        Assets that nest the sensor prim under the reference root place it alongside the housing
        geometry and author the vendor's mounting offset on it, so the transform arguments are
        applied to the reference root rather than the sensor prim. That keeps the housing and the
        sensor origin rigidly attached and preserves the mounting offset. When the sensor prim is
        the reference root, or ``usd_path`` is ``None``, the transform arguments are forwarded to
        the wrapper directly.

        Args:
            path: Target prim path on stage.
            usd_path: Path to the USD file, or ``None`` to create or wrap a prim at ``path``
                without referencing an asset.
            variant: Variant selection. Either a flat string (applied against
                ``variant_set_name``), or a dict of ``{set_name: variant_name}``
                pairs for USDs with multiple variant sets, or ``None``. Nested
                variants supported via dictionary; pairs applied in dict
                insertion order, so outer variant sets must come first.
            variant_set_name: Variant set name used when ``variant`` is a string.
            positions: Positions in the world frame.
            translations: Translations in the local frame.
            orientations: Orientations in the world frame.
            scales: Scales to be applied.
            reset_xform_op_properties: Whether to reset the transformation operation attributes.
            **kwargs: Sensor-specific keyword arguments forwarded to the constructor.

        Returns:
            Sensor instance wrapping the sensor prim found within the reference.

        Raises:
            ValueError: If no prim of the sensor's type is found within the reference.
        """
        asset_root_path = None
        if usd_path is not None:
            asset_root_path = path
            prim_type = cls._PRIM_TYPE if usd_path.endswith(".usda") else "Xform"
            if variant is None:
                variants = []
            elif isinstance(variant, str):
                variants = [(variant_set_name, variant)]
            else:
                variants = list(variant.items())
            stage_utils.add_reference_to_stage(usd_path=usd_path, path=path, prim_type=prim_type, variants=variants)
            predicate = lambda prim, path: prim.GetTypeName() == cls._PRIM_TYPE
            sensor_prim = prim_utils.get_first_matching_child_prim(path, predicate=predicate, include_self=True)
            if sensor_prim is None:
                raise ValueError(
                    f"Unable to find {cls._PRIM_TYPE} prim in the USD file {usd_path} (variant: {variant})"
                )
            path = prim_utils.get_prim_path(sensor_prim)
        author_on_root = asset_root_path is not None and asset_root_path != path
        sensor = cls(
            path=path,
            positions=None if author_on_root else positions,
            translations=None if author_on_root else translations,
            orientations=None if author_on_root else orientations,
            scales=None if author_on_root else scales,
            reset_xform_op_properties=reset_xform_op_properties,
            **kwargs,
        )
        if author_on_root:
            XformPrim(
                asset_root_path,
                positions=positions,
                translations=translations,
                orientations=orientations,
                scales=scales,
                reset_xform_op_properties=reset_xform_op_properties,
            )
        sensor._asset_root_path = asset_root_path
        return sensor

    @property
    def asset_root_path(self) -> str | None:
        """The reference root of the USD asset this sensor was loaded from.

        Transform arguments passed to ``create()`` are authored on this prim when the asset
        nests the sensor prim beneath it.

        Returns:
            The reference root prim path, or ``None`` when the sensor was not loaded from an asset.

        Example:

        .. code-block:: python

            >>> lidar.asset_root_path  # doctest: +NO_CHECK
            '/World/lidar'
        """
        return self._asset_root_path

    @property
    def aux_output_level(self) -> str:
        """The auxiliary output level configured on the GenericModelOutput RenderVar.

        Returns:
            The configured level (e.g. ``"NONE"``, ``"BASIC"``, ``"EXTRA"``, ``"FULL"``).
        """
        return self._aux_output_level

    def author_spg(
        self,
        nodes: SPGNode | list[SPGNode],
        connections: list[tuple[str, str]] | None = None,
        *,
        render_product: str | None = None,
        resolution: tuple[int, int] = (1920, 1080),
        copy_to: str | None = None,
    ) -> str:
        """Author an RTX Sensor Processing Graph (SPG) onto this sensor's render product.

        Authors one ``UsdShade.Shader`` prim per :class:`~isaacsim.sensors.experimental.rtx.SPGNode`
        directly under a render product tied to this sensor, wires the requested
        connections, and ensures the referenced ``RenderVar`` AOVs exist. The render
        product is resolved in this order: an explicit ``render_product`` path, an
        existing render product whose ``camera`` relationship targets this sensor prim,
        or a newly created one at ``<sensor>_RenderProduct``.

        Connections use the declarative style of ``omni.graph``'s ``Controller.edit``:
        each ``(src, dst)`` pair connects a producer to a consumer, where a bare
        endpoint (e.g. ``"LdrColor"``) names an AOV ``RenderVar`` and a
        ``"Node.inputs:X"`` / ``"Node.outputs:Y"`` endpoint names a shader port. A
        source AOV (consumed by a shader input) that has no ``RenderVar`` yet triggers
        a warning and is created; shader-output AOVs are created silently.

        Only structural validation is performed (``.cu`` / co-located ``.cu.lua``
        existence and prim wiring). CUDA / Lua / SPG semantic validation is left to the
        ``omni.rtx.spg`` extension at render time.

        Args:
            nodes: A single :class:`SPGNode` or a list of them. Node names must be unique.
            connections: ``(src, dst)`` endpoint pairs to wire. When ``None``, no
                connections are authored (useful when a caller wires them separately).
            render_product: Explicit render product path to author onto, or ``None`` to
                find-or-create one tied to this sensor.
            resolution: Resolution as ``(width, height)`` for a newly created render product.
            copy_to: Optional directory into which each distinct ``.cu`` and its
                co-located ``.cu.lua`` are copied (e.g. to assemble a self-contained
                sensor asset). Each shader's ``info:spg:sourceAsset`` then points at the
                copied file.

        Returns:
            The render product prim path the graph was authored onto.

        Raises:
            ValueError: If this wrapper holds more than one sensor prim.
            ValueError: If node names are not unique; if a node name collides with an AOV
                name used in ``connections``; if the prim at an explicit ``render_product``
                path exists but is not a ``RenderProduct``; or if a connection endpoint names
                a shader port not declared by the supplied nodes.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.experimental.rtx import RtxCamera, SPGNode
            >>>
            >>> cam = RtxCamera("/World/camera")
            >>> cam.author_spg(  # doctest: +SKIP
            ...     nodes=[
            ...         SPGNode("Grayscale", "kernels/GrayscaleKernel.cu",
            ...                 inputs=["LdrColor"], outputs=["LdrGrayscale"]),
            ...         SPGNode("Invert", "kernels/InvertKernel.cu",
            ...                 inputs=["Image"], outputs=["Inverted"], params={"strength": 1.0}),
            ...     ],
            ...     connections=[
            ...         ("LdrColor", "Grayscale.inputs:LdrColor"),
            ...         ("Grayscale.outputs:LdrGrayscale", "Invert.inputs:Image"),
            ...         ("Invert.outputs:Inverted", "LdrInverted"),
            ...     ],
            ... )
        """
        if len(self.paths) > 1:
            raise ValueError(
                f"author_spg supports a single sensor prim, but {type(self).__name__} wraps "
                f"{len(self.paths)} prims: {self.paths}"
            )
        nodes = [nodes] if isinstance(nodes, SPGNode) else list(nodes)
        names = [node.name for node in nodes]
        if len(names) != len(set(names)):
            raise ValueError(f"SPGNode names must be unique, got {names}")
        # Shader prims are authored at ``{rp}/{node.name}`` and bare AOV endpoints author a
        # RenderVar at ``{rp}/{aov}``; a shared name would collide at the same prim path.
        aov_names = {ep for pair in (connections or []) for ep in pair if not _endpoint_is_port(ep)}
        collisions = set(names) & aov_names
        if collisions:
            raise ValueError(
                f"SPGNode name(s) {sorted(collisions)} collide with AOV name(s) used in connections; "
                "a Shader prim and a RenderVar would be authored at the same path. Rename the node(s) or AOV(s)."
            )
        return _author_spg(
            self.paths[0],
            nodes,
            connections,
            render_product=render_product,
            resolution=resolution,
            copy_to=copy_to,
            asset_root_path=getattr(self, "_asset_root_path", None),
        )


class SensorRuntime(ABC):
    """Base class for RTX sensor runtime (annotator management and data retrieval).

    Wraps a sensor authoring object, creates a Replicator render product, and
    provides methods to attach/detach annotators and fetch sensor data. Subclasses
    define the authoring class type and a typed property to expose it.

    Do not instantiate this class directly; use a concrete subclass such as
    :class:`LidarSensor`, :class:`RadarSensor`, :class:`AcousticSensor`, or
    :class:`CameraSensor`.

    Subclasses must define:
        _AUTHORING_CLASS: type — the authoring class (e.g. ``Lidar``)
        _AUTHORING_ATTR: str — attribute name for the encapsulated object (e.g. ``"_lidar"``)

    Subclasses may override:
        _ALLOW_MULTIPLE_AUTHORING_OBJECTS: bool — whether the runtime may wrap more than one prim
        (e.g. :class:`TiledCameraSensor`, which batches several cameras into a single render product)

    Args:
        path: Sensor authoring object, or path (or paths) to sensor prims. Multiple prims are only
            accepted by subclasses that set ``_ALLOW_MULTIPLE_AUTHORING_OBJECTS``.
        annotators: Annotator types to configure.
        annotator_init_params: Per-annotator initialization parameters forwarded to Replicator annotators.
            Semantic filtering is the exception: ``semanticTypes``/``semanticFilter`` applies to the whole
            render product, so bounding box and segmentation annotators sharing one render product cannot
            be filtered independently.
        writers: Writer types to attach.
        render_vars: Render variables to pass to the render product.
    """

    _AUTHORING_CLASS: type[XformPrim]
    _AUTHORING_ATTR: str
    # batched runtimes (e.g. `TiledCameraSensor`) opt out of the single-prim restriction
    _ALLOW_MULTIPLE_AUTHORING_OBJECTS: bool = False
    # Optional schema filter for pre-authored render-product attachment.
    _ASSET_RP_SCHEMA: str | None = None

    def __init_subclass__(cls, **kwargs: Any) -> None:
        """Require subclasses to define authoring class wiring.

        Args:
            **kwargs: Keyword arguments forwarded to the parent class.
        """
        super().__init_subclass__(**kwargs)
        for attr in ("_AUTHORING_CLASS", "_AUTHORING_ATTR"):
            if not hasattr(cls, attr):
                raise TypeError(f"{cls.__name__} must define class attribute {attr}")

    def __init__(
        self,
        path: str | list[str] | XformPrim,
        *,
        annotators: ANNOTATOR | list[ANNOTATOR] | None = None,
        annotator_init_params: dict[str, dict[str, Any]] | None = None,
        writers: str | list[str] | None = None,
        render_vars: list[str] | None = None,
    ) -> None:
        if type(self) is SensorRuntime:
            raise TypeError(
                "SensorRuntime is abstract and cannot be instantiated directly. "
                "Use a concrete subclass such as LidarSensor, RadarSensor, "
                "AcousticSensor, or CameraSensor."
            )
        # define properties early so __del__ is safe if __init__ raises
        self._hydra_texture = None
        self._data_ready = False
        self._rendering_subscription = None
        self._annotators = {}
        self._writers = {}
        if not hasattr(self, "_annotators_spec"):
            self._annotators_spec = dict(ANNOTATOR_SPEC)
        if not hasattr(self, "_writers_spec"):
            self._writers_spec = dict(WRITER_SPEC)
        # check for supported annotators / writers
        if annotators is not None:
            self._validate_annotators(annotators)
            self._validate_annotator_init_params(annotators, annotator_init_params)
        if writers is not None:
            self._validate_writers(writers)
        # Resolve the prim count before constructing the authoring object. Authoring classes that
        # accept multiple prims (e.g. `RtxCamera`) create prims and apply schemas on construction,
        # so a single-prim runtime must reject the argument before the stage is modified.
        if not self._ALLOW_MULTIPLE_AUTHORING_OBJECTS:
            if isinstance(path, self._AUTHORING_CLASS):
                resolved_paths = list(path.paths)
            else:
                existent_paths, nonexistent_paths = XformPrim.resolve_paths(path)
                resolved_paths = existent_paths + nonexistent_paths
            if len(resolved_paths) > 1:
                raise ValueError(
                    f"Only one {self._AUTHORING_CLASS.__name__} prim is supported, "
                    f"but the provided argument refers to {len(resolved_paths)} prims: {resolved_paths}",
                )
        # get or create authoring object
        authoring_obj = self._get_or_create_authoring_object(path)
        setattr(self, self._AUTHORING_ATTR, authoring_obj)
        # initialize instance from arguments
        self._initialize_sensor(annotators or [], render_vars=render_vars, annotator_init_params=annotator_init_params)
        self._subscribe_to_render_events()
        # attach writers requested at construction time
        if writers is not None:
            writers = [writers] if isinstance(writers, str) else writers
            for writer_name in writers:
                self.attach_writer(writer_name)

    def __del__(self) -> None:
        """Clean up instance."""
        self._invalidate_sensor()

    @property
    def authoring_object(self) -> XformPrim:
        """The authoring object (prim wrapper) encapsulated by the sensor.

        Subclasses expose the same object through a typed property (e.g. :attr:`LidarSensor.lidar`).

        Returns:
            The sensor's authoring object (e.g. :class:`Lidar`, :class:`Radar`, :class:`Acoustic`,
            or :class:`RtxCamera`).
        """
        return getattr(self, self._AUTHORING_ATTR)

    @property
    def annotators(self) -> list[str]:
        """Annotators.

        Returns:
            Sorted list of registered annotators.
        """
        return sorted(self._annotators.keys())

    @property
    def render_product(self) -> UsdRender.Product:
        """Render product.

        Returns:
            Render product of the sensor.
        """
        prim = prim_utils.get_prim_at_path(self._hydra_texture.path)
        if prim.IsValid() and prim.IsA(UsdRender.Product):
            return UsdRender.Product(prim)
        raise RuntimeError(f"Invalid render product at path '{self._hydra_texture.path}'")

    def has_data(self) -> bool:
        """Return whether the sensor has rendered data and still owns its render product.

        The state becomes true when the render product completes a frame containing at least one
        annotator output, without requiring :meth:`get_data` to be called. It also becomes true when
        :meth:`get_data` retrieves a non-empty payload on render-product implementations that do not
        expose render-completion events.

        This is a render-product-level gate intended to bound an annotator warm-up wait: it reports
        that the render product is bound to a working render engine and is producing output. It does
        not guarantee that a particular annotator has a frame ready, so a sensor with several
        annotators may report true while :meth:`get_data` still returns ``None`` for some of them.
        Call :meth:`get_data` to establish per-annotator readiness.

        The state is scoped to the currently attached annotators, so it reverts to false once they
        have all been detached.

        Returns:
            True if the sensor has rendered data and the render product exists.

        Example:

        .. code-block:: python

            >>> sensor.has_data()  # doctest: +NO_CHECK
            False
        """
        return bool(self._annotators) and self._render_product_exists() and self._data_ready

    def attach_annotators(
        self,
        annotators: str | list[str],
        *,
        annotator_init_params: dict[str, dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        """Attach annotators to the sensor.

        Args:
            annotators: Annotator/sensor types to attach.
            annotator_init_params: Per-annotator initialization parameters forwarded to Replicator annotators.
                Semantic filtering is the exception: ``semanticTypes``/``semanticFilter`` applies to the whole
                render product, so bounding box and segmentation annotators sharing one render product cannot
                be filtered independently.

        Returns:
            Mapping from annotator name to attached annotator instance.

        Raises:
            ValueError: If the specified annotator is not supported.
        """
        annotators = [annotators] if isinstance(annotators, str) else annotators
        self._validate_annotators(annotators)
        self._validate_annotator_init_params(annotators, annotator_init_params)
        for annotator in annotators:
            spec = self._get_annotator_spec(annotator)
            init_params = self._get_annotator_init_params(annotator, annotator_init_params)
            kwargs = {
                "device": spec.get("device", "cuda"),
                "do_array_copy": False,
            }
            if init_params:
                kwargs["init_params"] = init_params
            self._annotators[annotator] = rep.AnnotatorRegistry.get_annotator(spec["name"], **kwargs)
        for annotator in annotators:
            self._annotators[annotator].attach(self._hydra_texture.path)

        return {annotator: self._annotators[annotator] for annotator in annotators}

    def detach_annotators(self, annotators: str | list[str]) -> None:
        """Detach annotators from the sensor.

        Args:
            annotators: Annotator/sensor types to detach. If the annotator is not attached,
                or it has already been detached, a warning is logged and the method does nothing.

        Raises:
            ValueError: If the specified annotator is not supported.
        """
        annotators = [annotators] if isinstance(annotators, str) else annotators
        self._validate_annotators(annotators)
        for annotator in annotators:
            if annotator not in self._annotators:
                carb.log_warn(f"Unable to detach annotator '{annotator}'. It might have been already detached")
                continue
            self._annotators[annotator].detach([self._hydra_texture.path])
            del self._annotators[annotator]
        # drop the render-completion state so re-attaching starts a fresh warm-up
        if not self._annotators:
            self._data_ready = False

    def get_data(self, annotator: str) -> tuple[wp.array | None, dict[str, Any]]:
        """Fetch the specified annotator/sensor data for the sensor.

        Args:
            annotator: Annotator/sensor type from which fetch the data.

        Returns:
            Two-elements tuple. 1) Array containing the fetched data. While the annotator is warming up
            this is ``None`` when the annotator reports no payload, or an empty array when it reports an
            empty one; use :meth:`has_data` to bound the wait. 2) Dictionary containing additional
            information according to the requested annotator/sensor.

        Raises:
            ValueError: If the specified annotator is not supported.
            ValueError: If the specified annotator is not configured.
        """
        self._validate_annotators(annotator)
        if annotator not in self._annotators:
            raise ValueError(f"The annotator '{annotator}' was not configured. Enable it when instantiating the class")
        data = self._annotators[annotator].get_data("cuda")
        if isinstance(data, dict):
            info = data["info"]
            data = data["data"]
        else:
            info = {}
        # Latch for has_data() only: the returned tuple keeps whatever the annotator reported,
        # including any info accompanying an empty payload during warm-up.
        self._record_fetched_annotator_data(data)
        return data, info

    def attach_writer(self, writer_name: str, **kwargs: Any) -> rep.Writer:
        """Attach a writer to the sensor's render product.

        ``writer_name`` can be either a short name registered in :data:`WRITER_SPEC`
        (e.g. ``"draw-point-cloud"``) or a Replicator writer registry name
        (e.g. ``"RtxSensorDebugDrawPointCloud"``).  When a spec is found, its
        ``"defaults"`` are merged with the provided *kwargs* (explicit kwargs win).

        Args:
            writer_name: Writer spec name or Replicator writer registry name.
            **kwargs: Keyword arguments forwarded to ``writer.initialize()``.

        Returns:
            Attached Replicator writer instance.

        Example:

        .. code-block:: python

            >>> sensor.attach_writer(
            ...     "draw-point-cloud",
            ...     size=0.05,
            ...     color=[0.0, 1.0, 0.5, 1.0],
            ... )  # doctest: +NO_CHECK
        """
        spec = self._writers_spec.get(writer_name)
        if spec is not None:
            registry_name = spec["name"]
            merged = {**spec.get("defaults", {}), **kwargs}
        else:
            registry_name = writer_name
            merged = kwargs
        writer = rep.writers.get(registry_name)
        writer.initialize(**merged)
        writer.attach([self._hydra_texture.path])
        self._writers[writer_name] = writer

        return writer

    def detach_writer(self, writer_name: str) -> None:
        """Detach a previously attached writer.

        Args:
            writer_name: Writer spec name or Replicator writer registry name used in :meth:`attach_writer`.
        """
        writer = self._writers.pop(writer_name, None)
        if writer is not None:
            writer.detach()
        else:
            carb.log_warn(f"Unable to detach writer '{writer_name}'. It might have been already detached")

    def _record_fetched_annotator_data(self, data: Any) -> bool:
        """Record a fetched annotator payload and return whether it is usable.

        Args:
            data: Annotator payload, or ``None`` when no frame is available.

        Returns:
            True when ``data`` holds a frame. Payloads without a shape (e.g. packed sensor buffers)
            count as data whenever they are not ``None``.
        """
        if data is None:
            return False
        shape = getattr(data, "shape", None)
        if shape is not None and len(shape) > 0 and not shape[0]:
            return False
        self._data_ready = True
        return True

    def _render_product_exists(self) -> bool:
        """Return whether the hydra texture still points at a USD RenderProduct prim.

        Returns:
            True if the sensor's render product prim is present on the stage.
        """
        path = getattr(self._hydra_texture, "path", None)
        if not path:
            return False
        prim = prim_utils.get_prim_at_path(path)
        return bool(prim.IsValid() and prim.IsA(UsdRender.Product))

    def _subscribe_to_render_events(self) -> None:
        """Subscribe to native Hydra texture events when the render product exposes them."""
        hydra_texture = getattr(self._hydra_texture, "hydra_texture", None)
        # `get_event_key` filters the subscription; `get_aov_info` inspects the rendered frame
        required_attrs = ("get_event_key", "get_aov_info")
        if hydra_texture is None or not all(hasattr(hydra_texture, attr) for attr in required_attrs):
            return

        owner_ref = weakref.ref(self)

        def on_event(event: Any) -> None:
            owner = owner_ref()
            if owner is not None:
                owner._on_render_done(event)

        self._rendering_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            observer_name=f"{type(self).__name__}:{id(self)}",
            event_name=omni.hydratexture.GLOBAL_EVENT_DRAWABLE_CHANGED,
            filter=hydra_texture.get_event_key(),
            on_event=on_event,
        )

    def _on_render_done(self, event: Any) -> None:
        """Record render completion when at least one annotator output is available.

        Args:
            event: Hydra texture drawable-change event.
        """
        # this runs for every rendered frame, so skip the AOV query once the state is latched
        if self._data_ready:
            return
        hydra_texture = getattr(self._hydra_texture, "hydra_texture", None)
        result_handle = event.payload.get("result_handle")
        if hydra_texture is None or result_handle is None:
            return
        if hydra_texture.get_aov_info(result_handle):
            self._data_ready = True

    def _invalidate_sensor(self) -> None:
        """Invalidate sensor by detaching writers, annotators, and destroying the hydra texture."""
        self._rendering_subscription = None
        if self._hydra_texture is not None:
            for writer in self._writers.values():
                writer.detach()
            self.detach_annotators(list(self._annotators.keys()))
            self._hydra_texture.destroy()
        self._writers = {}
        self._annotators = {}
        self._hydra_texture = None
        self._data_ready = False

    def _get_or_create_authoring_object(self, path: str | list[str] | XformPrim) -> XformPrim:
        """Get the supplied authoring object or construct one from paths.

        Args:
            path: Sensor authoring object, or path input accepted by the authoring class.

        Returns:
            Sensor authoring object.
        """
        return path if isinstance(path, self._AUTHORING_CLASS) else self._AUTHORING_CLASS(path)

    def _find_asset_render_product(self) -> Usd.Prim | None:
        """Find a pre-authored render product in the loaded USD asset for this sensor.

        Looks for a ``RenderProduct`` prim whose ``camera`` relationship targets the wrapped
        sensor prim. When :attr:`_ASSET_RP_SCHEMA` is set, only render products with that schema
        applied are matched; when it is ``None`` (the default), any matching render product is
        accepted.

        When the authoring object was created via a ``create()`` classmethod with a
        ``usd_path`` (i.e., ``_asset_root_path`` is set), the search is scoped to that
        asset subtree; otherwise the whole stage is searched, skipping transient render
        products under ``/Render``.

        Returns:
            The matching ``RenderProduct`` prim, or ``None`` when none is found.
        """
        stage = stage_utils.get_current_stage(backend="usd")
        sensor_prim_path = self.authoring_object.paths[0]
        asset_root_path = getattr(self.authoring_object, "_asset_root_path", None)
        if asset_root_path is not None:
            root_prim = stage.GetPrimAtPath(asset_root_path)
            if not root_prim.IsValid():
                carb.log_warn(
                    f"Asset root prim at '{asset_root_path}' is not valid. "
                    "Cannot discover a pre-authored render product."
                )
                return None
            search = Usd.PrimRange(root_prim)
        else:
            search = stage.Traverse()

        for prim in search:
            if str(prim.GetPath()).startswith("/Render"):
                continue
            if (
                prim.GetTypeName() == "RenderProduct"
                and (self._ASSET_RP_SCHEMA is None or prim.HasAPI(self._ASSET_RP_SCHEMA))
                and prim.HasRelationship("camera")
            ):
                targets = prim.GetRelationship("camera").GetTargets()
                if len(targets) == 1 and str(targets[0]) == sensor_prim_path:
                    return prim
        return None

    def _annotators_from_render_vars(self, render_product_prim: Usd.Prim) -> list[str]:
        """Derive annotator keys from a render product's authored render vars.

        Maps each render var's ``sourceName`` back to an annotator key using this
        sensor's annotator spec. Unmapped render vars are skipped with a warning.

        Args:
            render_product_prim: Render product whose ``orderedVars`` declare the render vars.

        Returns:
            List of annotator keys in the render product's ``orderedVars`` order.
        """
        stage = stage_utils.get_current_stage(backend="usd")
        name_to_key = {spec["name"]: key for key, spec in self._annotators_spec.items()}
        annotators = []
        for target in render_product_prim.GetRelationship("orderedVars").GetTargets():
            source_name = stage.GetPrimAtPath(target).GetAttribute("sourceName").Get()
            key = name_to_key.get(source_name)
            if key is None:
                carb.log_warn(f"Render var '{source_name}' has no matching annotator spec; skipping.")
            else:
                annotators.append(key)
        return annotators

    def _on_asset_render_product_found(self, render_product_prim: Usd.Prim) -> None:
        """Hook called when a pre-authored render product is found, before attachment.

        Override in subclasses to adopt properties from the pre-authored render product
        (e.g., :class:`CameraSensor` uses this to adopt the authored resolution).

        Args:
            render_product_prim: The matching pre-authored ``RenderProduct`` prim.
        """

    def _create_render_product_and_attach(
        self,
        annotators: str | list[str],
        *,
        render_vars: list[str] | None = None,
        annotator_init_params: dict[str, dict[str, Any]] | None = None,
    ) -> None:
        """Create a new render product and attach annotators.

        Override in subclasses to customize render product creation (e.g.,
        :class:`CameraSensor` creates a resolution-aware render product).

        Args:
            annotators: Annotators to attach after creating the render product.
            render_vars: Render variable names to author, or ``None`` to use Replicator defaults.
            annotator_init_params: Per-annotator initialization parameters forwarded to Replicator annotators.
                Semantic filtering is the exception: ``semanticTypes``/``semanticFilter`` applies to the whole
                render product, so bounding box and segmentation annotators sharing one render product cannot
                be filtered independently.
        """
        self._hydra_texture = rep.create.render_product(
            camera=self.authoring_object.paths[0],
            resolution=(300, 300),  # (width, height), needed but unused by the RTX sensor
            name=f"rtx_sensor_{hash(self)}",
            render_vars=render_vars,
        )
        self.attach_annotators(annotators, annotator_init_params=annotator_init_params)

    def _initialize_sensor(
        self,
        annotators: str | list[str],
        *,
        render_vars: list[str] | None = None,
        annotator_init_params: dict[str, dict[str, Any]] | None = None,
    ) -> None:
        """Initialize sensor by creating or attaching to a render product, then attaching annotators.

        When a matching pre-authored render product is found in the loaded USD asset (see
        :meth:`_find_asset_render_product`), a hydra texture is attached directly to it.
        Otherwise :meth:`_create_render_product_and_attach` is called to create a new one.

        Args:
            annotators: Annotators to attach. If empty when reusing an asset render product, infer them from
                its authored render variables.
            render_vars: Render variables for a newly created render product, or ``None`` to use Replicator
                defaults.
            annotator_init_params: Per-annotator initialization parameters forwarded to Replicator annotators.
                Semantic filtering is the exception: ``semanticTypes``/``semanticFilter`` applies to the whole
                render product, so bounding box and segmentation annotators sharing one render product cannot
                be filtered independently.
        """
        asset_render_product = self._find_asset_render_product()

        if asset_render_product is None:
            self._create_render_product_and_attach(
                annotators, render_vars=render_vars, annotator_init_params=annotator_init_params
            )
            return

        render_product_path = str(asset_render_product.GetPath())
        self._on_asset_render_product_found(asset_render_product)

        # SRTX backs the render product with a hydra texture itself; otherwise attach one explicitly.
        # attach_hydra_texture may return a real HydraTexture or a plain path string (viewport-backed).
        # In the SRTX case, and whenever a plain string is returned, we hold a path-only wrapper so
        # teardown never destroys the asset prim.
        hydra_texture = None
        if not carb.settings.get_settings().get_as_bool("/exts/omni.replicator.srtx/enabled"):
            try:
                hydra_texture = rep.vp_manager.attach_hydra_texture(render_product_path)
            except Exception as error:
                carb.log_warn(
                    f"Failed to attach a hydra texture to pre-authored render product "
                    f"'{render_product_path}' ({error}); falling back to creating a new render product."
                )
                self._create_render_product_and_attach(
                    annotators or [], render_vars=render_vars, annotator_init_params=annotator_init_params
                )
                return
        if hydra_texture is None or isinstance(hydra_texture, str):
            hydra_texture = rep.vp_manager.HydraTexture(
                name=render_product_path.rsplit("/", maxsplit=1)[-1],
                render_product_path=render_product_path,
                create_texture=False,
            )

        self._hydra_texture = hydra_texture
        if not annotators:
            annotators = self._annotators_from_render_vars(asset_render_product)
        self.attach_annotators(annotators, annotator_init_params=annotator_init_params)

    def _get_annotator_spec(self, annotator: str) -> dict[str, Any]:
        """Get the specification of the given annotator.

        Args:
            annotator: Annotator name.

        Returns:
            Annotator specification.
        """
        try:
            return self._annotators_spec[annotator]
        except KeyError:
            raise ValueError(
                f"Unsupported annotator '{annotator}'. Supported annotator are {list(self._annotators_spec.keys())}"
            )

    def _get_annotator_init_params(
        self, annotator: str, annotator_init_params: dict[str, dict[str, Any]] | None
    ) -> dict[str, Any]:
        """Get initialization parameters for an annotator.

        Args:
            annotator: Annotator whose initialization parameters are requested.
            annotator_init_params: Optional caller-provided initialization parameters.

        Returns:
            Default initialization parameters merged with caller-provided values.
        """
        spec = self._get_annotator_spec(annotator)
        return {**spec.get("init_params", {}), **(annotator_init_params or {}).get(annotator, {})}

    def _validate_annotators(self, annotators: str | list[str]) -> None:
        """Validate the given annotators.

        Args:
            annotators: Annotator names to validate.
        """
        annotators = [annotators] if isinstance(annotators, str) else annotators
        for annotator in annotators:
            if annotator not in self._annotators_spec:
                raise ValueError(
                    f"Unsupported annotator '{annotator}'. Supported annotator are {list(self._annotators_spec.keys())}"
                )

    def _validate_annotator_init_params(
        self,
        annotators: str | list[str],
        annotator_init_params: dict[str, dict[str, Any]] | None,
    ) -> None:
        """Validate that initialization parameters target configured annotators.

        Args:
            annotators: Configured annotator names.
            annotator_init_params: Initialization parameters to validate.
        """
        if annotator_init_params is None:
            return
        annotators = [annotators] if isinstance(annotators, str) else annotators
        unknown = sorted(set(annotator_init_params) - set(annotators))
        if unknown:
            raise ValueError(
                "Annotator initialization parameters were provided for unconfigured annotators: "
                f"{unknown}. Configured annotators are {annotators}."
            )

    def _validate_writers(self, writers: str | list[str]) -> None:
        """Validate the given writers.

        Args:
            writers: Writer names to validate.
        """
        writers = [writers] if isinstance(writers, str) else writers
        for writer in writers:
            if writer not in self._writers_spec:
                raise ValueError(
                    f"Unsupported writer '{writer}'. Supported writers are {list(self._writers_spec.keys())}"
                )
