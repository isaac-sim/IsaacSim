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
from collections.abc import Iterable
from typing import Any, Literal

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.replicator.core as rep
import warp as wp
from isaacsim.core.experimental.prims import XformPrim
from pxr import Sdf, UsdRender

from ._common import ANNOTATOR_SPEC, WRITER_SPEC

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


class _SensorAuthoring(XformPrim):
    """Base class for RTX sensor authoring (USD prim creation and wrapping).

    Handles path resolution, prim type/schema validation, attribute application,
    tick rate configuration, and USD reference loading. Subclasses define the
    sensor-specific prim type, schema, and creation logic.

    Subclasses must define:
        _PRIM_TYPE: str — USD prim type name (e.g. ``"OmniLidar"``)
        _SCHEMA: str — API schema name (e.g. ``"OmniSensorGenericLidarCoreAPI"``)
        _VALID_AUX_OUTPUT_LEVELS: tuple[str, ...] — valid ``aux_output_level`` values
        _create_prim(path, attributes) -> str — create a new prim and return its actual path

    Args:
        path: USD path for the sensor prim to create or wrap.
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

    def __init__(
        self,
        path: str,
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
        if len(existent_paths) > 1 or len(nonexistent_paths) > 1:
            raise ValueError(
                f"Only one {self._PRIM_TYPE} prim is supported, "
                f"but the provided argument refers to {len(existent_paths) + len(nonexistent_paths)} prims: "
                f"{existent_paths + nonexistent_paths}",
            )
        # wrap existing prims
        if existent_paths:
            paths = existent_paths
            for path in existent_paths:
                prim = prim_utils.get_prim_at_path(path)
                type_name = prim.GetPrimTypeInfo().GetTypeName()
                if type_name != self._PRIM_TYPE:
                    raise ValueError(f"Prim at {path} is not an '{self._PRIM_TYPE}' prim but a '{type_name}' prim")
                if self._SCHEMA not in prim.GetAppliedSchemas():
                    raise ValueError(f"Prim at {path} does not have the '{self._SCHEMA}' schema")
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
                if prim.HasAttribute("omni:sensor:tickRate"):
                    prim.GetAttribute("omni:sensor:tickRate").Set(tick_rate)
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

    def _create_prim(self, path: str, attributes: dict[str, Any] | None) -> str:
        """Create a new sensor prim. Override in subclasses.

        Args:
            path: USD path for the new prim.
            attributes: Attributes to set on the prim.

        Returns:
            Actual prim path after creation.
        """
        raise NotImplementedError

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
        usd_path: str,
        variant: str | dict[str, str] | None = None,
        variant_set_name: str = "sensor",
    ) -> str:
        """Add a USD reference to the stage and find the sensor prim within it.

        Args:
            path: Target prim path on stage.
            usd_path: Path to the USD file.
            variant: Variant selection. Either a flat string (applied against
                ``variant_set_name``), or a dict of ``{set_name: variant_name}``
                pairs for USDs with multiple variant sets, or ``None``. Nested
                variants supported via dictionary; pairs applied in dict
                insertion order, so outer variant sets must come first.
            variant_set_name: Variant set name used when ``variant`` is a string.

        Returns:
            Path to the sensor prim found within the reference.
        """
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
            raise ValueError(f"Unable to find {cls._PRIM_TYPE} prim in the USD file {usd_path} (variant: {variant})")
        return prim_utils.get_prim_path(sensor_prim)

    @property
    def aux_output_level(self) -> str:
        """The auxiliary output level configured on the GenericModelOutput RenderVar.

        Returns:
            The configured level (e.g. ``"NONE"``, ``"BASIC"``, ``"EXTRA"``, ``"FULL"``).
        """
        return self._aux_output_level


class _SensorRuntime:
    """Base class for RTX sensor runtime (annotator management and data retrieval).

    Wraps a sensor authoring object, creates a Replicator render product, and
    provides methods to attach/detach annotators and fetch sensor data. Subclasses
    define the authoring class type and a typed property to expose it.

    Subclasses must define:
        _AUTHORING_CLASS: type — the authoring class (e.g. ``Lidar``)
        _AUTHORING_ATTR: str — attribute name for the encapsulated object (e.g. ``"_lidar"``)

    Args:
        path: Sensor authoring object or path to a sensor prim.
        annotators: Annotator types to configure.
        writers: Writer types to attach.
        render_vars: Render variables to pass to the render product.
    """

    _AUTHORING_CLASS: type
    _AUTHORING_ATTR: str

    def __init__(
        self,
        path: str | _SensorAuthoring,
        *,
        annotators: ANNOTATOR | list[ANNOTATOR] | None = None,
        writers: str | list[str] | None = None,
        render_vars: list[str] | None = None,
    ) -> None:
        # define properties early so __del__ is safe if __init__ raises
        self._hydra_texture = None
        self._annotators = {}
        self._writers = {}
        if not hasattr(self, "_annotators_spec"):
            self._annotators_spec = dict(ANNOTATOR_SPEC)
        if not hasattr(self, "_writers_spec"):
            self._writers_spec = dict(WRITER_SPEC)
        # check for supported annotators / writers
        if annotators is not None:
            self._validate_annotators(annotators)
        if writers is not None:
            self._validate_writers(writers)
        # get or create authoring object
        authoring_obj = path if isinstance(path, self._AUTHORING_CLASS) else self._AUTHORING_CLASS(path)
        setattr(self, self._AUTHORING_ATTR, authoring_obj)
        if len(authoring_obj) > 1:
            raise ValueError(
                f"The sensor only supports one {self._AUTHORING_CLASS._PRIM_TYPE} prim, "
                f"but the provided argument refers to {len(authoring_obj)} prims: {authoring_obj.paths}",
            )
        # initialize instance from arguments
        self._initialize_sensor(annotators or [], render_vars=render_vars)
        # attach writers requested at construction time
        if writers is not None:
            writers = [writers] if isinstance(writers, str) else writers
            for writer_name in writers:
                spec = self._writers_spec[writer_name]
                self.attach_writer(spec["name"], **spec.get("defaults", {}))

    def __del__(self) -> None:
        """Clean up instance."""
        self._invalidate_sensor()

    @property
    def authoring_object(self) -> _SensorAuthoring:
        """The authoring object (prim wrapper) encapsulated by the sensor.

        Returns:
            The sensor's authoring object (e.g. :class:`Lidar`, :class:`Radar`, or :class:`Acoustic`).
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

    def attach_annotators(self, annotators: str | list[str]) -> dict[str, Any]:
        """Attach annotators to the sensor.

        Args:
            annotators: Annotator/sensor types to attach.

        Returns:
            Mapping from annotator name to attached annotator instance.

        Raises:
            ValueError: If the specified annotator is not supported.
        """
        annotators = [annotators] if isinstance(annotators, str) else annotators
        self._validate_annotators(annotators)
        for annotator in annotators:
            spec = self._get_annotator_spec(annotator)
            device = "cuda"
            if annotator in ["stable-id-map", "generic-model-output"]:
                device = "cpu"
            self._annotators[annotator] = rep.AnnotatorRegistry.get_annotator(
                spec["name"], device=device, do_array_copy=False
            )
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

    def get_data(self, annotator: str) -> tuple[wp.array | None, dict[str, Any]]:
        """Fetch the specified annotator/sensor data for the sensor.

        Args:
            annotator: Annotator/sensor type from which fetch the data.

        Returns:
            Two-elements tuple. 1) Array containing the fetched data.
            If no data is available at the moment of calling the method, ``None`` is returned.
            2) Dictionary containing additional information according to the requested annotator/sensor.

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

    def _invalidate_sensor(self) -> None:
        """Invalidate sensor by detaching writers, annotators, and destroying the hydra texture."""
        if self._hydra_texture is not None:
            for writer in self._writers.values():
                writer.detach()
            self.detach_annotators(list(self._annotators.keys()))
            self._hydra_texture.destroy()
        self._writers = {}
        self._annotators = {}
        self._hydra_texture = None

    def _initialize_sensor(self, annotators: str | list[str], *, render_vars: list[str] | None = None) -> None:
        """Initialize sensor by creating the hydra texture and attaching annotators.

        Args:
            annotators: Annotator/sensor types to attach.
            render_vars: Render variables to pass to the render product.
        """
        self._hydra_texture = rep.create.render_product(
            camera=self.authoring_object.paths[0],
            resolution=(300, 300),  # (width, height), needed but unused by the RTX sensor
            name=f"rtx_sensor_{hash(self)}",
            render_vars=render_vars,
        )
        self.attach_annotators(annotators)

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
