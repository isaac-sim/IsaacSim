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

"""Common utilities and constants for ROS 2 nodes extension."""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass

import carb
import omni.replicator.core as rep
import omni.syntheticdata
import omni.usd
from isaacsim.core.nodes.scripts.utils import register_node_writer_with_telemetry
from pxr import Sdf, Usd

# Nodes extension constants
BRIDGE_NAME = "isaacsim.ros2.bridge"
BRIDGE_PREFIX = "ROS2"
USE_SRTX_SETTING = "/exts/omni.replicator.srtx/enabled"

# Carb setting that lets the host application (e.g. the Mega Isaac Sim bridge)
# override the SRTX sensor-set name used by the ROS 2 OmniGraph helpers.
#
# In multi-bridge deployments (Mega) every bridge process must publish a
# DIFFERENT name, otherwise the bridges' SensorSet registrations clobber each
# other on the shared SRTX runtime stage and only the last writer wins. See
# `framework/services/extensions/isaac.mega.bridge/source/MegaFrontendClient.cpp`
# in the mega-dev repo for the producer side.
SRTX_SENSOR_SET_NAME_SETTING = "/exts/omni.replicator.srtx/sensorSetName"
SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING = "/exts/omni.replicator.srtx/sensorSetNameByRenderProductPath"
SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING = "/exts/omni.replicator.srtx/sensorSetRenderProductPathsByName"


def is_srtx_supported_platform() -> bool:
    """Return whether omni.replicator.srtx is expected to be available.

    Returns:
        True if SRTX is supported on the current platform, otherwise False.
    """
    return sys.platform.startswith("linux")


def validate_srtx_platform() -> bool:
    """Validate that SRTX is enabled on a supported platform.

    Returns:
        True when the current platform supports SRTX, otherwise False.

    """
    if is_srtx_supported_platform():
        return True
    carb.log_error(
        "SRTX is enabled, but omni.replicator.srtx is only available on Linux. "
        "Disable /exts/omni.replicator.srtx/enabled on this platform."
    )
    return False


@dataclass(frozen=True)
class SrtxSensorSetConfig:
    """Resolved SRTX sensor-set configuration for a render product."""

    name: str
    render_product_paths: list[str] | None = None


def _get_srtx_string_setting(setting_name: str) -> str | None:
    """Return a non-empty string value from an SRTX carb setting.

    Args:
        setting_name: Carb setting name to read.

    Returns:
        Non-empty setting value, or None if the value is missing or invalid.
    """
    settings = carb.settings.get_settings()
    if settings is None:
        return None
    try:
        value = settings.get(setting_name)
    except Exception:
        return None
    return value if isinstance(value, str) and value else None


def _get_default_srtx_sensor_set_name() -> str:
    """Return the process-wide SRTX sensor-set name override or the default name.

    Reads the carb setting :data:`SRTX_SENSOR_SET_NAME_SETTING` if present and
    non-empty; falls back to ``"default-sensor-set"`` (preserves the historical
    behavior for standalone Isaac Sim use). The host application is responsible
    for ensuring the value is valid for the SRTX server (lowercase letters,
    digits, hyphens, length 4-63, first char letter, last char letter/digit).

    Returns:
        Configured sensor-set name, or the default sensor-set name.
    """
    default = "default-sensor-set"
    override = _get_srtx_string_setting(SRTX_SENSOR_SET_NAME_SETTING)
    return override or default


def _get_srtx_json_object_setting(setting_name: str) -> dict[str, object] | None:
    """Parse an SRTX carb setting as a JSON object, logging malformed values.

    Args:
        setting_name: Carb setting name to parse.

    Returns:
        Parsed JSON object, or None if the setting is missing or invalid.
    """
    raw_value = _get_srtx_string_setting(setting_name)
    if raw_value is None:
        return None
    try:
        parsed = json.loads(raw_value)
    except json.JSONDecodeError:
        carb.log_warn(f"Invalid JSON in carb setting '{setting_name}'")
        return None
    if not isinstance(parsed, dict):
        carb.log_warn(f"Expected JSON object in carb setting '{setting_name}'")
        return None
    return parsed


def get_srtx_sensor_set_config(render_product_path: str | None = None) -> SrtxSensorSetConfig:
    """Resolve the SRTX sensor-set configuration for *render_product_path*.

    If *render_product_path* is not mapped to a configured shared set, returns
    the historical per-bridge/default sensor-set name with no declaration paths.

    Args:
        render_product_path: Render product path to resolve.

    Returns:
        Sensor-set configuration for the render product.
    """
    default_name = _get_default_srtx_sensor_set_name()
    if not render_product_path:
        return SrtxSensorSetConfig(name=default_name)

    sensor_set_name_by_render_product_path = _get_srtx_json_object_setting(
        SRTX_SENSOR_SET_NAME_BY_RENDER_PRODUCT_PATH_SETTING
    )
    if sensor_set_name_by_render_product_path is None:
        return SrtxSensorSetConfig(name=default_name)

    configured_name = sensor_set_name_by_render_product_path.get(render_product_path)
    if not isinstance(configured_name, str) or not configured_name:
        return SrtxSensorSetConfig(name=default_name)

    sensor_set_render_product_paths_by_name = _get_srtx_json_object_setting(
        SRTX_SENSOR_SET_RENDER_PRODUCT_PATHS_BY_NAME_SETTING
    )
    if sensor_set_render_product_paths_by_name is None:
        carb.log_error(
            f"Configured SRTX sensor set '{configured_name}' for '{render_product_path}' is missing declaration paths"
        )
        return SrtxSensorSetConfig(name=default_name)

    configured_paths = sensor_set_render_product_paths_by_name.get(configured_name)
    if (
        not isinstance(configured_paths, list)
        or not configured_paths
        or not all(isinstance(path, str) and path for path in configured_paths)
    ):
        carb.log_error(
            f"Configured SRTX sensor set '{configured_name}' for '{render_product_path}' has invalid declaration paths"
        )
        return SrtxSensorSetConfig(name=default_name)

    return SrtxSensorSetConfig(name=configured_name, render_product_paths=list(configured_paths))


def get_srtx_sensor_set_name(render_product_path: str | None = None) -> str:
    """Return the SRTX sensor-set name to use for ROS 2 OmniGraph publishers.

    For configured shared sensor sets, this consults the per-render-product map
    published by the host application. Otherwise it falls back to the
    historical process-wide override in :data:`SRTX_SENSOR_SET_NAME_SETTING`,
    then to ``"default-sensor-set"`` for standalone Isaac Sim use.

    Args:
        render_product_path: Render product path to resolve.

    Returns:
        Sensor-set name to use.
    """
    return get_srtx_sensor_set_config(render_product_path).name


def prepare_srtx_sensor_set(srtx_instance: object, render_product_path: str) -> str | None:
    """Resolve and, if configured, declare the SRTX sensor set for a render product.

    Args:
        srtx_instance: SRTX runtime instance.
        render_product_path: Render product path to prepare.

    Returns:
        Prepared sensor-set name, or None if preparation fails.
    """
    try:
        from omni.replicator.srtx import prepare_configured_sensorset
    except ImportError:
        prepare_configured_sensorset = None

    if prepare_configured_sensorset is not None:
        sensor_set_name = prepare_configured_sensorset(srtx_instance, render_product_path)
        if sensor_set_name is not None:
            return sensor_set_name

    sensor_set_config = get_srtx_sensor_set_config(render_product_path)
    if sensor_set_config.render_product_paths is not None:
        try:
            declared = srtx_instance.declare_sensor_set(sensor_set_config.name, sensor_set_config.render_product_paths)
        except Exception as exc:
            carb.log_error(
                f"Failed to declare configured SRTX sensor set '{sensor_set_config.name}' for "
                f"'{render_product_path}': {exc}"
            )
            return None
        if not declared:
            carb.log_error(
                f"Configured SRTX sensor set declaration was rejected for '{sensor_set_config.name}' "
                f"(render product '{render_product_path}')"
            )
            return None
    return sensor_set_config.name


NON_IMAGE_COMPRESSION_FALLBACK = "blosc"

# When searching existing RenderVars on a render product, a match against any
# name in the alias set is accepted in addition to the canonical AOV name.
# This handles render products that use "LdrColor" rather than "LdrColorSD".
AOV_ALIASES: dict[str, set[str]] = {
    "LdrColorSD": {"LdrColor"},
    "LdrColor": {"LdrColorSD"},
}


class SrtxCaptureState:
    """Encapsulates mutable SRTX continuous-capture tracking state.

    This avoids bare module-level globals and makes stage-change resets explicit.
    """

    def __init__(self) -> None:
        self._stage_id: str = ""
        self._output_paths: dict[str, list[str]] = {}

    def _refresh_stage(self) -> None:
        """Clear stale state when the current stage changes."""
        stage = omni.usd.get_context().get_stage()
        current_stage_id = str(stage.GetRootLayer().identifier) if stage else ""
        if current_stage_id != self._stage_id:
            self._output_paths.clear()
            CompressedImageManager.reset()
            self._stage_id = current_stage_id

    def start_or_extend(self, srtx_instance: object, sensor_set_name: str, output_path: str) -> None:
        """Add *output_path* to the SRTX continuous capture for *sensor_set_name*.

        Because ``start_continuous_capture`` replaces (rather than merges)
        output paths and silently ignores subsequent calls while active,
        we track the accumulated paths and do a stop/restart cycle when
        a new path needs to be added.

        Args:
            srtx_instance: The SRTX instance to control.
            sensor_set_name: Name of the sensor set.
            output_path: Output path to add to the capture.

        """
        self._refresh_stage()

        paths = self._output_paths.setdefault(sensor_set_name, [])
        if output_path in paths:
            return
        paths.append(output_path)
        srtx_instance.stop_continuous_capture(sensor_set_name)
        srtx_instance.start_continuous_capture(sensor_set_name, paths)

    def stop_or_shrink(self, srtx_instance: object, sensor_set_name: str, output_paths_to_remove: list[str]) -> None:
        """Remove *output_paths_to_remove* from the SRTX continuous capture for *sensor_set_name*.

        If other output paths remain active on the same sensor set the capture is
        restarted with only those paths.  If no paths remain, continuous capture is
        simply stopped.

        Args:
            srtx_instance: The SRTX instance to control.
            sensor_set_name: Name of the sensor set.
            output_paths_to_remove: Output paths to remove from the capture.

        """
        paths = self._output_paths.get(sensor_set_name)
        if paths is None:
            return

        for p in output_paths_to_remove:
            try:
                paths.remove(p)
            except ValueError:
                pass

        srtx_instance.stop_continuous_capture(sensor_set_name)

        if paths:
            srtx_instance.start_continuous_capture(sensor_set_name, paths)
        else:
            self._output_paths.pop(sensor_set_name, None)


_srtx_capture_state = SrtxCaptureState()


def _start_or_extend_continuous_capture(srtx_instance: object, sensor_set_name: str, output_path: str) -> None:
    _srtx_capture_state.start_or_extend(srtx_instance, sensor_set_name, output_path)


def _stop_or_shrink_continuous_capture(
    srtx_instance: object, sensor_set_name: str, output_paths_to_remove: list[str]
) -> None:
    _srtx_capture_state.stop_or_shrink(srtx_instance, sensor_set_name, output_paths_to_remove)


def _add_render_var(stage: object, rendervar_path: str, aov_name: str) -> bool:
    """Create a RenderVar USD prim at *rendervar_path* for *aov_name* if it does not already exist.

    Args:
        stage: The USD stage.
        rendervar_path: Absolute USD path for the new RenderVar prim.
        aov_name: AOV source name to set on the prim's ``sourceName`` attribute.

    Returns:
        True on success, False if the prim could not be created.

    """
    if not stage.GetPrimAtPath(rendervar_path).IsValid():
        render_var_prim = stage.DefinePrim(rendervar_path, "RenderVar")
        if not render_var_prim.IsValid():
            carb.log_error(f"Failed to create RenderVar at {rendervar_path}")
            return False
        render_var_prim.CreateAttribute("sourceName", Sdf.ValueTypeNames.String).Set(aov_name)

    carb.log_info(f"Created SRTX RenderVar at {rendervar_path}")
    return True


def ensure_render_var_on_product(
    stage: object, render_product_path: str, aov_name: str, compression_type: str | None = None, is_image: bool = False
) -> tuple[bool, str | None]:
    """Ensure a RenderVar for the given AOV exists as a child of the render product and is in orderedVars.

    Args:
        stage: The USD stage.
        render_product_path: Path to the render product prim.
        aov_name: The AOV source name to match or create.
        compression_type: Optional SRTX compression type to set on the render var.
        is_image: Whether this AOV represents image data (affects compression validation).

    Returns:
        A (success, rendervar_path) tuple.

    """
    rp_prim = stage.GetPrimAtPath(render_product_path)
    aov_name_aliases = AOV_ALIASES.get(aov_name, set())
    rendervar_path = None
    for child in rp_prim.GetChildren():
        if child.HasAttribute("sourceName"):
            src = child.GetAttribute("sourceName").Get()
            if src == aov_name or src in aov_name_aliases:
                rendervar_path = str(child.GetPath())
                break
    if not rendervar_path:
        rendervar_path = render_product_path + "/" + aov_name
        if not _add_render_var(stage, rendervar_path, aov_name):
            return False, None

    if compression_type:
        render_var_prim = stage.GetPrimAtPath(rendervar_path)
        if not render_var_prim.HasAttribute("srtx:compression:type"):
            render_var_prim.CreateAttribute("srtx:compression:type", Sdf.ValueTypeNames.String)
        if compression_type in ["hevc", "h264", "h265"] and not is_image:
            carb.log_warn(
                f"Compression type {compression_type} is not valid for non-image data {aov_name}. "
                f"Using {NON_IMAGE_COMPRESSION_FALLBACK} instead."
            )
            compression_type = NON_IMAGE_COMPRESSION_FALLBACK
        render_var_prim.GetAttribute("srtx:compression:type").Set(compression_type)

    rp_orderedvars = rp_prim.GetRelationship("orderedVars").GetTargets()
    if rendervar_path not in rp_orderedvars:
        rp_prim.GetRelationship("orderedVars").AddTarget(rendervar_path)
    return True, rendervar_path


def cleanup_srtx_state(state: object) -> None:
    """Unregister SRTX frame callbacks and clean up capture state on an OG node's internal state object.

    Expects the state object to have ``_srtx_callback_handle``, ``_srtx_sensor_set``,
    ``_srtx_output_path``, and ``_srtx_capsule`` attributes (initialised to ``None``).

    Args:
        state: The OG node internal state object to clean up.

    """
    handle = state._srtx_callback_handle
    sensor_set = state._srtx_sensor_set
    output_path = state._srtx_output_path
    if handle is not None and sensor_set is not None:
        try:
            from omni.replicator.srtx import SrtxCore

            stage = omni.usd.get_context().get_stage()
            if stage:
                usd_scene = str(stage.GetRootLayer().identifier)
                srtx_instance = SrtxCore.get_instance(usd_scene)
                if srtx_instance is not None:
                    srtx_instance.unregister_frame_callback(sensor_set, handle)
                    if output_path:
                        _stop_or_shrink_continuous_capture(srtx_instance, sensor_set, [output_path])
        except Exception as e:
            carb.log_warn(f"Error during SRTX cleanup: {e}")
    state._srtx_callback_handle = None
    state._srtx_capsule = None
    state._srtx_output_path = None
    state._srtx_sensor_set = None


class CompressedImageManager:
    """Manage per-camera H.264 compression annotators and writers.

    Each render product gets its own annotator instance (unique rendervar hash)
    so encoder pipelines and writers are fully independent across cameras.
    Annotator instances are cached so the hash stays stable across stop/play cycles.
    """

    _annotators: dict = {}
    #: Per-render-product annotator instances keyed by render product path.

    @classmethod
    def reset(cls) -> None:
        """Clear all cached annotators (e.g. on stage change)."""
        cls._annotators.clear()

    @classmethod
    def attach(cls, render_product_path: str) -> None:
        """Attach the H.264 encoder pipeline to a render product.

        Creates the annotator on first call for this render product, then attaches it.
        This activates the POST_RENDER encoder templates and the ON_DEMAND Ptr template.

        Args:
            render_product_path: Path to the render product.

        """
        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            if render_product_path not in cls._annotators:
                cls._annotators[render_product_path] = rep.AnnotatorRegistry.get_annotator(
                    "LdrColor", init_params={"compression": "h264"}
                )
            cls._annotators[render_product_path].attach([render_product_path])

    @classmethod
    def detach(cls, render_product_path: str) -> None:
        """Detach the H.264 encoder pipeline from a specific render product.

        Only detaches from the specified render product — other cameras are not affected.

        Args:
            render_product_path: Path to the render product.

        """
        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):

            annotator = cls._annotators.get(render_product_path)
            if annotator is not None:
                try:
                    annotator.detach([render_product_path])
                except Exception:
                    pass

    @classmethod
    def get_writer(cls, render_product_path: str, use_system_time: bool = False) -> rep.Writer:
        """Get a compressed image writer for a specific render product.

        Registers the writer on first call (unique name per annotator hash).
        The writer's Ptr template hash matches the annotator's encoder pipeline.

        Args:
            render_product_path: Path to the render product.
            use_system_time: If True, use system time for timestamps.

        Returns:
            The replicator writer instance for this render product.

        """
        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):

            annotator = cls._annotators.get(render_product_path)
            if annotator is None:
                raise RuntimeError(
                    f"H.264 annotator not attached for render product '{render_product_path}'. "
                    "Call CompressedImageManager.attach() first."
                )

            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar("Rgb")
            time_type = "SystemTime" if use_system_time else ""
            time_source = "systemTime" if use_system_time else "simulationTime"

            writer_name = f"{rv}{BRIDGE_PREFIX}{time_type}PublishCompressedImage_{annotator.template_name}"
            if writer_name not in rep.WriterRegistry.get_writers():
                register_node_writer_with_telemetry(
                    name=writer_name,
                    node_type_id=f"{BRIDGE_NAME}.{BRIDGE_PREFIX}PublishCompressedImage",
                    annotators=[
                        omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                            annotator.template_name,
                            attributes_mapping={
                                "outputs:dataPtr": "inputs:dataPtr",
                                "outputs:bufferSize": "inputs:bufferSize",
                            },
                        ),
                        omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                            f"IsaacRead{time_source[0].upper()}{time_source[1:]}",
                            attributes_mapping={f"outputs:{time_source}": "inputs:timeStamp"},
                        ),
                    ],
                    input_format="h264",
                    category=BRIDGE_NAME,
                )

            return rep.writers.get(writer_name)
