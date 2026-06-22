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

"""NuRec render setup: stage detection, launch args, prereq gating, and config.

`setup_for_rendering(stage, ...)` is the entry point. The detection helpers (`classify_stage`,
`is_nurec`, `is_particle_field`, `is_volume`, `has_spg`) and `load_config` are public.

Kit-free at import (`carb`/`omni` imported lazily), so the config and launch-arg helpers work
before a `SimulationApp` exists.
"""

from __future__ import annotations

import os
from typing import Any, NamedTuple

import yaml

# Stage-detection signals.
SPG_SOURCE_ATTR = "info:spg:sourceAsset"  # SPG/PPISP graph -> the render path
PARTICLE_FIELD_TYPE_PREFIX = "ParticleField"  # gaussian-splat geometry (UsdVolParticleField subclasses)
NUREC_VOLUME_FIELD_TYPE = "OmniNuRecFieldAsset"  # NuRec volume field prim
NUREC_VOLUME_FLAG = "omni:nurec:isNuRecVolume"  # authored on a NuRec Volume prim

# The shipped render config lives alongside this module.
DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config", "nurec_config.yaml")


def build_extra_args() -> list[str]:
    """Return the launch-time `extra_args` for SimulationApp (enable SPG, disable multi-GPU).

    Returns:
        The SimulationApp `extra_args` token list.
    """
    return ["--enable", "omni.rtx.spg", "--/renderer/multiGpu/enabled=false"]


def enable_omni_rtx_spg(simulation_app: Any) -> None:
    """Enable the `omni.rtx.spg` extension.

    Args:
        simulation_app: The running SimulationApp.

    Raises:
        RuntimeError: If the extension fails to enable.
    """
    import carb
    import omni.kit.app

    em = omni.kit.app.get_app().get_extension_manager()
    ok = em.set_extension_enabled_immediate("omni.rtx.spg", True)
    if not ok:
        raise RuntimeError(
            "omni.rtx.spg failed to enable. Confirm Kit registry connectivity; "
            "expected omni.rtx.spg-0.2.0 (>= Kit 110.1.2)."
        )
    simulation_app.update()
    carb.log_warn("omni.rtx.spg enabled.")


def setup_for_rendering(stage: Any, config_path: str | None = None) -> tuple[bool, bool, bool, list[str]]:
    """Prepare an open stage for rendering, applying NuRec carb overrides when needed.

    Args:
        stage: The open USD stage.
        config_path: Optional YAML overriding the shipped config.

    Returns:
        `(success, nurec, spg, problems)`: `success` is False only when a NuRec launch prerequisite
        is unmet (the caller should abort); `nurec` whether the stage is a NuRec USD; `spg`
        whether it uses the SPG/PPISP render path; `problems` the unmet-prerequisite messages
        (empty unless aborting).
    """
    import carb

    kinds = classify_stage(stage)
    carb.log_info(f"[nurec] stage kinds: particle_field={kinds.particle_field} volume={kinds.volume} spg={kinds.spg}")
    if not kinds.is_nurec:
        return True, False, False, []  # nothing to set up — render normally
    spg = kinds.spg
    problems = _nurec_launch_prereq_errors(spg)
    if problems:
        label = stage.GetRootLayer().identifier
        for p in problems:
            carb.log_error(f"[nurec] {label}: {p}")
        return False, True, spg, problems
    cfg = load_config(config_path)
    apply_carb_overrides(cfg.get("spg_pre_hydra_sync_overrides" if spg else "no_spg_pre_hydra_sync_overrides"))
    return True, True, spg, []


class StageKinds(NamedTuple):
    """The NuRec prim kinds found on a stage in one traversal.

    Args:
        particle_field: Whether the stage holds gaussian-splat geometry.
        volume: Whether the stage holds a NuRec volume.
        spg: Whether the stage uses the SPG/PPISP render path.
    """

    particle_field: bool
    volume: bool
    spg: bool

    @property
    def is_nurec(self) -> bool:
        """Return True for a gaussian-splat, volume, or SPG/PPISP stage.

        Returns:
            True when any NuRec prim kind is present.
        """
        return self.particle_field or self.volume or self.spg


def _prim_is_particle_field(prim: Any) -> bool:
    """Return True when `prim` is gaussian-splat geometry.

    Args:
        prim: The USD prim to check.

    Returns:
        True when the prim's type name starts with `ParticleField`.
    """
    return str(prim.GetTypeName()).startswith(PARTICLE_FIELD_TYPE_PREFIX)


def _prim_is_volume(prim: Any) -> bool:
    """Return True when `prim` is a NuRec volume.

    Args:
        prim: The USD prim to check.

    Returns:
        True when the prim is an `OmniNuRecFieldAsset`, or a `Volume` flagged `omni:nurec:isNuRecVolume`.
    """
    type_name = str(prim.GetTypeName())
    if type_name == NUREC_VOLUME_FIELD_TYPE:
        return True
    flag = prim.GetAttribute(NUREC_VOLUME_FLAG)
    return type_name == "Volume" and bool(flag and flag.Get())


def _prim_has_spg(prim: Any) -> bool:
    """Return True when `prim` authors `info:spg:sourceAsset`.

    Args:
        prim: The USD prim to check.

    Returns:
        True when the SPG/PPISP source attribute is authored on the prim.
    """
    attr = prim.GetAttribute(SPG_SOURCE_ATTR)
    return bool(attr and attr.HasAuthoredValue())


def classify_stage(stage: Any) -> StageKinds:
    """Return the NuRec prim kinds on `stage` from a single traversal.

    Args:
        stage: The open USD stage.

    Returns:
        The gaussian-splat, volume, and SPG/PPISP flags for the stage.
    """
    particle_field = volume = spg = False
    for prim in stage.Traverse():
        particle_field = particle_field or _prim_is_particle_field(prim)
        volume = volume or _prim_is_volume(prim)
        spg = spg or _prim_has_spg(prim)
        if particle_field and volume and spg:
            break
    return StageKinds(particle_field, volume, spg)


def is_nurec(stage: Any) -> bool:
    """Return True if `stage` is a NuRec USD.

    Args:
        stage: The open USD stage.

    Returns:
        True for a gaussian-splat, volume, or SPG/PPISP stage.
    """
    return classify_stage(stage).is_nurec


def has_spg(stage: Any) -> bool:
    """Return True if `stage` uses the SPG/PPISP render path.

    Args:
        stage: The open USD stage.

    Returns:
        True when a prim authors `info:spg:sourceAsset`.
    """
    return any(_prim_has_spg(prim) for prim in stage.Traverse())


def is_particle_field(stage: Any) -> bool:
    """Return True if `stage` holds gaussian-splat geometry.

    Args:
        stage: The open USD stage.

    Returns:
        True when a prim's type name starts with `ParticleField`.
    """
    return any(_prim_is_particle_field(prim) for prim in stage.Traverse())


def is_volume(stage: Any) -> bool:
    """Return True if `stage` holds a NuRec volume.

    Args:
        stage: The open USD stage.

    Returns:
        True when a prim is an `OmniNuRecFieldAsset`, or a `Volume` flagged `omni:nurec:isNuRecVolume`.
    """
    return any(_prim_is_volume(prim) for prim in stage.Traverse())


def apply_carb_overrides(overrides: dict | None) -> None:
    """Apply carb setting overrides for a NuRec scene.

    Call after `open_stage` but before the first `simulation_app.update()`.

    Args:
        overrides: Mapping of carb setting path to value; `None` values, and an empty or `None`
            mapping, apply nothing.
    """
    import carb

    settings = carb.settings.get_settings()
    for path, target in (overrides or {}).items():
        if target is None:  # explicit "leave default" — do not set
            continue
        current = settings.get(path)
        if current == target:
            carb.log_info(f"[spg-overrides]    {path} = {target!r}  (unchanged)")
            continue
        settings.set(path, target)
        carb.log_warn(f"[spg-overrides]  * {path}: {current!r} -> {target!r}  (FLIPPED)")


def load_config(cli_config: str | None = None) -> dict:
    """Load `nurec_config.yaml`, overlaid with an optional `--config` file.

    Args:
        cli_config: Path to an override YAML, or None to use only the defaults.

    Returns:
        The merged config dict.
    """
    with open(DEFAULT_CONFIG) as f:
        cfg = yaml.safe_load(f) or {}
    if cli_config:
        with open(os.path.expanduser(cli_config)) as f:
            override = yaml.safe_load(f) or {}
        for key, value in override.items():
            if isinstance(value, dict) and isinstance(cfg.get(key), dict):
                cfg[key].update(value)
            else:
                cfg[key] = value
    return cfg


def select_cases(cfg: dict, case_name: str | None = None) -> list[dict]:
    """Return the per-case config dicts to run from a (possibly multi-case) config.

    Args:
        cfg: The loaded config dict.
        case_name: When given, keep only the case whose `name` matches.

    Returns:
        The list of flattened per-case config dicts.

    Raises:
        SystemExit: If `case_name` matches no case.
    """
    cases = cfg.get("cases")
    if not cases:
        return [cfg]
    flat = []
    for c in cases:
        merged = dict(cfg)
        merged.pop("cases", None)
        merged.update(c)
        flat.append(merged)
    if case_name is not None:
        flat = [c for c in flat if c.get("name") == case_name]
        if not flat:
            raise SystemExit(f"--case {case_name!r} not found; available: {[c.get('name') for c in cases]}")
    return flat


def resolve_path(path: str, base_dir: str) -> str:
    """Resolve a config path against ``base_dir`` or the Isaac assets root.

    Resolution order:
    1. URL (contains ``://``) — returned as-is.
    2. Isaac-relative path (starts with ``/Isaac/``) — prepended with
       ``get_assets_root_path()`` so the same YAML value works against both
       Nucleus and the production S3 mirror.
    3. Absolute local path — returned as-is.
    4. Relative local path — joined to ``base_dir``.

    Args:
        path: The path to resolve.
        base_dir: Directory that relative local paths are resolved against.

    Returns:
        The resolved path or URL.
    """
    if "://" in path:
        return path
    expanded = os.path.expanduser(path)
    if expanded.startswith("/Isaac/"):
        from isaacsim.storage.native import get_assets_root_path

        assets_root = get_assets_root_path()
        if assets_root is None:
            raise RuntimeError(
                f"get_assets_root_path() returned None while resolving {path!r}. "
                "Check Nucleus connectivity, or use a local path / omniverse:// URL instead."
            )
        return assets_root + expanded
    if expanded != path:
        expanded = os.path.normpath(expanded)
    return expanded if os.path.isabs(expanded) else os.path.normpath(os.path.join(base_dir, expanded))


# --- Internal helpers ---------------------------------------------------------------------


def _nurec_launch_prereq_errors(spg: bool) -> list[str]:
    """Return the unmet launch prerequisites for NuRec rendering (empty list = OK).

    Args:
        spg: Whether the stage is an SPG (PPISP) NuRec USD.

    Returns:
        Human-readable problem strings (empty when all prerequisites hold).
    """
    import carb.settings
    import omni.kit.app

    problems: list[str] = []
    if carb.settings.get_settings().get("/renderer/multiGpu/enabled"):
        problems.append("/renderer/multiGpu/enabled is true (launch with: --/renderer/multiGpu/enabled=false)")
    if spg and not omni.kit.app.get_app().get_extension_manager().is_extension_enabled("omni.rtx.spg"):
        problems.append("omni.rtx.spg is not enabled (launch with: --enable omni.rtx.spg)")
    return problems
