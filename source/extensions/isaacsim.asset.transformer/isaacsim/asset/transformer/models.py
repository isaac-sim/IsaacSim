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

"""Data models for rule configuration and execution reporting."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Any


@dataclass
class RuleConfigurationParam:
    """Descriptor for a single rule configuration parameter.

    Args:
        name: Unique parameter name.
        display_name: Human-readable label.
        param_type: Expected Python type for the parameter.
        description: Optional description of the parameter.
        default_value: Default value for the parameter.

    """

    name: str
    display_name: str
    param_type: type
    description: str | None = None
    """Optional description of the parameter."""
    default_value: object | None = None
    """Default value for the parameter."""


@dataclass
class RuleSpec:
    """Specification for a single rule in a profile.

    Args:
        name: Rule display name.
        type: Fully qualified rule class path.
        destination: Optional output path override.
        params: Rule parameter overrides.
        enabled: Whether the rule is active.

    """

    name: str
    type: str
    destination: str | None = None
    """Optional output path override."""
    params: dict[str, Any] = field(default_factory=dict)
    enabled: bool = True
    """Whether the rule is active."""

    def to_dict(self) -> dict[str, Any]:
        """Convert this specification to a plain dictionary.

        Returns:
            Dictionary suitable for JSON serialization.

        Example:

        .. code-block:: python

            >>> spec = RuleSpec(
            ...     name="MoveMeshes",
            ...     type="isaacsim.asset.transformer.rules.perf.geometries.GeometriesRoutingRule",
            ...     params={"scope": "/World"},
            ... )
            >>> isinstance(spec.to_dict(), dict)
            True

        """
        return asdict(self)

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "RuleSpec":
        """Create a :class:`RuleSpec` from a dictionary.

        Args:
            data: Mapping with rule fields.

        Returns:
            Parsed :class:`RuleSpec` instance.

        Raises:
            ValueError: If required fields are missing.

        Example:

        .. code-block:: python

            spec = RuleSpec.from_dict(
                {"name": "MoveMeshes", "type": "my.rule.Class", "params": {"scope": "/World"}}
            )

        """
        name = data.get("name")
        type_ = data.get("type")
        if not name or not type_:
            raise ValueError("RuleSpec requires 'name' and 'type'.")
        return RuleSpec(
            name=name,
            type=type_,
            destination=data.get("destination"),
            params=dict(data.get("params", {})),
            enabled=bool(data.get("enabled", True)),
        )


@dataclass
class RuleProfile:
    """Collection of rules and metadata for a transformation run.

    Args:
        profile_name: Display name for the profile.
        version: Optional profile version string.
        rules: Rule specifications to execute.
        interface_asset_name: Optional interface asset identifier.
        output_package_root: Optional output root for packages.
        flatten_source: Whether to flatten source stages before rules.
        base_name: Optional base name for generated outputs.

    """

    profile_name: str
    version: str | None = None
    """Optional profile version string."""
    rules: list[RuleSpec] = field(default_factory=list)
    interface_asset_name: str | None = None
    """Optional interface asset identifier."""
    output_package_root: str | None = None
    """Optional output root for packages."""
    flatten_source: bool = False
    """Whether to flatten source stages before rules."""
    base_name: str | None = None
    """Optional base name for generated outputs."""

    def to_dict(self) -> dict[str, Any]:
        """Serialize profile to a dictionary.

        Returns:
            Dictionary suitable for JSON serialization.

        Example:

        .. code-block:: python

            payload = profile.to_dict()

        """
        return {
            "profile_name": self.profile_name,
            "version": self.version,
            "rules": [r.to_dict() for r in self.rules],
            "interface_asset_name": self.interface_asset_name,
            "output_package_root": self.output_package_root,
            "flatten_source": self.flatten_source,
            "base_name": self.base_name,
        }

    def to_json(self) -> str:
        """Serialize profile to a deterministic JSON string.

        Returns:
            JSON string with sorted keys and no trailing spaces.

        Example:

        .. code-block:: python

            json_str = profile.to_json()

        """
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))

    @staticmethod
    def from_dict(data: dict[str, Any]) -> "RuleProfile":
        """Parse a :class:`RuleProfile` from a dictionary.

        Args:
            data: Mapping with profile fields.

        Returns:
            Parsed :class:`RuleProfile` instance.

        Raises:
            ValueError: If required fields are missing.

        Example:

        .. code-block:: python

            profile = RuleProfile.from_dict({"profile_name": "Default", "rules": []})

        """
        profile_name = data.get("profile_name") or ""
        if not profile_name:
            raise ValueError("RuleProfile requires 'profile_name'.")
        rules_raw = data.get("rules", [])
        rules = [RuleSpec.from_dict(r) for r in rules_raw]
        return RuleProfile(
            profile_name=profile_name,
            version=data.get("version"),
            rules=rules,
            interface_asset_name=data.get("interface_asset_name"),
            output_package_root=data.get("output_package_root"),
            flatten_source=data.get("flatten_source", False),
            base_name=data.get("base_name"),
        )

    @staticmethod
    def from_json(json_str: str) -> "RuleProfile":
        """Parse a :class:`RuleProfile` from a JSON string.

        Args:
            json_str: JSON payload encoding a profile.

        Returns:
            Parsed :class:`RuleProfile` instance.

        Example:

        .. code-block:: python

            profile = RuleProfile.from_json('{"profile_name":"Default","rules":[]}')

        """
        return RuleProfile.from_dict(json.loads(json_str))


@dataclass
class RuleExecutionResult:
    """Outcome of running a single rule.

    Args:
        rule: Rule specification that was executed.
        success: Whether the rule completed successfully.
        log: Log entries recorded during execution.
        affected_stages: Identifiers of affected stages.
        error: Error message if the rule failed.
        started_at: Start timestamp in ISO format.
        finished_at: Finish timestamp in ISO format.

    """

    rule: RuleSpec
    success: bool
    log: list[dict[str, Any]] = field(default_factory=list)
    affected_stages: list[str] = field(default_factory=list)
    error: str | None = None
    """Error message if the rule failed."""
    started_at: str = field(default_factory=lambda: datetime.utcnow().isoformat(timespec="milliseconds") + "Z")
    finished_at: str | None = None
    """Finish timestamp in ISO format."""

    def close(self) -> None:
        """Mark the result as finished by setting the ``finished_at`` timestamp.

        Example:

        .. code-block:: python

            result.close()

        """
        self.finished_at = datetime.utcnow().isoformat(timespec="milliseconds") + "Z"


@dataclass
class ExecutionReport:
    """Report for a full transformation run.

    Args:
        profile: Profile used for the run.
        input_stage_path: Path to the input stage.
        package_root: Package output root path.
        started_at: Start timestamp in ISO format.
        finished_at: Finish timestamp in ISO format.
        results: Rule execution results.
        output_stage_path: File path of the final working stage after all rules
            have executed. Callers can use this to load the transformed asset.

    """

    profile: RuleProfile
    input_stage_path: str
    package_root: str
    started_at: str = field(default_factory=lambda: datetime.utcnow().isoformat(timespec="milliseconds") + "Z")
    finished_at: str | None = None
    """Finish timestamp in ISO format."""
    results: list[RuleExecutionResult] = field(default_factory=list)
    output_stage_path: str | None = None
    """File path of the final working stage after all rules have executed. Callers can use this to load the transformed asset."""

    def to_dict(self) -> dict[str, Any]:
        """Serialize the report to a dictionary suitable for JSON.

        Returns:
            Dictionary with execution details.

        Example:

        .. code-block:: python

            payload = report.to_dict()

        """
        return {
            "profile": self.profile.to_dict(),
            "input_stage_path": self.input_stage_path,
            "package_root": self.package_root,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "results": [asdict(r) for r in self.results],
            "output_stage_path": self.output_stage_path,
        }

    def to_json(self) -> str:
        """Serialize the report to a deterministic JSON string.

        Returns:
            JSON string with sorted keys and compact separators.

        Example:

        .. code-block:: python

            json_str = report.to_json()

        """
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))

    def close(self) -> None:
        """Mark the report as finished by setting the ``finished_at`` timestamp.

        Example:

        .. code-block:: python

            report.close()

        """
        self.finished_at = datetime.utcnow().isoformat(timespec="milliseconds") + "Z"
