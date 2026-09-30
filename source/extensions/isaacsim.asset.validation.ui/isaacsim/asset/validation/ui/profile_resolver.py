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

"""Resolve bundled SimReady profiles into registered validation requirements."""

from __future__ import annotations

import json
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from omni.asset_validator.core import Requirement, RequirementsRegistry
from simready.foundation import tier_core, tier_isaac

_SIMREADY_NAMESPACE = "com.nvidia.simready."


def _tier_content_paths(directory_name: str) -> tuple[Path, ...]:
    """Get a content directory from each bundled SimReady tier package.

    Container packaging deduplicates identical Python files with symlinks. Reading
    the package locations from ``__path__`` preserves the imported package paths;
    the tier descriptors resolve their symlinked ``_tier.py`` files and can both
    incorrectly point at the core tier.

    Args:
        directory_name: Content directory name within each tier package.

    Returns:
        Content paths for the core and Isaac tiers.
    """
    return tuple(Path(package.__path__[0]) / directory_name for package in (tier_core, tier_isaac))


class ProfileResolutionError(RuntimeError):
    """Error raised when a bundled profile cannot be resolved."""


@dataclass(frozen=True)
class FeatureReference:
    """Reference to a versioned SimReady feature.

    Args:
        feature_id: Stable feature identifier.
        version: Requested feature version.
        optional: Whether users may exclude the feature from validation.
    """

    feature_id: str
    version: str
    optional: bool = False


@dataclass(frozen=True)
class FeatureDefinition:
    """Resolved SimReady feature definition.

    Args:
        feature_id: Stable feature identifier.
        version: Feature version.
        display_name: User-facing feature name.
        path: Relative documentation path.
        requirements: Requirement codes declared by the feature.
        dependencies: Versioned feature dependencies.
    """

    feature_id: str
    version: str
    display_name: str
    path: str
    requirements: tuple[str, ...]
    dependencies: tuple[FeatureReference, ...]


@dataclass(frozen=True)
class ProfileDefinition:
    """Versioned SimReady profile definition.

    Args:
        profile_id: Stable profile identifier.
        version: Profile version.
        features: Features selected by the profile.
    """

    profile_id: str
    version: str
    features: tuple[FeatureReference, ...]


@dataclass(frozen=True)
class ResolvedFeature:
    """Feature with its resolved requirement objects.

    Args:
        definition: Source feature definition.
        optional: Whether users may exclude this feature.
        requirements: Registered requirements implemented by the feature.
        missing_requirements: Requirement codes absent from the registry.
    """

    definition: FeatureDefinition
    optional: bool
    requirements: tuple[Requirement, ...]
    missing_requirements: tuple[str, ...]


@dataclass(frozen=True)
class ResolvedProfile:
    """Profile expanded into features and registered requirements.

    Args:
        definition: Source profile definition.
        features: Dependency-expanded profile features.
    """

    definition: ProfileDefinition
    features: tuple[ResolvedFeature, ...]

    @property
    def requirements(self) -> tuple[Requirement, ...]:
        """Get unique requirements across all resolved features.

        Returns:
            Requirements in profile feature order.
        """
        resolved: dict[tuple[str, str | None], Requirement] = {}
        for feature in self.features:
            for requirement in feature.requirements:
                resolved.setdefault((requirement.code, requirement.version), requirement)
        return tuple(resolved.values())


def _parse_version(value: str) -> tuple[int, ...]:
    """Convert a semantic version into a sortable integer tuple.

    Args:
        value: Dotted semantic version string.

    Returns:
        Integer tuple usable as a sort key.
    """
    try:
        return tuple(int(part) for part in value.split("."))
    except ValueError:
        return (0,)


def _parse_feature_reference(raw: dict[str, Any], inherited_optional: bool = False) -> FeatureReference:
    """Parse one feature reference from profile or dependency data.

    Args:
        raw: Raw feature entry mapping a feature ID to its settings.
        inherited_optional: Whether the parent context marks the feature optional.

    Returns:
        Parsed feature reference.
    """
    feature_items = [(key, value) for key, value in raw.items() if key != "optional"]
    if len(feature_items) != 1:
        raise ProfileResolutionError(f"Expected one feature per entry, got {raw!r}")
    feature_id, settings = feature_items[0]
    if not isinstance(settings, dict) or "version" not in settings:
        raise ProfileResolutionError(f"Feature {feature_id!r} does not specify a version")
    return FeatureReference(
        feature_id=feature_id,
        version=str(settings["version"]),
        optional=inherited_optional or bool(settings.get("optional", False)) or bool(raw.get("optional", False)),
    )


class ProfileResolver:
    """Discover and resolve profiles shipped by the SimReady tier wheels.

    Args:
        profile_paths: Profile directories to search. Defaults to bundled tier paths.
        feature_paths: Feature directories to search. Defaults to bundled tier paths.
    """

    def __init__(
        self, profile_paths: tuple[Path, ...] | None = None, feature_paths: tuple[Path, ...] | None = None
    ) -> None:
        self._profile_paths = profile_paths or _tier_content_paths("profiles")
        self._feature_paths = feature_paths or _tier_content_paths("features")
        self._profiles = self._load_profiles()
        self._features = self._load_features()

    @property
    def profile_ids(self) -> tuple[str, ...]:
        """Get all available profile identifiers.

        Returns:
            Alphabetically sorted profile identifiers.
        """
        return tuple(sorted({profile_id for profile_id, _ in self._profiles}))

    def versions(self, profile_id: str) -> tuple[str, ...]:
        """Get available versions for a profile.

        Args:
            profile_id: Profile identifier.

        Returns:
            Versions sorted from newest to oldest.
        """
        versions = [version for candidate_id, version in self._profiles if candidate_id == profile_id]
        return tuple(sorted(versions, key=_parse_version, reverse=True))

    def latest_version(self, profile_id: str) -> str:
        """Get the latest available profile version.

        Args:
            profile_id: Profile identifier.

        Returns:
            Latest profile version.

        Raises:
            ProfileResolutionError: If the profile is unavailable.
        """
        versions = self.versions(profile_id)
        if not versions:
            raise ProfileResolutionError(f"Unknown SimReady profile: {profile_id}")
        return versions[0]

    def resolve(self, profile_id: str, version: str | None = None) -> ResolvedProfile:
        """Resolve a profile and all transitive feature dependencies.

        Args:
            profile_id: Profile identifier.
            version: Profile version, or None to use the latest version.

        Returns:
            Dependency-expanded profile with registered requirements.

        Raises:
            ProfileResolutionError: If the profile, feature, or dependency cannot be resolved.
        """
        selected_version = version or self.latest_version(profile_id)
        definition = self._profiles.get((profile_id, selected_version))
        if definition is None:
            raise ProfileResolutionError(f"Unknown SimReady profile: {profile_id}@{selected_version}")

        optional_by_feature: dict[tuple[str, str], bool] = {}
        ordered_features: list[FeatureDefinition] = []

        def add_feature(reference: FeatureReference, ancestry: tuple[str, ...] = ()) -> None:
            key = (reference.feature_id, reference.version)
            if reference.feature_id in ancestry:
                cycle = " -> ".join((*ancestry, reference.feature_id))
                raise ProfileResolutionError(f"Feature dependency cycle: {cycle}")
            feature = self._features.get(key)
            if feature is None:
                raise ProfileResolutionError(f"Missing SimReady feature: {reference.feature_id}@{reference.version}")

            previous_optional = optional_by_feature.get(key)
            optional_by_feature[key] = (
                reference.optional if previous_optional is None else previous_optional and reference.optional
            )
            if previous_optional is None or (previous_optional and not reference.optional):
                for dependency in feature.dependencies:
                    add_feature(
                        FeatureReference(dependency.feature_id, dependency.version, reference.optional),
                        (*ancestry, reference.feature_id),
                    )
            if previous_optional is None:
                ordered_features.append(feature)

        for reference in definition.features:
            add_feature(reference)

        registry = RequirementsRegistry()
        resolved_features: list[ResolvedFeature] = []
        for feature in ordered_features:
            requirements: list[Requirement] = []
            missing: list[str] = []
            for code in feature.requirements:
                requirement = registry.find_requirement(code) or registry.find_requirement(
                    f"{_SIMREADY_NAMESPACE}{code}"
                )
                if requirement is None:
                    missing.append(code)
                else:
                    requirements.append(requirement)
            resolved_features.append(
                ResolvedFeature(
                    definition=feature,
                    optional=optional_by_feature[(feature.feature_id, feature.version)],
                    requirements=tuple(requirements),
                    missing_requirements=tuple(missing),
                )
            )
        return ResolvedProfile(definition=definition, features=tuple(resolved_features))

    def _load_profiles(self) -> dict[tuple[str, str], ProfileDefinition]:
        profiles: dict[tuple[str, str], ProfileDefinition] = {}
        for directory in self._profile_paths:
            for path in sorted(directory.glob("*.toml")):
                with path.open("rb") as stream:
                    document = tomllib.load(stream)
                for profile_id, versions in document.items():
                    for version, value in versions.items():
                        features = tuple(_parse_feature_reference(raw) for raw in value.get("features", ()))
                        profiles[(profile_id, version)] = ProfileDefinition(profile_id, version, features)
        return profiles

    def _load_features(self) -> dict[tuple[str, str], FeatureDefinition]:
        features: dict[tuple[str, str], FeatureDefinition] = {}
        for directory in self._feature_paths:
            for path in sorted(directory.glob("*.json")):
                with path.open(encoding="utf-8") as stream:
                    document = json.load(stream)
                dependencies = tuple(_parse_feature_reference(raw) for raw in document.get("dependencies", ()))
                definition = FeatureDefinition(
                    feature_id=document["id"],
                    version=str(document["version"]),
                    display_name=document.get("display_name", document["id"]),
                    path=document.get("path", ""),
                    requirements=tuple(document.get("requirements", ())),
                    dependencies=dependencies,
                )
                features[(definition.feature_id, definition.version)] = definition
        return features
