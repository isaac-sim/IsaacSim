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

"""Data recorder interfaces and registry utilities."""

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from ..metrics.measurements import Measurement, MetadataBase


@dataclass
class MeasurementData:
    """Container for recorder measurements, metadata, and artifacts.

    Args:
        measurements: Recorded measurement values.
        metadata: Recorded metadata values.
        artefacts: Artifact tuples of (path, label).
    """

    measurements: Sequence[Measurement] = field(default_factory=lambda: [])
    metadata: Sequence[MetadataBase] = field(default_factory=lambda: [])
    artefacts: Sequence[tuple[Path, str]] = field(default_factory=lambda: [])  # (path, artefact-label)


@dataclass
class InputContext:
    """Miscellaneous input data required by recorders.

    Args:
        artifact_prefix: Prefix for artifact filenames.
        kit_version: Kit version string.
        phase: Current benchmark phase name.
    """

    artifact_prefix: str = ""
    """Prefix for artifact filenames."""
    kit_version: str = ""
    """Kit version string."""
    phase: str = ""
    """Current benchmark phase name."""


class MeasurementDataRecorder:
    """Base class for recording metrics, metadata, and file-based artifacts.

    There are two common recorder styles: instantaneous measurements taken at
    a point in time, and sampling-based measurements gathered over a period.

    Args:
        context: Input context for the recorder.
        root_dir: Root directory for output artifacts.
    """

    def __init__(
        self,
        context: InputContext | None = None,
        root_dir: Path | None = None,
    ) -> None:
        pass

    def get_data(self) -> MeasurementData:
        """Return measurements, metadata, and artifacts collected so far.

        Returns:
            The collected measurement data container.

        Example:

        .. code-block:: python

            data = recorder.get_data()
        """
        return MeasurementData()


class MeasurementDataRecorderRegistry:
    """Registry for measurement data recorders with decorator-based registration."""

    name_to_class: dict[str, type[MeasurementDataRecorder]] = {}
    """Mapping from recorder names to their corresponding recorder classes."""

    @classmethod
    def add(cls, name: str, recorder: type[MeasurementDataRecorder]) -> None:
        """Register a recorder class by name.

        Args:
            name: Unique identifier for the recorder.
            recorder: Recorder class to register.

        Example:

        .. code-block:: python

            MeasurementDataRecorderRegistry.add("custom", CustomRecorder)
        """
        cls.name_to_class[name] = recorder

    @classmethod
    def get(cls, name: str) -> type[MeasurementDataRecorder] | None:
        """Get a recorder class by name.

        Args:
            name: Recorder identifier.

        Returns:
            Recorder class if found, None otherwise.

        Example:

        .. code-block:: python

            recorder_cls = MeasurementDataRecorderRegistry.get("runtime")
        """
        return cls.name_to_class.get(name)

    @classmethod
    def get_many(cls, names: list[str]) -> list[type[MeasurementDataRecorder]]:
        """Get multiple recorder classes by name.

        Args:
            names: List of recorder identifiers.

        Returns:
            List of recorder classes, skipping missing recorders.

        Example:

        .. code-block:: python

            recorders = MeasurementDataRecorderRegistry.get_many(["runtime", "memory"])
        """
        classes = [cls.get(x) for x in names]
        return [c for c in classes if c is not None]

    @classmethod
    def register(cls, name: str) -> Callable[[type[MeasurementDataRecorder]], type[MeasurementDataRecorder]]:
        """Decorator for registering recorder classes.

        Args:
            name: Unique identifier for the recorder.

        Returns:
            Decorator function.

        Example:

        .. code-block:: python

            @MeasurementDataRecorderRegistry.register("my_recorder")
            class MyRecorder(MeasurementDataRecorder):
                pass
        """

        def decorator(recorder_class: type[MeasurementDataRecorder]) -> type[MeasurementDataRecorder]:
            cls.add(name, recorder_class)
            return recorder_class

        return decorator

    @classmethod
    def list_available(cls) -> list[str]:
        """List all registered recorder names.

        Returns:
            List of registered recorder identifiers.

        Example:

        .. code-block:: python

            names = MeasurementDataRecorderRegistry.list_available()
        """
        return list(cls.name_to_class.keys())
