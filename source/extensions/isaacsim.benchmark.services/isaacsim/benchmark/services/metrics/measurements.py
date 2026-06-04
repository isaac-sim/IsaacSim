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

"""Measurement and metadata models for benchmark results."""

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol, cast

from .. import utils


class _MetadataWithData(Protocol):
    """Protocol for metadata objects that contain a name and data.

    This protocol defines the interface that all metadata objects must implement to be used with the
    benchmark measurement system. Any class implementing this protocol must have a `name` attribute
    for identifying the metadata and a `data` attribute containing the actual metadata value.

    Args:
        *args: Variable length argument list passed to the implementing class.

    Keyword Args:
        **kwargs: Additional keyword arguments passed to the implementing class.
    """

    name: str
    data: Any


logger = utils.set_up_logging(__name__)


@dataclass
class Measurement(object):
    """Base measurement record.

    Args:
        name: Measurement name.
    """

    name: str


@dataclass
class SingleMeasurement(Measurement):
    """Single floating-point measurement.

    Args:
        name: Measurement name.
        value: Measurement value.
        unit: Unit string.
        type: Measurement type label.
    """

    value: float | int | str
    unit: str
    type: str = "single"
    """Measurement type label."""


@dataclass
class BooleanMeasurement(Measurement):
    """Boolean measurement.

    Args:
        name: Measurement name.
        bvalue: Measurement value.
        type: Measurement type label.
    """

    bvalue: bool
    type: str = "boolean"
    """Measurement type label."""


@dataclass
class DictMeasurement(Measurement):
    """Dictionary measurement.

    Args:
        name: Measurement name.
        value: Measurement value.
        type: Measurement type label.
    """

    value: dict
    type: str = "dict"
    """Measurement type label."""


@dataclass
class ListMeasurement(Measurement):
    """List measurement.

    Args:
        name: Measurement name.
        value: Measurement value.
        type: Measurement type label.
    """

    value: list
    type: str = "list"
    """Measurement type label."""

    def __repr__(self) -> str:
        """Return a compact string representation.

        Returns:
            String representation of the measurement.

        Example:

        .. code-block:: python

            repr_str = repr(ListMeasurement(name="samples", value=[1, 2, 3]))
        """
        return f"{self.__class__.__name__}(name={self.name!r}, length={len(self.value)})"


@dataclass
class MetadataBase(object):
    """Base metadata record.

    Args:
        name: Metadata name.
    """

    name: str


@dataclass
class StringMetadata(MetadataBase):
    """String metadata.

    Args:
        name: Metadata name.
        data: Metadata value.
        type: Metadata type label.
    """

    data: str
    type: str = "string"
    """Metadata type label."""


@dataclass
class IntMetadata(MetadataBase):
    """Integer metadata.

    Args:
        name: Metadata name.
        data: Metadata value.
        type: Metadata type label.
    """

    data: int
    type: str = "int"
    """Metadata type label."""


@dataclass
class FloatMetadata(MetadataBase):
    """Float metadata.

    Args:
        name: Metadata name.
        data: Metadata value.
        type: Metadata type label.
    """

    data: float
    type: str = "float"
    """Metadata type label."""


@dataclass
class DictMetadata(MetadataBase):
    """Dictionary metadata.

    Args:
        name: Metadata name.
        data: Metadata value.
        type: Metadata type label.
    """

    data: dict
    type: str = "dict"
    """Metadata type label."""


@dataclass
class TestPhase(object):
    """Represent a single test phase with associated metrics and metadata.

    Args:
        phase_name: Name of the phase.
        measurements: Measurements recorded for the phase.
        metadata: Metadata recorded for the phase.
    """

    phase_name: str
    measurements: list[Measurement] = field(default_factory=list)
    metadata: list[_MetadataWithData] = field(default_factory=list)

    def get_metadata_field(self, name: str, default: Any = KeyError) -> Any:
        """Get a metadata field's value.

        Args:
            name: Field name. Note that fields are named internally like 'Empty_Scene Stage DSSIM Status', however
                `name` is case-insensitive, and drops the stage name. In this eg it would be 'stage dssim status'.
            default: Default value to return when the field is missing.

        Returns:
            Metadata value, or default if provided.

        Raises:
            KeyError: If the field is not found and no default is provided.

        Example:

        .. code-block:: python

            status = phase.get_metadata_field("stage dssim status", default=None)
        """
        name = name.lower()
        for m in self.metadata:
            name2 = m.name.replace(self.phase_name, "").strip().lower()
            if name == name2:
                return cast(Any, m).data

        if default is KeyError:
            raise KeyError(name)
        else:
            return default

    @classmethod
    def metadata_from_dict(cls, m: dict) -> list[_MetadataWithData]:
        """Build metadata objects from a metadata dictionary.

        Args:
            m: Dictionary containing a "metadata" list.

        Returns:
            List of metadata objects.

        Example:

        .. code-block:: python

            metadata = TestPhase.metadata_from_dict({"metadata": [{"name": "gpu", "data": "A10"}]})
        """
        metadata: list[_MetadataWithData] = []
        metadata_mapping = {str: StringMetadata, int: IntMetadata, float: FloatMetadata, dict: DictMetadata}
        for meas in m["metadata"]:
            if "data" in meas:
                metadata_type = metadata_mapping.get(type(meas["data"]))
                if metadata_type:
                    curr_meta = metadata_type(name=meas["name"], data=meas["data"])
                    metadata.append(curr_meta)
        return metadata

    @classmethod
    def from_json(cls, m: dict) -> "TestPhase":
        """Deserialize measurements and metadata from a JSON structure.

        Args:
            m: JSON-compatible dictionary containing phase data.

        Returns:
            Deserialized test phase object.

        Example:

        .. code-block:: python

            phase = TestPhase.from_json(phase_dict)
        """
        curr_run = TestPhase(m["phase_name"])

        for meas in m["measurements"]:
            if "value" in meas:
                if isinstance(meas["value"], float):
                    curr_meas: Measurement = SingleMeasurement(
                        name=meas["name"], value=meas["value"], unit=meas["unit"]
                    )
                    curr_run.measurements.append(curr_meas)
                elif isinstance(meas["value"], dict):
                    curr_meas = DictMeasurement(name=meas["name"], value=meas["value"])
                    curr_run.measurements.append(curr_meas)
                elif isinstance(meas["value"], list):
                    curr_meas = ListMeasurement(name=meas["name"], value=meas["value"])
                    curr_run.measurements.append(curr_meas)
            elif "bvalue" in meas:
                curr_meas = BooleanMeasurement(name=meas["name"], bvalue=meas["bvalue"])
                curr_run.measurements.append(curr_meas)

            curr_run.metadata = TestPhase.metadata_from_dict(m["metadata"])
        return curr_run

    @classmethod
    def aggregate_json_files(cls, json_folder_path: str | Path) -> list["TestPhase"]:
        """Aggregate test phases from JSON files in a folder.

        Args:
            json_folder_path: Folder containing metrics JSON files.

        Returns:
            List of aggregated test phases.

        Example:

        .. code-block:: python

            phases = TestPhase.aggregate_json_files("/tmp/metrics")
        """
        # Gather the separate metrics files for each test
        test_runs = []
        metric_files = os.listdir(json_folder_path)
        for f in metric_files:
            metric_path = os.path.join(json_folder_path, f)
            if os.path.isfile(metric_path):
                if f.startswith("metrics") and f.endswith(".json"):
                    with open(metric_path, encoding="utf-8") as json_file:
                        try:
                            test_run_json_list = json.load(json_file)
                            for m in test_run_json_list:
                                run = cls.from_json(m)
                                test_runs.append(run)
                        except json.JSONDecodeError:
                            logger.error(
                                f'aggregate_json_files, problems parsing field {f} with content "{json_file.read()}"'
                            )
        return test_runs


class TestPhaseEncoder(json.JSONEncoder):
    """JSON encoder for test phases and measurement objects."""

    def default(self, o: object) -> dict:
        """Serialize objects by exposing their dictionary representation.

        Args:
            o: Object to serialize.

        Returns:
            Dictionary representation of the object.

        Example:

        .. code-block:: python

            json.dumps(phase, cls=TestPhaseEncoder)
        """
        return o.__dict__
