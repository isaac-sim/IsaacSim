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

ANNOTATOR_SPEC = {
    "generic-model-output": {"name": "GenericModelOutput"},
    "stable-id-map": {"name": "StableIdMap"},
}

WRITER_SPEC: dict[str, dict] = {}
"""Mutable registry of writer specs available to the sensor runtime classes.

Companion extensions populate this at startup via :func:`register_writer_spec`
so that writer names can be passed to the ``writers`` parameter of
``LidarSensor``, ``RadarSensor``, and ``AcousticSensor``.
"""


def register_annotator_spec(name: str, spec: dict) -> None:
    """Register an additional annotator type for the sensor runtime classes.

    Modifies :data:`ANNOTATOR_SPEC` in place so that subsequent ``LidarSensor``,
    ``RadarSensor``, or ``AcousticSensor`` instances accept the new name in
    their ``annotators`` parameter.

    Args:
        name: Short annotator name (e.g. ``"draw-point-cloud"``).
        spec: Spec dict with at least a ``"name"`` key matching the Replicator
            annotator registry name.
    """
    ANNOTATOR_SPEC[name] = spec


def unregister_annotator_spec(name: str) -> None:
    """Remove a previously registered annotator spec.

    Args:
        name: Short annotator name to remove.
    """
    ANNOTATOR_SPEC.pop(name, None)


def register_writer_spec(name: str, spec: dict) -> None:
    """Register a writer type for the sensor runtime classes.

    Modifies :data:`WRITER_SPEC` in place so that subsequent sensor instances
    accept the new name in their ``writers`` parameter.

    Args:
        name: Short writer name (e.g. ``"debug-draw"``).
        spec: Spec dict with at least a ``"name"`` key matching the Replicator
            writer registry name.  Optional ``"defaults"`` dict provides default
            ``initialize()`` kwargs.
    """
    WRITER_SPEC[name] = spec


def unregister_writer_spec(name: str) -> None:
    """Remove a previously registered writer spec.

    Args:
        name: Short writer name to remove.
    """
    WRITER_SPEC.pop(name, None)
