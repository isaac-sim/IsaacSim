# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility functions for USD prim relationships and Replicator node registration with telemetry tracking."""

import omni
import omni.replicator.core as rep


def set_target_prims(primPath: str, targetPrimPaths: list, inputName: str = "inputs:targetPrim"):
    """Set target prim relationships for a USD prim.

    Creates a relationship attribute on the specified prim and sets its targets to the provided
    prim paths.

    Args:
        primPath: Path to the USD prim that will have the relationship created.
        targetPrimPaths: List of prim paths to set as targets for the relationship.
        inputName: Name of the relationship attribute to create.
    """
    stage = omni.usd.get_context().get_stage()
    try:
        input_rel = stage.GetPrimAtPath(primPath).CreateRelationship(inputName)
        input_rel.SetTargets(targetPrimPaths)
    except Exception as e:
        print(e, primPath)


def register_node_writer_with_telemetry(*args, **kwargs):
    """Register a node writer with Replicator and enable telemetry tracking.

    Registers a node writer using the Replicator core API and adds it to the default writers
    list for telemetry tracking purposes.

    Args:
        *args: Arguments passed to the Replicator node writer registration.
        **kwargs: Keyword arguments passed to the Replicator node writer registration.
            Must include 'name' key for telemetry tracking.
    """
    rep.writers.register_node_writer(*args, **kwargs)
    # Register writer for Replicator telemetry tracking
    if kwargs["name"] not in rep.WriterRegistry._default_writers:
        rep.WriterRegistry._default_writers.append(kwargs["name"])


def register_annotator_from_node_with_telemetry(*args, **kwargs):
    """Register an annotator from a node with Replicator and enable telemetry tracking.

    Registers an annotator from a node using the Replicator core API and adds it to the
    default annotators list for telemetry tracking purposes.

    Args:
        *args: Arguments passed to the Replicator annotator registration.
        **kwargs: Keyword arguments passed to the Replicator annotator registration.
            Must include 'name' key for telemetry tracking.
    """
    rep.AnnotatorRegistry.register_annotator_from_node(*args, **kwargs)
    # Register annotator for Replicator telemetry tracking
    if kwargs["name"] not in rep.AnnotatorRegistry._default_annotators:
        rep.AnnotatorRegistry._default_annotators.append(kwargs["name"])
