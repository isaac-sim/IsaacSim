# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Collection of Isaac Sim Replicator writers for various data formats and visualization tasks."""

from .data_visualization_writer import *
from .dope_writer import *
from .pose_writer import *
from .pytorch_listener import *
from .pytorch_writer import *
from .ycb_video_writer import *


# Register writers and add them to the default writers for Replicator telemetry tracking
def register_writers():
    """Register all Isaac Sim Replicator writers with the WriterRegistry.

    Registers DataVisualizationWriter, DOPEWriter, PoseWriter, PytorchWriter, and YCBVideoWriter
    with the omni.replicator.core WriterRegistry and adds them to the default writers list for
    Replicator telemetry tracking.
    """
    from omni.replicator.core import WriterRegistry

    # DataVisualizationWriter
    WriterRegistry.register(DataVisualizationWriter)
    (
        WriterRegistry._default_writers.append("DataVisualizationWriter")
        if "DataVisualizationWriter" not in WriterRegistry._default_writers
        else None
    )

    # DOPEWriter
    WriterRegistry.register(DOPEWriter)
    (
        WriterRegistry._default_writers.append("DOPEWriter")
        if "DOPEWriter" not in WriterRegistry._default_writers
        else None
    )

    # PoseWriter
    WriterRegistry.register(PoseWriter)
    (
        WriterRegistry._default_writers.append("PoseWriter")
        if "PoseWriter" not in WriterRegistry._default_writers
        else None
    )

    # PytorchWriter
    WriterRegistry.register(PytorchWriter)
    (
        WriterRegistry._default_writers.append("PytorchWriter")
        if "PytorchWriter" not in WriterRegistry._default_writers
        else None
    )

    # YCBVideoWriter
    WriterRegistry.register(YCBVideoWriter)
    (
        WriterRegistry._default_writers.append("YCBVideoWriter")
        if "YCBVideoWriter" not in WriterRegistry._default_writers
        else None
    )
