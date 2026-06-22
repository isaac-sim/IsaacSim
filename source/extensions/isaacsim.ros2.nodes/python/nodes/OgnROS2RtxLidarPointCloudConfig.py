# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""OmniGraph node for configuring ROS 2 RTX lidar point cloud metadata."""

from typing import Any


class OgnROS2RtxLidarPointCloudConfig:
    """OmniGraph node that configures RTX lidar point cloud metadata selection."""

    @staticmethod
    def compute(db: Any) -> bool:
        """Populate selected RTX lidar metadata fields for downstream PointCloud2 publishing.

        Args:
            db: OmniGraph database for the node.

        Returns:
            True after the selected metadata output is populated.
        """
        selectedMetadata = []
        if db.inputs.outputIntensity:
            selectedMetadata.append("Intensity")
        if db.inputs.outputTimestamp:
            selectedMetadata.append("Timestamp")
        if db.inputs.outputEmitterId:
            selectedMetadata.append("EmitterId")
        if db.inputs.outputChannelId:
            selectedMetadata.append("ChannelId")
        if db.inputs.outputMaterialId:
            selectedMetadata.append("MaterialId")
        if db.inputs.outputTickId:
            selectedMetadata.append("TickId")
        if db.inputs.outputHitNormal:
            selectedMetadata.append("HitNormal")
        if db.inputs.outputVelocity:
            selectedMetadata.append("Velocity")
        if db.inputs.outputObjectId:
            selectedMetadata.append("ObjectId")
        if db.inputs.outputEchoId:
            selectedMetadata.append("EchoId")
        if db.inputs.outputTickState:
            selectedMetadata.append("TickState")
        db.outputs.selectedMetadata = selectedMetadata
        return True
