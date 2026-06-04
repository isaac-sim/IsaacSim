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

from ._common import register_annotator_spec as register_annotator_spec
from ._common import register_writer_spec as register_writer_spec
from ._common import unregister_annotator_spec as unregister_annotator_spec
from ._common import unregister_writer_spec as unregister_writer_spec
from .acoustic import Acoustic as Acoustic
from .acoustic_sensor import AcousticSensor as AcousticSensor
from .camera_sensor import CameraSensor as CameraSensor
from .camera_utils import draw_annotator_data_to_image as draw_annotator_data_to_image
from .lidar import Lidar as Lidar
from .lidar_sensor import LidarSensor as LidarSensor
from .radar import Radar as Radar
from .radar_sensor import RadarSensor as RadarSensor
from .rtx_acoustic_configs import SUPPORTED_ACOUSTIC_CONFIGS as SUPPORTED_ACOUSTIC_CONFIGS
from .rtx_acoustic_configs import SUPPORTED_ACOUSTIC_VARIANT_SET_NAME as SUPPORTED_ACOUSTIC_VARIANT_SET_NAME
from .rtx_camera import RtxCamera as RtxCamera
from .rtx_camera_configs import SUPPORTED_CAMERA_CONFIGS as SUPPORTED_CAMERA_CONFIGS
from .rtx_camera_configs import SUPPORTED_CAMERA_VARIANT_SET_NAME as SUPPORTED_CAMERA_VARIANT_SET_NAME
from .rtx_camera_configs import get_camera_metadata as get_camera_metadata
from .rtx_lidar_configs import SUPPORTED_LIDAR_CONFIGS as SUPPORTED_LIDAR_CONFIGS
from .rtx_lidar_configs import SUPPORTED_LIDAR_VARIANT_SET_NAME as SUPPORTED_LIDAR_VARIANT_SET_NAME
from .rtx_radar_configs import SUPPORTED_RADAR_CONFIGS as SUPPORTED_RADAR_CONFIGS
from .rtx_radar_configs import SUPPORTED_RADAR_VARIANT_SET_NAME as SUPPORTED_RADAR_VARIANT_SET_NAME
from .single_view_depth_camera_sensor import SingleViewDepthCameraSensor as SingleViewDepthCameraSensor
from .structured_light_camera import StructuredLightCamera as StructuredLightCamera
from .tiled_camera_sensor import TiledCameraSensor as TiledCameraSensor
from .utils import parse_generic_model_output_data as parse_generic_model_output_data
from .utils import parse_object_ids as parse_object_ids
from .utils import parse_stable_id_map_data as parse_stable_id_map_data
