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

"""Writer for the DOPE (Deep Object Pose Estimation) dataset format."""

import io
import json
from typing import Dict, List

import carb
import numpy as np
from omni.replicator.core import AnnotatorRegistry, BackendDispatch, Writer, WriterRegistry
from omni.syntheticdata import SyntheticData

NodeTemplate, NodeConnectionTemplate = SyntheticData.NodeTemplate, SyntheticData.NodeConnectionTemplate


class _NumpyEncoder(json.JSONEncoder):
    """JSON encoder that handles numpy arrays by converting them to lists."""

    def default(self, obj):
        """Serialize numpy arrays to JSON-compatible lists.

        Args:
            obj: The object to serialize. If it's a numpy array, converts to list.

        Returns:
            JSON-serializable representation of the object.
        """
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


__version__ = "0.0.1"


class DOPEWriter(Writer):
    """Basic writer capable of writing built-in annotator groundtruth.

    .. deprecated::
        This class has been deprecated and will be removed in the next major release.

    Args:
        output_dir: Output directory string that indicates the directory to save the results.
            If use_s3 == True, this will be the bucket name.
        class_name_to_index_map: Dictionary mapping class names to their corresponding indices.
        semantic_types: List of semantic types to consider when filtering annotator data.
        image_output_format: String that indicates the format of saved RGB images.
        use_s3: Boolean value that indicates whether output will be written to s3 bucket.
        bucket_name: Name of the S3 bucket when using S3 output.
        endpoint_url: Custom endpoint URL for S3 service.
        s3_region: AWS region for S3 bucket.

    Example:
        .. code-block:: python

            >>> import omni.replicator.core as rep
            >>> camera = rep.create.camera()
            >>> render_product = rep.create.render_product(camera, (512, 512))
            >>> writer = rep.WriterRegistry.get("DOPEWriter")
            >>> import carb
            >>> tmp_dir = carb.tokens.get_tokens_interface().resolve("${temp}/rgb")
            >>> writer.initialize(output_dir=tmp_dir, class_name_to_index_map=class_name_to_index_map)
            >>> writer.attach([render_product])
            >>> rep.orchestrator.run()
    """

    def __init__(
        self,
        output_dir: str,
        class_name_to_index_map: Dict,
        semantic_types: List[str] = None,
        image_output_format: str = "png",
        use_s3: bool = False,
        bucket_name: str = "",
        endpoint_url: str = "",
        s3_region: str = "us-east-1",
    ):
        carb.log_warn(
            "Deprecation warning: DOPEWriter has been deprecated and will be removed in the next major release."
        )

        self._output_dir = output_dir
        self._frame_id = 0
        self._image_output_format = image_output_format
        self._last_frame_is_valid = True
        self.annotators = []
        self.class_to_index = class_name_to_index_map
        self.index_to_class = {i: c for c, i in class_name_to_index_map.items()}

        self.use_s3 = use_s3

        if self.use_s3:
            if len(bucket_name) < 3 or len(bucket_name) > 63:
                raise Exception(
                    "Name of s3 bucket must be between 3 and 63 characters long. Please pass in a new bucket name to --output_folder."
                )

            self.backend = BackendDispatch(
                output_dir=output_dir,
                key_prefix=output_dir,
                bucket=bucket_name,
                region=s3_region,
                endpoint_url=endpoint_url,
                overwrite=True,
            )
        else:
            self.backend = BackendDispatch({"paths": {"out_dir": output_dir}}, overwrite=True)

        self._backend = self.backend  # Kept for backwards compatibility

        # Specify the semantic types that will be included in output
        if semantic_types is None:
            semantic_types = ["class"]

        # RGB
        self.annotators.append(AnnotatorRegistry.get_annotator("rgb"))

        # Pose Data
        self.annotators.append(AnnotatorRegistry.get_annotator("dope", init_params={"semanticTypes": semantic_types}))

    def register_pose_annotator(config_data: dict):
        AnnotatorRegistry.register_annotator_from_node(
            name="DopeSync",
            input_rendervars=[
                NodeConnectionTemplate(
                    "PostProcessDispatcher",
                    attributes_mapping={
                        "outputs:referenceTimeNumerator": "inputs:rationalTimeNumerator",
                        "outputs:referenceTimeDenominator": "inputs:rationalTimeDenominator",
                    },
                ),
                NodeConnectionTemplate("CameraParams", attributes_mapping={"outputs:exec": "inputs:execIn"}),
                NodeConnectionTemplate("InstanceMapping", attributes_mapping={"outputs:exec": "inputs:execIn"}),
                NodeConnectionTemplate("bounding_box_3d", attributes_mapping={"outputs:exec": "inputs:execIn"}),
            ],
            node_type_id="omni.graph.action.RationalTimeSyncGate",
        )
        # Register annotator for Replicator telemetry tracking
        (
            AnnotatorRegistry._default_annotators.append("DopeSync")
            if "DopeSync" not in AnnotatorRegistry._default_annotators
            else None
        )

        AnnotatorRegistry.register_annotator_from_node(
            name="dope",
            input_rendervars=[
                NodeConnectionTemplate("DopeSync", attributes_mapping={"outputs:execOut": "inputs:exec"}),
                "CameraParams",
                "InstanceMapping",
                NodeConnectionTemplate("bounding_box_3d", attributes_mapping={"outputs:data": "inputs:boundingBox3d"}),
            ],
            node_type_id="isaacsim.replicator.writers.Dope",
            init_params={
                "width": config_data["WIDTH"],
                "height": config_data["HEIGHT"],
                "cameraRotation": np.array(config_data["CAMERA_ROTATION"]),
            },
            output_data_type=np.dtype(
                [
                    ("semanticId", "<u4"),
                    ("visibility", "<f4"),
                    ("location", "<f4", (3,)),
                    ("rotation", "<f4", (4,)),  # Quaternion
                    ("projected_cuboid", "<f4", (9, 2)),  # Includes Center
                ]
            ),
            output_is_2d=False,
        )
        # Register annotator for Replicator telemetry tracking
        (
            AnnotatorRegistry._default_annotators.append("dope")
            if "dope" not in AnnotatorRegistry._default_annotators
            else None
        )

    def setup_writer(config_data: dict, writer_config: dict):
        """Initialize writer and attach render product.

        Args:
            config_data: A dictionary containing the general configurations for the script.
            writer_config: A dictionary containing writer-specific configurations.
        """
        writer = WriterRegistry.get("DOPEWriter")
        writer.initialize(
            output_dir=writer_config["output_folder"],
            class_name_to_index_map=config_data["CLASS_NAME_TO_INDEX"],
            use_s3=writer_config["use_s3"],
            bucket_name=writer_config["bucket_name"],
            endpoint_url=writer_config["endpoint_url"],
        )

        return writer

    def write(self, data: dict):
        """Write function called from the OgnWriter node on every frame to process annotator output.

        Args:
            data: A dictionary containing the annotator data for the current frame.
        """
        if not self._check_frame_validity(data):
            print(f"No training data in frame {self._frame_id} (object(s) fully occluded), skipping writing..")
            return

        for annotator in data.keys():
            annotator_split = annotator.split("-")
            render_product_path = ""
            multi_render_prod = 0
            # multiple render_products
            if len(annotator_split) > 1:
                multi_render_prod = 1
                render_product_name = annotator_split[-1]
                render_product_path = f"{render_product_name}/"

            if annotator.startswith("rgb"):
                if multi_render_prod:
                    render_product_path += "rgb/"
                self._write_rgb(data, render_product_path, annotator)

            if annotator.startswith("dope"):
                self._write_dope(data, render_product_path, annotator)

        self._frame_id += 1

    def _write_rgb(self, data: dict, render_product_path: str, annotator: str):
        image_id = "{:06d}".format(self._frame_id)

        file_path = f"{render_product_path}{image_id}.{self._image_output_format}"
        self._backend.write_image(file_path, data[annotator])

    def _write_dope(self, data: dict, render_product_path: str, annotator: str):
        image_id = "{:06d}".format(self._frame_id)

        dope_data = data[annotator]["data"]
        id_to_labels = data[annotator]["info"]["idToLabels"]

        objects = []

        for object in dope_data:
            semanticId = object["semanticId"]

            class_name = id_to_labels[str(semanticId)]["class"]
            class_name = f"0{class_name.lstrip('_')}" if class_name[0] == "_" else class_name

            groundtruth = {
                "class": class_name,
                "visibility": object["visibility"].astype(np.float64),
                "location": object["location"].astype(np.float64),
                "quaternion_wxyz": object["rotation"].astype(np.float64),
                "projected_cuboid": object["projected_cuboid"].astype(np.float64),
            }

            objects.append(groundtruth)

        output = {"camera_data": {}, "objects": objects}  # TO-DO: Add camera_data. This is not used for training script

        file_path = f"{render_product_path}{image_id}.json"
        buf = io.BytesIO()
        buf.write(json.dumps(output, indent=2, cls=_NumpyEncoder).encode())
        self._backend.write_blob(file_path, buf.getvalue())

    def _check_frame_validity(self, data: dict) -> bool:
        """Check and flag frame as valid if training data is present in the frame.

        Args:
            data (dict): The frame data to check.

        Returns:
            bool: True if frame is valid, False otherwise.
        """
        self._last_frame_is_valid = False
        if "dope" in data and "data" in data["dope"]:
            for val in data["dope"]["data"]:
                if val["visibility"] > 0.0:
                    self._last_frame_is_valid = True
                    break

        return self._last_frame_is_valid

    def is_last_frame_valid(self) -> bool:
        """Check if the last frame was valid (training data was present).

        Returns:
            bool: True if the last frame was valid, False otherwise.
        """
        return self._last_frame_is_valid
