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

"""Define a Replicator writer that converts RGB data to PyTorch tensor batches and sends them to a listener."""

from __future__ import annotations

import carb
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from omni.replicator.core import AnnotatorRegistry, BackendDispatch, Writer

from .pytorch_listener import PytorchListener

__version__ = "0.0.1"


class PytorchWriter(Writer):
    """A custom writer that uses omni.replicator API to retrieve RGB data via render products.

            and formats them as tensor batches. The writer takes a PytorchListener which is able
            to retrieve pytorch tensors for the user directly after each writer call.

    .. deprecated:: 1.5.0

        This class is deprecated and will be removed in a future version. No replacement is provided.

    Args:
        listener: A PytorchListener that is sent pytorch batch tensors at each write() call.
        output_dir: Directory in which rgb data will be saved in PNG format by the backend dispatch.
            If not specified, the writer will not write rgb data as png and only ping the
            listener with batched tensors.
        tiled_sensor: Whether to use tiled sensor mode.
        device: Device in which the pytorch tensor data will reside. Can be "cpu", "cuda", or any
            other format that pytorch supports for devices.
    """

    def __init__(
        self, listener: PytorchListener, output_dir: str = None, tiled_sensor: bool = False, device: str = "cuda"
    ) -> None:
        # If output directory is specified, writer will write annotated data to the given directory
        if output_dir:
            self.backend = BackendDispatch({"paths": {"out_dir": output_dir}})
            self._backend = self.backend
            self._output_dir = self.backend.output_dir
        else:
            self._output_dir = None
        self._frame_id = 0

        self.annotators = []
        if tiled_sensor:
            self.annotators = [AnnotatorRegistry.get_annotator("RtxSensorGpu", device="cuda", do_array_copy=False)]
        else:
            self.annotators = [AnnotatorRegistry.get_annotator("LdrColor", device="cuda", do_array_copy=False)]

        self.listener = listener
        self.device = device
        self.version = __version__

    def write(self, data: dict) -> None:
        """Send data captured by the attached render products to the PytorchListener and will write data to.

        the output directory if specified during initialization.

        Args:
            data: Data to be pinged to the listener and written to the output directory if specified.
        """
        # breakpoint()
        for annotator in data:
            if annotator.startswith("rp"):
                rp_info = data[annotator]

        if self._output_dir:
            # Write RGB data to output directory as png
            self._write_rgb(data, rp_info)
        pytorch_rgb = self._convert_to_pytorch(data, rp_info).to(self.device)
        self.listener.write_data({"pytorch_rgb": pytorch_rgb, "device": self.device})
        self._frame_id += 1

    @carb.profiler.profile
    def _write_rgb(self, data: dict, rp_info: dict) -> None:
        """Write RGB data to the output directory as PNG files.

        Args:
            data: Dictionary containing annotator data with RGB information.
            rp_info: Render product information containing resolution and other metadata.
        """
        for annotator in data:
            if annotator.startswith("LdrColor"):
                render_product_name = annotator.split("-")[-1]
                file_path = f"rgb_{self._frame_id}_{render_product_name}.png"
                img_data = data[annotator]
                if isinstance(img_data, wp.array):
                    img_data = img_data.numpy()
                self._backend.write_image(file_path, img_data)
            elif annotator.startswith("RtxSensor"):
                width, height = rp_info["resolution"][0], rp_info["resolution"][1]
                file_path = f"rgb_{self._frame_id}.png"
                img_data = data[annotator].reshape(height, width, -1)
                self._backend.write_image(file_path, img_data)

    @carb.profiler.profile
    def _convert_to_pytorch(self, data: dict, rp_info: dict) -> "torch.Tensor":
        """Convert annotator data to a PyTorch tensor batch.

        Args:
            data: Dictionary containing annotator data to convert.
            rp_info: Render product information containing resolution and other metadata.

        Returns:
            Concatenated PyTorch tensor containing all converted data.

        Raises:
            Exception: If data is None.
        """
        if data is None:
            raise Exception("Data is Null")
        # breakpoint()
        data_tensors = []
        for annotator in data:
            if annotator.startswith("LdrColor"):
                data_tensors.append(wp.to_torch(data[annotator]).unsqueeze(0))
            elif annotator.startswith("RtxSensor"):
                width, height = rp_info["resolution"][0], rp_info["resolution"][1]
                img_data = data[annotator]
                data_tensors.append(wp.to_torch(img_data).reshape(height, width, -1).unsqueeze(0))
            elif annotator.startswith("distance"):
                width, height = rp_info["resolution"][0], rp_info["resolution"][1]
                data_tensors.append(wp.to_torch(data[annotator]).reshape(height, width, -1).unsqueeze(0))

        # Move all tensors to the same device for concatenation
        device = "cuda:0" if self.device == "cuda" else self.device
        data_tensors = [t.to(device) for t in data_tensors]

        torch = import_module("torch")
        data_tensor = torch.cat(data_tensors, dim=0)
        return data_tensor
