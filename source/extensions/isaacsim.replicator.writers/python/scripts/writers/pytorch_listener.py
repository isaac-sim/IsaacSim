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

"""Provides a PyTorch-compatible listener for tracking and retrieving data from Replicator writers."""

from __future__ import annotations


class PytorchListener:
    """An Observer/Listener that keeps track of updated data sent by the writer. Is passed in the.

    itialization of a PytorchWriter at which point it is pinged by the writer after any data is
    passed to the writer.

    .. deprecated:: 1.5.0

        This class is deprecated and will be removed in a future version. No replacement is provided.
    """

    def __init__(self):
        self.data = {}

    def write_data(self, data: dict):
        """Update the existing data in the listener with the new data provided.

        Args:
            data: New data retrieved from writer.
        """

        self.data.update(data)

    def get_rgb_data(self) -> "torch.Tensor | None":
        """Return RGB data as a batched tensor from the current data stored.

        Returns:
            Images in batched pytorch tensor form.
        """

        if "pytorch_rgb" in self.data:
            images = self.data["pytorch_rgb"]
            images = images[..., :3]
            images = images.permute(0, 3, 1, 2)
            return images
        else:
            return None
