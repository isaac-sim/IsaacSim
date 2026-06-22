# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Common utilities and base test case for UCX node tests."""

import os
import socket

import numpy as np

try:
    from isaacsim.test.utils import TimedAsyncTestCase
except ImportError:
    raise ImportError(
        "isaacsim.test.utils is required to use UCXTestCase. " "This module should only be imported in test contexts."
    )

# UCX imports
import omni
import omni.timeline
import ucxx._lib.libucxx as ucx_api


def _read_tensor_f32(tensor: object) -> list:
    """Read float32 values from a FlatBuffers Tensor's ubyte data vector.

    Args:
        tensor: FlatBuffers Tensor object with ubyte data.

    Returns:
        Tensor data interpreted as float32 values.
    """
    n_bytes = tensor.DataLength()
    raw = bytes(tensor.Data(i) for i in range(n_bytes))
    return np.frombuffer(raw, dtype=np.float32).tolist()


def find_available_port() -> int:
    """Find an available port for a UCX listener.

    Creates a temporary socket bound to port 0, allowing the OS to assign
    an available ephemeral port, then closes the socket and returns the port.

    Returns:
        An available port number.

    Example:

    .. code-block:: python

        >>> from isaacsim.ucx.nodes.tests.common import find_available_port
        >>>
        >>> port = find_available_port()
        >>> isinstance(port, int) and 1024 <= port <= 65535
        True
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("", 0))
        return sock.getsockname()[1]


class UCXTestCase(TimedAsyncTestCase):
    """Base class for UCX node tests."""

    async def setUp(self) -> None:
        """Set up UCX test environment and find an available port."""
        await super().setUp()

        # UCX transport selection for tests. `tcp` + `self` cover host-to-host TCP
        # and same-process loopback. `cuda_copy` enables UCX's CUDA memtype module
        # so it can stage GPU buffers through host memory when one side of a
        # transfer is `cuda/dev[*]` — required by the GPU-direct two-message path
        # in OgnUCXPublishImage (`sendCudaBuffer=True`). Without it UCX aborts
        # the rendezvous recv with "cannot find remote protocol for ... rndv_recv
        # into host memory from cuda/dev[0]".
        os.environ["UCX_TLS"] = "tcp,self,cuda_copy"
        os.environ["UCX_NET_DEVICES"] = "all"

        self.port = find_available_port()

        # Initialize UCX client tracking
        self.client_context = None
        self.client_worker = None
        self.client_endpoint = None

    async def tearDown(self) -> None:
        """Clean up UCX client resources and stop the timeline."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()

        await omni.kit.app.get_app().next_update_async()

        # Clean up UCX client resources
        if self.client_worker:
            try:
                self.client_worker.stop_progress_thread()
            except Exception:
                pass

        if self.client_endpoint:
            self.client_endpoint = None

        self.client_context = None
        self.client_worker = None

        await super().tearDown()

    def create_ucx_client(self, port: int) -> tuple:
        """Create a UCX client connection to the specified port.

        Args:
            port: Port number to connect to.

        Returns:
            Tuple of (context, worker, endpoint).
        """
        self.client_context = ucx_api.UCXContext()
        self.client_worker = ucx_api.UCXWorker(self.client_context)

        self.client_endpoint = ucx_api.UCXEndpoint.create(
            self.client_worker,
            "127.0.0.1",
            port,
            endpoint_error_handling=True,
        )

        # Start the progress thread to handle async communication
        self.client_worker.start_progress_thread()

        return self.client_context, self.client_worker, self.client_endpoint


_ENCODING_MAP = {
    0: "custom",
    1: "rgb8",
    2: "rgba8",
    3: "bgr8",
    4: "bgra8",
    5: "r8_g8_b8",
    6: "b8_g8_r8",
    7: "mono8",
    8: "mono16",
    9: "mono32",
    10: "mono32f",
}


def unpack_image_message(buffer: object) -> tuple:
    """Unpack a UCX image FlatBuffers message.

    Args:
        buffer: Buffer containing the FlatBuffers-encoded Image message.

    Returns:
        Tuple of (timestamp, width, height, encoding, step, image_data, shape).
        encoding is a lowercase string (e.g. "rgb8").
        step is derived as total_bytes / height.
        image_data is a bytes object containing the raw pixel data.
        shape is the Tensor's shape vector as a tuple of int
        (``(height, width, bytes_per_pixel)`` on both the CPU and the
        GPU-direct metadata paths).

    Note:
        When the publisher uses the GPU-direct two-message protocol
        (``sendCudaBuffer=True``, the default for ``UCXCameraHelper``), this
        message carries only metadata: ``image_data`` is empty and ``step`` is
        derived from the ubyte vector length, which is 0. The expected pixel
        byte count is the product of the Tensor's ``shape`` dimensions — callers
        can use :py:func:`get_image_pixel_data_size` to compute it and post a
        second ``tag_recv`` on the same tag for the raw pixel buffer.
    """
    from isaacsim.ucx.nodes.messages.isaac import Image as ImageFb

    buf = bytearray(buffer.tobytes())
    msg = ImageFb.Image.GetRootAs(buf, 0)

    timestamp = msg.Header().Stamp().TimeNs() / 1e9
    height = msg.Height()
    width = msg.Width()
    encoding = _ENCODING_MAP.get(msg.Encoding(), "custom")

    tensor = msg.Data()
    n_bytes = tensor.DataLength()
    image_data = bytes(tensor.Data(i) for i in range(n_bytes))

    step = n_bytes // height if height > 0 else 0
    shape = tuple(int(tensor.Shape(i)) for i in range(tensor.ShapeLength()))

    return timestamp, width, height, encoding, step, image_data, shape


def get_image_pixel_data_size(buffer: object) -> int:
    """Return the expected pixel byte count from an Image FlatBuffer's tensor shape.

    Args:
        buffer: Buffer containing the FlatBuffers-encoded Image message.

    Returns:
        The pixel data size in bytes, computed as the product of the Tensor's
        ``shape`` dimensions (``[height, width, bytes_per_pixel]``).

    The publisher always sets the Tensor's shape vector even on the GPU-direct
    path where the embedded ``data`` ubyte vector is empty. Receivers compare
    this against ``len(image_data)`` from :py:func:`unpack_image_message`: when
    ``len(image_data) == 0`` and this returns a non-zero value, the publisher
    used the two-message protocol and the raw pixel buffer is the next message
    on the same UCX tag.
    """
    from isaacsim.ucx.nodes.messages.isaac import Image as ImageFb

    buf = bytearray(buffer.tobytes())
    msg = ImageFb.Image.GetRootAs(buf, 0)
    tensor = msg.Data()
    n = tensor.ShapeLength()
    if n == 0:
        return 0
    total = 1
    for i in range(n):
        total *= int(tensor.Shape(i))
    return total
