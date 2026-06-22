# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Socket-based kernel provisioner for embedded Omniverse Python."""

import os
import sys
from typing import Any

from jupyter_client.connect import KernelConnectionInfo
from jupyter_client.launcher import launch_kernel
from jupyter_client.provisioning import LocalProvisioner


class Provisioner(LocalProvisioner):
    """Provision a kernel using a socket connection to the Omniverse runtime."""

    async def launch_kernel(self, cmd: list[str], **kwargs: Any) -> KernelConnectionInfo:
        """Launch the kernel process with the correct Python executable path.

        Args:
            cmd: Kernel launch command received from Jupyter.
            **kwargs: Additional launch options passed to Jupyter's kernel launcher.

        Returns:
            Kernel connection information for the launched process.
        """
        # set paths
        if sys.platform == "win32":
            cmd[0] = os.path.abspath(os.path.join(os.path.dirname(os.__file__), "..", "python.exe"))
        else:
            cmd[0] = os.path.abspath(os.path.join(os.path.dirname(os.__file__), "..", "..", "bin", "python3"))
        cmd[1] = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "launchers", "ipykernel_launcher.py")
        )

        self.log.info("Launching kernel: %s", " ".join(cmd))

        scrubbed_kwargs = LocalProvisioner._scrub_kwargs(kwargs)
        self.process = launch_kernel(cmd, **scrubbed_kwargs)
        pgid = None
        if hasattr(os, "getpgid"):
            try:
                pgid = os.getpgid(self.process.pid)
            except OSError:
                pass

        self.pid = self.process.pid
        self.pgid = pgid
        return self.connection_info
