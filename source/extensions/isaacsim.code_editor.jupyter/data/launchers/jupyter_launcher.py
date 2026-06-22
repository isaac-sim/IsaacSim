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

"""Jupyter notebook launcher for Isaac Sim."""

import os
import sys
from typing import Any

PACKAGES_PATH = []
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# add packages to sys.path
with open(os.path.join(SCRIPT_DIR, "packages.txt")) as f:
    for p in f.readlines():
        p = p.strip()
        if p:
            PACKAGES_PATH.append(p)
            if p not in sys.path:
                print(f"Adding package to sys.path: {p}")
                sys.path.append(p)

# add provisioners to sys.path
sys.path.append(os.path.abspath(os.path.join(SCRIPT_DIR, "..", "provisioners")))


from jupyter_client.kernelspec import KernelSpecManager as _KernelSpecManager


class KernelSpecManager(_KernelSpecManager):
    """Custom kernel spec manager that loads Isaac Sim kernels.

    Args:
        *args: Positional arguments passed to the Jupyter kernel spec manager.
        **kwargs: Keyword arguments passed to the Jupyter kernel spec manager.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        kernel_dir = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "kernels"))
        if kernel_dir not in self.kernel_dirs:
            self.kernel_dirs.append(kernel_dir)


def main(ip: str = "127.0.0.1", port: int = 8228, argv: list[str] | None = None) -> None:
    """Entry point for launching Jupyter Notebook/Lab.

    Args:
        ip: Notebook server IP address.
        port: Notebook server port number.
        argv: Command line arguments to pass to Jupyter Notebook/Lab.
    """
    if argv is None:
        argv = []

    def load_jupyter_lab() -> Any:
        from jupyter_server.serverapp import ServerApp
        from jupyterlab.labapp import LabApp

        jpserver_extensions = {LabApp.get_extension_package(): True}
        find_extensions = LabApp.load_other_extensions
        if "jpserver_extensions" in LabApp.serverapp_config:
            jpserver_extensions.update(LabApp.serverapp_config["jpserver_extensions"])
            LabApp.serverapp_config["jpserver_extensions"] = jpserver_extensions
            find_extensions = False

        app = ServerApp.instance(
            ip=ip, port=port, kernel_spec_manager_class=KernelSpecManager, jpserver_extensions=jpserver_extensions
        )

        app.aliases.update(LabApp.aliases)
        app.initialize(argv=argv, starter_extension=LabApp.name, find_extensions=find_extensions)
        return app

    # load app
    app = load_jupyter_lab()

    # write url to file in script directory
    with open(os.path.join(SCRIPT_DIR, "notebook.txt"), "w") as f:
        f.write(app.display_url)

    app.start()


if __name__ == "__main__":

    argv = sys.argv[1:]

    # testing the launcher
    if not len(argv):
        print("Testing the launcher")
        argv = [
            "127.0.0.1",  # ip
            "8888",  # port
            "",  # token
            "",  # notebook_dir
            "--allow-root --no-browser",
        ]  # extra arguments

    # get function arguments
    ip = argv[0]
    port = int(argv[1])
    token = argv[2]

    # get notebook_dir
    if not argv[3]:
        notebook_dir = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "notebooks"))
        notebook_dir = f"--notebook-dir={notebook_dir}"

    # get token
    token = f"--ServerApp.token={token}"

    # assets path
    app_dir = []
    for p in PACKAGES_PATH:
        print(f"Checking package to app_dir: {p}")
        if os.path.exists(os.path.join(p, "share", "jupyter", "lab")):
            app_dir = ["--app-dir={}".format(os.path.join(p, "share", "jupyter", "lab"))]
        if os.path.exists(os.path.join(p, "jupyterlab", "static")):
            app_dir = ["--app-dir={}".format(os.path.join(p, "jupyterlab"))]
        if app_dir:
            break
    print(f"app_dir: {app_dir}")

    # clean up the argv
    argv = app_dir + [token] + [notebook_dir] + argv[4].split(" ")

    # run the launcher
    print(f"Starting Jupyter Lab at {ip}:{port}")
    print(" with argv: {}".format(" ".join(argv)))

    main(ip=ip, port=port, argv=argv)

    # delete notebook.txt
    try:
        os.remove(os.path.join(SCRIPT_DIR, "notebook.txt"))
    except BaseException:
        pass
