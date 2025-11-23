# SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os
import sys
from typing import List

PACKAGES_PATH = []
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# add packages to sys.path
with open(os.path.join(SCRIPT_DIR, "packages.txt"), "r") as f:
    for p in f.readlines():
        p = p.strip()
        if p:
            PACKAGES_PATH.append(p)
            if p not in sys.path:
                print("Adding package to sys.path: {}".format(p))
                sys.path.append(p)

# add provisioners to sys.path
sys.path.append(os.path.abspath(os.path.join(SCRIPT_DIR, "..", "provisioners")))


from jupyter_client.kernelspec import KernelSpecManager as _KernelSpecManager


class KernelSpecManager(_KernelSpecManager):
    def __init__(self, *args, **kwargs):
        """Custom kernel spec manager to allow for loading of custom kernels"""
        super().__init__(*args, **kwargs)
        kernel_dir = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "kernels"))
        if kernel_dir not in self.kernel_dirs:
            self.kernel_dirs.append(kernel_dir)


def main(ip: str = "127.0.0.1", port: int = 8228, argv: List[str] = []) -> None:
    """Entry point for launching Jupyter Notebook/Lab

    :param ip: Notebook server IP address (default: "127.0.0.1")
    :type code: str, optional
    :param port: Notebook server port number (default: 8228)
    :type code: int, optional
    :param argv: Command line arguments to pass to Jupyter Notebook/Lab (default: [])
    :type code: List of strings, optional
    """

    def load_jupyter_lab():
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
        notebook_dir = "--notebook-dir={}".format(notebook_dir)
    else:
        notebook_dir = argv[3]

    # get token
    token = "--ServerApp.token={}".format(token)

    # assets path
    app_dir = []
    for p in PACKAGES_PATH:
        print("Checking package to app_dir: {}".format(p))
        if os.path.exists(os.path.join(p, "share", "jupyter", "lab")):
            app_dir = ["--app-dir={}".format(os.path.join(p, "share", "jupyter", "lab"))]
        if os.path.exists(os.path.join(p, "jupyterlab", "static")):
            app_dir = ["--app-dir={}".format(os.path.join(p, "jupyterlab"))]
        if app_dir:
            break
    print("app_dir: {}".format(app_dir))

    # clean up the argv
    argv = app_dir + [token] + [notebook_dir] + argv[4].split(" ")

    # run the launcher
    print("Starting Jupyter Lab at {}:{}".format(ip, port))
    print(" with argv: {}".format(" ".join(argv)))

    main(ip=ip, port=port, argv=argv)

    # delete notebook.txt
    try:
        os.remove(os.path.join(SCRIPT_DIR, "notebook.txt"))
    except:
        pass
