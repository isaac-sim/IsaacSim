# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Extension for integrating Jupyter Notebook functionality into Isaac Sim."""

from __future__ import annotations

import asyncio
import glob
import json
import os
import secrets
import socket
import subprocess
import sys

import carb
import carb.eventdispatcher
import jedi
import omni.ext
import omni.kit.app

from . import executor, ui_builder


def _get_event_loop() -> asyncio.AbstractEventLoop:
    """Backward compatible function for getting the event loop.

    Returns:
        The current event loop, falling back to the policy's event loop if no loop is running.
    """
    try:
        return asyncio.get_event_loop()
    except RuntimeError:
        return asyncio.get_event_loop_policy().get_event_loop()


class Extension(omni.ext.IExt):
    """Extension for integrating Jupyter Notebook functionality into Isaac Sim.

    This extension provides a complete Jupyter Notebook environment within Isaac Sim, enabling interactive
    Python development and execution. It creates a socket server for code execution, completion, and
    introspection, while launching a separate Jupyter Notebook process for the web interface.

    The extension supports:

    - Code execution with output capture and error handling
    - Intelligent code completion using Jedi
    - Code introspection for documentation and help
    - Configurable host, port, and directory settings
    - Automatic process cleanup and port management
    - Cross-platform compatibility (Windows and Linux)

    The extension automatically configures the Python environment with access to Isaac Sim's modules and
    extensions, ensuring seamless integration between Jupyter notebooks and the simulation environment.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the Jupyter extension and starts all necessary components.

        Sets up socket server for code execution, configures Jedi autocompletion, launches Jupyter notebook process,
        and initializes the UI builder.

        Args:
            ext_id: Extension identifier used to get extension path and settings.
        """
        self._globals = {**globals()}

        # get extension path
        self._extension_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(ext_id)

        # get extension settings
        settings = carb.settings.get_settings()
        self._socket_host = settings.get("/exts/isaacsim.code_editor.jupyter/host")
        self._socket_port = settings.get("/exts/isaacsim.code_editor.jupyter/port")

        self._notebook_ip = settings.get("/exts/isaacsim.code_editor.jupyter/notebook_ip")
        self._notebook_port = settings.get("/exts/isaacsim.code_editor.jupyter/notebook_port")
        self._notebook_token = settings.get("/exts/isaacsim.code_editor.jupyter/notebook_token")
        self._notebook_dir = settings.get("/exts/isaacsim.code_editor.jupyter/notebook_dir")
        self._command_line_options = settings.get("/exts/isaacsim.code_editor.jupyter/command_line_options")

        kill_processes_with_port_in_use = settings.get(
            "/exts/isaacsim.code_editor.jupyter/kill_processes_with_port_in_use"
        )

        # ensure port is free
        if kill_processes_with_port_in_use:
            # windows
            if sys.platform == "win32":
                pids = []
                cmd = ["netstat", "-ano"]
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
                for line in p.stdout:
                    if f":{self._socket_port}".encode() in line or f":{self._notebook_port}".encode() in line:
                        if "listening".encode() in line.lower():
                            carb.log_info(f"Open port: {line.strip().decode()}")
                            pids.append(line.strip().split(b" ")[-1].decode())
                p.wait()
                for pid in pids:
                    if not pid.isnumeric():
                        continue
                    carb.log_warn(f"Forced process shutdown with PID {pid}")
                    cmd = ["taskkill", "/PID", pid, "/F"]
                    subprocess.Popen(cmd).wait()
            # linux
            elif sys.platform == "linux":
                pids = []
                cmd = ["netstat", "-ltnup"]
                try:
                    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
                    for line in p.stdout:
                        if f":{self._socket_port}".encode() in line or f":{self._notebook_port}".encode() in line:
                            carb.log_info(f"Open port: {line.strip().decode()}")
                            pids.append(
                                [chunk for chunk in line.strip().split(b" ") if b"/" in chunk][-1]
                                .decode()
                                .split("/")[0]
                            )
                    p.wait()
                except FileNotFoundError as e:
                    carb.log_warn(f"Command (netstat) not available. Install it using `apt install net-tools`")
                for pid in pids:
                    if not pid.isnumeric():
                        continue
                    carb.log_warn(f"Forced process shutdown with PID {pid}")
                    cmd = ["kill", "-9", pid]
                    subprocess.Popen(cmd).wait()

        # shutdown stream subscription
        self._shutdown_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_POST_QUIT,
            on_event=self._on_shutdown_event,
            observer_name="isaacsim.code_editor.jupyter._on_shutdown_event",
            order=0,
        )

        ext_name = omni.ext.get_extension_name(ext_id)
        self._ui_builder = ui_builder.UIBuilder(
            ext_name, "Window", "Jupyter Notebook", self._notebook_ip, self._notebook_port, self._get_display_url
        )
        self._ui_builder.startup()

        # create socket (code execution)
        self._token = ""
        self._server = None
        _get_event_loop().create_task(self._create_socket())

        # jedi (autocompletion and introspection)
        # application root path
        app_folder = settings.get_as_string("/app/folder")
        if not app_folder:
            app_folder = carb.tokens.get_tokens_interface().resolve("${app}")
        path = os.path.normpath(os.path.join(app_folder, os.pardir))
        # get extension paths
        folders = [
            "exts",
            "extscache",
            os.path.join("kit", "exts"),
            os.path.join("kit", "extscore"),
            os.path.join("kit", "extensions"),
            os.path.join("kit", "kernel", "py"),
        ]
        added_sys_path = []
        for folder in folders:
            sys_paths = glob.glob(os.path.join(path, folder, "*"))
            for sys_path in sys_paths:
                if os.path.isdir(sys_path):
                    added_sys_path.append(sys_path)
        # python environment
        python_exe = "python.exe" if sys.platform == "win32" else "bin/python3"
        environment_path = os.path.join(path, "kit", "python", python_exe)
        # jedi project
        carb.log_info("Autocompletion: jedi.Project")
        carb.log_info(f"  |-- path: {path}")
        carb.log_info(f"  |-- added_sys_path: {len(added_sys_path)} items")
        carb.log_info(f"  |-- environment_path: {environment_path}")
        self._jedi_project = jedi.Project(
            path=path, environment_path=environment_path, added_sys_path=added_sys_path, load_unsafe_extensions=False
        )

        # run jupyter in a separate process
        self._launch_jupyter_process()

    def on_shutdown(self) -> None:
        """Shuts down the Jupyter extension and cleans up all resources.

        Closes socket server, terminates Jupyter notebook process, and shuts down UI components.
        """
        self._shutdown_subscription = None
        self._ui_builder.shutdown()
        # close socket
        if self._server is not None:
            self._server.close()
            asyncio.run_coroutine_threadsafe(self._server.wait_closed(), _get_event_loop())
            self._server = None
        # end jupyter notebook (external process)
        if self._process is not None:
            process_pid = self._process.pid
            try:
                self._process.terminate()  # .kill()
            except OSError as e:
                if sys.platform == "win32":
                    if e.winerror != 5:
                        raise
                else:
                    from errno import ESRCH

                    if not isinstance(e, ProcessLookupError) or e.errno != ESRCH:
                        raise
            # make sure the process is not running anymore in Windows
            if sys.platform == "win32":
                subprocess.call(["taskkill", "/F", "/T", "/PID", str(process_pid)])
            # wait for the process to terminate
            self._process.wait()
            self._process = None

    def _on_shutdown_event(self, event: carb.eventdispatcher.Event) -> None:
        """Handles shutdown event from the application.

        Args:
            event: The shutdown event from the event dispatcher.
        """
        self.on_shutdown()

    async def _create_socket(self) -> None:
        """Create a socket server to listen for incoming connections."""

        class ServerProtocol(asyncio.Protocol):
            def __init__(self, parent: Extension) -> None:
                super().__init__()
                self._parent = parent

            def connection_made(self, transport: asyncio.BaseTransport) -> None:
                carb.log_info(f"Connection from {transport.get_extra_info('peername')}")
                self.transport = transport

            def data_received(self, data: bytes) -> None:
                data = data.decode()
                token = data[: len(self._parent._token)]
                source = data[len(self._parent._token) :]
                if not secrets.compare_digest(token, self._parent._token):
                    reply = json.dumps(
                        {
                            # cell output
                            "status": "error",
                            "output": "",
                            "ename": "AuthenticationError",
                            "evalue": "Invalid or missing authentication token",
                            "traceback": [],
                            # completion
                            "matches": [],
                            "delta": 0,
                            # introspection
                            "found": False,
                            "data": {},
                        },
                        separators=(",", ":"),
                    )
                    self.transport.write(reply.encode())
                    self.transport.close()
                    return
                # completion
                if source[:3] == "%!c":
                    source = source[3:]
                    asyncio.run_coroutine_threadsafe(
                        self._parent._complete_code_async(source, self.transport), _get_event_loop()
                    )
                # introspection
                elif source[:3] == "%!i":
                    source = source[3:]
                    pos = source.find("%")
                    line, column = [int(i) for i in source[:pos].split(":")]
                    source = source[pos + 1 :]
                    asyncio.run_coroutine_threadsafe(
                        self._parent._introspect_code_async(source, line, column, self.transport), _get_event_loop()
                    )
                # execution
                else:
                    asyncio.run_coroutine_threadsafe(
                        self._parent._process_code(source, self.transport), _get_event_loop()
                    )

        try:
            self._server = await _get_event_loop().create_server(
                protocol_factory=lambda: ServerProtocol(self),
                host=self._socket_host,
                port=self._socket_port,
                family=socket.AF_INET,
                reuse_port=None if sys.platform == "win32" else True,
            )
            carb.log_info(f"Serving at {self._socket_host}:{self._socket_port}")
            await self._server.start_serving()
        except Exception as e:
            carb.log_error(str(e))
            self._server = None

    async def _process_code(self, source: str, transport: asyncio.Transport) -> None:
        """Execute the source code in the Kit Python scope and send the result back to the client.

        Args:
            source: Python source code to execute.
            transport: Transport connection to send results back to the client.
        """
        _executor = executor.Executor(self._globals, self._globals)
        output, exception, trace = await _executor.execute(source)
        if output.endswith("\n"):
            output = output[:-1]
        # build reply
        reply = {"status": "ok" if exception is None else "error", "output": output}
        if exception is not None:
            reply["traceback"] = [trace]
            reply["ename"] = str(type(exception).__name__)
            reply["evalue"] = str(exception)
        # send the reply to the client
        reply = json.dumps(reply, separators=(",", ":"))
        transport.write(reply.encode())
        # close the connection
        transport.close()

    async def _complete_code_async(self, source: str, transport: asyncio.Transport) -> None:
        """Complete objects under the cursor and send the result back to the client.

        Args:
            source: Python source code for completion.
            transport: Transport connection to send completion results back to the client.
        """
        # generate completions
        script = jedi.Script(source, project=self._jedi_project)
        completions = script.complete()
        delta = completions[0].get_completion_prefix_length() if completions else 0
        # build reply
        reply = {"matches": [c.name for c in completions], "delta": delta}
        # send the reply to the client
        reply = json.dumps(reply)
        transport.write(reply.encode())
        # close the connection
        transport.close()

    async def _introspect_code_async(self, source: str, line: int, column: int, transport: asyncio.Transport) -> None:
        """Introspect code under the cursor and send the result back to the client.

        Args:
            source: Python source code for introspection.
            line: Line number of cursor position.
            column: Column number of cursor position.
            transport: Transport connection to send introspection results back to the client.
        """
        # generate introspection
        script = jedi.Script(source, project=self._jedi_project)
        definitions = script.infer(line=line, column=column)
        # build reply
        reply = {"found": False, "data": "TODO"}
        if len(definitions):
            reply["found"] = True
            reply["data"] = definitions[0].docstring()
        # send the reply to the client
        reply = json.dumps(reply)
        transport.write(reply.encode())
        # close the connection
        transport.close()

    # Jupyter Notebook methods

    def _launch_jupyter_process(self) -> None:
        """Launch the Jupyter notebook in a separate process."""
        launchers_dir = os.path.join(self._extension_path, "data", "launchers")

        # generate token
        self._token = secrets.token_hex(16)
        token_txt = os.path.join(launchers_dir, "token.txt")
        with open(token_txt, "w") as f:
            f.write(self._token)

        # get packages path
        paths = [p for p in sys.path if "pip3-envs" in p]
        packages_txt = os.path.join(launchers_dir, "packages.txt")
        with open(packages_txt, "w") as f:
            f.write("\n".join(paths))

        if sys.platform == "win32":
            executable_path = os.path.abspath(os.path.join(os.path.dirname(os.__file__), "..", "python.exe"))
        else:
            executable_path = os.path.abspath(os.path.join(os.path.dirname(os.__file__), "..", "..", "bin", "python3"))

        cmd = [
            executable_path,
            os.path.join(launchers_dir, "jupyter_launcher.py"),
            self._notebook_ip,
            str(self._notebook_port),
            self._notebook_token,
            self._notebook_dir,
            self._command_line_options,
        ]

        carb.log_info("Starting Jupyter server in separate process")
        carb.log_info("  |-- command: " + " ".join(cmd))
        try:
            self._process = subprocess.Popen(cmd, cwd=launchers_dir)
        except Exception as e:
            carb.log_error(f"Error starting Jupyter server: {e}")
            self._process = None

    def _get_display_url(self) -> str:
        """Get the Jupyter notebook app.display_url.

        Returns:
            The display URL for the Jupyter notebook or empty string if not available.
        """
        display_url = ""
        if self._process is not None:
            notebook_txt = os.path.join(self._extension_path, "data", "launchers", "notebook.txt")
            if os.path.exists(notebook_txt):
                with open(notebook_txt) as f:
                    urls = f.readlines()
                    display_url = "\n".join([url.strip() for url in urls])
        return display_url
