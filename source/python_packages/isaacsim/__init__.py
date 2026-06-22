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

import builtins
import glob
import multiprocessing
import os
import platform
import sys

# Workaround for PyTorch >=2.9 c10.dll WinError 1114 (pytorch/pytorch#166628).
# Kit's sitecustomize.py registers DLL directories via os.add_dll_directory()
# before Python code runs. When torch's _load_dll_libraries() later calls
# LoadLibraryExW with LOAD_LIBRARY_SEARCH_DEFAULT_DIRS, conflicting DLLs from
# Kit's directories can shadow torch's, causing c10.dll's DllMain to fail.
# Fix: pre-load all torch DLLs using LoadLibraryW (which uses PATH instead of
# the restricted search), so they are already in memory when torch imports.
if platform.system() == "Windows":
    try:
        from importlib.util import find_spec

        _torch_spec = find_spec("torch")
        if _torch_spec and _torch_spec.origin:
            _torch_lib = os.path.normpath(os.path.join(os.path.dirname(_torch_spec.origin), "lib"))
            if os.path.isdir(_torch_lib):
                os.environ["PATH"] = _torch_lib + os.pathsep + os.environ.get("PATH", "")
                if hasattr(os, "add_dll_directory"):
                    os.add_dll_directory(_torch_lib)

                import ctypes

                _kernel32 = ctypes.WinDLL("kernel32.dll", use_last_error=True)
                _kernel32.LoadLibraryW.restype = ctypes.c_void_p
                for _dll in sorted(os.listdir(_torch_lib)):
                    if _dll.lower().endswith(".dll"):
                        _kernel32.LoadLibraryW(os.path.join(_torch_lib, _dll))
                del _kernel32, _dll
        del _torch_spec, _torch_lib
        import torch  # noqa: F401
    except Exception:
        pass


def _aarch_preload_checking(queue):
    try:
        import torch
    except Exception as e:
        queue.put(f"Unable to check for LD_PRELOAD state ({', '.join(platform.uname())}): {e}")
        return

    message = None
    shared_libraries = [
        *glob.glob("/lib/*/libgomp.so.1"),
        *glob.glob(os.path.join(f"{torch.__path__[0]}.libs", "libgomp*")),
    ]
    preload_libraries = [item for item in os.environ.get("LD_PRELOAD", "").split(":") if item.strip()]
    paths = ":".join([item for item in shared_libraries if item not in preload_libraries])
    if paths:
        message = f"""
========================================================================
WARNING: For the application to run, some shared libraries must be
loaded before others, using one of the following options.
========================================================================

* Set the environment variable for the current terminal:

    export LD_PRELOAD="$LD_PRELOAD:{paths}"

* Execute a scoped operation (affecting the current process only):

    LD_PRELOAD="{paths}" <COMMAND>

========================================================================
"""
    queue.put(message)


def aarch_preload_checking():
    machine = platform.machine().lower()
    if sys.platform == "linux" and ("arm" in machine or "aarch" in machine):
        queue = multiprocessing.Queue()
        process = multiprocessing.Process(target=_aarch_preload_checking, args=(queue,))
        process.start()
        process.join()
        msg = queue.get()
        if msg:
            sys.exit(msg)


def bootstrap_kernel():
    # isaac-sim path
    isaacsim_path = os.path.dirname(os.path.abspath(os.path.realpath(__file__)))

    # check for non-Python package manager installation
    if isaacsim_path.split(os.sep)[-2:] == ["python_packages", "isaacsim"]:
        # DGX/ARM LD_PRELOAD checking when a (virtual) Python environment is used
        if os.path.join("kit", "python") not in sys.executable:
            aarch_preload_checking()
        return

    # DGX/ARM LD_PRELOAD checking
    aarch_preload_checking()

    # kit path (internal kernel)
    if os.path.isdir(os.path.join(isaacsim_path, "kit", "extscore")):
        kit_path = os.path.join(isaacsim_path, "kit")
        if kit_path not in sys.path:
            sys.path.append(kit_path)
        try:
            import kit_app  # importing 'kit_app' will bootstrap kernel

        except Exception as e:
            sys.exit(f"Unable to bootstrap inner kit kernel: {e}")
    # kit path (omniverse-kit kernel package)
    else:
        try:
            import omni.kit_app  # importing 'kit_app' will bootstrap kernel

            kit_path = os.path.dirname(os.path.abspath(os.path.realpath(omni.kit_app.__file__)))
        except ModuleNotFoundError:
            sys.exit("Unable to find 'omniverse-kit' package")

    # set environment variables
    if not os.environ.get("CARB_APP_PATH", None):
        os.environ["CARB_APP_PATH"] = kit_path
    if not os.environ.get("EXP_PATH", None):
        os.environ["EXP_PATH"] = os.path.join(isaacsim_path, "apps")
    if not os.environ.get("ISAAC_PATH", None):
        os.environ["ISAAC_PATH"] = os.path.join(isaacsim_path)

    # set environment variables (Jupyter)
    if os.environ.get("JPY_PARENT_PID", None):
        os.environ["ISAAC_JUPYTER_PYTHON_PACKAGE"] = "1"

    # set PYTHONPATH
    # isaac-sim
    paths = [
        os.path.join(isaacsim_path, "exts", "isaacsim.simulation_app"),
    ]
    # update sys.path
    for path in paths:
        if not path in sys.path:
            if not os.path.exists(path):
                print(f"[Warning] PYTHONPATH: path doesn't exist ({path})")
                continue
            sys.path.insert(0, path)

    # log info
    import carb

    carb.log_info(f"Isaac Sim path: {isaacsim_path}")
    carb.log_info(f"Kit path: {kit_path}")


def expose_api():
    AppFramework, SimulationApp = None, None
    try:
        # try a direct import
        from isaacsim.simulation_app import AppFramework, SimulationApp
    except ImportError:
        # try to import API from isaacsim/simulation_app folder instead
        try:
            # get isaacsim/simulation_app folder path
            isaacsim_path = os.path.dirname(os.path.abspath(os.path.realpath(__file__)))
            path = glob.glob(
                os.path.join(
                    os.environ.get("ISAAC_PATH", isaacsim_path), "exts*", "isaacsim.simulation_app*", "isaacsim"
                )
            )
            if len(path) and os.path.exists(path[0]):
                # register path
                sys.path.insert(0, path[0])
                # import API
                from simulation_app import AppFramework, SimulationApp

                # register module to support 'from isaacsim.simulation_app import SimulationApp'
                sys.modules["isaacsim.simulation_app"] = type(sys)("isaacsim.simulation_app")
                sys.modules["isaacsim.simulation_app.SimulationApp"] = SimulationApp
                sys.modules["isaacsim.simulation_app.AppFramework"] = AppFramework
            else:
                print(
                    "[Warning] Unable to expose 'isaacsim.simulation_app' API: Extension not found. "
                    "Please install 'isaacsim[all,extscache]' to install the full Isaac Sim bundle"
                )
        except ImportError as e:
            print(f"[Warning] Unable to expose 'isaacsim.simulation_app' API: {e}")
    return AppFramework, SimulationApp


def main():
    args = sys.argv[1:]
    using_inner_kernel = False

    # get paths
    # isaac-sim path
    isaacsim_path = os.path.dirname(os.path.abspath(os.path.realpath(__file__)))
    # kit path (internal kernel)
    if os.path.isdir(os.path.join(isaacsim_path, "kit", "extscore")):
        kit_path = os.path.join(isaacsim_path, "kit")
        using_inner_kernel = True
    # kit path (omniverse-kit kernel package)
    else:
        try:
            import omni.kit_app  # importing 'omni.kit_app' will bootstrap kernel

            kit_path = os.path.dirname(os.path.abspath(os.path.realpath(omni.kit_app.__file__)))
        except ModuleNotFoundError:
            sys.exit("Unable to find 'omniverse-kit' package")

    # experience file
    experience = args[0] if len(args) and not args[0].startswith("-") else "isaacsim.exp.full"
    experience = experience if experience.endswith(".kit") else f"{experience}.kit"
    if not os.path.isfile(experience):
        for experience_dir in [os.path.join(isaacsim_path, "apps"), os.path.join(kit_path, "apps")]:
            if os.path.isfile(os.path.join(experience_dir, experience)):
                experience = os.path.join(experience_dir, experience)
                if len(args) and not args[0].startswith("-"):
                    args = args[1:]
                break
    if not os.path.isfile(experience):
        sys.exit(f"Unable to find experience (.kit) file: '{args[0] if len(args) else experience}'")

    # launch app
    if using_inner_kernel:
        if kit_path not in sys.path:
            sys.path.append(kit_path)
        from kit_app import KitApp
    else:
        from omni.kit_app import KitApp

    app = KitApp()
    app.startup([experience, "--ext-folder", os.path.join(isaacsim_path, "apps")] + args)
    while app.is_running():
        app.update()
    sys.exit(app.shutdown())


"""
Setup Isaac Sim launch.
"""

bootstrap_kernel()

# make isaacsim.simulation_app discoverable
AppFramework, SimulationApp = expose_api()


# register custom exception handler
def exception_handler(exc_type, exc_value, exc_traceback):
    ret = _excepthook(exc_type, exc_value, exc_traceback)
    if issubclass(exc_type, (ImportError, ModuleNotFoundError)):
        if not hasattr(builtins, "ISAACSIM_APP_LAUNCHED"):
            print("""
========================================================================
WARNING: Omniverse/Isaac Sim import statements must take place after the
`SimulationApp` class has been instantiated. It is a requirement of the
Carbonite framework's extension/runtime plugin system.
========================================================================

Ensure that the `SimulationApp` class is instantiated before importing
any other Omniverse/Isaac Sim modules, as shown below:

    ------------------------------------------------------------------
    from isaacsim import SimulationApp

    # instantiate the SimulationApp helper class
    simulation_app = SimulationApp({"headless": False})

    # execute other Omniverse/Isaac Sim imports after instantiating it
    from isaacsim...
    ------------------------------------------------------------------

========================================================================
""")
    return ret


_excepthook = sys.excepthook
sys.excepthook = exception_handler
