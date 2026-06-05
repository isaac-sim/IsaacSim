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

"""Provides the SimulationApp class for launching and managing an Omniverse Toolkit application instance."""

from __future__ import annotations  # This allows us to hint types that do not yet exist like omni.usd etc

import argparse
import asyncio
import atexit
import builtins
import faulthandler
import os
import re
import signal
import sys
import time
from typing import Any

import carb
import omni.kit.app


class SimulationApp:
    """Helper class to launch Omniverse Toolkit.

    Omniverse loads various plugins at runtime which cannot be imported unless
    the Toolkit is already running. Thus, it is necessary to launch the Toolkit first from
    your python application and then import everything else.

    Launches the Omniverse Toolkit with the provided configuration settings.
    The settings in DEFAULT_LAUNCHER_CONFIG are overwritten by those in launch_config.

    Args:
        launch_config: A dictionary containing the configuration for the app.
            See DEFAULT_LAUNCHER_CONFIG for available options.
        experience: Path to the application config loaded by the launcher.
            If not specified, the launcher will load one of the following experience files in order
            (where ``$EXP_PATH`` points to the ``apps`` folder in a default Isaac Sim setup):

            - ``$EXP_PATH/omni.isaac.sim.python.kit``
            - ``$EXP_PATH/isaacsim.exp.base.python.kit``
            - ``$EXP_PATH/isaacsim.exp.base.kit``

    Usage:

    .. code-block:: python

        # At top of your application
        from isaacsim.simulation_app import SimulationApp
        config = {
             width: "1280",
             height: "720",
             headless: False,
        }
        simulation_app = SimulationApp(config)

        # Rest of the code follows
        ...
        simulation_app.close()

    Note:
            The settings in :obj:`DEFAULT_LAUNCHER_CONFIG` are overwritten by those in :obj:`config`.

    Example:

    .. code-block:: python

        >>> from isaacsim.simulation_app import SimulationApp
        >>> config = {
        ...     "width": 1280,
        ...     "height": 720,
        ...     "headless": False,
        ... }
        >>> simulation_app = SimulationApp(config)
        >>> # Rest of the code follows
        >>> simulation_app.close()
    """

    DEFAULT_LAUNCHER_CONFIG = {
        "headless": True,
        "hide_ui": None,
        "active_gpu": None,
        "physics_gpu": 0,
        "multi_gpu": True,
        "max_gpu_count": None,
        "sync_loads": True,
        "width": 1280,
        "height": 720,
        "window_width": 1440,
        "window_height": 900,
        "display_options": 3094,
        "subdiv_refinement_level": 0,
        "renderer": "RealTimePathTracing",  # Can also be PathTracing, RaytracedLighting, or MinimalRendering
        "minimal_shading_mode": 0,
        "anti_aliasing": 3,
        "samples_per_pixel_per_frame": 64,
        "denoiser": True,
        "max_bounces": 3,
        "max_specular_transmission_bounces": 3,
        "max_volume_bounces": 15,
        "open_usd": None,
        "fast_shutdown": True,
        "profiler_backend": [],
        "create_new_stage": True,
        "extra_args": [],
        "enable_crashreporter": True,
        "limit_cpu_threads": 32,
        "disable_viewport_updates": False,
    }
    """Default configuration dictionary for launching the SimulationApp.

    Contains default values for rendering settings, window dimensions, physics configuration,
    and other application launch parameters. These settings are overwritten by values
    provided in the launch_config parameter during initialization."""

    RENDERER_DEFAULTS = {
        "pathtracing": {
            "max_bounces": 4,
            "max_specular_transmission_bounces": 6,
            "max_volume_bounces": 64,
        },
        "realtimepathtracing": {
            "max_bounces": 3,
            "max_specular_transmission_bounces": 3,
            "max_volume_bounces": 15,
        },
    }
    """
    The config variable is a dictionary containing the following entries

    Args:
        headless (bool): Disable window creation and UI when running. Defaults to True
        hide_ui (bool): Hide UI when running to improve performance, when headless is set to true, the UI is hidden, set to false to override this behavior when live streaming. Defaults to None
        active_gpu (int): Specify the GPU to use when running, set to None to use default value which is usually the first gpu, default is None
        physics_gpu (int): Specify the GPU to use when running physics simulation. Defaults to 0 (first GPU).
        multi_gpu (bool): Set to true to enable Multi GPU support, Defaults to true
        max_gpu_count (int): Maximum number of GPUs to use, Defaults to None which will use all available
        sync_loads (bool): When enabled, will pause rendering until all assets are loaded. Defaults to True
        width (int): Width of the viewport and generated images. Defaults to 1280
        height (int): Height of the viewport and generated images. Defaults to 720
        window_width (int): Width of the application window, independent of viewport, defaults to 1440,
        window_height (int): Height of the application window, independent of viewport, defaults to 900,
        display_options (int): used to specify whats visible in the stage by default. Defaults to 3094 so extra objects do not appear in synthetic data. 3286 is another good default, used for the regular isaac-sim editor experience
        subdiv_refinement_level (int): Number of subdivisons to perform on supported geometry. Defaults to 0
        renderer (str): Rendering mode, can be `RaytracedLighting`, `PathTracing`, `RealTimePathTracing`, or `MinimalRendering` (also accepts `Minimal`). Defaults to `RealTimePathTracing`
        minimal_shading_mode (int): Minimal shading mode for `MinimalRendering`, maps to `/rtx/minimal/mode`. 0: Real-Time 2.0 (reference), 1: Diffuse/Glossy/Emission, 2: Textured Diffuse, 3: Constant Diffuse, 4: No Rendering. Defaults to 0
        anti_aliasing (int): Antialiasing mode, 0: Disabled, 1: TAA, 2: FXAA, 3: DLSS, 4:RTXAA
        samples_per_pixel_per_frame (int): The number of samples to render per frame, increase for improved quality, used for `PathTracing` only. Defaults to 64
        denoiser (bool):  Enable this to use AI denoising to improve image quality, used for `PathTracing` only. Defaults to True
        max_bounces (int): Maximum number of bounces, used for `PathTracing` (defaults to 4)and `RealTimePathTracing` (defaults to 3)
        max_specular_transmission_bounces (int): Maximum number of bounces for specular or transmission, used for `PathTracing` (defaults to 6) or `RealTimePathTracing` (defaults to 3)
        max_volume_bounces (int): Maximum number of bounces for volumetric materials, used for `PathTracing` (defaults to 64) and 'RealTimePathTracing' (defaults to 15).
        open_usd (str): This is the name of the usd to open when the app starts. It will not be saved over. Default is None and an empty stage is created on startup.
        fast_shutdown (bool): True to exit process immediately, false to shutdown each extension. If running in a jupyter notebook this is forced to false.
        profiler_backend (list): List of profiler backends to enable currently only supports the following backends: ["tracy", "nvtx"]
        create_new_stage (bool): Set False to not create a new stage on application startup. Defaults to True, does not have an effect if open_usd is not None.
        extra_args: (list): List of extra command line arguments to pass down to the kit process
        enable_crashreporter (bool): Enable crash reporter. Defaults to True
        limit_cpu_threads (int): Limit the number of CPU threads created to the lesser of cpu core count or specified value. Defaults to 32.
        disable_viewport_updates (bool): Disable viewport updates to improve performance. Defaults to False.
    """

    def __init__(self, launch_config: dict = None, experience: str = "") -> None:
        # Enable callstack on crash
        faulthandler.enable()
        # Sanity check to see if any extra omniverse modules are loaded
        # Warn users if so because this will usually cause issues.
        # Base list of modules that can be loaded before kit app starts, might need to be updated in the future
        ok_list = [
            "omni",
            "omni.isaac",
            "omni.kit",
            "omni.ext._extensions",
            "omni.ext._impl.utils",
            "omni.ext._impl.fast_importer",
            "omni.ext._impl.ext_settings",
            "omni.ext._impl.custom_importer",
            "omni.ext._impl.leak_detection",
            "omni.ext._impl.stat_cache",
            "omni.ext._impl._internal",
            "omni.ext._impl",
            "omni.ext",
            "omni.kit.app._app",
            "omni.kit.app._impl.app_iface",
            "omni.kit.app._impl.telemetry_helpers",
            "omni.kit.app._impl",
            "omni.kit.app",
            "omni.isaac.kit.app_framework",
            "omni.isaac.kit.simulation_app",
            "omni.isaac.kit",
            "isaacsim.simulation_app.app_framework",
            "isaacsim.simulation_app.simulation_app",
            "isaacsim.simulation_app",
            "omni.isaac.gym",
            "omniisaacgymenvs",
            "omniisaacgymenvs.utils",
            "omniisaacgymenvs.utils.hydra_cfg",
            "omniisaacgymenvs.utils.hydra_cfg.hydra_utils",
            "omniisaacgymenvs.utils.hydra_cfg.reformat",
            "omniisaacgymenvs.utils.rlgames",
            "omniisaacgymenvs.utils.rlgames.rlgames_utils",
            "omniisaacgymenvs.utils.task_util",
            "omniisaacgymenvs.utils.config_utils",
            "omniisaacgymenvs.utils.config_utils.path_utils",
            "omniisaacgymenvs.envs",
            "omniisaacgymenvs.envs.vec_env_rlgames",
            "omni.isaac.gym.vec_env",
            "omni.isaac.gym.vec_env.vec_env_base",
            "omni.isaac.gym.vec_env.vec_env_mt",
        ]
        r = re.compile("omni.*|pxr.*")
        found_modules = list(filter(r.match, list(sys.modules.keys())))
        result = []
        for item in found_modules:
            if item not in ok_list:
                result.append(item)
        # Made this a warning instead of an error as the above list might be incomplete
        if len(result) > 0:
            carb.log_warn(
                f"Modules: {result} were loaded before SimulationApp was started and might not be loaded correctly."
            )
            carb.log_warn(
                "Please check to make sure no extra omniverse or pxr modules are imported before the call to SimulationApp(...)"
            )
        else:
            carb.log_info("SimulationApp.__init__: Module validation passed - no problematic modules found")

        # Initialize variables
        builtins.ISAAC_LAUNCHED_FROM_TERMINAL = False
        self._exiting = False

        # Override settings from input config
        self.config = self.DEFAULT_LAUNCHER_CONFIG
        if experience == "":
            for exp in [
                f'{os.environ["EXP_PATH"]}/omni.isaac.sim.python.kit',
                f'{os.environ["EXP_PATH"]}/isaacsim.exp.base.python.kit',
                f'{os.environ["EXP_PATH"]}/isaacsim.exp.base.kit',
            ]:
                if os.path.isfile(exp):
                    experience = exp
                    break
        # Set or remove experience based on final value
        if experience and experience != "":
            self.config.update({"experience": experience})
        else:
            # Remove experience key if None or empty string (and no default found)
            self.config.pop("experience", None)

        if launch_config is not None:
            self.config.update(launch_config)
        self._apply_renderer_defaults(launch_config)
        if builtins.ISAAC_LAUNCHED_FROM_JUPYTER:
            if self.config["headless"] is False:
                carb.log_warn("Non-headless mode not supported with jupyter notebooks")
            if self.config["fast_shutdown"] is True:
                carb.log_warn("fast shutdown not supported with jupyter notebooks")
                self.config["fast_shutdown"] = False
                # self.config.update({"headless": True}) # Disable this, in case the user really wants to run non-headless

        # Load omniverse application plugins
        self._framework = carb.get_framework()
        wildcards = ["omni.kit.app.plugin"]
        if self.config["enable_crashreporter"]:
            wildcards.append("carb.crashreporter-*")

        self._framework.load_plugins(
            loaded_file_wildcards=wildcards,
            search_paths=[os.path.abspath(f'{os.environ["CARB_APP_PATH"]}/kernel/plugins')],
        )
        carb.log_info("SimulationApp.__init__: Loaded framework plugins")
        # Get Omniverse application
        self._app = omni.kit.app.get_app()
        self._start_app()

        # Register signal handler to exit when ctrl-c happens
        # This needs to happen after the app starts so that we can overide the default handler
        def signal_handler(signal: int, frame: Any) -> None:
            # Disable logging as we are forcefully exiting
            _logging = carb.logging.acquire_logging()
            _logging.set_log_enabled(False)
            self._app.shutdown()
            self._framework.unload_all_plugins()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

        # once app starts, we can set settings
        from .utils import create_new_stage, open_stage

        self._carb_settings = carb.settings.get_settings()
        # apply render settings specified in config
        self.reset_render_settings()

        self._app.print_and_log("Simulation App Starting")

        self._update_without_ready()

        self.open_usd = self.config.get("open_usd")
        if self.open_usd is not None:
            print("Opening usd file at ", self.open_usd, " ...", end="")
            if open_stage(self.open_usd) is False:
                print("Could not open", self.open_usd, "creating a new empty stage")
                create_new_stage()
            else:
                print("Done.")
        elif self.config["create_new_stage"] is True:
            carb.log_info("SimulationApp.__init__: Creating new stage")
            create_new_stage()
        # Update the app
        self._update_without_ready()
        self._prepare_ui()

        # Increase hang detection timeout
        omni.client.set_hang_detection_time_ms(10000)

        # Set the window title to something simpler
        try:
            from isaacsim.core.version import get_version
            from omni.kit.window.title import get_main_window_title

            window_title = get_main_window_title()
            app_version_core, _, _, _, _, _, _, _ = get_version()
            window_title.set_app_version(app_version_core)
        except Exception:
            pass

        self._wait_for_viewport()

        if self.config["disable_viewport_updates"] and self.config["headless"]:
            try:
                from omni.kit.viewport.utility import get_active_viewport

                viewport = get_active_viewport()
                if viewport:
                    viewport.updates_enabled = False
                    self.app.print_and_log("Viewport updates disabled")
                else:
                    self.app.print_and_log("Unable to disable viewport updates, no viewports found")
            except Exception as e:
                self.app.print_and_log(f"Error disabling default viewport: {e}")

        # Notify toolkit is running
        self._app.print_and_log("Simulation App Startup Complete")

        # Record startup time as time at which app is ready for use
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        if ext_manager.is_extension_enabled("isaacsim.benchmark.services"):
            from isaacsim.benchmark.services import BaseIsaacBenchmark

            benchmark = BaseIsaacBenchmark(
                benchmark_name="app_startup",
                workflow_metadata={
                    "metadata": [
                        {"name": "mode", "data": "non-async"},
                    ]
                },
            )
            benchmark.set_phase("startup", start_recording_frametime=False, start_recording_runtime=False)
            benchmark.store_measurements()
            benchmark.stop()

        self.update()  # This app update triggers app ready status.
        builtins.ISAACSIM_APP_LAUNCHED = True

        atexit.register(self._atexit_close)

    def _apply_renderer_defaults(self, launch_config: dict | None) -> None:
        """Apply renderer-specific defaults when values are not provided in the launch config.

        Args:
            launch_config: User-provided configuration overrides.
        """
        renderer_key = str(self.config.get("renderer", "")).lower()
        renderer_defaults = self.RENDERER_DEFAULTS.get(renderer_key)
        if not renderer_defaults:
            return
        override_keys = launch_config or {}
        for setting_key, default_value in renderer_defaults.items():
            if setting_key not in override_keys:
                self.config[setting_key] = default_value

    def __del__(self) -> None:
        """Destructor for the SimulationApp class.

        Warns if the application is still running when Python exits,
        indicating that close() was not called properly.
        """
        if self._exiting is False and sys.meta_path is None:
            print(
                "\033[91m"
                + "WARNING: Python exiting while SimulationApp was still running without an explicit call to close()"
                + "\033[0m"
            )

    def _atexit_close(self) -> None:
        """Automatically close the application during interpreter shutdown if close() was not called."""
        if not self._exiting:
            carb.log_warn("SimulationApp.close() was not called explicitly. Shutting down automatically")
            self.close(wait_for_replicator=False)

    ### Private methods

    def _start_app(self) -> None:
        """Launch the Omniverse application."""
        carb.log_info("SimulationApp._start_app: Starting app launch process")
        exe_path = os.path.abspath(f'{os.environ["CARB_APP_PATH"]}')
        carb.log_info(f"SimulationApp._start_app: Using executable path: {exe_path}")
        # input arguments to the application
        args = [os.path.abspath(__file__)]
        if "experience" in self.config:
            args.append(f'{self.config["experience"]}')
        args.extend(
            [
                f"--/app/tokens/exe-path={exe_path}",  # this is needed so dlss lib is found
                f'--/persistent/app/viewport/displayOptions={self.config["display_options"]}',  # hide extra stuff in viewport
                # Forces kit to not render until all USD files are loaded
                f'--/rtx/materialDb/syncLoads={self.config["sync_loads"]}',
                f'--/rtx/hydra/materialSyncLoads={self.config["sync_loads"]}',
                f'--/omni.kit.plugin/syncUsdLoads={self.config["sync_loads"]}',
                f'--/app/renderer/resolution/width={self.config["width"]}',
                f'--/app/renderer/resolution/height={self.config["height"]}',
                f'--/app/window/width={self.config["window_width"]}',
                f'--/app/window/height={self.config["window_height"]}',
                f'--/renderer/multiGpu/enabled={self.config["multi_gpu"]}',
                f'--/app/fastShutdown={self.config["fast_shutdown"]}',
                "--/app/installSignalHandlers=0",
                "--ext-folder",
                f'{os.path.abspath(os.environ["ISAAC_PATH"])}/exts',  # adding to json doesn't work
                "--ext-folder",
                f'{os.path.abspath(os.environ["ISAAC_PATH"])}/apps',  # so we can reference other kit files
            ]
        )
        # Add additional command line arguments
        extra_args = self.config.get("extra_args", [])
        if isinstance(extra_args, list):
            args.extend(self.config.get("extra_args", []))
        else:
            print("Ignoring extra_args, extra_args must be of type list")
        if self.config["create_new_stage"] is False:
            args.append("--/app/content/emptyStageOnStart=false")
        if self.config.get("active_gpu") is not None:
            args.append(f'--/renderer/activeGpu={self.config["active_gpu"]}')
        if self.config.get("physics_gpu") is not None:
            args.append(f'--/physics/cudaDevice={self.config["physics_gpu"]}')
        if self.config.get("max_gpu_count") is not None:
            args.append(f'--/renderer/multiGpu/maxGpuCount={self.config["max_gpu_count"]}')
        if self.config.get("enable_motion_bvh"):
            args.append("--/renderer/raytracingMotion/enabled=true")
            args.append("--/renderer/raytracingMotion/enableHydraEngineMasking=true")
            args.append("--/renderer/raytracingMotion/enabledForHydraEngines='0'")

        # limit the number of CPU threads created to lesser of: num_cpu_cores or config-set limit
        num_cpu_cores = os.cpu_count()
        num_threads = min(num_cpu_cores, self.config.get("limit_cpu_threads"))
        # set env variables to limit threads
        os.environ["PXR_WORK_THREAD_LIMIT"] = str(num_threads)
        os.environ["OPENBLAS_NUM_THREADS"] = str(num_threads)
        # pass to kit args
        args.append(f"--/plugins/carb.tasking.plugin/threadCount={num_threads}")
        args.append(f"--/plugins/omni.tbb.globalcontrol/maxThreadCount={num_threads}")

        # parse any extra command line args here
        # user script should provide its own help, otherwise we default to printing the kit app help output
        carb.log_info("SimulationApp._start_app: Parsing command line arguments")
        parser = argparse.ArgumentParser(add_help=False)
        _, unknown_args = parser.parse_known_args()
        carb.log_info(f"SimulationApp._start_app: Unknown args: {unknown_args}")
        # is user did not request portable root,
        # we still run apps as portable to prevent them writing extra files to user directory
        if "--portable-root" not in unknown_args:
            args.append("--portable")

        # Check for DISPLAY environment variable on Linux
        display_not_available = sys.platform.startswith("linux") and os.environ.get("DISPLAY") is None
        headless_mode = self.config.get("headless")

        if "--no-window" not in unknown_args and (headless_mode or display_not_available):
            args.append("--no-window")
            if display_not_available and not headless_mode:
                carb.log_warn(
                    "DISPLAY environment variable is not set, running in headless mode with --no-window flag. "
                    "Set DISPLAY environment variable if you want to run in non-headless mode."
                )

        # if the user forces hideUi via commandline, use that setting
        if "--/app/window/hideUi" not in unknown_args:
            # Hide the ui by default if headless
            # Else: If the user specified a value for hide_ui, override with that value
            hide_ui = self.config.get("hide_ui")
            if hide_ui is None:
                if "--no-window" in args or "--no-window" in unknown_args:
                    args.append("--/app/window/hideUi=1")
            else:
                args.append(f"--/app/window/hideUi={hide_ui}")

        # get the effective uid of this process, if its root, then we automatically add the allow root flag
        # if the flag is already in unknown_args, we don't need to add it again.
        if sys.platform.startswith("linux") and os.geteuid() == 0 and "--allow-root" not in unknown_args:
            args.append("--allow-root")

        # Add args to enable profiling
        profiler_backends = self.config.get("profiler_backend")
        # Common args
        if "tracy" in profiler_backends or "nvtx" in profiler_backends:
            args += [
                "--/app/profileFromStart=true",
                "--/profiler/enabled=true",
            ]
        # Args needed if tracy is enabled
        if "tracy" in profiler_backends:
            args += [
                "--/rtx/addTileGpuAnnotations=true",
                "--/profiler/gpu/tracyInject/enabled=true",
                "--/profiler/gpu/tracyInject/msBetweenClockCalibration=0",
                "--/app/profilerMask=1",
                "--/plugins/carb.profiler-tracy.plugin/fibersAsThreads=false",
                "--/profiler/channels/carb.events/enabled=false",
                "--/profiler/channels/carb.tasking/enabled=false",
                "--/profiler/gpu=true",
            ]
        # Enable the supported backend
        if "tracy" in profiler_backends and "nvtx" not in profiler_backends:
            args += [
                "--/app/profilerBackend=tracy",
            ]
        elif "tracy" not in profiler_backends and "nvtx" in profiler_backends:
            args += [
                "--/app/profilerBackend=nvtx",
            ]
        elif "tracy" in profiler_backends and "nvtx" in profiler_backends:
            args += [
                "--/app/profilerBackend=[tracy,nvtx]",
            ]

        # look for --ovd="directory" and replace with the proper settings
        index = None
        for i, s in enumerate(unknown_args):
            if s.startswith("--ovd"):
                index = i
                break
        if index is not None:
            # remove --opvd from the incoming arguments and replace with the following expanded settings
            pvdString = unknown_args.pop(index)
            # parse out the directory string, so find the first =
            try:
                indexEqual = pvdString.index("=")
            except ValueError:
                carb.log_error('Malformed --ovd argument. Expected: --ovd="/path/to/capture/"')
            else:
                pvdDirArg = "--/persistent/physics/omniPvdOvdRecordingDirectory" + pvdString[indexEqual:]
                pvdEnabled = "--/physics/omniPvdOutputEnabled=true"
                print("Passing the OmniPVD arguments:", pvdDirArg, pvdEnabled)
                args.append(pvdDirArg)
                args.append(pvdEnabled)

        # pass all extra arguments onto the main kit app
        print("Starting kit application with the following args: ", args)
        print("Passing the following args to the base kit application: ", unknown_args)
        args.extend(unknown_args)
        self.app.startup("kit", os.environ["CARB_APP_PATH"], args)
        carb.log_info("SimulationApp._start_app: Kit application startup completed")

        # Check if the app actually started successfully (e.g. dependency solver may have failed)
        if not self._app.is_running():
            carb.log_error("SimulationApp._start_app: Application failed to start. " "Check the log above for errors.")
            sys.exit(1)

        # if user called with -h kit auto exits so we force exit the script here as well
        if "-h" in unknown_args or "--help" in unknown_args:
            carb.log_info("SimulationApp._start_app: Help requested, exiting")
            self.close(skip_cleanup=True)

    def _set_render_settings(self, default: bool = False) -> None:
        """Set render settings to those in config.

        Note:
            This should be used in case a new stage is opened and the desired config needs
            to be re-applied.

        Args:
            default: Whether to setup RTX default or non-default settings.
        """
        from .utils import set_carb_setting

        # Define mode to configure settings into.
        if default:
            rtx_mode = "/rtx-defaults"
        else:
            rtx_mode = "/rtx"

        # Set renderer mode, handle case where user may have entered incorrect case
        renderer_lower = self.config["renderer"].lower()
        if renderer_lower == "raytracedlighting":
            render_mode = "RaytracedLighting"
        elif renderer_lower == "pathtracing":
            render_mode = "PathTracing"
        elif renderer_lower == "realtimepathtracing":
            render_mode = "RealTimePathTracing"
        elif renderer_lower in ("minimal", "minimalrendering"):
            render_mode = "MinimalRendering"
        else:
            render_mode = self.config["renderer"]
        set_carb_setting(self._carb_settings, rtx_mode + "/rendermode", render_mode)
        if render_mode == "MinimalRendering":
            set_carb_setting(self._carb_settings, rtx_mode + "/minimal/mode", self.config["minimal_shading_mode"])
        # Raytrace mode settings
        set_carb_setting(self._carb_settings, rtx_mode + "/post/aa/op", self.config["anti_aliasing"])

        # Realtime Path Tracing mode settings
        set_carb_setting(self._carb_settings, rtx_mode + "/rtpt/maxBounces", self.config["max_bounces"])
        set_carb_setting(
            self._carb_settings,
            rtx_mode + "/rtpt/maxSpecularAndTransmissionBounces",
            self.config["max_specular_transmission_bounces"],
        )
        set_carb_setting(self._carb_settings, rtx_mode + "/rtpt/maxVolumeBounces", self.config["max_volume_bounces"])

        # Pathtrace mode settings
        set_carb_setting(self._carb_settings, rtx_mode + "/pathtracing/spp", self.config["samples_per_pixel_per_frame"])
        set_carb_setting(
            self._carb_settings, rtx_mode + "/pathtracing/totalSpp", self.config["samples_per_pixel_per_frame"]
        )
        set_carb_setting(
            self._carb_settings, rtx_mode + "/pathtracing/clampSpp", self.config["samples_per_pixel_per_frame"]
        )
        set_carb_setting(self._carb_settings, rtx_mode + "/pathtracing/maxBounces", self.config["max_bounces"])
        set_carb_setting(
            self._carb_settings,
            rtx_mode + "/pathtracing/maxSpecularAndTransmissionBounces",
            self.config["max_specular_transmission_bounces"],
        )
        set_carb_setting(
            self._carb_settings, rtx_mode + "/pathtracing/maxVolumeBounces", self.config["max_volume_bounces"]
        )
        set_carb_setting(self._carb_settings, rtx_mode + "/pathtracing/optixDenoiser/enabled", self.config["denoiser"])
        set_carb_setting(
            self._carb_settings, rtx_mode + "/hydra/subdivision/refinementLevel", self.config["subdiv_refinement_level"]
        )

        # Experimental, forces kit to not render until all USD files are loaded
        set_carb_setting(self._carb_settings, rtx_mode + "/materialDb/syncLoads", self.config["sync_loads"])
        set_carb_setting(self._carb_settings, rtx_mode + "/hydra/materialSyncLoads", self.config["sync_loads"])
        set_carb_setting(self._carb_settings, "/omni.kit.plugin/syncUsdLoads", self.config["sync_loads"])
        carb.log_info("SimulationApp._set_render_settings: Completed render settings configuration")

    def _prepare_ui(self) -> None:
        """Dock the windows in the UI if they exist."""
        carb.log_info("SimulationApp._prepare_ui: Starting UI setup")
        try:
            import omni.ui

            self._update_without_ready()
            content = omni.ui.Workspace.get_window("Content")
            console = omni.ui.Workspace.get_window("Console")
            samples = omni.ui.Workspace.get_window("Samples")

            if content:
                content.dock_order = 0
                content.focus()
            if console:
                console.dock_order = 1
            if samples:
                samples.visible = False
        except Exception:
            pass

        self._update_without_ready()
        carb.log_info("SimulationApp._prepare_ui: UI setup completed")

    def _update_without_ready(self) -> None:
        """Update the application without waiting for the app to be ready.

        This is a convenience function that updates the application without waiting for the app to be ready.
        """
        app = omni.kit.app.get_app()
        if not app.is_app_ready():
            app.delay_app_ready("IsaacSim")
        self._app.update()

    def _wait_for_viewport(self) -> None:
        """Wait for the viewport to become available and properly initialized."""
        MAX_FRAMES = 240 if os.name == "nt" else 120
        DOCKING_FRAMES = 10
        frame_count = 0
        carb.log_info("SimulationApp._wait_for_viewport: Starting viewport wait")
        try:
            from omni.kit.viewport.utility import get_active_viewport

            # avoid infinite loop if no new stage was created
            if self.config["create_new_stage"] is False:
                raise Exception("create_new_stage is False")

            # Get every ViewportWindow, regardless of UsdContext it is attached to
            viewport_api = get_active_viewport()

            while viewport_api.frame_info.get("viewport_handle", None) is None and frame_count < MAX_FRAMES:
                carb.log_info(f"SimulationApp._wait_for_viewport: Waiting for viewport... frame {frame_count}")
                self._update_without_ready()
                # Sleep to reduce the overall number of frames it takes for the renderer to render its first frame
                time.sleep(0.02)
                frame_count += 1
            if frame_count == MAX_FRAMES:
                raise Exception("Timeout waiting for viewport")
        except Exception as e:
            carb.log_info(f"SimulationApp._wait_for_viewport: Exception during viewport wait: {e}")

        # once we load, we need a few frames so everything docks itself
        if frame_count < DOCKING_FRAMES:
            carb.log_info("SimulationApp._wait_for_viewport: Running final update frames for docking")
            for _ in range(DOCKING_FRAMES):
                self._update_without_ready()
        carb.log_info("SimulationApp._wait_for_viewport: Viewport wait completed")

    ### Public methods

    def update(self) -> None:
        """Step the application forward by one frame.

        This is a convenience function that advances the simulation by a single frame,
        updating all systems and rendering.

        Example:

        .. code-block:: python

            >>> simulation_app = SimulationApp()
            >>> # Run simulation for 10 frames
            >>> for _ in range(10):
            ...     simulation_app.update()
            >>> simulation_app.close()
        """
        self._app.update()

    def set_setting(self, setting: str, value: Any) -> None:
        """Set a Carbonite framework setting.

        Sets a configuration value in the Carbonite settings system.
        The value type is automatically detected and used for proper setting assignment.

        Args:
            setting: The Carbonite setting path (e.g., "/app/window/width").
            value: The value to set. Type is automatically detected.

        Example:

        .. code-block:: python

            >>> simulation_app = SimulationApp()
            >>> simulation_app.set_setting("/app/window/width", 1920)
            >>> simulation_app.set_setting("/rtx/rendermode", "PathTracing")
            >>> simulation_app.close()
        """
        from .utils import set_carb_setting

        set_carb_setting(self._carb_settings, setting, value)

    def reset_render_settings(self) -> None:
        """Reset render settings to those specified in the launch configuration.

        Re-applies the rendering settings from the initial configuration.
        This should be used when a new stage is opened and the desired
        render configuration needs to be re-applied.

        Example:

        .. code-block:: python

            >>> config = {"renderer": "PathTracing", "samples_per_pixel_per_frame": 128}
            >>> simulation_app = SimulationApp(config)
            >>> # Open a new stage
            >>> simulation_app.reset_render_settings()  # Re-apply config settings
            >>> simulation_app.close()
        """
        # Set rtx settings renderer settings
        self._set_render_settings(default=False)

    def run_coroutine(
        self, coroutine: asyncio.Coroutine, run_until_complete: bool = True
    ) -> asyncio.Task | asyncio.Future | Any:
        """Run a coroutine using Kit's asynchronous task engine.

        Executes the provided coroutine within Kit's event loop system.
        The Kit's asynchronous task engine runs the event loop every IApp update.

        Args:
            coroutine: The coroutine to execute.
            run_until_complete: Whether to block and wait for coroutine completion.

        Returns:
            If run_until_complete is True, returns the coroutine result.
            Otherwise returns an asyncio.Task (from main thread) or asyncio.Future
            (from other threads).

        Raises:
            Exception: Any exception raised by the coroutine.

        Example:

        .. code-block:: python

            >>> import asyncio
            >>> async def my_async_task():
            ...     await asyncio.sleep(0.1)
            ...     return "completed"
            >>>
            >>> simulation_app = SimulationApp()
            >>> result = simulation_app.run_coroutine(my_async_task())
            >>> print(result)
            completed
            >>> simulation_app.close()
        """
        task_or_future = omni.kit.async_engine.run_coroutine(coroutine)
        if run_until_complete:
            # Since Kit's asynchronous task engine runs the event loop every IApp update,
            # it is necessary to call `app.update()` while waiting for the coroutine to complete.
            while not task_or_future.done():
                self._app.update()
            # Even if the task/future does not return a value,
            # `.result()` must be called to re-raise any exception raised by the coroutine.
            return task_or_future.result()
        return task_or_future

    @staticmethod
    def _flush_stdio() -> None:
        """Flush Python's stdout and stderr, swallowing errors from closed/detached streams."""
        for stream in (sys.stdout, sys.stderr):
            try:
                stream.flush()
            except (ValueError, OSError, AttributeError):
                pass

    def close(self, wait_for_replicator: bool = True, skip_cleanup: bool = False, exit_code: int = 0) -> None:
        """Close the running Omniverse Toolkit application.

        Performs cleanup and shuts down the application. Can either perform
        a graceful shutdown with full cleanup or an immediate exit.

        Args:
            wait_for_replicator: Whether to wait for replicator workflows to complete
                before shutdown.
            skip_cleanup: If True, performs immediate exit without cleanup.
                If False, performs graceful shutdown with full cleanup.
            exit_code: Process exit status to preserve when fast shutdown terminates
                the process. Nonzero values flush stdio and exit with the supplied
                status before Kit's fast-shutdown path can replace it with 0.

        Example:

        .. code-block:: python

            >>> simulation_app = SimulationApp()
            >>> # Do simulation work...
            >>> simulation_app.close(wait_for_replicator=True)
            >>>
            >>> # For immediate exit without cleanup:
            >>> simulation_app = SimulationApp()
            >>> simulation_app.close(skip_cleanup=True)
        """
        carb.log_info("SimulationApp.close: Closing application")
        if self._exiting:
            carb.log_info("SimulationApp.close: already exiting, skipping duplicate close call")
            return

        # Flush Python stdio before any shutdown path that may terminate the process via _exit().
        # When stdout/stderr are piped (e.g. through `tee` in test runners), CPython uses block
        # buffering; the fast-shutdown path calls quickReleaseFrameworkAndTerminate which bypasses
        # the interpreter's normal flush-on-exit, so pending print() output would otherwise be lost.
        self._flush_stdio()

        if exit_code != 0 and self.config.get("fast_shutdown", False):
            self._exiting = True
            os._exit(exit_code)

        # `post_quit()` can already stop Kit's run loop before callers reach `close()`.
        # In that state, forcing shutdown may block indefinitely.
        if not self._app.is_running():
            self._exiting = True
            carb.log_info("SimulationApp.close: app already stopped, skipping framework shutdown")
            if self.config.get("fast_shutdown", False):
                self._app.print_and_log("Simulation App Shutting Down")
                self._flush_stdio()
                os._exit(exit_code)
            return

        if skip_cleanup:
            self._exiting = True
            carb.log_info("SimulationApp.close: immediate_exit")
            _logging = carb.logging.acquire_logging()
            _logging.set_log_enabled(False)
            self._flush_stdio()
            self._app.shutdown()
            return
        try:
            # Ensure replicator workflows complete any queued writes.
            import omni.replicator.core as rep

            # Stop the workflow if it is still running.
            if rep.orchestrator.get_status() not in {rep.orchestrator.Status.STOPPED, rep.orchestrator.Status.STOPPING}:
                rep.orchestrator.stop()

            # Always wait when requested; queued writes can remain after stop/step.
            if wait_for_replicator:
                rep.orchestrator.wait_until_complete()
                time.sleep(0.05)

            # Disable capture on play to avoid replicator engaging on any new timeline events
            rep.orchestrator.set_capture_on_play(False)
        except Exception:
            pass
        carb.log_info("SimulationApp.close: Replicator shutdown finished")
        # Prevents issues when exiting
        if self.context.can_close_stage():
            self.context.close_stage()
            carb.log_info("SimulationApp.close: Stage closed")
        else:
            carb.log_info("SimulationApp.close: Stage could not be closed")
        # check if exited already
        self._exiting = True
        self._app.print_and_log("Simulation App Shutting Down")

        # Cleanup any running tracy instances so data is not lost
        try:
            _profiler_tracy = carb.profiler.acquire_profiler_interface(plugin_name="carb.profiler-tracy.plugin")
            if _profiler_tracy:
                _profiler_tracy.set_capture_mask(0)
                _profiler_tracy.end(0)
                _profiler_tracy.shutdown()
        except RuntimeError:
            pass

        carb.log_info("SimulationApp.close: shutting down app")
        _logging = carb.logging.acquire_logging()
        _logging.set_log_enabled(False)

        # Use app.shutdown() instead of shutdown_and_release_framework() to avoid a
        # GIL deadlock (NVBug 5948099): the Python binding for
        # shutdown_and_release_framework holds the GIL while
        # releaseFrameworkAndShutdown() joins carb.tasking worker threads that may
        # themselves be blocked waiting for the GIL.
        #
        # app.shutdown() handles /app/fastShutdown internally — when true (the
        # default) it calls quickReleaseFrameworkAndTerminate which exits the process
        # immediately.  When false it performs full extension teardown and returns;
        # the framework/plugin unload is left to process exit.
        self._flush_stdio()
        self._app.shutdown()

    def is_running(self) -> bool:
        """Check if the simulation application is currently running.

        Returns True if the application is running and not exiting.

        This reflects the lifetime of the underlying Kit application and does
        not require an active USD stage.

        Returns:
            True if the application is running, False otherwise.

        Example:

        .. code-block:: python

            >>> simulation_app = SimulationApp()
            >>> simulation_app.is_running()
            True
            >>> simulation_app.close()
            >>> simulation_app.is_running()
            False
        """
        return self._app.is_running() and not self.is_exiting()

    def is_exiting(self) -> bool:
        """Check if the simulation application is in the process of exiting.

        Returns True if close() has been called and the application is shutting down,
        False if the application is still running normally.

        Returns:
            True if close() was called previously, False otherwise.

        Example:

        .. code-block:: python

            >>> simulation_app = SimulationApp()
            >>> simulation_app.is_exiting()
            False
            >>> simulation_app.close()
            >>> simulation_app.is_exiting()
            True
        """
        return self._exiting

    @property
    def app(self) -> omni.kit.app.IApp:
        """Underlying Omniverse Kit application object.

        Provides access to the low-level Kit application interface for advanced
        operations and direct framework access.

        Returns:
            The Omniverse Kit application interface object.

        Example:

        .. code-block:: python

            >>> simulation_app = SimulationApp()
            >>> app = simulation_app.app
            >>> app.update()  # Direct app update
            >>> simulation_app.close()
        """
        return self._app

    @property
    def context(self) -> omni.usd.UsdContext:
        """Current USD context for stage operations.

        Provides access to the USD context which manages the current stage
        and USD-related operations.

        Returns:
            The current USD context object.

        Example:

        .. code-block:: python

            >>> simulation_app = SimulationApp()
            >>> context = simulation_app.context
            >>> stage = context.get_stage()
            >>> print(stage.GetRootLayer().identifier)
            >>> simulation_app.close()
        """
        return omni.usd.get_context()
