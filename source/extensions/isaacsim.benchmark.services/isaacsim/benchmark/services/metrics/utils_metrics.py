# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility helpers for building benchmark metrics payloads."""

import calendar
import os
import pathlib
import platform
import uuid
from datetime import datetime

import carb
import omni.kit
import yaml  # type: ignore[import-untyped]

from .. import utils

logger = utils.set_up_logging(__name__)


def get_execution_environment() -> tuple[str, str]:
    """Create a source and build id for the metrics API.

    Returns:
        Tuple of (source, build_id).

    Example:

    .. code-block:: python

        source, build_id = get_execution_environment()
    """
    nvm_task_id = os.getenv("NVM_TASK_ID")
    if nvm_task_id is None:
        return "Other", datetime.utcnow().isoformat()
    return "GtlTask", nvm_task_id


def get_execution_purpose() -> str:
    """Get the execution purpose for this run.

    Returns:
        Execution purpose string.

    Example:

    .. code-block:: python

        purpose = get_execution_purpose()
    """
    # Check for CI environment variables
    if bool(os.getenv("TEAMCITY_VERSION")) or bool(os.getenv("ETM_ACTIVE")):
        return os.getenv("KB2_PURPOSE", "ci")
    return "local"


def get_kit_info() -> tuple[str, str, str]:
    """Get Kit information.

    Returns:
        Tuple of (full_branch, kit_build_version, kit_build_number).

    Example:

    .. code-block:: python

        full_branch, kit_build_version, kit_build_number = get_kit_info()
    """
    kit_build_version = omni.kit.app.get_app().get_build_version()
    kit_build_number = kit_build_version.rsplit(".", 3)[1]
    version = kit_build_version.split("+")[0]
    branch = kit_build_version.split("+")[1].split(".")[0]

    operating_system = platform.system()
    operating_system_version = "Unknown"
    if operating_system == "Windows":
        operating_system_version = platform.version().split(".")[0]
    elif operating_system == "Linux":
        # Supported for Ubuntu. Not tested for CentOS.
        # Doesn't work on TC machines anymore
        try:
            operating_system_version = platform.version().split("~")[1].split(".")[0]
        except Exception:
            pass

    full_branch = version + "-" + branch + "-" + operating_system + "-" + operating_system_version
    return full_branch, kit_build_version, kit_build_number


def get_app_info() -> tuple[str, str, str]:
    """Get App name and version.

    Returns:
        Tuple of (app_name, app_version, app_build).

    Example:

    .. code-block:: python

        app_name, app_version, app_build = get_app_info()
    """
    settings = carb.settings.get_settings()
    app_name = settings.get("/app/name")
    yaml_contents = get_package_info_yaml()
    try:
        ci_build_number = yaml_contents["CI Build Number"]
        app_version = ci_build_number.rsplit(".", 3)[0]
        app_build = ".".join(ci_build_number.rsplit(".", 3)[1:4])
        return app_name, app_version, app_build
    except Exception as e:
        logger.warning(f"Unable to find app_version and/or app_build. {str(e)}")
        return app_name, "Unknown", "Unknown"


def get_kit_build_date() -> int | None:
    """Get the Kit build date from PACKAGE-INFO.yaml or Kit executable if not found.

    Returns:
        Build date as milliseconds since epoch, or None if unavailable.

    Example:

    .. code-block:: python

        build_date = get_kit_build_date()
    """
    yaml_contents = get_package_info_yaml()
    try:
        time = yaml_contents["Time"]
        logger.info(f"PACKAGE-INFO.yaml Time = {time}")
        # Convert time from "Wed May 25 13:38:50 2022" to UNIX epoch time in millisecond.
        epoch_time_ms = int(calendar.timegm(datetime.strptime(time, "%a %b %d %H:%M:%S %Y").timetuple()) * 1000)
        logger.info(f"PACKAGE-INFO.yaml Epoch Time (ms) = {epoch_time_ms}")
        return epoch_time_ms
    except Exception as e:
        logger.warning(f"Unable to find Time with PACKAGE-INFO.yaml. {str(e)}")
        return None


def get_package_info_yaml(yaml_path: str | None = None) -> dict:
    """Get PACKAGE-INFO.yaml contents.

    Args:
        yaml_path: Explicit path to PACKAGE-INFO.yaml.

    Returns:
        Parsed YAML contents.

    Example:

    .. code-block:: python

        info = get_package_info_yaml()
    """
    if yaml_path is None:
        import carb.tokens

        yaml_path = str(pathlib.Path(carb.tokens.get_tokens_interface().resolve("${kit}")) / "PACKAGE-INFO.yaml")

    yaml_contents = {}
    try:
        logger.info(f"PACKAGE-INFO.yaml path = {yaml_path}")
        with open(yaml_path, encoding="utf-8") as f:
            yaml_contents = yaml.safe_load(f)
            logger.info(f"PACKAGE-INFO.yaml contents: {yaml_contents}")
    except Exception as e:
        logger.warning(f"Unable to find PACKAGE-INFO.yaml. {str(e)}")

    return yaml_contents


_run_uuid = uuid.uuid4().hex


def get_run_uuid() -> str:
    """Get the benchmark run UUID.

    If running in a test subprocess, the UUID is pulled from settings. Otherwise
    a process-global UUID is returned.

    Returns:
        Run UUID string.

    Example:

    .. code-block:: python

        run_id = get_run_uuid()
    """
    settings = carb.settings.get_settings()
    run_uuid = settings.get("/exts/omni.kit.tests.benchmark/metrics/run_uuid")
    return run_uuid or _run_uuid


def get_telem_logdir(metrics_output_dir: str, test_name: str) -> str:
    """Get the telemetry log directory for a test.

    Args:
        metrics_output_dir: Output directory for metrics.
        test_name: Test name to namespace logs.

    Returns:
        Telemetry log directory path.

    Example:

    .. code-block:: python

        path = get_telem_logdir("/tmp/metrics", "benchmark_a")
    """
    return os.path.join(metrics_output_dir, "telem_logs", test_name)
