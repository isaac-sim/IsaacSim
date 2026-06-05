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

"""Nucleus server utilities for Isaac Sim asset storage and management operations."""

import asyncio
import json
import os

# python
import typing
from collections import namedtuple
from urllib.parse import urlparse

# omniverse
import carb
import omni.client
import omni.kit.app
import omni.kit.commands
from isaacsim.core.version import get_version
from omni.client import CopyBehavior, Result

DEFAULT_ASSET_ROOT_PATH_SETTING = "/persistent/isaac/asset_root/default"
DEFAULT_ASSET_ROOT_TIMEOUT_SETTING = "/persistent/isaac/asset_root/timeout"
DEFAULT_ASSET_ROOT_RETRY_ATTEMPTS_SETTING = "/persistent/isaac/asset_root/retry_attempts"
DEFAULT_ASSET_ROOT_RETRY_BASE_DELAY_SETTING = "/persistent/isaac/asset_root/retry_base_delay"


class Version(namedtuple("Version", "major minor patch")):
    """Semantic version representation for comparing Isaac Sim assets versions.

    A named tuple that parses and stores version strings in the format "major.minor.patch".
    Supports comparison operations inherited from the namedtuple base class.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native.nucleus import Version
        >>>
        >>> v1 = Version("1.2.3")
        >>> v1.major, v1.minor, v1.patch
        (1, 2, 3)
        >>> str(v1)
        '1.2.3'
    """

    def __new__(cls, s: str):
        """Create a new Version instance from a version string.

        Args:
            s: Version string in the format "major.minor.patch" (e.g., "1.2.3").

        Returns:
            A new Version named tuple with major, minor, and patch components.
        """
        return super().__new__(cls, *map(int, s.split(".")))

    def __repr__(self):
        """Return the string representation of the version.

        Returns:
            Version string in the format "major.minor.patch".
        """
        return ".".join(map(str, self))


def get_url_root(url: str) -> str:
    """Get root from URL or path.

    Extracts the server root (protocol and netloc) from a full URL.

    Args:
        url: Full http or omniverse path.

    Raises:
        RuntimeError: If the root path is not found or protocol is unsupported.

    Returns:
        Root path or URL of the Nucleus server.

    Example:

    .. code-block:: python

        >>> from isaacsim.storage.native import get_url_root
        >>>
        >>> get_url_root("omniverse://localhost/NVIDIA/Assets")
        'omniverse://localhost'
        >>> get_url_root("https://example.com/path/to/asset")
        'https://example.com'
    """
    supported_list = ["omniverse", "http", "https"]
    protocol = urlparse(url).scheme
    if protocol not in supported_list:
        raise RuntimeError("Unable to find root for {}".format(url))
        return ""
    server = f"{protocol}://{urlparse(url).netloc}"
    return server


def create_folder(server: str, path: str) -> bool:
    """Create a folder on server.

    Args:
        server: Name of Nucleus server.
        path: Path to folder.

    Returns:
        True if folder is created successfully.
    """
    carb.log_info("Create {} folder on {} Server".format(path, server))
    # Increase hang detection timeout
    omni.client.set_hang_detection_time_ms(10000)
    result = omni.client.create_folder("{}{}".format(server, path))
    if result == Result.OK:
        carb.log_info("Success: {} Server has {} folder created".format(server, path))
        return True
    else:
        carb.log_warn("Failure: Server {} not able to create {} folder".format(server, path))
        return False


def delete_folder(server: str, path: str) -> bool:
    """Remove folder and all of its contents.

    Args:
        server: Name of Nucleus server.
        path: Path to folder.

    Returns:
        True if folder is deleted successfully.
    """
    carb.log_info("Cleanup {} folder on {} Server".format(path, server))
    # Increase hang detection timeout
    omni.client.set_hang_detection_time_ms(10000)
    result = omni.client.delete("{}{}".format(server, path))
    if result == Result.OK:
        carb.log_info("Success: {} Server has {} folder deleted".format(server, path))
        return True
    else:
        carb.log_warn("Failure: Server {} not able to delete {} folder".format(server, path))
        return False


async def _list_files(url: str) -> typing.Tuple[str, typing.List]:
    """List files under a URL.

    Args:
        url: URL of Nucleus server with path to folder.

    Returns:
        Tuple containing root URL of Nucleus server and list of paths to each file.
    """
    root, paths = await _collect_files(url)
    return root, paths


async def download_assets_async(
    src: str,
    dst: str,
    progress_callback: object,
    concurrency: int = 10,
    copy_behaviour: omni.client.CopyBehavior = CopyBehavior.OVERWRITE,
    copy_after_delete: bool = True,
    timeout: float = 300.0,
) -> omni.client.Result:
    """Download assets from S3 bucket.

    Args:
        src: URL of S3 bucket as source.
        dst: URL of Nucleus server to copy assets to.
        progress_callback: Callback function to keep track of progress of copy.
            The callback receives two arguments: current count and total count.
        concurrency: Number of concurrent copy operations.
        copy_behaviour: Behavior if the destination exists.
        copy_after_delete: True if destination needs to be deleted before a copy.
        timeout: Timeout in seconds for each copy operation.

    Returns:
        Result of the copy operation.
    """
    # omni.client is a singleton, import locally to allow to run with multiprocessing
    import omni.client

    count = 0
    result = Result.ERROR

    if copy_after_delete and check_server(dst, ""):
        carb.log_info("Deleting existing folder {}".format(dst))
        delete_folder(dst, "")

    sem = asyncio.Semaphore(concurrency)
    carb.log_info("Listing {} ...".format(src))
    root_source, paths = await _list_files("{}".format(src))
    total = len(paths)
    carb.log_info("Found {} files from {}".format(total, root_source))

    for entry in reversed(paths):
        count += 1
        path = os.path.relpath(entry, root_source).replace("\\", "/")

        carb.log_info(
            "Downloading asset {} of {} from {}/{} to {}/{}".format(count, total, root_source, path, dst, path)
        )
        try:
            async with sem:
                result = await asyncio.wait_for(
                    omni.client.copy_async(
                        "{}/{}".format(root_source, path), "{}/{}".format(dst, path), copy_behaviour
                    ),
                    timeout=timeout,
                )
            if result != Result.OK:
                carb.log_warn(f"Failed to copy {path} to {dst}.")
                return Result.ERROR_ACCESS_LOST
        except asyncio.CancelledError:
            carb.log_warn(f"Assets download cancelled.")
            return Result.ERROR
        except Exception as ex:
            carb.log_warn(f"Exception: {type(ex).__name__}")
            return Result.ERROR
        progress_callback(count, total)

    return result


def check_server(server: str, path: str, timeout: float = 10.0) -> bool:
    """Check a specific server for a path.

    Args:
        server: Name of Nucleus server.
        path: Path to search.
        timeout: Timeout in seconds.

    Returns:
        True if folder is found.
    """
    carb.log_info("Checking path: {}{}".format(server, path))
    # Increase hang detection timeout
    omni.client.set_hang_detection_time_ms(20000)
    result, _ = omni.client.stat("{}{}".format(server, path))
    if result == Result.OK:
        carb.log_info("Success: {}{}".format(server, path))
        return True
    else:
        carb.log_info("Failure: {}{} not accessible".format(server, path))
        return False


async def check_server_async(server: str, path: str, timeout: float = 10.0) -> bool:
    """Check a specific server for a path.

    This function retries transient failures using exponential backoff.
    It retries when the stat operation times out or returns
    ``omni.client.Result.ERROR_CONNECTION``. Retry behavior is controlled by:
    ``/persistent/isaac/asset_root/retry_attempts`` and
    ``/persistent/isaac/asset_root/retry_base_delay``.

    Args:
        server: Name of Nucleus server.
        path: Path to search.
        timeout: Per-attempt timeout in seconds.

    Returns:
        True if folder is found, False otherwise.
    """
    carb.log_info("Checking path: {}{}".format(server, path))

    settings = carb.settings.get_settings()
    retry_attempts = settings.get(DEFAULT_ASSET_ROOT_RETRY_ATTEMPTS_SETTING)
    if not isinstance(retry_attempts, int) or retry_attempts < 1:
        retry_attempts = 3
    retry_base_delay = settings.get(DEFAULT_ASSET_ROOT_RETRY_BASE_DELAY_SETTING)
    if not isinstance(retry_base_delay, (int, float)) or retry_base_delay <= 0:
        retry_base_delay = 0.5

    delay = float(retry_base_delay)
    server_path = "{}{}".format(server, path)
    for attempt in range(1, retry_attempts + 1):
        try:
            result, _ = await asyncio.wait_for(omni.client.stat_async(server_path), timeout)
            if result == Result.OK:
                carb.log_info("Success: {}".format(server_path))
                return True
            if result == Result.ERROR_CONNECTION and attempt < retry_attempts:
                carb.log_warn(
                    f"check_server_async() connection error on attempt {attempt}/{retry_attempts}, retrying in {delay:.2f}s"
                )
                await asyncio.sleep(delay)
                delay *= 2
                continue
            carb.log_info("Failure: {} not accessible".format(server_path))
            return False
        except asyncio.TimeoutError:
            if attempt < retry_attempts:
                carb.log_warn(
                    f"check_server_async() timeout {timeout} on attempt {attempt}/{retry_attempts}, retrying in {delay:.2f}s"
                )
                await asyncio.sleep(delay)
                delay *= 2
                continue
            carb.log_warn(f"check_server_async() timeout {timeout}")
            return False
        except Exception as ex:
            carb.log_warn(f"Exception: {type(ex).__name__}")
            return False

    return False


def build_server_list() -> typing.List:
    """Return list with all known servers to check.

    Retrieves the list of mounted Nucleus server drives from the persistent settings.

    Returns:
        List of servers found.
    """
    mounted_drives = carb.settings.get_settings().get_settings_dictionary("/persistent/app/omniverse/mountedDrives")
    all_servers = []
    if mounted_drives is not None:
        mounted_dict = json.loads(mounted_drives.get_dict())
        for drive in mounted_dict.items():
            all_servers.append(drive[1])
    else:
        carb.log_info("/persistent/app/omniverse/mountedDrives setting not found")

    return all_servers


def find_nucleus_server(suffix: str) -> typing.Tuple[bool, str]:
    """Attempts to determine best Nucleus server to use based on existing mountedDrives setting.

    .. deprecated::
        This function is deprecated. Use :func:`get_assets_root_path` instead.

    Args:
        suffix: Path to folder to search for.

    Returns:
        Tuple of (found, url) where found is True if Nucleus server with suffix is found
        and url is the URL of the found Nucleus server.
    """
    carb.log_warn("find_nucleus_server() is deprecated. Use get_assets_root_path().")
    return False, ""


def get_server_path(suffix: str = "") -> typing.Union[str, None]:
    """Tries to find a Nucleus server with specific path.

    Args:
        suffix: Path to folder to search for.

    Raises:
        RuntimeError: If the root path is not found.

    Returns:
        URL of Nucleus server with path to folder, or None if not found.
    """
    carb.log_info(f"Check {DEFAULT_ASSET_ROOT_PATH_SETTING} setting")
    default_asset_root = carb.settings.get_settings().get(DEFAULT_ASSET_ROOT_PATH_SETTING)
    server_root = get_url_root(default_asset_root)
    if server_root:
        result = check_server(server_root, suffix)
        if result:
            return server_root
    raise RuntimeError("Could not find Nucleus server with {} folder".format(suffix))


async def get_server_path_async(suffix: str = "") -> typing.Union[str, None]:
    """Tries to find a Nucleus server with specific path (asynchronous version).

    Args:
        suffix: Path to folder to search for.

    Raises:
        RuntimeError: If the root path is not found.

    Returns:
        URL of Nucleus server with path to folder, or None if not found.
    """
    carb.log_info(f"Check {DEFAULT_ASSET_ROOT_PATH_SETTING} setting")
    default_asset_root = carb.settings.get_settings().get(DEFAULT_ASSET_ROOT_PATH_SETTING)
    server_root = get_url_root(default_asset_root)
    if server_root:
        result = await check_server_async(server_root, suffix)
        if result:
            return server_root
    raise RuntimeError("Could not find Nucleus server with {} folder".format(suffix))


def verify_asset_root_path(path: str) -> typing.Tuple[omni.client.Result, str]:
    """Attempts to determine Isaac assets version and check if there are updates.

    Reads the version.txt file from the asset root path and compares it against
    the current Isaac Sim application version to verify compatibility.

    Args:
        path: URL or path of asset root to verify.

    Returns:
        Tuple containing the result (OK if assets verified) and the version string
        of the Isaac Sim assets.
    """
    # omni.client is a singleton, import locally to allow to run with multiprocessing
    import omni.client

    ver_asset = Version("0.0.0")
    version_core, _, _, _, _, _, _, _ = get_version()
    ver_app = Version(version_core)

    # Get asset version
    carb.log_info(f"Verifying {path}")
    try:
        # Increase hang detection timeout
        omni.client.set_hang_detection_time_ms(10000)
        omni.client.push_base_url(f"{path}/")
        file_path = omni.client.combine_with_base_url("version.txt")
        # carb.log_warn(f"Looking for version file at: {file_path}")
        result, _, file_content = omni.client.read_file(file_path)
        if result != omni.client.Result.OK:
            carb.log_info(f"Unable to find version file: {file_path}.")
        else:
            ver_asset = Version(memoryview(file_content).tobytes().decode())

    except ValueError:
        carb.log_info(f"Unable to parse version file: {file_path}.")
    except UnicodeDecodeError:
        carb.log_info(f"Unable to read version file: {file_path}.")
    except Exception as ex:
        carb.log_warn(f"Exception: {type(ex).__name__}")

    # Compare versions
    # carb.log_warn(f"ver_asset = {ver_asset.major}.{ver_asset.minor}.{ver_asset.patch}")
    # carb.log_warn(f"ver_app = {ver_app.major}.{ver_app.minor}.{ver_app.patch}")

    if ver_asset == Version("0.0.0"):
        carb.log_info(f"Error verifying Isaac Sim assets at {path}")
        return Result.ERROR_NOT_FOUND, ""
    elif ver_asset.major != ver_app.major:
        carb.log_info(f"Unsupported version of Isaac Sim assets found at {path}: {ver_asset}")
        return Result.ERROR_BAD_VERSION, ver_asset
    elif ver_asset.minor != ver_app.minor:
        carb.log_info(f"Unsupported version of Isaac Sim assets found at {path}: {ver_asset}")
        return Result.ERROR_BAD_VERSION, ver_asset
    else:
        return Result.OK, ver_asset


def get_full_asset_path(path: str) -> typing.Union[str, None]:
    """Tries to find the full asset path on connected servers.

    Searches for the asset path first in the default asset root, then in all
    mounted Nucleus drives.

    Args:
        path: Path of asset from root to verify.

    Raises:
        RuntimeError: If the root path is not found.

    Returns:
        URL or full path to assets, or None if assets not found.
    """
    # 1 - Check /persistent/isaac/asset_root/default setting
    default_asset_root = carb.settings.get_settings().get(DEFAULT_ASSET_ROOT_PATH_SETTING)
    if default_asset_root:
        result = check_server(default_asset_root, path)
        if result:
            carb.log_info("Asset path found at {}{}".format(default_asset_root, path))
            return default_asset_root + path

    # 2 - Check mountedDrives setting
    connected_servers = build_server_list()
    if len(connected_servers):
        for server_name in connected_servers:
            result = check_server(server_name, path)
            if result:
                carb.log_info("Asset path found at {}{}".format(server_name, path))
                return server_name + path

    raise RuntimeError("Could not find assets path: {}".format(path))


async def get_full_asset_path_async(path: str) -> typing.Union[str, None]:
    """Tries to find the full asset path on connected servers (asynchronous version).

    Searches for the asset path first in the default asset root, then in all
    mounted Nucleus drives.

    Args:
        path: Path of asset from root to verify.

    Raises:
        RuntimeError: If the root path is not found.

    Returns:
        URL or full path to assets, or None if assets not found.
    """
    # 1 - Check /persistent/isaac/asset_root/default setting
    default_asset_root = carb.settings.get_settings().get(DEFAULT_ASSET_ROOT_PATH_SETTING)
    if default_asset_root:
        result = await check_server_async(default_asset_root, path)
        if result:
            carb.log_info("Asset path found at {}{}".format(default_asset_root, path))
            return default_asset_root + path

    # 2 - Check mountedDrives setting
    connected_servers = build_server_list()
    if len(connected_servers):
        for server_name in connected_servers:
            result = await check_server_async(server_name, path)
            if result:
                carb.log_info("Asset path found at {}{}".format(server_name, path))
                return server_name + path

    raise RuntimeError("Could not find assets path: {}".format(path))


def get_nvidia_asset_root_path() -> typing.Union[str, None]:
    """Get the NVIDIA asset root path.

    .. deprecated::
        This function is deprecated. Use :func:`get_assets_root_path` instead.

    Returns:
        None. This function is deprecated and always returns None.
    """
    carb.log_warn("get_nvidia_asset_root_path() has been deprecated. Use get_assets_root_path().")

    return None


def get_isaac_asset_root_path() -> typing.Union[str, None]:
    """Get the Isaac Sim asset root path.

    .. deprecated::
        This function is deprecated. Use :func:`get_assets_root_path` instead.

    Returns:
        None. This function is deprecated and always returns None.
    """
    carb.log_warn("get_isaac_asset_root_path() has been deprecated. Use get_assets_root_path().")

    return None


def get_assets_root_path(*, skip_check: bool = False) -> str:
    """Tries to find the root path to the Isaac Sim assets on a Nucleus server.

    Args:
        skip_check: If True, skip the checking step to verify that the resolved path exists.

    Raises:
        RuntimeError: If the root path setting is not set.
        RuntimeError: If the root path is not found.

    Returns:
        URL of Nucleus server with root path to assets folder.
    """
    # get timeout
    timeout = carb.settings.get_settings().get(DEFAULT_ASSET_ROOT_TIMEOUT_SETTING)
    if not isinstance(timeout, (int, float)):
        timeout = 10.0

    # resolve path
    carb.log_info(f"Check {DEFAULT_ASSET_ROOT_PATH_SETTING} setting")
    default_asset_root = carb.settings.get_settings().get(DEFAULT_ASSET_ROOT_PATH_SETTING)
    if not default_asset_root:
        raise RuntimeError(f"The '{DEFAULT_ASSET_ROOT_PATH_SETTING}' setting is not set")
    if skip_check:
        return default_asset_root

    # check path
    result = check_server(default_asset_root, "/Isaac", timeout)
    if result:
        result = check_server(default_asset_root, "/NVIDIA", timeout)
        if result:
            carb.log_info(f"Assets root found at {default_asset_root}")
            return default_asset_root

    raise RuntimeError(f"Could not find assets root folder: {default_asset_root}")


async def get_assets_root_path_async(*, skip_check: bool = False) -> str:
    """Tries to find the root path to the Isaac Sim assets on a Nucleus server.

    Args:
        skip_check: If True, skip the checking step to verify that the resolved path exists.

    Raises:
        RuntimeError: If the root path setting is not set.
        RuntimeError: If the root path is not found.

    Returns:
        URL of Nucleus server with root path to assets folder.
    """
    # get timeout
    timeout = carb.settings.get_settings().get(DEFAULT_ASSET_ROOT_TIMEOUT_SETTING)
    if not isinstance(timeout, (int, float)):
        timeout = 10.0

    # resolve path
    carb.log_info(f"Check {DEFAULT_ASSET_ROOT_PATH_SETTING} setting")
    default_asset_root = carb.settings.get_settings().get(DEFAULT_ASSET_ROOT_PATH_SETTING)
    if not default_asset_root:
        raise RuntimeError(f"The '{DEFAULT_ASSET_ROOT_PATH_SETTING}' setting is not set")
    if skip_check:
        return default_asset_root

    # check path
    result = await check_server_async(default_asset_root, "/Isaac", timeout)
    if result:
        result = await check_server_async(default_asset_root, "/NVIDIA", timeout)
        if result:
            carb.log_info(f"Assets root found at {default_asset_root}")
            return default_asset_root
    raise RuntimeError(f"Could not find assets root folder: {default_asset_root}")


def get_assets_server() -> typing.Union[str, None]:
    """Tries to find a server with the Isaac Sim assets.

    .. deprecated::
        This function is deprecated. Use :func:`get_server_path` instead.

    Returns:
        None. This function is deprecated and always returns None.
    """
    carb.log_warn("get_assets_server() is deprecated. Use get_server_path().")
    return None


async def _collect_files(url: str) -> typing.Tuple[str, typing.List]:
    """Collect files under a URL.

    Args:
        url: URL of Nucleus server with path to folder.

    Returns:
        Tuple containing root URL of Nucleus server and list of paths to each file.
    """
    paths = []

    if await is_dir_async(url):
        root = url + "/"
        paths.extend(await recursive_list_folder(root))
        return url, paths
    else:
        if await is_file_async(url):
            root = os.path.dirname(url)
            return root, [url]


async def is_dir_async(path: str) -> bool:
    """Check if path is a folder.

    Args:
        path: Path to folder.

    Returns:
        True if path is a folder.

    Raises:
        Exception: If failed to determine if the path is a folder.
    """
    result, folder = await asyncio.wait_for(omni.client.list_async(path), timeout=10)
    if result != omni.client.Result.OK:
        raise Exception(f"Failed to determine if {path} is a folder: {result}")
    return True if len(folder) > 0 else False


def is_dir(path: str) -> bool:
    """Check if path is a folder.

    Args:
        path: Path to folder.

    Returns:
        True if path is a folder.

    Raises:
        Exception: If failed to determine if the path is a folder.
    """
    result, folder = omni.client.list(path)
    if result != omni.client.Result.OK:
        raise Exception(f"Failed to determine if {path} is a folder: {result}")
    return True if len(folder) > 0 else False


async def is_file_async(path: str) -> bool:
    """Check if path is a file.

    Args:
        path: Path to file.

    Returns:
        True if path is a file.

    Raises:
        Exception: If failed to determine if the path is a file.
    """
    result, file = await asyncio.wait_for(omni.client.stat_async(path), timeout=10)
    if result != omni.client.Result.OK:
        raise Exception(f"Failed to determine if {path} is a file: {result}")
    return False if file.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN > 0 else True


def is_file(path: str) -> bool:
    """Check if path is a file.

    Args:
        path: Path to file.

    Returns:
        True if path is a file.

    Raises:
        Exception: If failed to determine if the path is a file.
    """
    # Increase hang detection timeout
    omni.client.set_hang_detection_time_ms(10000)
    result, file = omni.client.stat(path)
    if result != omni.client.Result.OK:
        raise Exception(f"Failed to determine if {path} is a file: {result}")
    return False if file.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN > 0 else True


async def recursive_list_folder(path: str) -> typing.List:
    """Recursively list all files.

    Args:
        path: Path to folder.

    Returns:
        List of paths to each file.
    """
    if not path.endswith("/"):
        path += "/"
    paths = []
    files, dirs = await list_folder(path)
    paths.extend(files)

    tasks = []
    for dir in dirs:
        tasks.append(asyncio.create_task(recursive_list_folder(dir)))

    results = await asyncio.gather(*tasks)
    for result in results:
        paths.extend(result)

    return paths


async def list_folder(path: str) -> typing.Tuple[typing.List, typing.List]:
    """List files and sub-folders from root path.

    Args:
        path: Path to root folder.

    Returns:
        Tuple containing list of file paths and list of sub-folder paths.

    Raises:
        Exception: When unable to find files under the path.
    """
    # omni.client is a singleton, import locally to allow to run with multiprocessing
    import omni.client

    files = []
    dirs = []

    carb.log_info(f"Collecting files for {path}")
    result, entries = await asyncio.wait_for(omni.client.list_async(path), timeout=10)

    if result != omni.client.Result.OK:
        raise Exception(f"Failed to list entries for {path}: {result}")

    for entry in entries:
        # Increase hang detection timeout
        omni.client.set_hang_detection_time_ms(10000)
        full_path = omni.client.combine_urls(path, entry.relative_path)
        if entry.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN > 0:
            dirs.append(full_path + "/")
        else:
            carb.log_info(f"Enqueuing {full_path} for processing")
            files.append(full_path)

    return files, dirs
