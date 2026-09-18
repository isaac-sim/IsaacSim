# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Bootstrap the repository-pinned Pixi executable and run standalone library workflows."""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
import platform
import re
import stat
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO

from pixi_licenses import PixiLicenseError, collect_python_licenses

BOOTSTRAP_METADATA = "pixi-bootstrap.json"
CACHE_DIRECTORY = Path(".cache") / "pixi"
DOWNLOAD_TIMEOUT_SECONDS = 120
LOCK_TIMEOUT_SECONDS = 120
WINDOWS_CUDA_DRIVER_BLOCKER = (
    b"Intentionally invalid DLL: prevent CPU-only Pixi builds from loading the system CUDA driver.\n"
)
PIXI_LICENSE_DIRECTORY = Path("_cmake_build") / "PACKAGE-LICENSES"
PIXI_LICENSE_FILENAME = "isaacsim-libraries-PIP-LICENSES.txt"
PIXI_LICENSE_FILE_VARIABLE = "ISAACSIM_PIXI_LICENSE_FILE"
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
VERSION_PATTERN = re.compile(r"[0-9]+\.[0-9]+\.[0-9]+")
APPROVED_REDIRECT_HOSTS = frozenset(
    {
        "github.com",
        "objects.githubusercontent.com",
        "release-assets.githubusercontent.com",
    }
)
SCRUBBED_BUILD_VARIABLES = frozenset(
    {
        "CMAKE_PREFIX_PATH",
        "CPATH",
        "CPLUS_INCLUDE_PATH",
        "INCLUDE",
        "LD_LIBRARY_PATH",
        "LIB",
        "LIBPATH",
        "LIBRARY_PATH",
        "PKG_CONFIG_PATH",
        "PYTHONHOME",
        "PYTHONPATH",
    }
)


class PixiBootstrapError(RuntimeError):
    """Error raised when the pinned Pixi executable cannot be used safely."""


@dataclass(frozen=True)
class PixiRelease:
    """Pinned Pixi release artifact for one host.

    Args:
        version: Exact Pixi release version.
        platform_name: Repository platform identifier.
        url: Official release artifact URL.
        sha256: Expected artifact digest.
    """

    version: str
    platform_name: str
    url: str
    sha256: str


class _OfficialRedirectHandler(urllib.request.HTTPRedirectHandler):
    """URL redirect handler restricted to official GitHub release hosts."""

    def redirect_request(
        self,
        request: urllib.request.Request,
        file_pointer: Any,
        code: int,
        message: str,
        headers: Any,
        new_url: str,
    ) -> urllib.request.Request | None:
        """Validate a redirect before following it."""
        _validate_download_url(new_url, allow_release_asset_host=True)
        return super().redirect_request(request, file_pointer, code, message, headers, new_url)


def _repository_root() -> Path:
    """Return the repository root containing this launcher."""
    return Path(__file__).resolve().parents[1]


def _detect_platform() -> str:
    """Return the supported platform identifier for the current host.

    Raises:
        PixiBootstrapError: If the host operating system or architecture is unsupported.
    """
    machine = platform.machine().lower()
    if sys.platform == "win32" and machine in {"amd64", "x86_64"}:
        return "windows-x86_64"
    if sys.platform.startswith("linux") and machine in {"amd64", "x86_64"}:
        return "linux-x86_64"
    if sys.platform.startswith("linux") and machine in {"aarch64", "arm64"}:
        return "linux-aarch64"
    raise PixiBootstrapError(f"Unsupported Pixi host: {sys.platform}/{machine}")


def _read_nonempty_string(table: dict[str, Any], key: str, context: str) -> str:
    """Read a required nonempty string from bootstrap metadata."""
    value = table.get(key)
    if not isinstance(value, str) or not value:
        raise PixiBootstrapError(f"{context}.{key} must be a nonempty string")
    return value


def _validate_download_url(url: str, *, allow_release_asset_host: bool = False) -> None:
    """Validate an official Pixi artifact or redirect URL."""
    parsed = urllib.parse.urlparse(url)
    allowed_hosts = APPROVED_REDIRECT_HOSTS if allow_release_asset_host else frozenset({"github.com"})
    if parsed.scheme != "https" or parsed.hostname not in allowed_hosts or parsed.username or parsed.password:
        raise PixiBootstrapError(f"Pixi download URL is not an approved official HTTPS URL: {url}")


def _load_release(metadata_path: Path, platform_name: str) -> PixiRelease:
    """Load and validate the pinned release for one platform."""
    try:
        document = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise PixiBootstrapError(f"Cannot read Pixi bootstrap metadata {metadata_path}: {error}") from error
    if not isinstance(document, dict) or set(document) != {"version", "platforms"}:
        raise PixiBootstrapError(f"Pixi bootstrap metadata has invalid top-level fields: {metadata_path}")
    version = _read_nonempty_string(document, "version", str(metadata_path))
    if VERSION_PATTERN.fullmatch(version) is None:
        raise PixiBootstrapError(f"Pixi bootstrap version must be exact: {version}")
    platforms = document["platforms"]
    if not isinstance(platforms, dict) or platform_name not in platforms:
        raise PixiBootstrapError(f"Pixi {version} has no bootstrap artifact for {platform_name}")
    artifact = platforms[platform_name]
    if not isinstance(artifact, dict) or set(artifact) != {"url", "sha256"}:
        raise PixiBootstrapError(f"Pixi bootstrap artifact has invalid fields for {platform_name}")
    url = _read_nonempty_string(artifact, "url", platform_name)
    sha256 = _read_nonempty_string(artifact, "sha256", platform_name)
    _validate_download_url(url)
    expected_prefix = f"https://github.com/prefix-dev/pixi/releases/download/v{version}/pixi-"
    if not url.startswith(expected_prefix) or "/latest/" in url:
        raise PixiBootstrapError(f"Pixi artifact URL does not name the pinned official release: {url}")
    if SHA256_PATTERN.fullmatch(sha256) is None:
        raise PixiBootstrapError(f"Pixi artifact SHA-256 is invalid for {platform_name}")
    return PixiRelease(version, platform_name, url, sha256)


def _compute_sha256(path: Path) -> str:
    """Compute the SHA-256 digest of a file."""
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise PixiBootstrapError(f"Cannot hash Pixi executable {path}: {error}") from error
    return digest.hexdigest()


def _ensure_safe_directory(path: Path, repository_root: Path) -> None:
    """Create a cache directory without traversing symbolic links."""
    try:
        relative_parts = path.relative_to(repository_root).parts
    except ValueError as error:
        raise PixiBootstrapError(f"Pixi cache must remain inside the repository: {path}") from error
    current = repository_root
    for part in relative_parts:
        current = current / part
        try:
            info = current.lstat()
        except FileNotFoundError:
            try:
                current.mkdir(mode=0o700)
            except FileExistsError:
                pass
            info = current.lstat()
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise PixiBootstrapError(f"Pixi cache path is not a real directory: {current}")


@contextlib.contextmanager
def _exclusive_lock(path: Path) -> Iterator[None]:
    """Acquire an exclusive cross-process bootstrap lock."""
    deadline = time.monotonic() + LOCK_TIMEOUT_SECONDS
    descriptor = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        while True:
            try:
                if os.name == "nt":
                    import msvcrt

                    os.lseek(descriptor, 0, os.SEEK_SET)
                    if os.fstat(descriptor).st_size == 0:
                        os.write(descriptor, b"0")
                    os.lseek(descriptor, 0, os.SEEK_SET)
                    msvcrt.locking(descriptor, msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl

                    fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except (BlockingIOError, OSError):
                if time.monotonic() >= deadline:
                    raise PixiBootstrapError(f"Timed out waiting for Pixi bootstrap lock: {path}") from None
                time.sleep(0.1)
        yield
    finally:
        if os.name == "nt":
            import msvcrt

            os.lseek(descriptor, 0, os.SEEK_SET)
            with contextlib.suppress(OSError):
                msvcrt.locking(descriptor, msvcrt.LK_UNLCK, 1)
        else:
            import fcntl

            with contextlib.suppress(OSError):
                fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)


def _download_release(release: PixiRelease, output: BinaryIO) -> None:
    """Download one official Pixi release artifact into an exclusive file."""
    opener = urllib.request.build_opener(_OfficialRedirectHandler())
    request = urllib.request.Request(release.url, headers={"User-Agent": "isaac-sim-pixi-bootstrap/1"})
    try:
        with opener.open(request, timeout=DOWNLOAD_TIMEOUT_SECONDS) as response:
            _validate_download_url(response.geturl(), allow_release_asset_host=True)
            while chunk := response.read(1024 * 1024):
                output.write(chunk)
            output.flush()
            os.fsync(output.fileno())
    except (OSError, urllib.error.URLError) as error:
        raise PixiBootstrapError(
            f"Cannot download official Pixi {release.version} from {release.url}: {error}"
        ) from error


def _install_windows_cuda_driver_blocker(cache_root: Path) -> None:
    """Block Rattler's optional CUDA probe from loading a broken system driver."""
    blocker = cache_root / "nvcuda.dll"
    if blocker.is_file() and not blocker.is_symlink():
        if blocker.read_bytes() != WINDOWS_CUDA_DRIVER_BLOCKER:
            raise PixiBootstrapError(f"Unverified CUDA driver blocker exists at {blocker}")
        return
    if blocker.exists() or blocker.is_symlink():
        raise PixiBootstrapError(f"Unverified CUDA driver blocker exists at {blocker}")
    descriptor, temporary_name = tempfile.mkstemp(prefix="nvcuda-blocker-", dir=cache_root)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as output:
            output.write(WINDOWS_CUDA_DRIVER_BLOCKER)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, blocker)
    except Exception:
        with contextlib.suppress(OSError):
            temporary.unlink()
        raise


def ensure_pixi(
    *,
    repository_root: Path | None = None,
    platform_name: str | None = None,
    metadata_path: Path | None = None,
) -> Path:
    """Return a verified repository-pinned Pixi executable.

    Args:
        repository_root: Repository root override used by tests.
        platform_name: Host platform override used by tests.
        metadata_path: Bootstrap metadata override used by tests.

    Returns:
        Path to the verified executable.

    Raises:
        PixiBootstrapError: If metadata, cache safety, download, or verification fails.
    """
    root = (repository_root or _repository_root()).resolve()
    selected_platform = platform_name or _detect_platform()
    selected_metadata = metadata_path or root / "tools" / BOOTSTRAP_METADATA
    release = _load_release(selected_metadata, selected_platform)
    cache_root = root / CACHE_DIRECTORY / release.version / selected_platform
    _ensure_safe_directory(cache_root, root)
    executable = cache_root / ("pixi.exe" if selected_platform.startswith("windows-") else "pixi")
    lock_path = cache_root / "bootstrap.lock"
    with _exclusive_lock(lock_path):
        if executable.is_file() and not executable.is_symlink() and _compute_sha256(executable) == release.sha256:
            if os.name != "nt" and not os.access(executable, os.X_OK):
                executable.chmod(0o700)
            if selected_platform.startswith("windows-"):
                _install_windows_cuda_driver_blocker(cache_root)
            return executable
        if executable.exists() or executable.is_symlink():
            raise PixiBootstrapError(
                f"Unverified Pixi cache entry exists at {executable}. Remove it before bootstrapping again."
            )
        descriptor, temporary_name = tempfile.mkstemp(prefix="pixi-download-", dir=cache_root)
        temporary = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "wb") as output:
                _download_release(release, output)
            actual_digest = _compute_sha256(temporary)
            if actual_digest != release.sha256:
                raise PixiBootstrapError(
                    f"Pixi {release.version} digest mismatch for {release.platform_name}: "
                    f"expected {release.sha256}, received {actual_digest}"
                )
            temporary.chmod(0o700)
            os.replace(temporary, executable)
        except Exception:
            with contextlib.suppress(OSError):
                temporary.unlink()
            raise
        if selected_platform.startswith("windows-"):
            _install_windows_cuda_driver_blocker(cache_root)
    return executable


def _create_pixi_environment() -> dict[str, str]:
    """Create the controlled environment for standalone Pixi invocations."""
    environment = os.environ.copy()
    for variable in SCRUBBED_BUILD_VARIABLES:
        environment.pop(variable, None)
    environment["PIXI_NO_CONFIG"] = "1"
    environment["PIXI_TLS_ROOT_CERTS"] = "webpki"
    if os.name == "nt":
        # The CPU build pool has a stale NVIDIA driver whose nvcuda64.dll crashes during Rattler's host probes. Explicit
        # values are required: empty overrides are not preserved reliably by every Windows process launcher.
        environment["CONDA_OVERRIDE_CUDA"] = "0"
        environment["CONDA_OVERRIDE_CUDA_ARCH"] = "0.0"
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    return environment


def _run_pixi(
    executable: Path,
    arguments: Sequence[str],
    *,
    repository_root: Path,
    environment: dict[str, str] | None = None,
) -> int:
    """Run the verified Pixi executable with repository-controlled configuration."""
    if not arguments:
        raise PixiBootstrapError("Pixi arguments must include a subcommand")
    completed = subprocess.run(
        [
            str(executable),
            arguments[0],
            "--manifest-path",
            str(repository_root / "pixi.toml"),
            *arguments[1:],
        ],
        cwd=repository_root,
        env=environment if environment is not None else _create_pixi_environment(),
        check=False,
    )
    return completed.returncode


def _read_option(arguments: Sequence[str], name: str, default: str) -> str:
    """Read the last exact command-line option in split or assignment form.

    The library build uses ``argparse``'s last-value-wins behavior for repeated
    scalar options. Environment selection must make the same choice before the
    build starts. Abbreviated selectors are rejected here because accepting an
    abbreviation downstream can otherwise select a different dependency
    environment in this launcher.
    """
    value = default
    for index, argument in enumerate(arguments):
        option_name = argument.partition("=")[0]
        is_abbreviated = (
            len(option_name) > 2
            and option_name.startswith("--")
            and option_name != name
            and name.startswith(option_name)
        )
        if is_abbreviated:
            raise PixiBootstrapError(f"Abbreviated option {option_name!r} is not supported; use {name}")
        if argument == name:
            if index + 1 >= len(arguments) or arguments[index + 1].startswith("-"):
                raise PixiBootstrapError(f"{name} requires a value")
            value = arguments[index + 1]
        if argument.startswith(f"{name}="):
            value = argument.partition("=")[2]
            if not value:
                raise PixiBootstrapError(f"{name} requires a value")
    return value


def _has_option(arguments: Sequence[str], *names: str) -> bool:
    """Return whether command arguments contain one of the named options."""
    return any(argument in names or any(argument.startswith(f"{name}=") for name in names) for argument in arguments)


def _select_library_environments(arguments: Sequence[str]) -> tuple[str, ...]:
    """Select the isolated dependency environments for a library command."""
    if _has_option(arguments, "--pull-only"):
        return (
            "examples-runner",
            "build-driver",
            "native-runtime",
            "runtime",
            "schema-build",
            "test",
            "wheel-build",
            "minimum-runtime",
            "minimum-test",
            "minimum-wheel-build",
        )
    profile = _read_option(arguments, "--profile", "standard")
    dependency_profile = _read_option(arguments, "--dependency-profile", "locked")
    if dependency_profile == "latest":
        raise PixiBootstrapError(
            "--dependency-profile latest is not reproducible; update pixi.lock in the dependency-update workflow"
        )
    python_enabled = profile in {"standard", "python-modules", "werror"}
    minimum = dependency_profile == "minimum"
    runtime_environment = "minimum-runtime" if minimum else "runtime"
    test_environment = "minimum-test" if minimum else "test"
    wheel_environment = "minimum-wheel-build" if minimum else "wheel-build"
    selected = ["build-driver", "native-runtime"]
    if python_enabled:
        selected.extend((runtime_environment, "schema-build", test_environment))
    if _has_option(arguments, "--wheel", "--carrier"):
        selected.append(wheel_environment)
    return tuple(selected)


def _pixi_environment_prefix(repository_root: Path, environment_name: str) -> Path:
    """Return the installed prefix for a named Pixi environment."""
    return repository_root / ".pixi" / "envs" / environment_name


def _python_site_packages(prefix: Path) -> Path:
    """Return the site-packages directory in a Pixi environment prefix."""
    if os.name == "nt":
        site_packages = prefix / "Lib" / "site-packages"
    else:
        python_directories = (
            path
            for path in (prefix / "lib").iterdir()
            if path.is_dir() and not path.is_symlink() and re.fullmatch(r"python[0-9]+\.[0-9]+", path.name)
        )
        candidates = sorted(path / "site-packages" for path in python_directories if (path / "site-packages").is_dir())
        if len(candidates) != 1:
            raise PixiBootstrapError(f"Pixi environment has an ambiguous site-packages layout: {prefix}")
        site_packages = candidates[0]
    if not site_packages.is_dir():
        raise PixiBootstrapError(f"Pixi environment site-packages is missing: {site_packages}")
    return site_packages


def _license_output_path(repository_root: Path) -> Path:
    """Return a lock- and platform-specific native-runtime aggregate path."""
    digest = hashlib.sha256()
    try:
        digest.update((repository_root / "pixi.lock").read_bytes())
    except OSError as error:
        raise PixiBootstrapError(f"Cannot read the Pixi lock file: {repository_root / 'pixi.lock'}") from error
    digest.update(b"\0")
    digest.update(_detect_platform().encode("utf-8"))
    selection_key = digest.hexdigest()[:24]
    return repository_root / PIXI_LICENSE_DIRECTORY / selection_key / PIXI_LICENSE_FILENAME


def _collect_native_runtime_licenses(prefix: Path, output_path: Path) -> None:
    """Collect notices for the locked Python payload redistributed by native modules."""
    try:
        collect_python_licenses([_python_site_packages(prefix)], output_path)
    except PixiLicenseError as error:
        raise PixiBootstrapError(str(error)) from error


def _install_locked_environment(
    executable: Path,
    repository_root: Path,
    environment_name: str,
    *,
    offline: bool,
) -> None:
    """Install one named environment without permitting lock-file changes."""
    command = [
        "install",
        "--locked",
        "--environment",
        environment_name,
    ]
    if offline:
        command.append("--offline")
    status = _run_pixi(executable, command, repository_root=repository_root)
    if status != 0:
        raise PixiBootstrapError(
            f"Pixi failed to install the locked {environment_name!r} environment (exit code {status})"
        )


def _run_libraries(arguments: Sequence[str]) -> int:
    """Run the library build with isolated Pixi dependency prefixes."""
    repository_root = _repository_root()
    if _has_option(arguments, "-h", "--help", "-c", "--clean"):
        environment = os.environ.copy()
        environment["PYTHONDONTWRITEBYTECODE"] = "1"
        return subprocess.run(
            [sys.executable, str(repository_root / "source" / "libraries" / "tools" / "build.py"), *arguments],
            cwd=repository_root,
            env=environment,
            check=False,
        ).returncode
    executable = ensure_pixi(repository_root=repository_root)
    selected_environments = _select_library_environments(arguments)
    driver_environment = "build-driver"
    offline = _has_option(arguments, "--no-pull")
    for environment_name in selected_environments:
        _install_locked_environment(executable, repository_root, environment_name, offline=offline)
    prefixes = {name: _pixi_environment_prefix(repository_root, name) for name in selected_environments}
    license_output = None
    if not _has_option(arguments, "--pull-only", "--test-only"):
        license_output = _license_output_path(repository_root)
        _collect_native_runtime_licenses(prefixes["native-runtime"], license_output)
    environment = _create_pixi_environment()
    environment["ISAACSIM_PIXI_BUILD_PREFIX"] = str(prefixes[driver_environment])
    if os.name != "nt":
        environment["LD_LIBRARY_PATH"] = str(prefixes[driver_environment] / "lib")
    environment["ISAACSIM_PIXI_NATIVE_RUNTIME_DEPS"] = str(_python_site_packages(prefixes["native-runtime"]))
    if license_output is not None:
        environment[PIXI_LICENSE_FILE_VARIABLE] = str(license_output)
    if "schema-build" in selected_environments:
        environment["ISAACSIM_PIXI_SCHEMA_BUILD_DEPS"] = str(_python_site_packages(prefixes["schema-build"]))
    dependency_profile = _read_option(arguments, "--dependency-profile", "locked")
    if dependency_profile == "minimum":
        runtime_name = "minimum-runtime"
        test_name = "minimum-test"
        wheel_name = "minimum-wheel-build"
    else:
        runtime_name = "runtime"
        test_name = "test"
        wheel_name = "wheel-build"
    for variable, environment_name in (
        ("ISAACSIM_PIXI_RUNTIME_DEPS", runtime_name),
        ("ISAACSIM_PIXI_TEST_DEPS", test_name),
        ("ISAACSIM_PIXI_WHEEL_BUILD_DEPS", wheel_name),
    ):
        if environment_name in selected_environments:
            environment[variable] = str(_python_site_packages(prefixes[environment_name]))
    return _run_pixi(
        executable,
        [
            "run",
            "--locked",
            "--environment",
            driver_environment,
            "--",
            "python",
            str(repository_root / "source" / "libraries" / "tools" / "build.py"),
            *arguments,
        ],
        repository_root=repository_root,
        environment=environment,
    )


def _run_examples(arguments: Sequence[str]) -> int:
    """Run source examples with the locked runner environment."""
    repository_root = _repository_root()
    executable = ensure_pixi(repository_root=repository_root)
    return _run_pixi(
        executable,
        [
            "run",
            "--locked",
            "--environment",
            "examples-runner",
            "--",
            "python",
            str(repository_root / "source" / "examples" / "examples.py"),
            "--dev",
            *arguments,
        ],
        repository_root=repository_root,
        environment=_create_pixi_environment(),
    )


def _create_argument_parser() -> argparse.ArgumentParser:
    """Create the command-line parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("bootstrap", help="Download or verify the pinned Pixi executable.")
    examples_parser = subparsers.add_parser("examples", help="Run the published source examples.")
    examples_parser.add_argument("arguments", nargs=argparse.REMAINDER)
    libraries_parser = subparsers.add_parser("libraries", help="Run the standalone library build.")
    libraries_parser.add_argument("arguments", nargs=argparse.REMAINDER)
    return parser


def _main() -> int:
    """Execute the bootstrap launcher."""
    arguments = _create_argument_parser().parse_args()
    if arguments.command == "bootstrap":
        print(ensure_pixi())
        return 0
    command = list(arguments.arguments)
    if command[:1] == ["--"]:
        command = command[1:]
    if arguments.command == "examples":
        return _run_examples(command)
    return _run_libraries(command)


if __name__ == "__main__":
    try:
        raise SystemExit(_main())
    except PixiBootstrapError as error:
        print(f"Error: {error}", file=sys.stderr)
        raise SystemExit(1) from error
