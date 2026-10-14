# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Build h5py with a pinned HDF5 library from verified local source archives.

Run with CPython 3.12 in a provisioned build environment. This command neither
downloads dependencies nor publishes wheels. Use --unrepaired only for local
compatibility experiments; those wheels must not be distributed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
import zipfile
from pathlib import Path

H5PY_VERSION = "3.16.0+nv1"
HDF5_VERSION = "2.2.0"
SOURCES = {
    "hdf5": ("hdf5-2.2.0", "5b8d75125ae7b4fef55d3b39e8d3dbfd238cdf63b6518afd67fa69ac80f06542"),
    "h5py": ("h5py-3.16.0", "a0dbaad796840ccaa67a4c144a0d0c8080073c34c76d5a6941d6818678ef2738"),
}


def _hash(path: Path) -> str:
    """Compute a file's SHA-256 digest."""
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def _extract(archive: Path, name: str, destination: Path) -> Path:
    """Reject unexpected source bytes before safely extracting them."""
    directory, digest = SOURCES[name]
    # Verify and extract the same private snapshot, not a mutable input path.
    # The destination's parent is the configured work directory in the CLI.
    with tempfile.TemporaryFile(dir=destination.parent) as snapshot:
        with archive.open("rb") as source:
            shutil.copyfileobj(source, snapshot)
        snapshot.seek(0)
        if hashlib.file_digest(snapshot, "sha256").hexdigest() != digest:
            raise ValueError(f"Source hash mismatch: {name}")
        snapshot.seek(0)
        with tarfile.open(fileobj=snapshot) as source:
            source.extractall(destination, filter="data")
    result = destination / directory
    if not result.is_dir():
        raise ValueError(f"Missing source directory: {directory}")
    return result


def _set_version(source: Path) -> None:
    """Give the rebuilt distribution and runtime a distinct local version."""
    replacements = [
        (source / "pyproject.toml", "version = '3.16.0'", f"version = '{H5PY_VERSION}'"),
        (
            source / "h5py/version.py",
            'version = "{0.major:d}.{0.minor:d}.{0.bugfix:d}".format(version_tuple)',
            f'version = "{H5PY_VERSION}"',
        ),
    ]
    for path, old, new in replacements:
        text = path.read_text(encoding="utf-8")
        if text.count(old) != 1:
            raise ValueError(f"Unexpected upstream version declaration: {path.name}")
        path.write_text(text.replace(old, new), encoding="utf-8")


def _platform_tag(system: str, machine: str) -> str:
    """Reject unsupported build platforms rather than cross-tagging wheels."""
    if system == "Windows" and machine.lower() in {"amd64", "x86_64"}:
        return "win_amd64"
    if system == "Linux" and machine in {"x86_64", "aarch64"}:
        return f"manylinux_2_35_{machine}"
    raise ValueError(f"Unsupported native build platform: {system}/{machine}")


def _repair_command(wheel: Path, prefix: Path, output: Path, tag: str) -> list[str]:
    """Select native-library bundling for the requested platform."""
    if tag == "win_amd64":
        return [
            sys.executable,
            "-m",
            "delvewheel",
            "repair",
            "--add-path",
            str(prefix / "bin"),
            "--wheel-dir",
            str(output),
            str(wheel),
        ]
    return [
        sys.executable,
        "-m",
        "auditwheel",
        "repair",
        "--plat",
        tag,
        "--wheel-dir",
        str(output),
        str(wheel),
    ]


def _run(command: list[str], env: dict[str, str], log: Path) -> None:
    """Run one build stage and preserve failure output."""
    print(f"Running: {' '.join(command)}", flush=True)
    with log.open("a", encoding="utf-8") as stream:
        stream.write(f"\nCommand: {command!r}\n")
        stream.flush()
        subprocess.run(command, env=env, stdout=stream, stderr=subprocess.STDOUT, check=True)


def _check_bundled_hdf5(wheel: Path) -> None:
    """Require the repaired wheel to contain both HDF5 shared libraries."""
    with zipfile.ZipFile(wheel) as archive:
        libraries = [
            Path(name).name.lower() for name in archive.namelist() if name.endswith(".dll") or ".so" in Path(name).name
        ]
    if not any("hdf5" in name and "_hl" not in name for name in libraries):
        raise RuntimeError("Repaired wheel is missing the HDF5 library")
    if not any("hdf5_hl" in name for name in libraries):
        raise RuntimeError("Repaired wheel is missing the HDF5 high-level library")


def _main() -> None:
    """Parse the build inputs and produce a candidate wheel and manifest."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hdf5-source", required=True, type=Path)
    parser.add_argument("--h5py-source", required=True, type=Path)
    parser.add_argument("--work-dir", required=True, type=Path)
    parser.add_argument("--zlib-root", type=Path, help="Provisioned zlib prefix; required on Windows")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--unrepaired", action="store_true")
    args = parser.parse_args()
    if sys.version_info[:2] != (3, 12) or platform.python_implementation() != "CPython" or sys.maxsize <= 2**32:
        parser.error("Use 64-bit CPython 3.12")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    tag = _platform_tag(platform.system(), platform.machine())
    if tag == "win_amd64" and args.zlib_root is None:
        parser.error("Windows requires --zlib-root for a matching x64 zlib build")
    if args.zlib_root is not None and not args.zlib_root.is_dir():
        parser.error("--zlib-root must exist")
    args.work_dir.mkdir(parents=True, exist_ok=True)
    root = Path(tempfile.mkdtemp(prefix="h5py-", dir=args.work_dir.resolve()))
    print(f"Build directory: {root}", flush=True)
    hdf5 = _extract(args.hdf5_source, "hdf5", root)
    h5py = _extract(args.h5py_source, "h5py", root)
    _set_version(h5py)
    shutil.copyfile(hdf5 / "LICENSE", h5py / "licenses/hdf5.txt")
    prefix, build = root / "prefix", root / "native-build"
    raw, repaired = root / "unrepaired", root / "repaired"
    env = os.environ.copy()
    # Avoid ambient selectors overriding the verified HDF5 build.
    for key in list(env):
        if key.startswith(("HDF5_", "H5PY_")):
            del env[key]
    env.update(PIP_NO_INDEX="1", UV_OFFLINE="1", PYTHONNOUSERSITE="1")
    command = [
        "cmake",
        "-S",
        str(hdf5),
        "-B",
        str(build),
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DCMAKE_INSTALL_PREFIX={prefix}",
        "-DCMAKE_INSTALL_LIBDIR=lib",
        "-DBUILD_SHARED_LIBS=ON",
        "-DBUILD_STATIC_LIBS=OFF",
        "-DHDF5_BUILD_HL_LIB=ON",
        "-DBUILD_TESTING=OFF",
        "-DHDF5_BUILD_TOOLS=OFF",
        "-DHDF5_BUILD_EXAMPLES=OFF",
        "-DHDF5_BUILD_FORTRAN=OFF",
        "-DHDF5_BUILD_CPP_LIB=OFF",
        "-DHDF5_BUILD_JAVA=OFF",
        "-DHDF5_ENABLE_ZLIB_SUPPORT=ON",
        "-DHDF5_ENABLE_SZIP_SUPPORT=OFF",
        "-DHDF5_ENABLE_ROS3_VFD=OFF",
        "-DHDF5_ENABLE_PARALLEL=OFF",
        "-DHDF5_ALLOW_EXTERNAL_SUPPORT=NO",
    ]
    if args.zlib_root is not None:
        command.append(f"-DZLIB_ROOT={args.zlib_root.resolve()}")
    log = root / "build.log"
    _run(command, env, log)
    _run(["cmake", "--build", str(build), "--config", "Release", "--parallel", str(args.jobs)], env, log)
    _run(["cmake", "--install", str(build), "--config", "Release"], env, log)
    env.update(HDF5_DIR=str(prefix), H5PY_ROS3="0", H5PY_DIRECT_VFD="0", HDF5_MPI="OFF")
    if tag == "win_amd64":
        env["PATH"] = os.pathsep.join([str(prefix / "bin"), str(args.zlib_root.resolve() / "bin"), env.get("PATH", "")])
    else:
        env["LD_LIBRARY_PATH"] = str(prefix / "lib")
    _run([sys.executable, "-m", "build", "--no-isolation", "--wheel", "--outdir", str(raw), str(h5py)], env, log)
    wheels = list(raw.glob("*.whl"))
    if len(wheels) != 1:
        raise RuntimeError("Expected exactly one newly built wheel")
    if not args.unrepaired:
        _run(_repair_command(wheels[0], prefix, repaired, tag), env, log)
        wheels = list(repaired.glob("*.whl"))
        if len(wheels) != 1:
            raise RuntimeError("Expected exactly one repaired wheel")
        _check_bundled_hdf5(wheels[0])
    result = {
        "h5py": H5PY_VERSION,
        "hdf5": HDF5_VERSION,
        "platform": tag,
        "wheel": wheels[0].name,
        "sha256": _hash(wheels[0]),
        "repaired": not args.unrepaired,
        "validated": False,
        "sources": {name: digest for name, (_, digest) in SOURCES.items()},
    }
    (root / "manifest.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"Candidate: {wheels[0]}\nRuntime and clean-environment validation still required.")


if __name__ == "__main__":
    _main()
