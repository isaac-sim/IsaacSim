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

"""Schema dataclasses for benchmark metric payloads."""

import json
from dataclasses import dataclass
from typing import Any

import omni.kit.app


@dataclass
class OSConfiguration:
    """Operating system configuration details.

    Args:
        platform: Platform identifier.
        os: Operating system name.
        os_major: Major OS version.
        architecture: CPU architecture string.
    """

    platform: str
    os: str
    os_major: str
    architecture: str  # 64bit


@dataclass
class GPU:
    """GPU descriptor.

    Args:
        number: GPU index.
        gpu_id: GPU identifier string.
        product_architecture: GPU architecture name.
        product_brand: GPU brand.
        product_name: GPU product name.
    """

    number: int
    gpu_id: str
    product_architecture: str
    product_brand: str
    product_name: str


@dataclass
class GPUConfiguration:
    """GPU configuration details.

    Args:
        cuda_version: CUDA runtime version.
        driver_version: Driver version string.
        gpus: List of GPU descriptors.
        primary_gpu: Primary GPU descriptor.
        num_gpus: Total number of GPUs.
    """

    cuda_version: float
    driver_version: str
    gpus: list[GPU]
    primary_gpu: GPU
    num_gpus: int


@dataclass
class CPUConfiguration:
    """CPU configuration details.

    Args:
        model: CPU model string.
    """

    model: str


@dataclass
class MemoryConfiguration:
    """Memory configuration details.

    Args:
        ram_gb: Installed RAM in GB.
    """

    ram_gb: float


@dataclass
class HardwareConfiguration:
    """Hardware configuration details.

    Args:
        gpu_configuration: GPU configuration.
        cpu_configuration: CPU configuration.
        memory_configuration: Memory configuration.
    """

    gpu_configuration: GPUConfiguration
    cpu_configuration: CPUConfiguration
    memory_configuration: MemoryConfiguration


@dataclass
class Application:
    """Application build and version information.

    Args:
        name: Application short name.
        name_full: Full application name.
        kit_file: Kit file name.
        version_minor: Minor version string.
        version_major_minor_patch: Version string.
        version_full: Full version string.
        build_id: Build identifier.
        kit_version_minor: Kit minor version string.
        kit_version_patch: Kit patch version string.
        kit_build_id: Kit build identifier.
        package_name: Package name.
        package_full: Full package name.
        build_date: Build date timestamp.
    """

    name: str
    name_full: str
    kit_file: str
    version_minor: str
    version_major_minor_patch: str
    version_full: str
    build_id: str
    kit_version_minor: str
    kit_version_patch: str
    kit_build_id: str
    package_name: str
    package_full: str
    build_date: int


@dataclass
class ExecutionEnvironment:
    """Execution environment identifiers.

    Args:
        primary_system: Primary system name.
        primary_id: Primary system identifier.
        primary_url: Primary system URL.
        secondary_system: Secondary system name.
        secondary_id: Secondary system identifier.
        secondary_url: Secondary system URL.
        extension_identifier: Extension identifier string.
        etm_identifier: ETM identifier string.
        input_build_url: Input build URL.
        input_build_id: Input build identifier.
        hostname: Hostname string.
    """

    primary_system: str
    primary_id: str
    primary_url: str
    secondary_system: str
    secondary_id: str
    secondary_url: str
    extension_identifier: str
    etm_identifier: str
    input_build_url: str
    input_build_id: str
    hostname: str


@dataclass
class BenchmarkIdentifier:
    """Benchmark run identifier.

    Args:
        run_uuid: Run UUID string.
    """

    run_uuid: str


@dataclass
class Benchmark:
    """Benchmark metadata details.

    Args:
        name: Benchmark name.
        asset_url: Asset URL.
        version_identifier: Version identifier string.
        checkpoint: Checkpoint number.
        dssim_status: DSSIM status flag.
        dssim: DSSIM value.
        resolution: Render resolution string.
    """

    name: str
    asset_url: str
    version_identifier: str
    checkpoint: int
    dssim_status: bool
    dssim: float
    resolution: str


@dataclass
class Metric:
    """Metric payload containing name and value.

    Args:
        name: Metric name.
        value: Metric value.
    """

    name: str
    value: Any


@dataclass
class BenchData:
    """Top-level benchmark payload.

    Args:
        ts_created: Creation timestamp.
        test_name: Test name.
        schema: Schema version string.
        hardware_configuration: Hardware configuration payload.
        os_configuration: OS configuration payload.
        application: Application payload.
        execution_environment: Execution environment payload.
        benchmark_identifier: Benchmark identifier payload.
        benchmark: Benchmark payload.
        metric: Metric payload.
    """

    ts_created: int
    test_name: str
    schema: str
    hardware_configuration: HardwareConfiguration
    os_configuration: OSConfiguration
    application: Application
    execution_environment: ExecutionEnvironment
    benchmark_identifier: BenchmarkIdentifier
    benchmark: Benchmark
    metric: Metric

    def get_fingerprint(self) -> str:
        """Get a short session fingerprint hash.

        Returns:
            Fingerprint hash string.

        Example:

        .. code-block:: python

            fingerprint = bench_data.get_fingerprint()
        """
        import hashlib

        params = {
            "name": self.application.name,
            "version_minor": self.application.version_minor,
            "kit_version_minor": self.application.kit_version_minor,
            "platform": self.os_configuration.platform,
            "os": self.os_configuration.os,
            "architecture": self.os_configuration.architecture,
            "cpu": self.hardware_configuration.cpu_configuration.model,
            "gpu": self.hardware_configuration.gpu_configuration.primary_gpu.product_name,
            "driver": str(self.hardware_configuration.gpu_configuration.driver_version),
            "cuda": self.hardware_configuration.gpu_configuration.cuda_version,
            "python": omni.kit.app.get_app().get_platform_info()["python_version"],
        }

        h = hashlib.sha256(json.dumps(params, sort_keys=True).encode("utf-8")).hexdigest()[:8]
        return h
