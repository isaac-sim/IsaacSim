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
import time
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import psutil

if TYPE_CHECKING:
    from isaacsim.benchmark.services.settings import BenchmarkSettings

import carb
import omni.kit.app as omni_kit_app
from isaacsim.benchmark.services.datarecorders import cpu, frametime, interface, memory
from isaacsim.benchmark.services.metrics import measurements

from .collectors import IsaacUpdateFrametimeCollector


class IsaacFrameTimeRecorder(interface.MeasurementDataRecorder):
    def __init__(
        self,
        context: Optional[interface.InputContext] = None,
        root_dir: Optional[Path] = None,
        benchmark_settings: Optional["BenchmarkSettings"] = None,
        gpu_frametime: Optional[bool] = False,
    ):
        self.context = context
        self.root_dir = root_dir
        self.benchmark_settings = benchmark_settings
        self.gpu_frametime = gpu_frametime
        self.frametime_collector = IsaacUpdateFrametimeCollector(gpu_frametime=self.gpu_frametime)
        self.phase = None

        self.real_time_start = None
        self.elapsed_real_time = None

    def start_collecting(self):
        self.phase = self.context.phase
        self.real_time_start = time.perf_counter_ns()
        self.frametime_collector.start_collecting()

    def stop_collecting(self):
        if self.real_time_start is None:
            # Frametime collection never began, so skip.
            return
        self.elapsed_real_time = (time.perf_counter_ns() - self.real_time_start) / 1000000
        self.frametime_collector.stop_collecting()

    def get_data(self):
        if self.phase != self.context.phase:
            return interface.MeasurementData(measurements=[])

        frametime_stats = frametime.FrametimeStats()
        frametime_stats.app_frametime_samples = self.frametime_collector.app_frametimes_ms
        frametime_stats.physics_frametime_samples = self.frametime_collector.physics_frametimes_ms
        frametime_stats.gpu_frametime_samples = self.frametime_collector.gpu_frametimes_ms
        frametime_stats.renderer_frametime_samples = self.frametime_collector.render_frametimes_ms

        # Set per-GPU samples for multi-GPU support
        if self.gpu_frametime and self.frametime_collector.per_gpu_frametimes_ms:
            frametime_stats.per_gpu_frametime_samples = self.frametime_collector.per_gpu_frametimes_ms

        frametime_stats.calc_stats()

        measurements_out = []

        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Mean App_Update Frametime",
                value=frametime_stats.app_stats["mean"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Stdev App_Update Frametime",
                value=frametime_stats.app_stats["stdev"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Min App_Update Frametime",
                value=frametime_stats.app_stats["min"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Max App_Update Frametime",
                value=frametime_stats.app_stats["max"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.ListMeasurement(
                name=f"App_Update Frametime Samples",
                value=frametime_stats.app_frametime_samples,
            )
        )

        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Mean Physics Frametime",
                value=frametime_stats.physics_stats["mean"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Stdev Physics Frametime",
                value=frametime_stats.physics_stats["stdev"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Min Physics Frametime",
                value=frametime_stats.physics_stats["min"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Max Physics Frametime",
                value=frametime_stats.physics_stats["max"],
                unit="ms",
            )
        )
        measurements_out.append(
            measurements.ListMeasurement(
                name=f"Physics Frametime Samples",
                value=frametime_stats.physics_frametime_samples,
            )
        )

        if self.gpu_frametime:
            measurements_out.append(
                measurements.SingleMeasurement(
                    name=f"Mean GPU Frametime", value=frametime_stats.gpu_stats["mean"], unit="ms"
                )
            )

            measurements_out.append(
                measurements.SingleMeasurement(
                    name=f"Stdev GPU Frametime", value=frametime_stats.gpu_stats["stdev"], unit="ms"
                )
            )

            measurements_out.append(
                measurements.SingleMeasurement(
                    name=f"Min GPU Frametime", value=frametime_stats.gpu_stats["min"], unit="ms"
                )
            )

            measurements_out.append(
                measurements.SingleMeasurement(
                    name=f"Max GPU Frametime", value=frametime_stats.gpu_stats["max"], unit="ms"
                )
            )

            measurements_out.append(
                measurements.ListMeasurement(name=f"GPU Frametime Samples", value=frametime_stats.gpu_frametime_samples)
            )

            # Add per-GPU measurements for multi-GPU setups
            if len(frametime_stats.per_gpu_stats) > 1:
                for gpu_idx, gpu_stats in enumerate(frametime_stats.per_gpu_stats):
                    if gpu_stats and gpu_stats["mean"] > 0:
                        measurements_out.append(
                            measurements.SingleMeasurement(
                                name=f"Mean GPU{gpu_idx} Frametime", value=gpu_stats["mean"], unit="ms"
                            )
                        )
                        measurements_out.append(
                            measurements.SingleMeasurement(
                                name=f"Stdev GPU{gpu_idx} Frametime", value=gpu_stats["stdev"], unit="ms"
                            )
                        )
                        measurements_out.append(
                            measurements.SingleMeasurement(
                                name=f"Min GPU{gpu_idx} Frametime", value=gpu_stats["min"], unit="ms"
                            )
                        )
                        measurements_out.append(
                            measurements.SingleMeasurement(
                                name=f"Max GPU{gpu_idx} Frametime", value=gpu_stats["max"], unit="ms"
                            )
                        )
                        measurements_out.append(
                            measurements.ListMeasurement(
                                name=f"GPU{gpu_idx} Frametime Samples",
                                value=frametime_stats.per_gpu_frametime_samples[gpu_idx],
                            )
                        )

        if frametime_stats.renderer_stats["mean"]:
            measurements_out.append(
                measurements.SingleMeasurement(
                    name=f"Mean Render Frametime",
                    value=frametime_stats.renderer_stats["mean"],
                    unit="ms",
                )
            )
            measurements_out.append(
                measurements.SingleMeasurement(
                    name=f"Stdev Render Frametime",
                    value=frametime_stats.renderer_stats["stdev"],
                    unit="ms",
                )
            )
            measurements_out.append(
                measurements.SingleMeasurement(
                    name=f"Min Render Frametime",
                    value=frametime_stats.renderer_stats["min"],
                    unit="ms",
                )
            )
            measurements_out.append(
                measurements.SingleMeasurement(
                    name=f"Max Render Frametime",
                    value=frametime_stats.renderer_stats["max"],
                    unit="ms",
                )
            )
            measurements_out.append(
                measurements.SingleMeasurement(
                    name=f"Rendering FPS",
                    value=round(1000 / (frametime_stats.renderer_stats["mean"]), 3),
                    unit="FPS",
                )
            )

        if frametime_stats.app_stats["mean"]:
            # Scripts which don't trigger app update will have empty app stats, so app_stars["mean"] will never
            # be populated, and this will trigger a divison-by-zero error.
            measurements_out.append(
                measurements.SingleMeasurement(
                    name=f"Mean FPS", value=round(1000 / (frametime_stats.app_stats["mean"]), 3), unit="FPS"
                )
            )

        measurements_out.append(
            measurements.SingleMeasurement(
                name=f"Real Time Factor",
                value=round(self.frametime_collector.elapsed_sim_time / self.elapsed_real_time, 3),
                unit="",
            )
        )
        return interface.MeasurementData(measurements=measurements_out)


class IsaacMemoryRecorder(memory.MemoryRecorder):
    def __init__(
        self,
        context: Optional[interface.InputContext] = None,
        root_dir: Optional[Path] = None,
        benchmark_settings: Optional["BenchmarkSettings"] = None,
    ):
        self.context = context
        self.root_dir = root_dir
        self.benchmark_settings = benchmark_settings
        super().__init__(context, root_dir, benchmark_settings)

    def get_data(self) -> interface.MeasurementData:
        (
            rss,
            vms,
            uss,
            pb,
            tracked_gpu_memory,
            dedicated_gpu_memory,
        ) = self.get_hardware_stats()

        m1 = measurements.SingleMeasurement(name=f"System Memory RSS", value=rss, unit="GB")
        m2 = measurements.SingleMeasurement(name=f"System Memory VMS", value=vms, unit="GB")
        m3 = measurements.SingleMeasurement(name=f"System Memory USS", value=uss, unit="GB")
        m4 = measurements.SingleMeasurement(name=f"GPU Memory Tracked", value=tracked_gpu_memory, unit="GB")
        m5 = measurements.SingleMeasurement(name=f"GPU Memory Dedicated", value=dedicated_gpu_memory, unit="GB")
        measurements_out = [m1, m2, m3, m4, m5]

        # Only capture System Memory PB for Windows.
        # if platform.system() == "Windows":
        #     measurements_out.append(
        #         measurements.SingleMeasurement(name=f"System Memory PB", value=pb, unit="GB")
        #     )

        return interface.MeasurementData(measurements=measurements_out)


class IsaacCPUStatsRecorder(cpu.CPUStatsRecorder):
    def __init__(
        self,
        context: Optional[interface.InputContext] = None,
        root_dir: Optional[Path] = None,
        benchmark_settings: Optional["BenchmarkSettings"] = None,
    ):
        self.context = context
        self.root_dir = root_dir
        self.benchmark_settings = benchmark_settings
        super().__init__(context, root_dir, benchmark_settings)

    def get_data(self) -> interface.MeasurementData:
        (
            cpu_iowait_pct,
            cpu_system_pct,
            cpu_user_pct,
            cpu_idle_pct,
        ) = cpu.get_cpu_usage_in_pct(self.cpu_iowait, self.cpu_system, self.cpu_user, self.cpu_idle)

        m1 = measurements.SingleMeasurement(name=f"System CPU iowait", value=cpu_iowait_pct, unit="%")
        m2 = measurements.SingleMeasurement(name=f"System CPU system", value=cpu_system_pct, unit="%")
        m3 = measurements.SingleMeasurement(name=f"System CPU user", value=cpu_user_pct, unit="%")
        m4 = measurements.SingleMeasurement(name=f"System CPU idle", value=cpu_idle_pct, unit="%")

        return interface.MeasurementData(measurements=[m1, m2, m3, m4])


class IsaacRuntimeRecorder(interface.MeasurementDataRecorder):
    def __init__(
        self,
        context: Optional[interface.InputContext] = None,
        root_dir: Optional[Path] = None,
        benchmark_settings: Optional["BenchmarkSettings"] = None,
    ):
        self.context = context
        self.root_dir = root_dir
        self.benchmark_settings = benchmark_settings
        self.start = 0.0
        self.elapsed_time = None
        self.phase = None

    def start_time(self):
        self.phase = self.context.phase
        self.start = omni_kit_app.get_app().get_time_since_start_ms()

    def stop_time(self):
        if self.phase is None:
            self.phase = self.context.phase
        self.elapsed_time = omni_kit_app.get_app().get_time_since_start_ms() - self.start

    def get_data(self):
        if self.phase != self.context.phase:
            return interface.MeasurementData(measurements=[])

        m1 = measurements.SingleMeasurement(name=f"Runtime", value=round(self.elapsed_time, 3), unit="ms")
        return interface.MeasurementData(measurements=[m1])


class IsaacHardwareSpecRecorder(interface.MeasurementDataRecorder):
    def __init__(self, context: Optional[interface.InputContext] = None):
        self.context = context

    def get_data(self):
        import torch

        device_names = [torch.cuda.get_device_name(d) for d in range(torch.cuda.device_count())]

        if len(set(device_names)) > 1:
            carb.log_warn(f"Detected multiple GPU types: {device_names}.")
            carb.log_warn(f"Only recording GPU 0 type: {device_names[0]}")

        measurements_out = []

        measurements_out.append(measurements.SingleMeasurement(name=f"num_cpus", value=psutil.cpu_count(), unit=""))
        measurements_out.append(measurements.SingleMeasurement(name=f"gpu_device_name", value=device_names[0], unit=""))

        return interface.MeasurementData(measurements_out)
