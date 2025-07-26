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
from __future__ import annotations

import time
from typing import List, Optional, Tuple

import carb
import omni.kit.test


def get_last_gpu_time_ms(
    hydra_engine_stats: HydraEngineStats,
) -> Tuple[float, List[float]]:
    """
    Return the total RTX Renderer duration (in milliseconds) and per-GPU durations.
    """
    if hydra_engine_stats is None:
        return 0.0, []

    device_nodes = hydra_engine_stats.get_gpu_profiler_result()

    total_time = 0.0
    per_gpu_times = []

    for device_idx, device in enumerate(device_nodes):
        device_time = 0.0
        for node in device:
            if node["indent"] == 0:
                device_time += node["duration"]
        per_gpu_times.append(round(device_time, 6))
        total_time += device_time

    # For backward compatibility, return average for single total
    avg_time = round(total_time / len(device_nodes) if device_nodes else 0.0, 6)
    return avg_time, per_gpu_times


class IsaacUpdateFrametimeCollector:
    """
    Utility to collect
        app update time (in milliseconds)
        physics update time (in milliseconds)
        gpu frame time (in milliseconds)

    """

    def __init__(self, usd_context_name="", hydra_engine="rtx", gpu_frametime: bool = True) -> None:
        self.gpu_frametime = gpu_frametime
        self.hydra_engine_stats = None
        if self.gpu_frametime:
            try:
                from omni.hydra.engine.stats import HydraEngineStats

                self.hydra_engine_stats = HydraEngineStats(usd_context_name, hydra_engine)
            except Exception as e:
                carb.log_warn(f"Failed to initialize HydraEngineStats, GPU frametimes will not be measured: {e}")
                self.hydra_engine_stats = None

        try:
            import omni.physx

            self.__physx_benchmarks_iface = omni.physx.get_physx_benchmarks_interface()
        except:
            self.__physx_benchmarks_iface = None
            carb.log_warn("physx interface not loaded, physics frametimes will not be measured")

        self.app_frametimes_ms: List[float] = []
        self.gpu_frametimes_ms: List[float] = []
        self.per_gpu_frametimes_ms: List[List[float]] = []
        self.physics_frametimes_ms: List[float] = []
        self.render_frametimes_ms: List[float] = []

        self.__last_main_frametime_timestamp_ns = 0
        self.__last_render_frametime_timestamp_ns = 0

        self.__subscription: Optional[carb.events.ISubscription] = None
        self.__physx_subscription = None

        self.elapsed_sim_time = 0.0

    def __update_event_callback(self, event: carb.events.IEvent):
        timestamp_ns = time.perf_counter_ns()
        app_update_time_ms = round((timestamp_ns - self.__last_main_frametime_timestamp_ns) / 1000 / 1000, 6)
        self.__last_main_frametime_timestamp_ns = timestamp_ns
        if self.gpu_frametime:
            avg_gpu_frametime_ms, per_gpu_times = get_last_gpu_time_ms(self.hydra_engine_stats)
            self.gpu_frametimes_ms.append(avg_gpu_frametime_ms)

            # Initialize per-GPU lists if needed
            if len(self.per_gpu_frametimes_ms) != len(per_gpu_times):
                self.per_gpu_frametimes_ms = [[] for _ in range(len(per_gpu_times))]

            # Add per-GPU frametime data
            for i, gpu_time in enumerate(per_gpu_times):
                self.per_gpu_frametimes_ms[i].append(gpu_time)

        self.app_frametimes_ms.append(app_update_time_ms)
        self.elapsed_sim_time += event.payload["dt"]

    def __render_update_event_callback(self, event: carb.events.IEvent):
        timestamp_ns = time.perf_counter_ns()
        render_update_tims_ms = round((timestamp_ns - self.__last_render_frametime_timestamp_ns) / 1000 / 1000, 6)
        self.__last_render_frametime_timestamp_ns = timestamp_ns
        self.render_frametimes_ms.append(render_update_tims_ms)

    def __physics_stats_callback(self, profile_stats):
        if len(profile_stats) > 0:
            for stat in profile_stats:
                if stat.zone_name == "PhysX Update":
                    self.physics_frametimes_ms.append(stat.ms)

    def start_collecting(self):
        # reset our tracking variables
        self.app_frametimes_ms: List[float] = []
        self.gpu_frametimes_ms: List[float] = []
        self.per_gpu_frametimes_ms: List[List[float]] = []
        self.physics_frametimes_ms: List[float] = []
        self.render_frametimes_ms: List[float] = []
        self.__last_main_frametime_timestamp_ns = time.perf_counter_ns()
        self.__last_render_frametime_timestamp_ns = time.perf_counter_ns()

        self.__subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_PRE_UPDATE,
            on_event=self.__update_event_callback,
            observer_name="IsaacUpdateFrametimeCollector.__update_event_callback",
        )

        self.__physx_subscription = self.__physx_benchmarks_iface.subscribe_profile_stats_events(
            self.__physics_stats_callback
        )

        self.__pre_render_thread_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=f"runloop:rendering_0:update",
            on_event=self.__render_update_event_callback,
            observer_name="IsaacUpdateFrametimeCollector.__render_update_event_callback",
        )

        self.elapsed_sim_time = 0.0

    def stop_collecting(self) -> Tuple[List[float], List[float], List[float]]:
        self.__subscription = None
        self.__physx_subscription = None

        # drop the first frame since the interval approach doesn't work for
        # the render frame
        if len(self.app_frametimes_ms) > 0:
            self.app_frametimes_ms.pop(0)
        if len(self.gpu_frametimes_ms) > 0:
            self.gpu_frametimes_ms.pop(0)
        for per_gpu_list in self.per_gpu_frametimes_ms:
            if len(per_gpu_list) > 0:
                per_gpu_list.pop(0)
        if len(self.physics_frametimes_ms) > 0:
            self.physics_frametimes_ms.pop(0)
        if len(self.render_frametimes_ms) > 0:
            self.render_frametimes_ms.pop(0)

        # convert s to ms for consistency
        self.elapsed_sim_time *= 1000
        return self.app_frametimes_ms, self.gpu_frametimes_ms, self.physics_frametimes_ms, self.render_frametimes_ms
