# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""UI extension for the Episode Recorder system.

Provides the standalone :class:`EpisodeRecorderWindow` (``Tools > Replicator >
Episode Recorder``) and the underlying :class:`EpisodeRecorderPanel`. The
panel can be embedded into other windows if needed.
"""

from __future__ import annotations

from .episode_recorder_extension import EpisodeRecorderUIExtension
from .episode_recorder_panel import EpisodeRecorderPanel
from .episode_recorder_window import EpisodeRecorderWindow

__all__ = [
    "EpisodeRecorderPanel",
    "EpisodeRecorderUIExtension",
    "EpisodeRecorderWindow",
]
