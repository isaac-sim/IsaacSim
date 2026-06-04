# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UCX Bridge Extension.

This module provides Python bindings for UCX (Unified Communication X) functionality
in Isaac Sim, enabling high-performance, low-latency communication for distributed
simulation scenarios.
"""

from .bindings import _ucx_nodes  # noqa: F401

# Import the extension class to ensure it's registered
from .extension import UCXBridgeExtension  # noqa: F401

__all__ = []
