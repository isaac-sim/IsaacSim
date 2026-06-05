# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Import the implementation modules that will be externally visible.

The extension object must be visible so that this module properly starts up and shuts down.
The Python bindings are all imported so that they can be used in the omni.graph.examples.cpp import space.
Everything else is explicitly imported for visibility in the omni.graph.examples.cpp import space.
"""

# One line per import is used to make them easier to read and find, grouped by originating file
from .extension import Extension as Extension
