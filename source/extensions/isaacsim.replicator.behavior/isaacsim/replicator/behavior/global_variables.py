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

# Extension name used for event prefixing (e.g. "isaacsim.replicator.behavior.my_event")
# https://docs.omniverse.nvidia.com/dev-guide/latest/programmer_ref/events.html

"""Global constants for the replicator behavior extension."""

EXTENSION_NAME = "isaacsim.replicator.behavior"

# The namespace of the exposed variables used in the randomizer scripts (e.g. "exposedVar:behaviorScriptName:group:attrName")
EXPOSED_ATTR_NS = "exposedVar"

# The scope name to keep the created assets in
SCOPE_NAME = "/Behaviors"

# Event dispatched when exposed USD variables are created or removed. The UI extension subscribes to this
# event to refresh the property window. Using an event keeps the core extension decoupled from any UI module.
EXPOSED_VARS_CHANGED_EVENT = f"{EXTENSION_NAME}.EXPOSED_VARS_CHANGED"
