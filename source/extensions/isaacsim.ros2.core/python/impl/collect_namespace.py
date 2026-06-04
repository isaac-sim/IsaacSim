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


"""Utility for collecting ROS 2 namespaces from USD prim hierarchies."""

from isaacsim.core.rendering_manager import ViewportManager


def collect_namespace(namespace_input: str, render_product_path: str) -> str:
    """Collects the ROS 2 namespace from a USD Prim hierarchy by traversing upwards and appending any.

    'isaac:namespace' attributes found.

    If an input namespace is provided, it will be returned as-is without traversing the hierarchy.
    Otherwise, the function starts from the Camera prim associated with the render product and traverses
    upwards through the hierarchy, collecting all 'isaac:namespace' attributes.

    Args:
        namespace_input: An initial namespace. If this is non-empty, it will be returned as-is.
        render_product_path: The path of the render product, used to find the Camera prim associated with
            the render product.

    Returns:
        The accumulated namespace string.
    """
    # If the namespace_input is not empty, return it immediately
    if namespace_input:
        return namespace_input

    if not render_product_path:
        return ""

    namespace_string = ""

    start_prim = ViewportManager.get_camera(render_product_path).GetPrim()

    current_prim = start_prim

    # Traverse upwards until there are no more parents
    while current_prim.IsValid():

        # Retrieve the "isaac:namespace" attribute
        attr = current_prim.GetAttribute("isaac:namespace")
        namespace_value = ""

        # If the attribute has a value, append it to the namespace string
        if attr:
            namespace_value = attr.Get()
            if namespace_value:
                if namespace_string:
                    namespace_string = namespace_value + "/" + namespace_string
                else:
                    namespace_string = namespace_value

        # Move to the parent prim
        current_prim = current_prim.GetParent()

    return namespace_string
