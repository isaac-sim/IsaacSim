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

"""Common utilities for grasping tests."""


def check_grasp_pose_generation_dependencies() -> bool:
    """Check if grasp sampler dependencies are met, used to skip tests if not.

    Returns:
        True if the dependencies are installed, False otherwise.
    """
    try:
        import rtree  # noqa: F401
    except Exception as e:
        print(f"Could not import 'rtree': {e}.")
        return False
    return True
