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

"""Run the mixed C++ and Python JSON pipeline example."""

import json

import _json_pipeline_cpp
import jsonpointer


def main() -> None:
    """Build JSON in C++ and select its values from Python."""
    document = json.loads(_json_pipeline_cpp.build_document())
    name = jsonpointer.resolve_pointer(document, "/robot/name")
    joints = jsonpointer.resolve_pointer(document, "/robot/joints")
    print(f"Robot {name} has {joints} joints.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
