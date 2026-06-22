# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Read file contents from an OmniGraph path input into a string output."""

from os.path import exists
from typing import Any


class OgnIsaacReadFilePath:
    """look for file at path given, and return its contents."""

    @staticmethod
    def compute(db: Any) -> bool:
        """Populate `fileContents` from the input path, returning False for empty or missing paths.

        Args:
            db: OmniGraph database for this node.

        Returns:
            False when the input path is empty or missing.
        """
        # Empty input:
        db.outputs.fileContents = ""
        if len(db.inputs.path) == 0:
            db.log_warn("Empty input path, returning empty string.")
            return False
        elif not exists(db.inputs.path):
            db.log_warn(f"Could not find file at {db.inputs.path}, returning empty string.")
            return False
        else:
            with open(db.inputs.path) as f:
                db.outputs.fileContents = f.read()
