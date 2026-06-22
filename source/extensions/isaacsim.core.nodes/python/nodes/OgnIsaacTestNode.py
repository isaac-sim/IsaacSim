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


"""Echo a string input to an output for simple OmniGraph test graphs."""

from typing import Any


class OgnIsaacTestNode:
    """Test Isaac Sim Node."""

    @staticmethod
    def compute(db: Any) -> bool:
        """Copy the input value to the output and report successful execution.

        Args:
            db: OmniGraph database for this node.

        Returns:
            True after the value is copied.
        """
        # copy input to output
        db.outputs.output = db.inputs.input
        return True
