# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Define Franka behavior modules and context state monitoring."""

import sys

if __name__ == "__main__":
    print(
        "This file is not meant to be executed. Please check the files at the root of `isaacsim.cortex.framework` for the main entry points."
    )
    sys.exit(0)

from isaacsim.cortex.behaviors.franka import (
    block_stacking_behavior,
    peck_decider_network,
    peck_game,
    peck_state_machine,
)
from isaacsim.cortex.behaviors.franka.simple import simple_decider_network, simple_state_machine
from isaacsim.cortex.framework.dfb import DfDiagnosticsMonitor

behaviors = {
    "block_stacking_behavior": block_stacking_behavior,
    "peck_decider_network": peck_decider_network,
    "peck_game": peck_game,
    "peck_state_machine": peck_state_machine,
    "simple_decider_network": simple_decider_network,
    "simple_state_machine": simple_state_machine,
}


class ContextStateMonitor(DfDiagnosticsMonitor):
    """State monitor to read the context and pass it to the UI.

    For these behaviors, the context has a `diagnostic_message` that contains the text to be displayed, and each
    behavior implements its own monitor to update that.
    """

    def __init__(self, print_dt, diagnostic_fn=None):
        super().__init__(print_dt=print_dt)

    def print_diagnostics(self, context):
        """Print diagnostic messages from the context."""
        if hasattr(context, "diagnostics_message"):
            print("====================================")
            print(context.diagnostics_message)
