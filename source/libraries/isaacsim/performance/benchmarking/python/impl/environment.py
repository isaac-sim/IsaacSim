# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Collect descriptive host context alongside benchmark measurements."""

from __future__ import annotations

import os
import platform
import sys
from typing import Any


def collect_environment() -> dict[str, Any]:
    """Return lightweight, non-evaluative host and interpreter metadata.

    Returns:
        Host, hardware, and Python interpreter metadata.

    """
    uname = platform.uname()
    return {
        "operating_system": {
            "system": uname.system,
            "release": uname.release,
            "version": uname.version,
        },
        "hardware": {
            "machine": uname.machine,
            "processor": uname.processor,
            "cpu_count": os.cpu_count(),
        },
        "python": {
            "implementation": platform.python_implementation(),
            "version": platform.python_version(),
            "executable": sys.executable,
        },
    }
