# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run the C++ with Python Hello World example."""

import _hello_world_cpp


def main() -> None:
    """Call the example-local C++ binding."""
    _hello_world_cpp.say_hello()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
