# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Run the Python Hello World example."""

from isaacsim.common.logging import Logger


def main() -> None:
    """Print the Hello World result through the installed Isaac Sim Python API."""
    logger = Logger("isaacsim.examples.hello_world.python")
    logger.report("Hello World from Python.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
