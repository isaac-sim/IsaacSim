# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Author a cube on a new stage."""

from __future__ import annotations

from isaacsim.foundation.objects import Cube, Stage


def main() -> None:
    """Create a stage and author a cube."""
    stage = Stage("openusd").create_stage()
    try:
        position = [0.0, 0.0, 2.0]
        Cube("/World/Cube", sizes=1.0).set_world_poses(position)
        print(f"Authored /World/Cube at ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}).")
    finally:
        stage.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
