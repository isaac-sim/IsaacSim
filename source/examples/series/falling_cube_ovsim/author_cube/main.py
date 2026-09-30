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

"""Author a cube on a new stage."""

from __future__ import annotations

from isaacsim.ovsim.api import make_client


def main() -> None:
    """Create a stage and author a cube."""
    client = make_client("in-process")
    if not client.control.authoring.create_stage():
        raise RuntimeError("Stage creation failed.")
    try:
        cube = "/World/Cube"
        position = [0.0, 0.0, 2.0]
        client.control.authoring.define_prim(cube, "Cube")
        client.data.write(cube, "size", 1.0)
        client.data.write(cube, "position", position)

        print(f"Authored {cube} at ({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}).")
    finally:
        client.control.authoring.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
