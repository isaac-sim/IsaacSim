# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides a base class for sensor implementations in Isaac Sim."""

from __future__ import annotations

from collections.abc import Sequence

from isaacsim.core.prims import SingleXFormPrim


class BaseSensor(SingleXFormPrim):
    """Provides common properties and methods to deal with prims as a sensor.

    .. note::

        This class, which inherits from ``SingleXFormPrim``, does not currently add any new property/method to it.
        Its definition is oriented to future implementations.

    Args:
        prim_path: Prim path of the Prim to encapsulate or create.
        name: Shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        position: Position in the world frame of the prim. shape is (3, ).
        translation: Translation in the local frame of the prim
            (with respect to its parent prim). shape is (3, ).
        orientation: Quaternion orientation in the world/ local frame of the prim
            (depends if translation or position is specified).
            quaternion is scalar-first (w, x, y, z). shape is (4, ).
        scale: Local scale to be applied to the prim's dimensions. shape is (3, ).
        visible: Set to false for an invisible prim in the stage while rendering.

    Raises:
        Exception: If translation and position defined at the same time.

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "base_sensor",
        position: Sequence[float] | None = None,
        translation: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        scale: Sequence[float] | None = None,
        visible: bool | None = None,
    ) -> None:
        SingleXFormPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
        )
        return

    def initialize(self, physics_sim_view: object = None) -> None:
        """Create a physics simulation view if not passed and using PhysX tensor API.

        .. note::

            If the prim has been added to the world scene (e.g., ``world.scene.add(prim)``),
            it will be automatically initialized when the world is reset (e.g., ``world.reset()``).

        Args:
            physics_sim_view: current physics simulation view.

        Example:

        .. code-block:: python

            >>> prim.initialize()

        """
        SingleXFormPrim.initialize(self, physics_sim_view=physics_sim_view)
        return

    def post_reset(self) -> None:
        """Resets the sensor to its initial state after a simulation reset."""
        SingleXFormPrim.post_reset(self)
        return
