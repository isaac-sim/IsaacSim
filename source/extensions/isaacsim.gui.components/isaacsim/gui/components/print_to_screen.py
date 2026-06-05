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

"""Provides utilities for displaying text on the viewport screen using OmniGraph visualization nodes."""

import numpy as np
import omni
import omni.graph.core as og
from omni.usd.commands import DeletePrimsCommand


class ScreenPrinter:
    """Print text to the viewport using the omni.graph.visualization.nodes.DrawScreenSpaceText node.

    Args:
        screen_pos_x: X position of the text on the screen, given as a percent of screen width with 0 referring
            to the left hand side.
        screen_pos_y: Y position of the text on the screen, given as a percent of screen width with 0 referring
            to the top.
        text_size: Size of text.
        max_width: Maximum width of text before wrapping around and continuing on a new line. A value of 0 means
            there is no wraparound.
        color: Color of text, given in a (4x1) np.array of the form [r,g,b,luminosity]. All four values have
            a minimum of 0.0 and a maximum of 2.0.
    """

    def __init__(
        self,
        screen_pos_x: float = 78,
        screen_pos_y: float = 95,
        text_size: float = 14.0,
        max_width: int = 0,
        color: np.ndarray | None = None,
    ) -> None:
        if color is None:
            color = np.ones(4)

        self._keys = og.Controller.Keys
        self._controller = og.Controller()

        self._graph_path = omni.usd.get_stage_next_free_path(
            omni.usd.get_context().get_stage(), "/World/PrintActionGraph", False
        )

        self.graph, self.nodes, _, _ = self._controller.edit(
            {"graph_path": self._graph_path, "evaluator_name": "push"},
            {
                self._keys.CREATE_NODES: [
                    ("tick", "omni.graph.action.OnTick"),
                    ("print_to_screen", "omni.graph.visualization.nodes.DrawScreenSpaceText"),
                ],
                self._keys.CONNECT: [("tick.outputs:tick", "print_to_screen.inputs:execIn")],
                self._keys.SET_VALUES: [
                    ("print_to_screen.inputs:position", [screen_pos_x, screen_pos_y]),
                    ("print_to_screen.inputs:text", ""),
                    ("print_to_screen.inputs:size", text_size),
                    ("print_to_screen.inputs:boxWidth", max_width),
                    ("print_to_screen.inputs:textColor", color),
                ],
            },
        )

        self._print_node = self.nodes[1]

    def set_text(self, text: str) -> None:
        """Set the text on the screen.

        Args:
            text: Text to appear on the screen.
        """
        self._controller.edit(self.graph, {self._keys.SET_VALUES: (("inputs:text", self._print_node), text)})

    def set_text_position(self, screen_pos_x: float, screen_pos_y: float) -> None:
        """Set the x,y position of the text on the screen.

        Args:
            screen_pos_x: X position of the text on the screen, given as a percent of screen width with 0 refering to the
                left hand side.
            screen_pos_y: Y position of the text on the screen, given as a percent of screen width with 0 refering to the
                top.
        """
        self._controller.edit(
            self.graph, {self._keys.SET_VALUES: (("inputs:position", self._print_node), [screen_pos_x, screen_pos_y])}
        )

    def set_text_size(self, size: float) -> None:
        """Set the size of the text.

        Args:
            size: Pixel height of a line of text.
        """
        self._controller.edit(self.graph, {self._keys.SET_VALUES: (("inputs:size", self._print_node), size)})

    def set_text_max_width(self, max_width: int) -> None:
        """Set the maximum text width (in pixels) before wrap-around.

        Args:
            max_width: Maximum width of text before wrapping around and continuing on a new line. A value of 0 means
                there is no wrap-around.
        """
        self._controller.edit(self.graph, {self._keys.SET_VALUES: (("inputs:boxWidth", self._print_node), max_width)})

    def set_text_color(self, color4f: np.ndarray) -> None:
        """Set the color of the text.

        Args:
            color4f: Color of text, given in a (4x1) np.array of the form [r,g,b,luminocity]. All four values have a
                minimum of 0.0 and a maximum of 2.0.
        """
        self._controller.edit(self.graph, {self._keys.SET_VALUES: (("inputs:textColor", self.nodes[1]), color4f)})

    def clear_text(self) -> None:
        """Clear the text from the screen."""
        self.set_text("")

    def exit(self) -> None:
        """Delete OmniGraph used by this ScreenPrinter. After calling ``exit()``, all subsequent function calls will fail."""
        DeletePrimsCommand([self._graph_path]).do()
