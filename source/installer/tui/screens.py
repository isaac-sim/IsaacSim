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

"""Run-argument and setup dialogs."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import Button, Input, Label, Static


class ArgumentsScreen(ModalScreen[tuple[str, ...] | None]):
    """Edit ordered run arguments as discrete strings.

    Each row is one exact argument. Empty rows are preserved, as are duplicate,
    Unicode, and option-looking values.

    Args:
        example_id: Stable example ID to run.
    """

    BINDINGS = [
        ("escape", "cancel", "Cancel"),
        Binding("ctrl+a", "add_argument", "Add argument", priority=True),
        ("ctrl+enter", "submit", "Run"),
    ]

    def __init__(self, example_id: str) -> None:
        super().__init__()
        self._example_id = example_id

    def compose(self) -> ComposeResult:
        """Compose the ordered argument editor."""
        with Vertical(classes="dialog arguments-dialog"):
            yield Label("Run arguments", classes="dialog-title")
            yield Static(self._example_id, id="argument-example", markup=False)
            yield Static("One argument per row · spaces stay together", id="argument-help")
            with VerticalScroll(id="argument-rows"):
                yield Static("No extra arguments. This run will use the example defaults.", id="argument-empty")
            with Horizontal(classes="argument-tools"):
                yield Button("+ Add argument (Ctrl+A)", id="add")
            with Horizontal(classes="dialog-actions"):
                yield Button("Cancel (Esc)", id="cancel")
                yield Button("Run (Ctrl+Enter)", id="submit")

    def on_mount(self) -> None:
        """Start at Add so custom arguments are immediately accessible."""
        self.query_one("#add", Button).focus()

    def action_submit(self) -> None:
        """Return the exact ordered argument values to the caller."""
        self.dismiss(tuple(field.value for field in self.query(".argument-value").nodes))

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        """Handle argument editor controls.

        Args:
            event: Pressed button event.
        """
        event.stop()
        button_id = event.button.id
        if button_id == "cancel":
            self.dismiss(None)
        elif button_id == "submit":
            self.action_submit()
        elif button_id == "add":
            await self.action_add_argument()
        elif event.button.has_class("remove-argument"):
            row = event.button.parent
            if row is not None:
                await row.remove()
                for number, remaining in enumerate(self.query(".argument-row"), 1):
                    remaining.query_one(".argument-number", Static).update(str(number))
                self.query_one("#argument-empty").display = not bool(self.query(".argument-row"))
                self.query_one("#add", Button).focus()

    async def action_add_argument(self) -> None:
        """Append and focus one exact argument field."""
        self.query_one("#argument-empty").display = False
        row = Horizontal(
            Static(str(len(self.query(".argument-row")) + 1), classes="argument-number"),
            Input(placeholder="Argument value", classes="argument-value"),
            Button("Remove", classes="remove-argument"),
            classes="argument-row",
        )
        await self.query_one("#argument-rows", VerticalScroll).mount(row)
        row.query_one(Input).focus()

    def action_cancel(self) -> None:
        """Cancel run argument entry."""
        self.dismiss(None)


class SetupScreen(ModalScreen[None]):
    """Explain missing execution prerequisites without leaving the browser.

    Args:
        message: Brief backend-owned prerequisite description.
        instructions: Actionable steps to prepare the environment.
    """

    BINDINGS = [("escape", "close", "Close")]

    def __init__(self, message: str, instructions: str) -> None:
        super().__init__()
        self._message = message
        self._instructions = instructions

    def compose(self) -> ComposeResult:
        """Keep the recovery command visible in a scrollable setup dialog."""
        with Vertical(classes="dialog"):
            yield Label("Setup required", classes="dialog-title")
            with VerticalScroll():
                yield Static(self._message + "\n\n" + self._instructions, markup=False)
            yield Button("Back to examples", id="close", variant="primary")

    def on_mount(self) -> None:
        """Focus the return action for immediate keyboard dismissal."""
        self.query_one("#close", Button).focus()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Return to the browser without starting an operation."""
        event.stop()
        self.dismiss()

    def action_close(self) -> None:
        """Dismiss setup guidance."""
        self.dismiss()
