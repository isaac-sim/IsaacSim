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

"""Example browsing and actions independent of checkout or installer implementation."""

from __future__ import annotations

import asyncio
import contextlib

from isaacsim_installer.core.backend import ExamplesBackend, prepare_and_build
from isaacsim_installer.core.models import Example, Readiness
from isaacsim_installer.tui.screens import ArgumentsScreen, SetupScreen
from isaacsim_installer.tui.widgets import CatalogBrowser, CatalogList, set_enter_hint
from rich.text import Text
from textual.app import App, ComposeResult
from textual.containers import Horizontal
from textual.widgets import Button, Footer, RichLog, Static


class ExamplesApp(App[None]):
    """Browse and operate on examples through an injected backend.

    Args:
        backend: Discovery and execution provider; UI code knows no workspace paths.
    """

    TITLE = "Isaac Sim Examples"
    CSS_PATH = "theme.tcss"
    ENABLE_COMMAND_PALETTE = False
    BINDINGS = [
        ("b", "operate('build')", "Build"),
        ("r", "operate('run')", "Run"),
        ("a", "arguments", "Arguments"),
        ("t", "operate('test')", "Test"),
        ("escape", "escape", "Quit / cancel"),
        ("q", "quit", "Quit"),
    ]

    def __init__(self, backend: ExamplesBackend) -> None:
        super().__init__()
        self.backend = backend
        self.selected: Example | None = None
        self._built_examples: set[str] = set()
        self._readiness: Readiness | None = None
        self._task: asyncio.Task[None] | None = None
        self._busy = False
        self._quitting = False
        self._pending_output = ""
        self._pending_carriage_return = False

    def compose(self) -> ComposeResult:
        """Compose the retained browser with backend-neutral controls."""
        yield Static("ISAAC SIM / Examples", id="brand", markup=False)
        yield Static(self.backend.label, id="context", markup=False)
        yield CatalogBrowser()
        with Horizontal(id="actions"):
            yield Button("Build (b)", id="build", disabled=True)
            yield Button("Run (r)", id="run", disabled=True)
            yield Button("Test (t)", id="test", disabled=True)
            yield Button("Args (a)", id="arguments", disabled=True)
            yield Button("Cancel (Esc)", id="cancel", disabled=True)
        yield Static("Loading examples…", id="status", markup=False)
        yield RichLog(id="output", max_lines=1000, wrap=True, markup=False, highlight=False)
        yield Footer()

    def on_mount(self) -> None:
        """Discover asynchronously so navigation remains responsive."""
        self.query_one(CatalogBrowser).disabled = True
        self._set_busy(True)
        self._task = asyncio.create_task(self._discover())

    def _set_busy(self, busy: bool) -> None:
        """Update the operation lock and corresponding action availability."""
        self._busy = busy
        self._update_actions()

    def _update_actions(self) -> None:
        """Derive buttons and Enter hints from readiness, selection, and active work."""
        capabilities = self.backend.capabilities(self.selected) if self.selected is not None else frozenset()
        ready = self._readiness is not None and self._readiness.ready
        can_prepare = self._readiness is not None and not ready and self._readiness.can_prepare
        for action in ("build", "run", "test"):
            self.query_one(f"#{action}", Button).disabled = self._busy or not ready or action not in capabilities
        build = self.query_one("#build", Button)
        if can_prepare:
            build.disabled = self._busy or "build" not in capabilities
        built = self.selected is not None and self.selected.id in self._built_examples
        build.variant = "primary" if not build.disabled and (can_prepare or not built) else "default"
        run = self.query_one("#run", Button)
        run.disabled = run.disabled or not built
        run.variant = "success" if not run.disabled else "default"
        self.query_one("#arguments", Button).disabled = run.disabled
        self.query_one("#cancel", Button).disabled = not self._busy or self._quitting
        for catalog_list in self.query(CatalogList):
            set_enter_hint(catalog_list, "select_cursor", "Run" if built and not can_prepare else "Build")
        labels = {"build": "Build", "run": "Run", "test": "Test", "cancel": "Cancel", "arguments": "Arguments"}
        for button in self.query("#actions Button"):
            set_enter_hint(button, "press", labels[button.id])

    def _set_status(self, text: str) -> None:
        """Replace the status message without interpreting markup."""
        self.query_one("#status", Static).update(text)

    async def _discover(self) -> None:
        """Load the catalog and disk readiness while keeping the UI responsive."""
        self._set_status("Loading examples…")
        try:
            catalog = await self.backend.discover()
            self.query_one(CatalogBrowser).disabled = False
            self.query_one(CatalogBrowser).set_catalog(catalog)
            self._readiness = await self.backend.check_ready()
            if self._readiness.ready:
                self._built_examples.update(self._readiness.runnable_examples or ())
                self._set_status(f"{len(catalog.examples)} examples · Select an example to build, run, or test.")
            else:
                self._built_examples.clear()
                self._show_readiness()
            self.call_after_refresh(self.query_one(CatalogBrowser).focus_results)
        except asyncio.CancelledError:
            self._set_status("Discovery cancelled.")
        except Exception as error:
            self._set_status(f"Discovery failed: {error}")
            self._emit(str(error) + "\n")
        finally:
            self._set_busy(False)

    def on_catalog_browser_selected(self, event: CatalogBrowser.Selected) -> None:
        """Update available operations when the browser selection changes."""
        if self.query_one(CatalogBrowser).disabled:
            return
        self.selected = event.record
        self._update_actions()

    def on_catalog_browser_activated(self, event: CatalogBrowser.Activated) -> None:
        """Invoke Build or Run for an activated example, matching the primary button."""
        self.selected = event.record
        action = "run" if event.record.id in self._built_examples else "build"
        self.action_operate(action)

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """Dispatch toolbar actions without coupling widgets to a backend."""
        if event.button.id == "cancel":
            self.action_cancel()
        elif event.button.id == "arguments":
            self.action_arguments()
        elif event.button.id in {"build", "run", "test"}:
            self.action_operate(event.button.id)

    def action_operate(self, action: str) -> None:
        """Start one supported action for the current example."""
        if self._busy or self.selected is None:
            return
        if action not in self.backend.capabilities(self.selected):
            return
        if action != "build" and self._readiness and not self._readiness.ready and self._readiness.can_prepare:
            return
        if action == "run" and self.selected.id not in self._built_examples:
            self._set_status("Build the selected example first.")
            return
        example_id = self.selected.id
        self._set_busy(True)
        self._task = asyncio.create_task(self._prepare_action(action, example_id))

    async def _prepare_action(self, action: str, example_id: str, arguments: tuple[str, ...] = ()) -> None:
        """Check prerequisites before starting an operation in the output pane."""
        self._set_status("Checking example prerequisites…")
        try:
            readiness = await self.backend.check_ready()
            self._readiness = readiness
            if readiness.runnable_examples is not None:
                self._built_examples = set(readiness.runnable_examples)
            if not readiness.ready:
                self._built_examples.clear()
                if action == "build" and readiness.can_prepare:
                    await self._prepare_backend(example_id)
                else:
                    self._show_readiness()
                if not readiness.can_prepare:
                    self.push_screen(SetupScreen(readiness.message, readiness.instructions))
            elif action == "run":
                if example_id not in self._built_examples:
                    self._set_status("The selected example needs a build. Select Build (b).")
                else:
                    await self._execute("run", example_id, arguments=arguments)
            else:
                await self._execute(action, example_id)
        except asyncio.CancelledError:
            self._set_status("Prerequisite check cancelled.")
        except Exception as error:
            self._set_status(f"Could not check prerequisites: {error}")
            self._emit(str(error) + "\n")
        finally:
            self._set_busy(False)

    def _show_readiness(self) -> None:
        """Display prerequisite recovery guidance from the active backend."""
        if self._readiness is not None:
            status = (
                "Build required before running examples. Select Build (b)."
                if self._readiness.can_prepare
                else self._readiness.message
            )
            self._set_status(status)

    def _clear_output(self) -> None:
        """Reset the log and partial-line state for a new operation."""
        self.query_one("#output", RichLog).clear()
        self._pending_output = ""
        self._pending_carriage_return = False

    def _flush_output(self) -> None:
        """Publish any remaining partial line, including a trailing carriage return."""
        if self._pending_output or self._pending_carriage_return:
            self._emit("\n")

    async def _prepare_backend(self, example_id: str) -> None:
        """Build prerequisites and the originally selected example in one operation."""
        self._built_examples.clear()
        self._clear_output()
        self.query_one("#output", RichLog).border_title = f"Build · {example_id}"
        self._set_status(f"Building · {example_id} · Esc to cancel")
        try:
            code, self._readiness = await prepare_and_build(self.backend, example_id, self._emit)
            if self._readiness.ready:
                if self._readiness.runnable_examples is not None:
                    self._built_examples = set(self._readiness.runnable_examples)
                elif code == 0:
                    self._built_examples.add(example_id)
            if code != 0:
                self._set_status(f"Build failed · exit {code}. Review the output and retry.")
            elif self._readiness.ready:
                self._set_status(f"Build completed · exit {code} · {example_id}")
            else:
                self._show_readiness()
                self._emit(self._readiness.details + "\n")
        except asyncio.CancelledError:
            self._set_status("Build cancelled. You can retry when ready.")
        except Exception as error:
            self._set_status(f"Build failed: {error}")
            self._emit(str(error) + "\n")
        finally:
            self._flush_output()

    async def _execute(self, action: str, example_id: str, *, arguments: tuple[str, ...] = ()) -> None:
        """Stream one example operation and flush its output when it finishes."""
        self._clear_output()
        self.query_one("#output", RichLog).border_title = f"{action.capitalize()} · {example_id}"
        progress = {"build": "Building", "run": "Running", "test": "Testing"}[action]
        self._set_status(f"{progress} · {example_id} · Esc to cancel")
        if action == "build" or (
            action == "test" and self._readiness is not None and self._readiness.runnable_examples is not None
        ):
            self._built_examples.discard(example_id)
        try:
            code = (
                await self.backend.run(example_id, arguments, self._emit)
                if action == "run"
                else await self.backend.execute(action, example_id, self._emit)
            )
            if action in {"build", "test"}:
                self._readiness = await self.backend.check_ready()
                if not self._readiness.ready:
                    self._built_examples.clear()
                elif self._readiness.runnable_examples is not None:
                    self._built_examples = set(self._readiness.runnable_examples)
                elif action == "build" and code == 0:
                    self._built_examples.add(example_id)
            self._set_status(
                f"{action.capitalize()} {'completed' if code == 0 else 'failed'} · exit {code} · {example_id}"
            )
        except asyncio.CancelledError:
            self._set_status(f"Cancelled · {example_id}")
        except Exception as error:
            self._set_status(f"{action.capitalize()} failed: {error}")
            self._emit(str(error) + "\n")
        finally:
            self._flush_output()

    def _emit(self, text: str) -> None:
        """Normalize streamed line endings and write bounded chunks to the log."""
        # Defer a trailing CR so a split CRLF remains one newline.
        if self._pending_carriage_return:
            text = "\r" + text
            self._pending_carriage_return = False
        if text.endswith("\r"):
            text = text[:-1]
            self._pending_carriage_return = True
        self._pending_output += text.replace("\r\n", "\n").replace("\r", "\n")
        log = self.query_one("#output", RichLog)
        while "\n" in self._pending_output or len(self._pending_output) > 8192:
            end = self._pending_output.find("\n", 0, 8193)
            if end < 0:
                end = 8192
                line, self._pending_output = self._pending_output[:end], self._pending_output[end:]
            else:
                line, self._pending_output = self._pending_output[:end], self._pending_output[end + 1 :]
            log.write(Text.from_ansi(line))

    def action_arguments(self) -> None:
        """Offer custom arguments separately from the one-step Run action."""
        if not self._busy and self.selected is not None and self.selected.id in self._built_examples:
            example_id = self.selected.id
            self.push_screen(ArgumentsScreen(example_id), lambda args: self._arguments_ready(example_id, args))

    def _arguments_ready(self, example_id: str, arguments: tuple[str, ...] | None) -> None:
        """Recheck readiness and start Run after optional argument entry."""
        if arguments is not None and not self._busy and example_id in self._built_examples:
            self._set_busy(True)
            self._task = asyncio.create_task(self._prepare_action("run", example_id, arguments))

    async def action_escape(self) -> None:
        """Cancel active work, or quit from the idle browser."""
        if self._busy:
            self.action_cancel()
        else:
            await self.action_quit()

    def action_cancel(self) -> None:
        """Request cancellation once; the backend completes cleanup before unlocking."""
        if self._task is not None and not self._task.done() and not self._task.cancelling():
            self._set_status("Cancelling; waiting for subprocess cleanup…")
            self._task.cancel()

    async def action_quit(self) -> None:
        """Reap active work before exiting the application."""
        self._quitting = True
        self.action_cancel()
        if self._task is not None:
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
        self.exit()

    async def on_unmount(self) -> None:
        """Clean up if the application exits outside its normal quit action."""
        self.action_cancel()
        if self._task is not None:
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
