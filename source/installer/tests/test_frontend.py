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

"""Exercise UI behavior with a backend that has no checkout or installer."""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from unittest.mock import patch

import pytest
from isaacsim_installer.core.models import Catalog, Example, Readiness, Series, SeriesStep
from isaacsim_installer.tui.app import ExamplesApp
from isaacsim_installer.tui.screens import ArgumentsScreen
from isaacsim_installer.tui.widgets import CatalogBrowser, CatalogList, DisclosureHeader
from textual.widgets import Button, Input, RichLog, Static

EXAMPLE = Example("demo", "Demo [literal]", "A demo", "libraries/demo", "none", "python", tests=("smoke",))
STEP = Example("lesson", "Lesson", "Step one", "series/intro/lesson", "none", "python")


class FakeBackend:
    """Implement the UI contract using only in-memory records."""

    label = "Test examples"

    def __init__(self) -> None:
        self.started = asyncio.Event()
        self.cleaned = asyncio.Event()
        self.finish = asyncio.Event()
        self.calls: list[tuple[str, str]] = []
        self.fail_discovery = False
        self.readiness = Readiness(True)
        self.execute_result = 7
        self.prepare_result = 0

    async def discover(self) -> Catalog:
        """Return a small catalog or simulate a backend failure."""
        if self.fail_discovery:
            raise RuntimeError("Catalog unavailable")
        return Catalog(
            (EXAMPLE, STEP), (Series("intro", "Introduction", "Start here", (SeriesStep("lesson", "beginner"),)),)
        )

    async def check_ready(self) -> Readiness:
        """Return configurable prerequisites independently of catalog discovery."""
        return self.readiness

    def capabilities(self, example: Example) -> frozenset[str]:
        """Expose testing only on the standalone record."""
        return frozenset({"build", "run", "test"} if example.tests else {"build", "run"})

    async def prepare(self, emit: Callable[[str], None]) -> int:
        """Simulate an owned prerequisite build without invoking any SDK tools."""
        self.calls.append(("prepare", ""))
        emit("Building SDK…\n")
        self.started.set()
        try:
            await self.finish.wait()
            if self.prepare_result == 0:
                self.readiness = Readiness(True)
            return self.prepare_result
        finally:
            self.cleaned.set()

    async def execute(self, action: str, example_id: str, emit: Callable[[str], None]) -> int:
        """Stream output, block, and account for cleanup."""
        self.calls.append((action, example_id))
        emit("[red]literal log[/red]\r")
        emit("\nProgress 1\rProgress 2\r\n" + "x" * 20000)
        self.started.set()
        try:
            await self.finish.wait()
            return self.execute_result
        finally:
            self.cleaned.set()

    async def run(self, example_id: str, arguments: tuple[str, ...], emit: Callable[[str], None]) -> int:
        """Record exact arguments and stream output for Run tests."""
        self.run_arguments = arguments
        return await self.execute("run", example_id, emit)


async def ready(app: ExamplesApp, pilot: object) -> None:
    """Wait for catalog population and focus a real browser row."""
    await app._task
    await pilot.pause()
    browser = app.query_one(CatalogBrowser)
    browser.query(CatalogList).first().focus()
    await pilot.pause()


async def build_selected(app: ExamplesApp, pilot: object) -> None:
    """Complete a real UI build transition before testing Run interactions."""
    backend = app.backend
    previous_result = backend.execute_result
    was_finished = backend.finish.is_set()
    backend.execute_result = 0
    backend.finish.set()
    app.action_operate("build")
    await app._task
    await pilot.pause()
    backend.execute_result = previous_result
    if not was_finished:
        backend.finish.clear()
    backend.calls.clear()
    backend.started.clear()
    backend.cleaned.clear()


async def test_navigation_and_capabilities() -> None:
    """Navigate nested series with the keyboard and expose supported actions."""
    app = ExamplesApp(FakeBackend())
    async with app.run_test() as pilot:
        await ready(app, pilot)
        assert app.selected == EXAMPLE
        assert not app.query_one("#build", Button).disabled
        assert app.query_one("#cancel", Button).disabled
        browser = app.query_one(CatalogBrowser)
        headers = list(browser.query(DisclosureHeader))
        series_header = next(header for header in headers if header._title == "Series")
        series_header.focus()
        await pilot.press("enter", "down", "enter")
        await pilot.pause()
        assert app.selected is None
        assert str(browser.query_one("#record-detail", Static).render()) == ""
        await pilot.press("down")
        await pilot.pause()
        assert app.selected == STEP
        assert app.query_one("#test", Button).disabled
        assert app.query_one("#run", Button).disabled
        assert not app.query_one("#build", Button).disabled


async def test_cancel_keeps_ui_responsive_and_reaps_work() -> None:
    """Block overlapping work and wait for backend cleanup on Escape."""
    backend = FakeBackend()
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await ready(app, pilot)
        await pilot.press("b")
        await backend.started.wait()
        assert app.query_one("#run", Button).disabled
        app.action_operate("test")
        assert backend.calls == [("build", "demo")]
        await pilot.press("escape")
        await app._task
        assert backend.cleaned.is_set()
        assert not app._busy
        assert not app._quitting
        lines = app.query_one("#output", RichLog).lines
        assert len(lines) <= 1000
        assert [line.text for line in lines[:3]] == ["[red]literal log[/red]", "Progress 1", "Progress 2"]
        assert "" not in (line.text for line in lines)
        assert len(app._pending_output) <= 8192


async def test_discovery_failure_keeps_actions_disabled_and_error_visible() -> None:
    """Failed startup keeps its diagnostic visible without offering example actions."""
    backend = FakeBackend()
    backend.fail_discovery = True
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await app._task
        assert "Catalog unavailable" in str(app.query_one("#status", Static).render())
        assert all(app.query_one(f"#{action}", Button).disabled for action in ("build", "run", "test", "arguments"))
        await pilot.pause()
        assert "Catalog unavailable" in "\n".join(line.text for line in app.query_one("#output", RichLog).lines)
        await pilot.press("b", "r", "t", "a")
        assert backend.calls == []


async def test_run_arguments_default_and_literal_values() -> None:
    """Run starts immediately; optional arguments preserve exact strings."""
    backend = FakeBackend()
    backend.finish.set()
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await ready(app, pilot)
        await build_selected(app, pilot)
        await pilot.press("r")
        await app._task
        assert not isinstance(app.screen, ArgumentsScreen)
        assert backend.run_arguments == ()
        values = ("--name", "two words", "", "$(literal)", "é")
        await pilot.press("a")
        await pilot.pause()
        screen = app.screen
        assert isinstance(screen, ArgumentsScreen)
        edited_values = values[:2] + ("discard me",) + values[2:]
        for value in edited_values:
            await pilot.press("ctrl+a")
            await pilot.pause()
            screen.query(Input).last().value = value
        screen.query(".remove-argument")[2].press()
        await pilot.pause()
        numbers = [str(item.render()) for item in screen.query(".argument-number")]
        assert numbers == ["1", "2", "3", "4", "5"]
        await pilot.click("#submit")
        await app._task
        assert backend.run_arguments == values


async def test_quit_waits_for_cleanup() -> None:
    """Quit cannot abandon backend work."""
    backend = FakeBackend()
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await ready(app, pilot)
        app.action_operate("build")
        await backend.started.wait()
        await app.action_quit()
        assert backend.cleaned.is_set()


@pytest.mark.parametrize("header_title,interaction", [("Standalone", "focus"), ("Series", "click")])
async def test_section_header_clears_example_details(header_title: str, interaction: str) -> None:
    """Headers clear stale details and actions; returning to a row restores both."""
    backend = FakeBackend()
    backend.readiness = Readiness(False, can_prepare=True)
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await app._task
        await pilot.pause()
        assert app.selected is None
        assert not any(row.highlighted for row in app.query(CatalogList).first().children)
        await ready(app, pilot)
        browser = app.query_one(CatalogBrowser)
        assert app.selected == EXAMPLE
        header = next(item for item in browser.query(DisclosureHeader) if item._title == header_title)
        if interaction == "click":
            await pilot.click(header)
        else:
            header.focus()
        await pilot.pause()
        assert str(browser.query_one("#record-detail", Static).render()) == ""
        assert app.selected is None
        assert all(app.query_one(f"#{action}", Button).disabled for action in ("build", "run", "test", "arguments"))
        await pilot.press("b", "r", "t", "a")
        assert backend.calls == []
        if interaction == "click":
            await pilot.click(browser.query(DisclosureHeader).first())
            await pilot.click(browser.query(CatalogList).first().children[0])
        else:
            browser.query(CatalogList).first().focus()
        await pilot.pause()
        assert app.selected == EXAMPLE
        assert app.query_one("#run", Button).disabled
        assert not app.query_one("#build", Button).disabled


async def test_pointer_movement_preserves_selection_until_click() -> None:
    """Keep details and keyboard position stable while the pointer crosses other rows."""
    other = Example("other", "Other example", "Another example", "libraries/other", "none", "python")
    backend = FakeBackend()
    with patch.object(backend, "discover", return_value=Catalog((EXAMPLE, other), ())):
        app = ExamplesApp(backend)
        async with app.run_test() as pilot:
            await ready(app, pilot)
            browser = app.query_one(CatalogBrowser)
            catalog_list = browser.query(CatalogList).first()
            detail = str(browser.query_one("#record-detail", Static).render())
            await pilot.hover(catalog_list.children[1])
            await pilot.pause()
            assert app.selected == EXAMPLE
            assert catalog_list.index == 0
            assert str(browser.query_one("#record-detail", Static).render()) == detail
            await pilot.press("down")
            assert app.selected == other
            await pilot.hover(catalog_list.children[0])
            await pilot.pause()
            assert app.selected == other
            await pilot.click(catalog_list.children[0])
            await pilot.pause()
            assert app.selected == EXAMPLE
            assert catalog_list.index == 0
            assert backend.calls == []


@pytest.mark.parametrize("exit_code", [0, 3])
async def test_single_build_prepares_sdk_and_example(exit_code: int) -> None:
    """One Enter completes both stages; SDK failure keeps Run disabled."""
    backend = FakeBackend()
    backend.readiness = Readiness(False, "SDK build required.", can_prepare=True)
    backend.prepare_result = exit_code
    backend.execute_result = 0
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await ready(app, pilot)
        build = app.query_one("#build", Button)
        run = app.query_one("#run", Button)
        assert str(build.label) == "Build (b)"
        assert build.variant == "primary" and not build.disabled
        assert run.disabled and run.variant == "default"
        await pilot.press("enter")
        await backend.started.wait()
        assert backend.calls == [("prepare", "")]
        assert build.disabled and run.disabled
        backend.finish.set()
        await app._task
        await pilot.pause()
        expected = [("prepare", ""), ("build", "demo")] if exit_code == 0 else [("prepare", "")]
        assert backend.calls == expected
        success = exit_code == 0
        assert run.disabled is not success
        assert run.variant == ("success" if success else "default")
        assert str(build.label) == "Build (b)"
        if success:
            await pilot.press("enter")
            await app._task
            await pilot.pause()
            assert backend.calls[-1] == ("run", "demo")


async def test_cancelled_example_build_reuses_prepared_sdk(monkeypatch: pytest.MonkeyPatch) -> None:
    """Retry only the example when cancellation follows successful SDK preparation."""
    backend = FakeBackend()
    backend.readiness = Readiness(False, "SDK build required.", can_prepare=True)
    backend.execute_result = 0

    async def prepare(emit: Callable[[str], None]) -> int:
        backend.calls.append(("prepare", ""))
        backend.readiness = Readiness(True)
        return 0

    monkeypatch.setattr(backend, "prepare", prepare)
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await ready(app, pilot)
        await pilot.press("b")
        await backend.started.wait()
        assert backend.calls == [("prepare", ""), ("build", EXAMPLE.id)]
        await pilot.press("escape")
        await app._task
        assert backend.cleaned.is_set()
        assert app.query_one("#run", Button).disabled
        backend.finish.set()
        await pilot.press("b")
        await app._task
        assert backend.calls == [("prepare", ""), ("build", EXAMPLE.id), ("build", EXAMPLE.id)]
        assert not app.query_one("#run", Button).disabled


async def test_escape_dismisses_dialog_before_quitting() -> None:
    """The dialog handles the first Escape; the idle browser handles the next."""
    app = ExamplesApp(FakeBackend())
    async with app.run_test() as pilot:
        await ready(app, pilot)
        await build_selected(app, pilot)
        await pilot.press("a")
        await pilot.pause()
        assert isinstance(app.screen, ArgumentsScreen)
        await pilot.press("escape")
        await pilot.pause()
        assert not isinstance(app.screen, ArgumentsScreen)
        assert not app._quitting
        await pilot.press("escape")
        assert app._quitting


async def test_unbuilt_example_defaults_to_build_and_blocks_run() -> None:
    """Enter builds first; Run stays unavailable until the explicit build succeeds."""
    backend = FakeBackend()
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await ready(app, pilot)
        assert app.query_one("#build", Button).variant == "primary"
        assert app.query_one("#run", Button).disabled
        assert app.active_bindings["enter"].binding.description == "Build"
        await pilot.press("r")
        assert backend.calls == []
        await pilot.press("enter")
        await backend.started.wait()
        backend.finish.set()
        await app._task
        assert app.query_one("#run", Button).disabled
        assert "exit 7" in str(app.query_one("#status", Static).render())
        await build_selected(app, pilot)
        assert not app.query_one("#run", Button).disabled
        assert app.active_bindings["enter"].binding.description == "Run"
        backend.finish.set()
        await pilot.press("enter")
        await app._task
        assert backend.run_arguments == ()
        assert backend.calls == [("run", "demo")]


@pytest.mark.parametrize(
    "action,exit_code,finally_runnable",
    [
        ("build", 0, False),
        ("test", 7, True),
    ],
)
async def test_build_and_test_refresh_disk_readiness(action: str, exit_code: int, finally_runnable: bool) -> None:
    """Use artifact state after builds and tests, independently of their exit status."""
    backend = FakeBackend()
    backend.readiness = Readiness(True, runnable_examples=frozenset())
    backend.execute_result = exit_code
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await ready(app, pilot)
        app.action_operate(action)
        await backend.started.wait()
        backend.readiness = Readiness(True, runnable_examples=frozenset({EXAMPLE.id} if finally_runnable else ()))
        backend.finish.set()
        await app._task
        assert backend.calls == [(action, EXAMPLE.id)]
        assert app.query_one("#run", Button).disabled is not finally_runnable
        assert app.query_one("#build", Button).variant == ("default" if finally_runnable else "primary")


async def test_cancelled_test_requires_build_before_run() -> None:
    """Cancelling a test's build stage must not retain its old runnable snapshot."""
    backend = FakeBackend()
    backend.readiness = Readiness(True, runnable_examples=frozenset({EXAMPLE.id}))
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await ready(app, pilot)
        await pilot.press("t")
        await backend.started.wait()
        backend.readiness = Readiness(True, runnable_examples=frozenset())
        await pilot.press("escape")
        await app._task
        assert backend.cleaned.is_set()
        assert app.query_one("#run", Button).disabled
        assert app.query_one("#build", Button).variant == "primary"


async def test_removed_build_is_rechecked_before_run() -> None:
    """Block Run when its previously ready artifact disappears."""
    backend = FakeBackend()
    backend.readiness = Readiness(True, runnable_examples=frozenset({EXAMPLE.id}))
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await ready(app, pilot)
        backend.readiness = Readiness(True, runnable_examples=frozenset())
        await pilot.press("enter")
        await app._task
        await pilot.pause()
        assert not isinstance(app.screen, ArgumentsScreen)
        assert app.query_one("#run", Button).disabled
        assert not app.query_one("#build", Button).disabled
        assert backend.calls == []


async def test_run_streams_without_leaving_browser_and_restores_controls() -> None:
    """Run an existing build, retaining output and controls after failure."""
    backend = FakeBackend()
    backend.readiness = Readiness(True, runnable_examples=frozenset({EXAMPLE.id}))
    app = ExamplesApp(backend)
    async with app.run_test() as pilot:
        await ready(app, pilot)
        with patch.object(app, "suspend", side_effect=AssertionError("Must stay in the TUI")):
            await pilot.press("r")
            await backend.started.wait()
            await pilot.pause()
            assert not isinstance(app.screen, ArgumentsScreen)
            assert app._busy
            assert app.query_one("#output", RichLog).lines
            assert not app.query_one("#cancel", Button).disabled
            backend.finish.set()
            await app._task
            assert not app._busy
            assert not app.query_one("#run", Button).disabled
            status = str(app.query_one("#status", Static).render())
            assert "failed" in status
            assert "exit 7" in status
            assert app.query_one("#output", RichLog).lines


@pytest.mark.parametrize(
    "chunks,expected_lengths",
    [
        (("x" * 16384, "\n"), [8192, 8192]),
        (("x" * 8192, "\n\n"), [8192, 0]),
    ],
    ids=["multiple", "intentional-blank"],
)
def test_output_chunk_boundary_preserves_line_breaks(chunks: tuple[str, ...], expected_lengths: list[int]) -> None:
    """Chunking must not insert blank lines or consume intentional empty lines."""
    app = ExamplesApp(FakeBackend())
    with patch.object(app, "query_one") as query:
        for chunk in chunks:
            app._emit(chunk)
        app._flush_output()
    assert [len(call.args[0].plain) for call in query.return_value.write.call_args_list] == expected_lengths


async def test_shared_series_example_uses_selected_path_context() -> None:
    """A shared lesson retains its selected path's metadata and sole row cursor."""
    backend = FakeBackend()
    backend.readiness = Readiness(True, runnable_examples=frozenset({EXAMPLE.id}))
    backend.finish.set()
    paths = (
        Series("first", "First path", "", (SeriesStep(EXAMPLE.id, "beginner"),)),
        Series("second", "Second path", "", (SeriesStep(EXAMPLE.id, "advanced"),)),
    )
    with patch.object(backend, "discover", return_value=Catalog((EXAMPLE,), paths)):
        app = ExamplesApp(backend)
        async with app.run_test() as pilot:
            await app._task
            await pilot.pause()
            browser = app.query_one(CatalogBrowser)
            root = next(item for item in browser.query(DisclosureHeader) if item._title == "Series")
            root.focus()
            if not root.expanded:
                await pilot.press("enter")
            for path in paths:
                header = next(item for item in browser.query(DisclosureHeader) if item._title == path.title)
                header.focus()
                await pilot.press("enter")
                await pilot.pause()
                catalog_list = header.parent.content
                await pilot.click(catalog_list.children[0])
                await pilot.pause()
                detail = str(browser.query_one("#record-detail", Static).render())
                assert f"Step 1 of 1 in {path.title} · {path.steps[0].level}." in detail
                assert app.selected == EXAMPLE
                assert catalog_list.index == 0
                assert all(other.index is None for other in browser.query(CatalogList) if other is not catalog_list)
            assert backend.calls == []
            standalone = next(item for item in browser.query(DisclosureHeader) if item._title == "Standalone")
            standalone.focus()
            await pilot.press("enter", "down")
            await pilot.pause()
            detail = str(browser.query_one("#record-detail", Static).render())
            assert "STANDALONE EXAMPLE" in detail
            assert "First path" not in detail and "Second path" not in detail
