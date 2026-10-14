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

"""Provide reusable, presentation-only Textual widgets."""

from __future__ import annotations

from isaacsim_installer.core.models import Catalog, Example, Series
from textual import events
from textual.app import ComposeResult
from textual.binding import Binding
from textual.containers import Container, Horizontal, Vertical, VerticalScroll
from textual.content import Content
from textual.geometry import Offset
from textual.message import Message
from textual.reactive import reactive
from textual.widget import Widget
from textual.widgets import Button, Label, ListItem, ListView, Static

RowMetadata = tuple[str, str] | None


def set_enter_hint(widget: Widget, action: str, label: str) -> None:
    """Update the footer hint without changing the widget's Enter behavior."""
    binding = Binding("enter", action, label, key_display="Enter")
    if widget._bindings.key_to_bindings.get("enter") != [binding]:
        widget._bindings.key_to_bindings["enter"] = [binding]
        widget.refresh_bindings()


class MarqueeLabel(Container):
    """Single-line text that reveals overflow while hovered or keyboard-active.

    Args:
        text: Literal text to display.
    """

    DEFAULT_CSS = """
    MarqueeLabel {
        width: 1fr;
        height: 1;
        overflow: hidden hidden;
    }

    MarqueeLabel > .marquee-text {
        height: 1;
        text-wrap: nowrap;
        text-overflow: clip;
    }
    """

    def __init__(self, text: str) -> None:
        super().__init__()
        self._keyboard_active = False
        content = Content(text)
        self._title_widget = Static(content, classes="marquee-text", markup=False)
        self._title_widget.styles.width = max(1, content.cell_length)

    def compose(self) -> ComposeResult:
        """Compose the literal title."""
        yield self._title_widget

    def set_keyboard_active(self, active: bool) -> None:
        """Set whether keyboard navigation should reveal the full title.

        Args:
            active: Whether this title is the active keyboard row.
        """
        self._keyboard_active = active
        if active:
            self.call_after_refresh(self._start)
        elif not self.is_mouse_over:
            self.call_after_refresh(self._reset)

    async def on_enter(self, event: events.Enter) -> None:
        """Start revealing an overflowing title on pointer entry.

        Args:
            event: Pointer entry event.
        """
        await self._start()

    async def on_leave(self, event: events.Leave) -> None:
        """Restore the title when pointer and keyboard are inactive.

        Args:
            event: Pointer leave event.
        """
        if not self._keyboard_active:
            await self._reset()

    def on_resize(self, event: events.Resize) -> None:
        """Recalculate overflow after layout changes.

        Args:
            event: Widget resize event.
        """
        self.call_after_refresh(self._reset)
        if self._keyboard_active or self.is_mouse_over:
            self.call_after_refresh(self._start)

    async def _start(self) -> None:
        """Scroll an overflowing title to reveal its end."""
        title = self._title_widget
        if not title.is_mounted:
            return
        overflow = title.outer_size.width - self.content_size.width
        if overflow <= 0:
            return
        await title.stop_animation("offset", complete=False)
        title.animate("offset", Offset(-overflow, 0), speed=12, delay=0.35, easing="linear", level="basic")

    async def _reset(self) -> None:
        """Stop title animation and restore its starting position."""
        title = self._title_widget
        if title.is_mounted:
            await title.stop_animation("offset", complete=False)
        title.offset = (0, 0)


class DisclosureHeader(Horizontal, can_focus=True):
    """Accessible single-line disclosure header.

    Args:
        title: Literal title to display.
        count: Optional number of records below the header.
    """

    BINDINGS = [
        Binding("enter", "toggle", "Expand/collapse", key_display="Enter"),
        Binding("space", "toggle", show=False),
        Binding("right", "expand", show=False),
        Binding("left", "collapse", show=False),
    ]
    DEFAULT_CSS = """
    DisclosureHeader {
        width: auto;
        max-width: 100%;
        height: 1;
        padding: 0;
        color: #d7dde5;
        background: transparent;
    }

    DisclosureHeader:hover,
    DisclosureHeader:focus {
        color: #f5f7fa;
        background: #222b37;
    }

    DisclosureHeader:focus {
        text-style: bold;
    }

    DisclosureHeader > .disclosure-symbol {
        width: 2;
        height: 1;
        color: #76b900;
    }

    DisclosureHeader > .disclosure-count {
        width: auto;
        height: 1;
        margin-left: 1;
        color: #7f8997;
    }
    """

    expanded = reactive(False)

    class Active(Message):
        """Report that a header has focus."""

    class ToggleRequested(Message):
        """Request a change to this disclosure's open state.

        Args:
            header: Header that received the request.
            expanded: Requested open state.
        """

        def __init__(self, header: DisclosureHeader, expanded: bool) -> None:
            super().__init__()
            self.header = header
            self.expanded = expanded

    def __init__(self, title: str, count: int = 0) -> None:
        super().__init__()
        self._title = title
        self._count = count
        self.styles.width = 2 + Content(title).cell_length + 1 + Content(str(count)).cell_length

    def compose(self) -> ComposeResult:
        """Compose the symbol, title, and count."""
        yield Static("▸", classes="disclosure-symbol", markup=False)
        yield MarqueeLabel(self._title)
        yield Static(str(self._count), classes="disclosure-count", markup=False)

    def set_count(self, count: int) -> None:
        """Update the compact header count.

        Args:
            count: Visible child count.
        """
        self._count = count
        self.query_one(".disclosure-count", Static).update(str(count))
        self.styles.width = 2 + Content(self._title).cell_length + 1 + Content(str(count)).cell_length

    def on_focus(self, event: events.Focus) -> None:
        """Reveal an overflowing title while keyboard-focused.

        Args:
            event: Focus event.
        """
        self.query_one(MarqueeLabel).set_keyboard_active(True)
        self.post_message(self.Active())

    def on_blur(self, event: events.Blur) -> None:
        """Stop keyboard-driven title movement after focus leaves.

        Args:
            event: Blur event.
        """
        self.query_one(MarqueeLabel).set_keyboard_active(False)

    def on_click(self, event: events.Click) -> None:
        """Toggle the disclosure from any point in its header.

        Args:
            event: Click event.
        """
        event.stop()
        self.focus()
        self.action_toggle()

    def action_toggle(self) -> None:
        """Request the opposite open state."""
        self.post_message(self.ToggleRequested(self, not self.expanded))

    def action_expand(self) -> None:
        """Request the open state."""
        if not self.expanded:
            self.post_message(self.ToggleRequested(self, True))

    def action_collapse(self) -> None:
        """Request the closed state."""
        if self.expanded:
            self.post_message(self.ToggleRequested(self, False))

    def _watch_expanded(self, expanded: bool) -> None:
        """Keep the disclosure symbol and Enter hint aligned with its state."""
        set_enter_hint(self, "toggle", "Collapse" if expanded else "Expand")
        if self.is_mounted:
            self.query_one(".disclosure-symbol", Static).update("▾" if expanded else "▸")


class CatalogListItem(ListItem):
    """One literal catalog record, with optional two-line step context.

    Args:
        record: Catalog record represented by the row.
        metadata: Optional ``(ordinal, difficulty)`` series-step context.
    """

    DEFAULT_CSS = """
    CatalogListItem {
        height: 1;
        padding: 0 1;
        background: #181e27;
    }

    CatalogListItem.-highlight {
        background: #222b37;
        color: #f5f7fa;
        border-left: thick #76b900;
        padding-left: 0;
    }

    CatalogListItem .row-number {
        width: 3;
        height: 1;
        color: #9aa4b2;
        text-wrap: nowrap;
    }

    CatalogListItem .row-top,
    CatalogListItem .row-body {
        width: 100%;
        height: 1;
    }

    CatalogListItem .row-body.-series-step {
        height: 2;
    }

    CatalogListItem .row-difficulty {
        width: 100%;
        height: 1;
        padding-right: 1;
        color: #7f8997;
        text-align: right;
        text-style: italic;
    }
    """

    def __init__(self, record: Example, metadata: RowMetadata = None) -> None:
        super().__init__()
        self.styles.height = 2 if metadata else 1
        self.record = record
        self._metadata = metadata

    def compose(self) -> ComposeResult:
        """Compose ordinal/title first, then right-aligned difficulty."""
        body = Vertical(classes="row-body -series-step" if self._metadata else "row-body")
        with body:
            with Horizontal(classes="row-top"):
                if self._metadata:
                    ordinal, _difficulty = self._metadata
                    yield Static(Content(ordinal), classes="row-number", markup=False)
                yield MarqueeLabel(self.record.title)
            if self._metadata:
                _ordinal, difficulty = self._metadata
                yield Static(Content(difficulty), classes="row-difficulty", markup=False)

    def set_keyboard_active(self, active: bool) -> None:
        """Set keyboard marquee activity for this row.

        Args:
            active: Whether the containing list highlights this row with keyboard focus.
        """
        if not self.is_mounted:
            return
        marquees = self.query(MarqueeLabel).nodes
        if marquees:
            marquees[0].set_keyboard_active(active)


class CatalogList(ListView):
    """List of selectable catalog records with marquee-aware highlighting."""

    DEFAULT_CSS = """
    CatalogList {
        height: auto;
        max-height: 18;
        border: none;
        background: #181e27;
        scrollbar-color: #365700;
    }

    CatalogList:focus {
        border: none;
    }
    """

    def __init__(self) -> None:
        super().__init__(initial_index=None)

    class RecordSelected(Message):
        """Report a selected record.

        Args:
            catalog_list: List that produced the selection.
            record: Selected catalog record.
        """

        def __init__(self, catalog_list: CatalogList, record: Example) -> None:
            super().__init__()
            self.catalog_list = catalog_list
            self.record = record

    class RecordActivated(RecordSelected):
        """Request the primary action through Enter, rather than a pointer click."""

    def action_select_cursor(self) -> None:
        """Activate the highlighted example with the keyboard."""
        item = self.highlighted_child
        if isinstance(item, CatalogListItem):
            self.post_message(self.RecordActivated(self, item.record))

    class RecordHighlighted(Message):
        """Report a highlighted record.

        Args:
            catalog_list: List that produced the highlight.
            record: Highlighted catalog record.
        """

        def __init__(self, catalog_list: CatalogList, record: Example) -> None:
            super().__init__()
            self.catalog_list = catalog_list
            self.record = record

    def set_records(self, records: list[tuple[Example, RowMetadata]]) -> None:
        """Replace visible rows.

        Args:
            records: Records paired with optional compact metadata.
        """
        content_height = sum(2 if metadata else 1 for _record, metadata in records)
        self.styles.height = max(1, min(content_height, 18))
        self.clear()
        if records:
            self.extend(CatalogListItem(record, metadata) for record, metadata in records)

    def on_focus(self, event: events.Focus) -> None:
        """Activate the current row marquee on keyboard focus.

        Args:
            event: Focus event.
        """
        if self.index is None and self.children:
            self.index = 0
        self._update_keyboard_marquee(self.index)
        item = self.highlighted_child
        if isinstance(item, CatalogListItem):
            self.post_message(self.RecordHighlighted(self, item.record))

    def on_blur(self, event: events.Blur) -> None:
        """Stop the current row marquee after keyboard focus leaves.

        Args:
            event: Blur event.
        """
        self._update_keyboard_marquee(None)

    def on_list_view_highlighted(self, event: ListView.Highlighted) -> None:
        """Publish highlight changes and move keyboard marquee activity.

        Args:
            event: List highlight event.
        """
        item = event.item
        if item is not None and (not item.is_mounted or item not in self.children):
            return
        self._update_keyboard_marquee(event.list_view.index if self.has_focus else None)
        if isinstance(item, CatalogListItem):
            self.post_message(self.RecordHighlighted(self, item.record))

    def on_list_view_selected(self, event: ListView.Selected) -> None:
        """Publish the activated record.

        Args:
            event: List selection event.
        """
        if isinstance(event.item, CatalogListItem):
            self.post_message(self.RecordSelected(self, event.item.record))

    def _update_keyboard_marquee(self, active_index: int | None) -> None:
        """Animate only the keyboard-highlighted row title."""
        for index, item in enumerate(self.children):
            if isinstance(item, CatalogListItem):
                item.set_keyboard_active(index == active_index)


class AccordionSection(Vertical):
    """Disclosure header and collapsible content.

    Args:
        title: Literal section title.
        content: Widget revealed by the section.
        count: Number displayed in the header.
    """

    DEFAULT_CSS = """
    AccordionSection {
        height: auto;
    }

    AccordionSection > .accordion-content {
        height: auto;
        padding-left: 2;
    }
    """

    class OpenRequested(Message):
        """Request a new section state.

        Args:
            section: Section that received the request.
            expanded: Requested open state.
        """

        def __init__(self, section: AccordionSection, expanded: bool) -> None:
            super().__init__()
            self.section = section
            self.expanded = expanded

    def __init__(self, title: str, content: Widget, count: int) -> None:
        super().__init__()
        self.header = DisclosureHeader(title, count)
        self.content = content
        self.content.add_class("accordion-content")

    def compose(self) -> ComposeResult:
        """Compose the header and collapsible content."""
        yield self.header
        yield self.content

    def set_expanded(self, expanded: bool) -> None:
        """Set the visible state while keeping focus valid.

        Args:
            expanded: Whether to display the content.
        """
        if not expanded and self.content.query("*:focus"):
            self.header.focus()
        self.header.expanded = expanded
        self.content.display = expanded

    def on_mount(self) -> None:
        """Apply the initial collapsed state."""
        self.set_expanded(False)

    def on_disclosure_header_toggle_requested(self, event: DisclosureHeader.ToggleRequested) -> None:
        """Forward a header request with section identity.

        Args:
            event: Header state request.
        """
        event.stop()
        self.post_message(self.OpenRequested(self, event.expanded))


class CatalogBrowser(Vertical):
    """Accordion catalog with an adjacent detail view."""

    DEFAULT_CSS = """
    CatalogBrowser #catalog-accordion {
        height: 1fr;
        background: #181e27;
        scrollbar-color: #365700;
    }

    CatalogBrowser #series-sections {
        height: auto;
    }
    """

    class Selected(Message):
        """Report a selected catalog record.

        Args:
            record: Selected catalog record.
        """

        def __init__(self, record: Example | None) -> None:
            super().__init__()
            self.record = record

    class Activated(Message):
        """Request the primary action for an activated example row."""

        def __init__(self, record: Example) -> None:
            super().__init__()
            self.record = record

    def on_disclosure_header_active(self, event: DisclosureHeader.Active) -> None:
        """Clear details and actions when a header receives attention."""
        event.stop()
        self._select(None)

    def __init__(self) -> None:
        super().__init__(id="catalog-browser")
        self._catalog: Catalog | None = None
        self._series_membership: dict[tuple[CatalogList, str], tuple[Series, int, str]] = {}
        self._series_sections: list[AccordionSection] = []
        self._standalone_list = CatalogList()
        self._workflow_list = CatalogList()
        self._series_container = Vertical(id="series-sections")
        self._standalone_section = AccordionSection("Standalone", self._standalone_list, 0)
        self._workflow_section = AccordionSection("Workflows", self._workflow_list, 0)
        self._series_root = AccordionSection("Series", self._series_container, 0)
        self._root_sections: list[AccordionSection] = [
            self._standalone_section,
            self._workflow_section,
            self._series_root,
        ]

    def compose(self) -> ComposeResult:
        """Compose catalog results and details."""
        with Horizontal(id="catalog-content"):
            with Vertical(id="results-pane"):
                yield Label("EXAMPLES", classes="eyebrow")
                with VerticalScroll(id="catalog-accordion"):
                    yield from self._root_sections
            with VerticalScroll(id="detail-pane"):
                yield Static("", id="record-detail", markup=False)

    def set_catalog(self, catalog: Catalog) -> None:
        """Display a validated catalog.

        Args:
            catalog: Validated release or active-workspace catalog.
        """
        self._catalog = catalog
        self._rebuild_series_sections()
        self.call_after_refresh(self._populate_options)

    def focus_results(self) -> None:
        """Focus the active root header at the start of result navigation."""
        root = next((section for section in self._root_sections if section.header.expanded), None)
        if root is None:
            root = next((section for section in self._root_sections if section.display), self._root_sections[0])
        root.header.focus()

    def navigate_results(self, delta: int) -> bool:
        """Move focus through visible accordion headers and lists.

        Args:
            delta: Negative to move backward or positive to move forward.

        Returns:
            True if result focus was handled, otherwise false.
        """
        targets = self._result_focus_targets()
        focused = self.screen.focused
        if focused not in targets or not delta:
            return False
        index = targets.index(focused)
        target_index = max(0, min(len(targets) - 1, index + (-1 if delta < 0 else 1)))
        targets[target_index].focus()
        return True

    def on_key(self, event: events.Key) -> None:
        """Move between disclosures when arrows reach a list boundary.

        Args:
            event: Key event from a result descendant.
        """
        if event.key not in ("up", "down"):
            return
        focused = self.screen.focused
        should_navigate = isinstance(focused, DisclosureHeader)
        if isinstance(focused, CatalogList):
            at_start = focused.index is None or focused.index <= 0
            at_end = focused.index is None or focused.index >= len(focused.children) - 1
            should_navigate = at_start if event.key == "up" else at_end
        if should_navigate and self.navigate_results(-1 if event.key == "up" else 1):
            event.prevent_default()
            event.stop()

    def on_catalog_list_record_highlighted(self, event: CatalogList.RecordHighlighted) -> None:
        """Update details when a row is highlighted.

        Args:
            event: Highlighted record event.
        """
        if self._list_is_active(event.catalog_list) and event.catalog_list.has_focus:
            self._select(event.record, event.catalog_list)

    def on_catalog_list_record_selected(self, event: CatalogList.RecordSelected) -> None:
        """Publish a selected row without starting an operation.

        Args:
            event: Selected record event.
        """
        self._select(event.record, event.catalog_list)

    def on_catalog_list_record_activated(self, event: CatalogList.RecordActivated) -> None:
        """Forward Enter on an example to the primary action."""
        self._select(event.record, event.catalog_list)
        self.post_message(self.Activated(event.record))

    def on_accordion_section_open_requested(self, event: AccordionSection.OpenRequested) -> None:
        """Enforce one open peer at each accordion level.

        Args:
            event: Section state request.
        """
        event.stop()
        self._select(None)
        section = event.section
        if section in self._root_sections:
            if event.expanded:
                self._open_only(section, self._root_sections)
                if section is self._series_root and self._series_sections:
                    for nested in self._series_sections:
                        nested.set_expanded(False)
            else:
                section.set_expanded(False)
            return
        if section in self._series_sections:
            if event.expanded:
                self._open_only(self._series_root, self._root_sections)
                self._open_only(section, self._series_sections)
            else:
                section.set_expanded(False)

    def _populate_options(self) -> None:
        """Populate example lists and series steps from the current catalog."""
        if self._catalog is None:
            return
        standalones = [
            (example, None)
            for example in self._catalog.examples
            if example.path.split("/", 1)[0] not in ("workflows", "series")
        ]
        workflows = [
            (example, None) for example in self._catalog.examples if example.path.split("/", 1)[0] == "workflows"
        ]
        self._standalone_list.set_records(standalones)
        self._workflow_list.set_records(workflows)

        examples_by_id = {example.id: example for example in self._catalog.examples}
        self._series_membership.clear()
        for series, section in zip(self._catalog.series, self._series_sections, strict=True):
            catalog_list = section.content
            assert isinstance(catalog_list, CatalogList)
            examples = []
            for index, step in enumerate(series.steps, start=1):
                example = examples_by_id[step.example]
                examples.append((example, (f"{index:02}", step.level)))
                self._series_membership[catalog_list, example.id] = (series, index, step.level)
            catalog_list.set_records(examples)
            section.display = True

        counts = (len(standalones), len(workflows), len(self._series_sections))
        for section, count in zip(self._root_sections, counts, strict=True):
            section.display = bool(count)
            section.header.set_count(count)

        populated_roots = [section for section, count in zip(self._root_sections, counts, strict=True) if count]
        current_root = next((section for section in populated_roots if section.header.expanded), None)
        if populated_roots:
            self._open_only(current_root or populated_roots[0], self._root_sections)
            if self._series_root.header.expanded:
                for section in self._series_sections:
                    section.set_expanded(False)

    def _rebuild_series_sections(self) -> None:
        """Replace nested series accordions using the current catalog."""
        self._series_container.remove_children()
        self._series_sections.clear()
        if self._catalog is None:
            return
        for series in self._catalog.series:
            section = AccordionSection(series.title, CatalogList(), len(series.steps))
            self._series_sections.append(section)
        if self._series_sections:
            self._series_container.mount(*self._series_sections)

    def _select(self, record: Example | None, source_list: CatalogList | None = None) -> None:
        """Select one list's row and show its learning-path context."""
        for catalog_list in self.query(CatalogList):
            catalog_list.index = next(
                (
                    index
                    for index, item in enumerate(catalog_list.children)
                    if catalog_list is source_list
                    and isinstance(item, CatalogListItem)
                    and record is not None
                    and item.record.id == record.id
                ),
                None,
            )
        membership = (
            self._series_membership.get((source_list, record.id))
            if source_list is not None and record is not None
            else None
        )
        detail = _record_text(record, membership) if record is not None else ""
        self.query_one("#record-detail", Static).update(detail)
        self.post_message(self.Selected(record))

    def _list_is_active(self, catalog_list: CatalogList) -> bool:
        """Return whether a list belongs to a visible expanded section."""
        if catalog_list is self._standalone_list:
            return self._standalone_section.display and self._standalone_section.header.expanded
        if catalog_list is self._workflow_list:
            return self._workflow_section.display and self._workflow_section.header.expanded
        if not self._series_root.display or not self._series_root.header.expanded:
            return False
        return any(
            section.display and section.header.expanded and section.content is catalog_list
            for section in self._series_sections
        )

    def _result_focus_targets(self) -> list[Widget]:
        """Return visible headers and populated lists in navigation order."""
        targets: list[Widget] = []
        for root in self._root_sections:
            if not root.display:
                continue
            targets.append(root.header)
            if not root.header.expanded:
                continue
            if root is self._series_root:
                for section in self._series_sections:
                    if not section.display:
                        continue
                    targets.append(section.header)
                    has_visible_records = (
                        section.header.expanded
                        and isinstance(section.content, CatalogList)
                        and bool(section.content.children)
                    )
                    if has_visible_records:
                        targets.append(section.content)
            elif isinstance(root.content, CatalogList) and root.content.children:
                targets.append(root.content)
        return targets

    @staticmethod
    def _open_only(selected: AccordionSection, peers: list[AccordionSection]) -> None:
        """Expand the selected visible section and collapse its peers."""
        for section in peers:
            section.set_expanded(section is selected and section.display)


def _record_text(record: Example, membership: tuple[Series, int, str] | None = None) -> Content:
    """Format one catalog detail record without markup interpretation.

    Args:
        record: Example to format.
        membership: Optional series, step number, and learning level.

    Returns:
        Styled literal detail text.
    """
    kind, description, kind_style = _record_kind(record, membership)
    detail = Content()
    detail += Content.styled(kind, f"bold {kind_style}")
    detail += Content.styled(f"  {description}", "#9aa4b2")
    detail += Content("\n\n")
    detail += Content.styled(record.title, "bold #f5f7fa")
    detail += Content.styled(f"\n{record.id}", "#76b900")
    detail += Content(f"\n\n{record.summary}\n")
    fields = (
        ("PATH", (record.path,)),
        ("BUILD / RUN", (record.build_adapter, record.run_adapter)),
        ("CATEGORIES", record.categories),
        ("TOPICS", record.topics),
        ("SYSTEM REQUIREMENTS", record.system_requirements),
        ("TESTS", record.tests),
        ("OWNERS", record.owners),
    )
    for heading, values in fields:
        if not values:
            continue
        detail += Content.styled(f"\n{heading}\n", "bold #8d96a6")
        detail += Content("  " + "\n  ".join(values) + "\n")
    if record.module_requirements:
        detail += Content.styled("\nSDK MODULES\n", "bold #8d96a6")
        for requirement in record.module_requirements:
            surfaces = ", ".join(requirement.surfaces)
            detail += Content(f"  {requirement.name}: {surfaces}\n")
    return detail


def _record_kind(record: Example, membership: tuple[Series, int, str] | None = None) -> tuple[str, str, str]:
    """Return the visible catalog role, explanation, and accent color."""
    root = record.path.split("/", 1)[0]
    if membership is not None:
        series, index, level = membership
        description = f"Step {index} of {len(series.steps)} in {series.title} · {level}."
        return "SERIES STEP", description, "#c4b5fd"
    if root == "workflows":
        return "WORKFLOW", "Runnable end-to-end user goal.", "#7dd3fc"
    if root == "series":
        return "SERIES STEP", "Independently runnable lesson in a series.", "#c4b5fd"
    if root == "libraries":
        return "STANDALONE EXAMPLE", "One independently runnable example.", "#76b900"
    return "EXAMPLE", "Independently runnable catalog example.", "#9aa4b2"


__all__ = ["CatalogBrowser", "CatalogList", "set_enter_hint"]
