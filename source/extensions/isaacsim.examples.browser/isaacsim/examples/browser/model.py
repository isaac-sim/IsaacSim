# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Browser model implementation for organizing and managing examples in the Isaac Sim Examples Browser."""

import asyncio
import os
from typing import Optional

import carb.settings
import omni.kit.app
from omni.kit.browser.core import CategoryItem, CollectionItem, DetailItem
from omni.kit.browser.folder.core import TreeFolderBrowserModel

SETTING_FOLDER = "/exts/isaacsim.examples.browser/folders"

# Folder thumbnail used by ExampleFolderDetailItem. Resolved relative to the extension's data folder.
_DATA_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "data"))
FOLDER_THUMBNAIL = os.path.join(_DATA_DIR, "folder.svg")


class Example:
    """Represents an example that can be registered and displayed in the Isaac Sim Examples Browser.

    This class encapsulates the metadata and functionality for an individual example, including its display
    name, execution logic, UI customization, categorization, and visual representation. Examples are
    organized by category and can be executed through the browser interface.

    Args:
        name: Display name of the example.
        execute_entrypoint: Callable function that executes the example's main logic.
        ui_hook: Callable function for custom UI behavior or modifications.
        category: Category path for organizing the example in the browser hierarchy.
        thumbnail: File path to the example's thumbnail image for visual identification.
    """

    def __init__(
        self,
        name: str = "",
        execute_entrypoint: callable = None,
        ui_hook: callable = None,
        category: str = "",
        thumbnail: Optional[str] = None,
    ) -> None:
        self.name = name
        self.category = category if category else "General"
        self.execute_entrypoint = execute_entrypoint
        self.ui_hook = ui_hook
        self.thumbnail = thumbnail


class ExampleCategoryItem(CategoryItem):
    """A category item for organizing examples in the Isaac Sim examples browser.

    This class represents a hierarchical category node that can contain child categories and examples.
    It extends the browser core CategoryItem to provide example-specific functionality for managing
    hierarchical category structures in the examples browser interface.

    Args:
        name: The name of the category.
    """

    def __init__(self, name: str) -> None:
        super().__init__(name)
        # Examples registered directly at this category (excluding descendants).
        self.examples: list["ExampleDetailItem"] = []

    def add_child(self, child_name: str) -> "ExampleCategoryItem":
        """Adds a child category item to the current category.

        Creates a new child category with the specified name if it doesn't already exist,
        or returns the existing child category if it does.

        Args:
            child_name: Name of the child category to add.

        Returns:
            The child category item that was created or already existed.
        """
        if child_name not in [c.name for c in self.children]:
            child_category = ExampleCategoryItem(child_name)
            self.children.append(child_category)
            self.count += 1
            child_category.parent = self  # add self to the child's parent
        else:
            child_category = self.children[[c.name for c in self.children].index(child_name)]

        return child_category

    def collect_examples(self) -> list["ExampleDetailItem"]:
        """Returns examples registered at this category and recursively at all descendants.

        Returns:
            List of example detail items registered at this category and its descendants.
        """
        items = list(self.examples)
        for child in self.children:
            if isinstance(child, ExampleCategoryItem):
                items.extend(child.collect_examples())
        return items


class ExampleDetailItem(DetailItem):
    """A detail item representing an individual example in the Isaac Sim examples browser.

    This class extends the browser framework's DetailItem to display example-specific information including
    name, thumbnail, and UI hooks. It serves as the visual representation of an Example object within the
    browser's detail view, enabling users to interact with and execute individual examples.

    Args:
        example: The Example object containing the example's metadata and execution details.
    """

    def __init__(self, example: Example) -> None:
        super().__init__(example.name, "", example.thumbnail)
        self.example = example
        self.ui_hook = example.ui_hook


class ExampleFolderDetailItem(DetailItem):
    """A detail-view tile that represents a sub-category, like a folder in a file browser.

    When a user double-clicks one of these tiles in the detail view, the model's `execute` method
    looks up the matching live category by path and changes the tree selection to drill into it.

    Args:
        category_path: Slash-separated path identifying the sub-category (e.g. ``"ROS2/Navigation"``).
        name: Display name shown on the tile (typically the leaf segment of the path).
        thumbnail: Optional path to a folder icon used as the tile thumbnail.
    """

    def __init__(self, category_path: str, name: str, thumbnail: Optional[str] = None) -> None:
        super().__init__(name, "", thumbnail)
        self.category_path = category_path
        # Property delegates iterate selected items and call `item.ui_hook()` to render extra UI; folder
        # tiles don't contribute UI, but we expose a no-op so the shared delegate path keeps working.
        self.ui_hook = lambda: None


class ExampleBrowserModel(TreeFolderBrowserModel):
    """Represent asset browser model.

    Args:
        *args: Variable length argument list passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        settings = carb.settings.get_settings()
        self._examples = {}
        # Set by `set_widget()` once the BrowserWidget is constructed; used by `execute()` to drive
        # tree navigation when a folder tile is double-clicked.
        self._widget = None
        super().__init__(
            *args,
            setting_folders=SETTING_FOLDER,
            show_category_subfolders=True,
            hide_file_without_thumbnails=False,
            show_summary_folder=True,
            **kwargs,
        )

    def set_widget(self, widget: object) -> None:
        """Register the BrowserWidget so folder tiles can drive tree navigation on double-click.

        Args:
            widget: Browser widget used for tree selection updates.
        """
        self._widget = widget

    def register_example(self, **kwargs: object) -> None:
        """Registers a new example in the browser.

        Args:
            **kwargs: Example properties including name, execute_entrypoint, ui_hook, category, and thumbnail.
        """
        example = Example(**kwargs)
        if not example.name:
            return
        if example.category not in self._examples:
            self._examples[example.category] = []

        # check if there are already an example with the same name
        if example.name in [e.name for e in self._examples[example.category]]:
            carb.log_warn(f"Example with name {example.name} already exists in category {example.category}.")
        self._examples[example.category].append(ExampleDetailItem(example))
        self.refresh_browser()
        return

    def get_category_items(self, item: CollectionItem) -> list[CategoryItem]:
        """Override to get list of category items.

        Args:
            item: Collection item to get categories for.

        Returns:
            List of category items organized hierarchically.
        """
        category_items: list[ExampleCategoryItem] = []
        for category, examples in self._examples.items():
            parts = category.split("/")

            current_category = next((c for c in category_items if c.name == parts[0]), None)
            if current_category is None:
                current_category = ExampleCategoryItem(parts[0])
                category_items.append(current_category)

            for part in parts[1:]:
                current_category = current_category.add_child(part)

            # Bind the live example list so register/deregister stays reflected without rebuilding.
            current_category.examples = examples

        # Cache the live tree so `execute()` can resolve folder-tile paths back to current category items.
        self._category_root = category_items
        self.sort_items(category_items)
        return category_items

    def get_detail_items(self, item: ExampleCategoryItem) -> list[DetailItem]:
        """Override to get list of detail items.

        Returns the directly-registered examples at ``item``, plus a folder tile per sub-category. This
        mirrors a typical file-browser detail view: clicking a parent category shows its files (examples)
        and folders (sub-categories) at one level only, not recursively.

        Args:
            item: Category item to get examples for.

        Returns:
            List of detail items: ``ExampleDetailItem`` for direct examples and ``ExampleFolderDetailItem``
            for each immediate sub-category.
        """
        if item.name == self.SUMMARY_FOLDER_NAME:
            # Summary still shows everything flattened, matching the pre-existing convention.
            detail_items: list[DetailItem] = [example for examples in self._examples.values() for example in examples]
        elif isinstance(item, ExampleCategoryItem):
            detail_items = list(item.examples)
            parent_path = self._category_path(item)
            for child in item.children:
                if isinstance(child, ExampleCategoryItem):
                    child_path = f"{parent_path}/{child.name}" if parent_path else child.name
                    detail_items.append(ExampleFolderDetailItem(child_path, child.name, FOLDER_THUMBNAIL))
        else:
            detail_items = []

        self.sort_items(detail_items)
        return detail_items

    def execute(self, item: DetailItem) -> None:
        """Execute a detail item.

        For a regular example tile, runs the example's entrypoint. For a folder tile, navigates the tree
        to the corresponding sub-category so the user can drill into it.

        Args:
            item: The detail item that was activated (typically by a double-click).
        """
        if isinstance(item, ExampleFolderDetailItem):
            target = self._find_category_by_path(item.category_path)
            if target is not None and self._widget is not None:
                # Mutating category selection synchronously inside the double-click handler tears down
                # the very widget tree currently dispatching the event ("Container::destroy was called
                # during an event or draw"). Defer to the next update so the event chain completes
                # before the tree gets rebuilt.
                widget = self._widget

                async def _select_next_frame() -> None:
                    await omni.kit.app.get_app().next_update_async()
                    widget.category_selection = [target]

                asyncio.ensure_future(_select_next_frame())
            return

        if isinstance(item, ExampleDetailItem):
            example = item.example
            if example.execute_entrypoint:
                example.execute_entrypoint()

    def _category_path(self, item: "ExampleCategoryItem") -> str:
        """Reconstruct the slash-separated path for a category by walking its parent chain.

        Args:
            item: Category item whose path should be reconstructed.

        Returns:
            Slash-separated category path from the root category to ``item``.
        """
        parts: list[str] = []
        current: Optional[ExampleCategoryItem] = item
        while current is not None:
            parts.append(current.name)
            current = current.parent if isinstance(current.parent, ExampleCategoryItem) else None
        return "/".join(reversed(parts))

    def _find_category_by_path(self, path: str) -> Optional["ExampleCategoryItem"]:
        """Look up the live category instance matching ``path`` in the most recently built tree.

        Args:
            path: Slash-separated category path to find.

        Returns:
            Matching category item if found, None otherwise.
        """
        roots = getattr(self, "_category_root", None) or self.get_category_items(None)
        parts = path.split("/")
        current = next((c for c in roots if c.name == parts[0]), None)
        for part in parts[1:]:
            if current is None:
                return None
            current = next(
                (c for c in current.children if isinstance(c, ExampleCategoryItem) and c.name == part),
                None,
            )
        return current

    def deregister_example(self, name: str, category: str) -> None:
        """Removes an example from the browser.

        Args:
            name: Name of the example to remove.
            category: Category containing the example.
        """
        if category in self._examples:
            self._examples[category] = [e for e in self._examples[category] if e.name != name]
            if len(self._examples[category]) == 0:
                del self._examples[category]
        self.refresh_browser()

    def refresh_browser(self) -> None:
        """Refreshes the browser display to reflect current examples."""
        collections = self.get_item_children(None)
        if collections:
            self._item_changed(collections[0])
        else:
            self._item_changed(None)
        return
