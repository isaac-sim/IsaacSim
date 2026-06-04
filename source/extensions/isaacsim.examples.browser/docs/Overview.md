# Overview

The isaacsim.examples.browser extension provides a browser interface for discovering and navigating Isaac Sim robotics examples. It creates a window-based browser that organizes examples by category using a tree-structured interface, allowing users to explore different types of robotics examples available in Isaac Sim through both menu shortcuts and programmatic access.

## Functionality

**Example Organization**: The browser organizes examples hierarchically by category using the {class}`ExampleBrowserModel <isaacsim.examples.browser.ExampleBrowserModel>`, which maintains a tree structure where categories can contain subcategories and individual examples. Each example is represented by an Example object that contains metadata including display name, execution logic, UI customization hooks, category path, and thumbnail images.

**Window Interface**: The {class}`ExampleBrowserWindow <isaacsim.examples.browser.ExampleBrowserWindow>` provides the main user interface, built on top of the tree folder browser widget. Users can navigate through categories to discover examples, with each example displaying its name and thumbnail for visual identification.

**Dynamic Registration**: Examples can be registered and deregistered at runtime through the extension's public API methods. The register_example method accepts parameters like name, execute_entrypoint, ui_hook, category, and thumbnail, while deregister_example removes examples by name and category.

## Key Components

### {class}`ExampleBrowserModel <isaacsim.examples.browser.ExampleBrowserModel>`

The {class}`ExampleBrowserModel <isaacsim.examples.browser.ExampleBrowserModel>` extends TreeFolderBrowserModel to manage the data structure for Isaac Sim examples. It maintains categories as ExampleCategoryItem objects and examples as ExampleDetailItem objects, providing methods to retrieve category items and detail items for the browser display.

### {class}`ExampleBrowserWindow <isaacsim.examples.browser.ExampleBrowserWindow>`

The {class}`ExampleBrowserWindow <isaacsim.examples.browser.ExampleBrowserWindow>` uses a specialized BrowserWidget that extends TreeFolderBrowserWidgetEx. This widget is customized for displaying robotics examples with consistent thumbnail display settings and visible labels across different thumbnail sizes.

## Integration

The extension integrates with Omniverse Kit's menu system through **omni.kit.menu.utils**, registering menu items under "Window > Examples > Robotics Examples" that allow users to toggle the browser window visibility. The browser window can be shown or hidden based on user settings, with the visibility state configurable through the extension's settings.
