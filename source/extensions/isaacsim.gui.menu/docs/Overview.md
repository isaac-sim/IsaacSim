# Overview

The isaacsim.gui.menu extension provides Isaac Sim-specific menu bar functionality for robotics applications. It creates and manages multiple specialized menus including File, Edit, Create, Window, Tools, Utilities, Layouts, and Help menus, each tailored with robotics-focused operations and Isaac Sim assets.

## Key Components

### Menu Extensions

**{class}`CreateMenuExtension <isaacsim.gui.menu.CreateMenuExtension>`** builds and manages the Create menu for Isaac Sim, providing access to robotics assets and specialized creation functions. It includes utilities for creating Isaac Sim assets at specified stage paths and generating AprilTag materials with custom textures.

**{class}`EditMenuExtension <isaacsim.gui.menu.EditMenuExtension>`** handles the Edit menu with enhanced functionality for robotics workflows. It provides comprehensive prim manipulation capabilities including selection validation, duplication with layer options, parenting operations, grouping/ungrouping, and visibility controls. The extension also includes screenshot capture functionality and live session detection for collaborative workflows.

**{class}`FileMenuExtension <isaacsim.gui.menu.FileMenuExtension>`** manages the File menu with Isaac Sim-specific file operations. It includes validation methods for stage operations and uses a custom FileMenuDelegate that extends IconMenuDelegate to control text width limits for menu items like "Open Recent".

**{class}`FixmeMenuExtension <isaacsim.gui.menu.FixmeMenuExtension>`** builds and manages a specialized FixMe menu using a custom MenuDelegate that hides menu entries by overriding the build_item method and providing right-aligned menu positioning.

**{class}`WindowMenuExtension <isaacsim.gui.menu.WindowMenuExtension>`**, **{class}`ToolsMenuExtension <isaacsim.gui.menu.ToolsMenuExtension>`**, **{class}`UtilitiesMenuExtension <isaacsim.gui.menu.UtilitiesMenuExtension>`**, **{class}`LayoutMenuExtension <isaacsim.gui.menu.LayoutMenuExtension>`**, and **{class}`HelpMenuExtension <isaacsim.gui.menu.HelpMenuExtension>`** handle their respective menu categories. The {class}`HelpMenuExtension <isaacsim.gui.menu.HelpMenuExtension>` includes functionality to open documentation URLs using the system browser and provides physics documentation URL resolution for version-specific links.

### Menu Infrastructure

**{class}`HookMenuHandler <isaacsim.gui.menu.HookMenuHandler>`** registers hooks to adjust menu item appearance across the menu system, providing a way to customize how menu items are displayed and behave.

The extension uses **omni.kit.menu.utils** for menu item creation and management, integrating with the broader Omniverse Kit menu framework through MenuItemDescription objects and standard menu registration patterns.

### Asset Creation Functions

The extension provides specialized functions for creating Isaac Sim assets, including `create_asset()` for referencing USD assets with optional camera positioning and `create_apriltag()` for generating AprilTag materials with custom tag textures.

## Functionality

Each menu extension is responsible for registering its menu items, managing menu state, and handling menu-specific actions. The extensions provide validation methods to determine when menu items should be enabled or disabled based on current stage state, selection, and live session status.

The Edit menu includes advanced prim operations like instancing, duplication across layers, parenting with validation, and screenshot capture with configurable save paths. The Create menu focuses on robotics asset creation and material generation specific to Isaac Sim workflows.

All menu extensions follow a consistent lifecycle pattern with initialization that takes an extension ID and a shutdown method that removes menu layouts and cleans up resources.
