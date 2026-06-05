# Overview

```{deprecated} 6.0.0
This extension is deprecated. Use the `repo.{sh,bat}` template system instead:

    ./repo.sh template new

The CLI-based template system provides the same extension scaffolding capabilities with better
maintainability, no runtime dependencies, and support for the latest extension conventions.
```

The isaacsim.examples.extension provided a UI-based code generation tool for creating Isaac Sim extension templates. It offered a window inside Isaac Sim that generated boilerplate code and directory structures for different workflow types.

```{image} ../../../../source/deprecated/isaacsim.examples.extension/data/preview.png
---
align: center
---
```

## Migration

Replace all usage of this extension with the CLI Extension Templates (``./repo.sh template new``),
which will interactively prompt for the extension name, description, and template type.

## Legacy Functionality

The extension created a dockable window with forms that allowed developers to generate four different types of extension templates:

- **Configuration Tooling Template** - For building configuration-based workflow extensions
- **Loaded Scenario Template** - For creating scenario loading and management extensions
- **Scripting Template** - For developing script-based automation extensions
- **UI Component Library Template** - For building reusable UI component extensions

## Key Components

### {class}`TemplateGenerator <isaacsim.examples.extension.TemplateGenerator>`

The {class}`TemplateGenerator <isaacsim.examples.extension.TemplateGenerator>` class handled the core template generation functionality. It managed file system operations including directory creation, template file copying, and keyword replacement within template files.
