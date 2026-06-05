# Overview

The isaacsim.core.version extension provides tools to query Isaac Sim version and build information. It enables developers to programmatically access detailed version components from Isaac Sim applications, including major, minor, patch versions, prerelease tags, and build metadata.

## Key Components

### {class}`Version <isaacsim.core.version.Version>` Object

The {class}`Version <isaacsim.core.version.Version>` class serves as a structured data container for parsed version information. It breaks down complex version strings into individual components:

```python
version = Version()
# Contains attributes: core, prerelease, major, minor, patch, pretag, prebuild, buildtag
```

Each attribute stores a specific part of the version string, making it easy to access individual components for comparison, display, or version-specific logic implementation.

### {class}`Version <isaacsim.core.version.Version>` Parsing

The {func}`parse_version <isaacsim.core.version.parse_version>` function converts raw version strings into structured {class}`Version <isaacsim.core.version.Version>` objects:

```python
version_obj = parse_version("1.2.3-alpha.1+build123")
# Breaks down the string into major=1, minor=2, patch=3, prerelease info, etc.
```

This parsing follows semantic versioning patterns, extracting meaningful components from complex version strings that may include prerelease identifiers and build metadata.

### {class}`Version <isaacsim.core.version.Version>` Retrieval

The {func}`get_version <isaacsim.core.version.get_version>` function directly accesses Isaac Sim's VERSION file to retrieve current application version information:

```python
core, prerelease, major, minor, patch, pretag, prebuild, buildtag = get_version()
```

This provides immediate access to the running application's version details without requiring manual file reading or parsing.

## Functionality

The extension handles version strings that follow semantic versioning conventions, supporting complex formats with prerelease tags and build metadata. The parsed components enable version comparison logic, compatibility checks, and feature detection based on specific Isaac Sim releases.

{class}`Version <isaacsim.core.version.Version>` information is accessed directly from Isaac Sim's internal VERSION file, ensuring accuracy and consistency with the actual application build being executed.
