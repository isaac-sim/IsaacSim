# Overview

The isaacsim.asset.transformer extension provides the core framework for transforming USD assets through configurable rule pipelines. It defines the rule interface, profile model, rule registry, and execution manager that orchestrate ordered sequences of transformation rules over USD stages.

## Key Components

### RuleInterface

Abstract base class that all transformation rules must subclass. A rule receives a source `Usd.Stage`, a package root directory, a destination path, and a parameter dictionary. Subclasses implement `process_rule()` to read from the source stage, write opinions to destination layers, and optionally return a new stage path for subsequent rules in the pipeline.

### RuleRegistry

A singleton registry that maps fully qualified rule class names to their implementation classes. Rules register themselves on extension startup (typically in `isaacsim.asset.transformer.rules`) and are resolved by the manager at execution time.

### AssetTransformerManager

Coordinates the execution of a `RuleProfile` against a USD stage. The manager:

1. Opens the input stage and creates a flattened base layer in the output package
2. Collects external asset dependencies into the package directory
3. Canonicalizes orientation quaternions for deterministic output
4. Iterates through enabled rules in profile order, instantiating each rule class and calling `process_rule()`
5. Supports mid-pipeline stage replacement when a rule returns a new stage path
6. Produces an `ExecutionReport` with per-rule logs, affected stages, and success/error status

### Data Models

- **RuleSpec** — Specification for a single rule: type, display name, destination path, parameters, and enabled flag
- **RuleProfile** — Ordered collection of `RuleSpec` entries plus global settings (output root, base name, flatten option)
- **RuleConfigurationParam** — Descriptor for a rule's configurable parameter (name, type, default, description)
- **ExecutionReport** / **RuleExecutionResult** — Structured output from a profile run, including timing, logs, and error details

## Usage

```python
from isaacsim.asset.transformer import AssetTransformerManager, RuleProfile

manager = AssetTransformerManager()
profile = RuleProfile.from_json("path/to/profile.json")
report = manager.run("input.usd", profile, package_root="/tmp/output")

for result in report.results:
    print(f"{result.rule.name}: {'OK' if result.success else result.error}")
```

## Related Extensions

- `isaacsim.asset.transformer.rules` — Built-in rule implementations (structure, routing, performance, Isaac Sim–specific)
- `isaacsim.asset.transformer.ui` — Interactive UI for configuring and executing transformation pipelines
