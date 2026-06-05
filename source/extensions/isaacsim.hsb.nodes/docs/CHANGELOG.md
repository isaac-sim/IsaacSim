# Changelog

## [1.0.2] - 2026-05-14
### Changed
- Address ruff lint errors: add type annotations and docstrings to `OgnHSBCameraHelper`, extension lifecycle methods, and tests; rewrite `dict()` call as a dict literal.

## [1.0.1] - 2026-05-05
### Removed
- Enabled multitick, removed `frameSkipCount` input as it's controlled directly on sensor prim.

## [1.0.0] - 2026-04-03
### Added
- Initial release. OmniGraph nodes extracted from `isaacsim.hsb.bridge`.
- `HSBSend`: Send DLTensor data buffers via HSB emulator.
- `RGBToVB1940`: GPU-accelerated RGB→VB1940 CSI format conversion node.
- `HSBCameraHelper`: Python helper node for Replicator-based camera publishing.
- Node type IDs use `isaacsim.hsb.nodes` namespace.
