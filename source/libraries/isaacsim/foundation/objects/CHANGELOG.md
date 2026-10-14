# Changelog

## [Unreleased]

### Fixed

- Preserve the M1 display-color getter names and `shapes::ColorType` spelling while retaining the newer color setters.
  The getters retain their historical placeholder values; they do not read authored colors. The Mesh Python getter
  returns a zero-dimensional NumPy int32 array because Warp cannot represent the native scalar placeholder.

- Initialize the dependency-owned OVStage runtime before loading the Python bindings.

## [7.0.0a1] - 2026-09-16

### Added

- C++ and Python wrappers for common USD objects: `Stage`, `Prim`, `Xform`, `PhysicsScene`, `Camera`,
  `Mesh`, geometry shapes (`Sphere`, `Cube`, `Capsule`, `Cone`, `Cylinder`, `Plane`),
  and lights (`SphereLight`, `DiskLight`, `RectLight`, `CylinderLight`, `DistantLight`, `DomeLight`).
- The alpha C++ API uses descriptive, fully spelled identifiers such as `PhysxGpuConfiguration`, `setDeltaTimes()`,
  and the mesh `*Specifications()` methods; Python bindings use `snake_case` names.
- Installed C++ headers contain the public object and physics-scene interfaces; conversion and validation helpers are
  private implementation details.
