# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- C++ and Python wrappers for common USD objects: `Stage`, `Prim`, `Xform`, `PhysicsScene`, `Camera`,
  `Mesh`, geometry shapes (`Sphere`, `Cube`, `Capsule`, `Cone`, `Cylinder`, `Plane`),
  and lights (`SphereLight`, `DiskLight`, `RectLight`, `CylinderLight`, `DistantLight`, `DomeLight`).
- The alpha C++ API uses descriptive, fully spelled identifiers such as `PhysxGpuConfiguration`, `setDeltaTimes()`,
  and the mesh `*Specifications()` methods; Python bindings use `snake_case` names.
- Installed C++ headers contain the public object and physics-scene interfaces; conversion and validation helpers are
  private implementation details.
