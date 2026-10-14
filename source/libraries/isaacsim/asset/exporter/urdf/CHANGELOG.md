# Changelog

## [Unreleased]

### Added
- Record UDIM tile-group metadata in mesh materials for USD reconstruction and resolve texture paths through material interfaces during re-export.

### Fixed
- Preserve distinct same-named texture payloads during URDF export and reference the delivered filenames in mesh materials.
- Prefer canonical robot-schema joint names before legacy fallback and warn when legacy joint names are used.

### Security
- Reject remote texture filenames that decode to paths outside the export directory.

### Changed
- Package `isaacsim.asset.exporter.urdf` in the independently built `isaacsim-asset` wheel.
- Export geometry, joint offsets, and inertial data in SI units while preserving authored URDF limits and normalizing source joint/drive metadata for reconstruction.
- Reject incomplete joint limits before replacing an existing URDF export, while exporting zero independent effort for unactuated mimic followers with missing or unbounded effort. Preserve authored finite limits and continue validating actuated joints.
- Allow continuous joints without a limit element and preserve authored effort and velocity on expanded multi-DOF axes.
- Preserve PhysX mimic coupling direction and convert mimic offsets to URDF radians/meters.
- Derive missing passive mimic velocity and required position bounds from source limits and the affine mimic relationship, including chains, negative ratios, offsets, and mixed angular/linear units. Preserve authored finite limits and reject unresolved or cyclic references.
- Record inferred velocity and position limits in round-trip metadata, bound to the emitted values and mimic relationship so edited URDF limits remain authoritative.
- Reject one-sided position bounds instead of inferring a missing side, and validate passive mimic position intervals.
- Export effort=1e6 for actuated joints without authored force limits and record explicit unauthored-state provenance.
- Preserve omission of the optional limit element for continuous joints with neither force nor velocity limits.
