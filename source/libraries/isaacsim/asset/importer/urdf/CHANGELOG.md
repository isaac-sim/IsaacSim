# Changelog

## [Unreleased]

### Added
- Reconstruct exporter-tagged diffuse UDIM materials with shared bindings and portable tile sets, including textures renamed to avoid filename collisions.

### Fixed
- Honor declared XML encodings and byte-order marks when scanning URDF files for UDIM metadata.
- Preserve backend input-validation errors and accept XML declarations preceded by whitespace or comments when reading UDIM metadata.

### Changed
- Package `isaacsim.asset.importer.urdf` in the independently built `isaacsim-asset` wheel.
- Restore unbounded independent mimic limits identified by exporter metadata instead of applying inferred URDF limits as additional solver constraints. Preserve explicitly authored limits.
- Remove synthesized force-limit opinions after physics conversion when verified export metadata records an unauthored source limit.
