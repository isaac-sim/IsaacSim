# Changelog

## [0.11.1] - 2026-09-23
### Fixed
- `VisualMaterial`: handle shaderless USD materials in discovery APIs without raising an exception.

## [0.11.0] - 2026-09-16
### Removed
- `NonVisualMaterial`: no longer authors a `UsdPreviewSurface` when the material has no shader, since non-visual material IDs now resolve without a surface shader. Define a visual material (`PreviewSurface`, `OmniPBR`, or `OmniGlass`) before wrapping it if the prim must also render.

## [0.10.0] - 2026-08-11
### Added
- Declare the `rtx.materialDb.nonVisualMaterialSemantics.prefix` setting, defaulting to `omni:simready:nonvisual`.

### Fixed
- `NonVisualMaterial`: fall back to the default attribute prefix when the RTX setting is unset, instead of building attribute names from `None`.

## [0.9.0] - 2026-07-10
### Changed
- `NonVisualMaterial`: author a minimal `UsdPreviewSurface` connected to `outputs:surface` when the material has no shader, so non-visual material IDs resolve after a cold stage load (not only when authored live). Materials that already have a shader are left untouched.

## [0.8.0] - 2026-07-08
### Changed
- `NonVisualMaterial`: author attributes using the SimReady spec USD types (`base`/`coating` as `token`, `attributes` as `token[]`), and support multiple attributes per material combined into the material-ID bitfield.
- `NonVisualMaterial.decode_material_ids`: return the decoded attributes as a list of strings instead of a single string.
- Fix base material spelling `calibration_lambertion` -> `calibration_lambertian`.

## [0.7.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [0.7.0] - 2026-03-18
### Added
- Add non-visual material

## [0.6.0] - 2026-03-04
### Changed
- Add Overview.md, python_api.md and updated docstrings

## [0.5.1] - 2025-12-11
### Removed
- Remove checking for the deformable beta feature, as it is now active by default

## [0.5.0] - 2025-11-24
### Changed
- Define ranges for visual material inputs and clip them accordingly

## [0.4.1] - 2025-10-29
### Changed
- Standardize test args in extension.toml

## [0.4.0] - 2025-09-18
### Added
- Add support for input data expressed as basic Python types (bool, int, float)

## [0.3.0] - 2025-07-16
### Added
- Add surface and volume deformable physics materials

## [0.2.1] - 2025-06-07
### Changed
- Set test timeout to 900 seconds

## [0.2.0] - 2025-06-06
### Changed
- Update source code to use the experimental core utils API

## [0.1.2] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [0.1.1] - 2025-05-16
### Changed
- Make extension target a specific kit version

## [0.1.0] - 2025-05-12
### Added
- Initial release
