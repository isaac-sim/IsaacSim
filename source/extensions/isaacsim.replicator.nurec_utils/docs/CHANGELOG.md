# Changelog
## [0.1.2] - 2026-06-10
### Fixed
- Fixed pydoclint errors and updated docstrings.

## [0.1.1] - 2026-06-10
### Changed
- Moved matplotlib, PIL, and the URL-check helper to module-top / shared imports.
- Simplified the copy-rp test result (enum) and the timer's enabled check.
- Added full license headers and tidied docs and comments.

## [0.1.0] - 2026-06-05
### Added
- Render NuRec assets at recorded or user-supplied camera poses, saving images and a manifest.
- Detect the NuRec asset type and apply the matching render setup automatically.
- Score rendered images against ground truth (PSNR, SSIM, image difference) with plots.
- Tests, sample assets, and user documentation.
