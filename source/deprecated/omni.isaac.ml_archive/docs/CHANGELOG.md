# Changelog

## [3.3.0] - 2026-04-13
### Changed
- Update to pytorch 2.11.0+cu128, torchaudio 2.11.0+cu128, torchvision 0.26.0+cu128 and update dependencies

## [3.2.1] - 2026-04-13
### Deprecated
- Extension deprecated

## [3.2.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md

## [3.1.0] - 2026-01-26
### Changed
- Update to pytorch 2.10.0+cu128, torchaudio 2.10.0+cu128, torchvision 0.25.0+cu128 and update dependencies

## [3.0.4] - 2025-08-22
### Changed
- Version bump to fix extension publishing issues

## [3.0.3] - 2025-08-19
### Changed
- Update to sympy 1.13.3 for ARM64

## [3.0.2] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [3.0.1] - 2025-04-10
### Changed
- Update unit test

## [3.0.0] - 2025-04-07
### Changed
- Update to pytorch 2.7.0+cu128, remove gymnasium, add nvidia-cusparselt-cu12, nvidia-cufile-cu12, nvidia-nvjitlink-cu12

## [2.2.4] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [2.2.3] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [2.2.2] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [2.2.1] - 2025-01-30
### Changed
- Updated to latest release 4.5 changes

## [2.2.0] - 2025-01-15
### Changed
- Update to Kit 107.x, Python 3.11

## [2.1.2] - 2025-01-14
### Changed
- Update extension description and add extension specific test settings

## [2.1.1] - 2024-12-01
### Changed
- Make this extension python version specific

## [2.1.0] - 2024-11-14
### Changed
- Update to torch==2.5.1+cu118, torchvision==0.20.1+cu118, torchaudio==2.5.0+cu118
- Update to filelock==3.13.1, fsspec==2024.2.0, networkx==3.2.1, sympy==1.12
- Update to nvidia-nccl-cu11==2.20.5

## [2.0.2] - 2024-10-28
### Changed
- Remove test imports from runtime

## [2.0.1] - 2024-05-16
### Fixed
- Make platform specific

## [2.0.0] - 2024-04-19
### Changed
- Removed omni.pip.torch dependency, torch is now directly part of this archive
- Added torch==2.2.2+cu118, torchvision==0.17.2+cu118, torchaudio==2.2.2+cu118, filelock==3.13.4, fsspec==2024.3.1, mpmath==1.3.0, networkx==3.3, sympy==1.12
- Added nvidia lib pip packages

## [1.1.3] - 2023-08-10
### Changed
- Added version to omni.pip.torch dependency

## [1.1.2] - 2023-07-05
### Changed
- Removed unused omni.kit.pipapi dependency

## [1.1.1] - 2023-06-12
### Changed
- No longer depend on omni.pip.torch

## [1.1.0] - 2022-07-11
### Changed
- Use pip_torch extension instead of packaging torch directly.
- Make extension kit sdk version specific

## [1.0.0] - 2022-07-01
### Changed
- Updating version for publishing

## [0.1.0] - 2022-01-13
### Added
- Initial version of Isaac Pip Archive
