# Changelog

## [1.5.4] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings.

## [1.5.3] - 2026-04-30
### Changed
- Update cryptography to 46.0.7

## [1.5.2] - 2026-04-14
### Changed
- Downgrade boto3 to 1.40.61
- Downgrade botocore to 1.40.61

## [1.5.1] - 2026-03-06
### Changed
- Update cryptography to 46.0.5
- Update azure-core to 1.38.0
- Update msal to 1.35.1

## [1.5.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md and updated docstrings

## [1.4.5] - 2026-02-22
### Changed
- Downgrage boto3 to 1.40.61
- Downgrage botocore to 1.40.61
- Downgrage s3transfer to 0.14.0
- Update aioboto3 to 15.5.0

## [1.4.4] - 2025-11-25
### Changed
- Update to aioboto3==15.2.0
- Update to aiobotocore==2.24.2
- Update to boto3==1.40.18
- Update to botocore==1.40.18
- Update to msal==1.29.0

## [1.4.3] - 2025-09-26
### Changed
- Update to aioboto3==15.1.0
- Update to aiobotocore==2.24.0
- Update to boto3==1.39.11
- Update to botocore==1.39.11
- Update to awscrt==0.23.8
- Update to s3transfer==0.13.1

## [1.4.2] - 2025-09-03
### Changed
- Update to boto3[crt]==1.40.16
- Update to botocore==1.40.16
- Update to s3transfer==0.13.1

## [1.4.1] - 2025-08-25
### Changed
- Update to s3transfer==0.11.3

## [1.4.0] - 2025-08-08
### Changed
- Update to msal==1.27.0
- Update to s3transfer==0.11.0
- Remove typing-extensions as its in omni.kit.pip_archive

## [1.3.7] - 2025-07-30
### Changed
- Version bump to fix aarch64 pip_*.toml platform inclusion

## [1.3.6] - 2025-05-19
### Changed
- Update copyright and license to apache v2.0

## [1.3.5] - 2025-04-03
### Changed
- Version bump to fix pywin32 issues

## [1.3.4] - 2025-04-02
### Changed
- Version bump to fix pywin32 issues

## [1.3.3] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.3.2] - 2025-03-04
### Changed
- Update to kit 107.1 and fix build issues

## [1.3.1] - 2025-01-30
### Changed
- Updated to latest release 4.5 changes

## [1.3.0] - 2025-01-16
### Changed
- Updated boto3[crt]==1.36.1, botocore==1.36.1
- Added jmespath==1.0.1, python-dateutil==2.9.0.post0, six==1.17.0

## [1.2.0] - 2025-01-15
### Changed
- Update to Kit 107.x, Python 3.11

## [1.1.7] - 2025-01-14
### Changed
- Update extension description and add extension specific test settings

## [1.1.6] - 2025-01-11
### Changed
- Make this extension kit version specific

## [1.1.5] - 2024-12-01
### Changed
- Make this extension python version specific

## [1.1.4] - 2024-10-28
### Changed
- Remove test imports from runtime

## [1.1.3] - 2024-05-24
### Fixed
- Manually add paths for pywintypes import

## [1.1.2] - 2024-05-15
### Changed
- Replace pypiwin32==223 with pywin32==306

## [1.1.1] - 2024-05-14
### Changed
- Update cryptography to 42.0.7

## [1.1.0] - 2024-04-19
### Changed
- Update typing extensions to typing_extensions==4.10.0

## [1.0.2] - 2023-08-02
### Fixed
- Unit test failure due to imports

## [1.0.1] - 2023-08-02
### Fixed
- typing-extensions not loading for azure cloud, force reload in extension startup

## [1.0.0] - 2023-08-02
### Added
- Initial version of Cloud Pip Archive
