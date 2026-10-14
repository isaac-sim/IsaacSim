# Changelog

## [1.12.2] - 2026-10-02
### Changed
- `persistent.isaac.asset_root.timeout`: increase the default asset-root check timeout from 5 to 10 seconds.

### Fixed
- `TestVerifyAssetRootPath`: mock callback-based metadata checks to keep asset-root verification tests isolated.

## [1.12.1] - 2026-10-02
### Fixed
- Enforce the timeout for synchronous asset-server checks and cancel unresponsive requests.

## [1.12.0] - 2026-10-02
### Changed
- Update the default, US, and China asset roots to Isaac Sim 6.2.

## [1.11.0] - 2026-09-01
### Changed
- Use Asset Region Profile naming for regional asset routing and add an explicit default profile.

## [1.10.4] - 2026-09-01
### Fixed
- Asset-root validation accepts roots containing either the Isaac or NVIDIA asset tree.

## [1.10.3] - 2026-08-31
### Changed
- Update the regional asset profile to the Isaac Sim 6.1 asset root.

## [1.10.2] - 2026-08-26
### Fixed
- `verify_asset_root_path`: validate versioned HTTP asset roots when `version.txt` is unavailable.

## [1.10.1] - 2026-08-24
### Changed
- Update the default Isaac Sim asset root to the production 6.1 URL.

## [1.10.0] - 2026-07-28
### Added
- Support selecting an asset region profile via the `ISAACSIM_ASSET_REGION_PROFILE` environment variable.
- Ship a regional profile that serves reads from a regional CDN while list and stat go direct to object storage, and that sets the asset root and USD Search endpoint.

## [1.9.4] - 2026-07-21
### Fixed
- Accept configured asset roots when the Storage provider does not support directory metadata checks.

## [1.9.3] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.9.2] - 2026-05-04
### Fixed
- `path_join`: treat any URL-scheme path (including `file://`) as forward-slash on Windows
- `path_join`: normalize backslashes in relative path components to forward slashes, fixing traversal failures when `omni.client.list` returns Windows-style paths on Nucleus/remote URLs

## [1.9.1] - 2026-04-30
### Fixed
- `path_join` uses forward-slash concatenation for all URL schemes, not just `omniverse://`, fixing backslash-corrupted paths on Windows when the assets root is an `https://` URL
- `path_join` `../` traversal uses `rsplit` instead of `os.path.dirname` to avoid the same backslash issue

## [1.9.0] - 2026-04-02
### Added
- Support overriding the default asset root via the `ISAACSIM_ASSET_ROOT` environment variable

## [1.8.0] - 2026-03-04
### Changed
- Added Overview.md, python_api.md, and SETTINGS.md and updated docstrings

## [1.7.1] - 2026-03-02
### Changed
- Update assets path to 6.0 staging

## [1.7.0] - 2026-02-13
### Changed
- Add retry attempts and retry base delay settings for asset root connectivity checks

## [1.6.4] - 2026-01-09
### Changed
- Update assets path to development

## [1.6.3] - 2025-12-07
### Changed
- Update description

## [1.6.2] - 2025-12-06
### Changed
- Add missing docstrings
- Add docstring tests

## [1.6.1] - 2025-12-01
### Fixed
- Update assets path to 6.0 staging

## [1.6.0] - 2025-11-26
### Added
- Added `resolve_asset_path` function to synchronously resolve asset paths with the same logic as the async variant.

## [1.5.2] - 2025-11-05
### Changed
- Update assets path to development

## [1.5.1] - 2025-10-10
### Changed
- Update assets path to production

## [1.5.0] - 2025-10-07
### Added
- Added `resolve_asset_path_async` utility function to resolve asset paths by checking original location and falling back to asset root

## [1.4.1] - 2025-09-18
### Fixed
- Update is_absolute_path function to also check http:// and https://

## [1.4.0] - 2025-09-08
### Added
- Add `skip_check` optional argument to allow skipping the verification check step when resolving the asset root path

## [1.3.0] - 2025-09-05
### Added
- Added the find_filtered_files and find_filtered_files_async utility to recursively search for USD files with configurable depth and pattern matching.
- Added is_local_path utility to determine if a given path is a local file path or a remote (e.g., Omniverse) URL.

## [1.2.9] - 2025-09-05
### Fixed
- Update assets path to staging

## [1.2.8] - 2025-07-25
### Fixed
- Update assets path to production

## [1.2.7] - 2025-07-07
### Fixed
- Correctly enable omni.kit.loop-isaac in test dependency (fixes issue from 1.2.6)

## [1.2.6] - 2025-07-03
### Changed
- Make omni.kit.loop-isaac an explicit test dependency

## [1.2.5] - 2025-06-25
### Changed
- Add --reset-user to test args

## [1.2.4] - 2025-05-31
### Changed
- Use default nucleus server for all tests

## [1.2.3] - 2025-05-20
### Changed
- Update copyright and license to apache v2.0

## [1.2.2] - 2025-05-20
### Changed
- Update assets path to staging

## [1.2.1] - 2025-05-10
### Changed
- Enable FSD in test settings

## [1.2.0] - 2025-05-09
### Changed
- Add file utility functions

## [1.1.0] - 2025-04-09
### Changed
- Add authentication callback for ETM tests

## [1.0.12] - 2025-04-09
### Changed
- Update all test args to be consistent

## [1.0.11] - 2025-04-04
### Changed
- Version bump to fix extension publishing issues

## [1.0.10] - 2025-03-26
### Changed
- Cleanup and standardize extension.toml, update code formatting for all code

## [1.0.9] - 2025-03-18
### Fixed
- recursive_list_folders now enforces trailing slashes on paths, preventing path clobber bug

## [1.0.8] - 2025-03-11
### Changed
- Switch asset root for tests to internal nucleus

## [1.0.7] - 2025-02-25
### Changed
- Removed remaining omni.client._omniclient type hints in favor of public API

## [1.0.6] - 2025-01-26
### Changed
- Update test settings

## [1.0.5] - 2025-01-21
### Changed
- Update extension description and add extension specific test settings

## [1.0.4] - 2024-12-20
### Added
- Added synchronous version of is_dir() function

## [1.0.3] - 2024-11-18
### Changed
- omni.client._omniclient to omni.client

## [1.0.2] - 2024-11-01
### Changed
- Deprecated get_nvidia_asset_root_path()
- Deprecated get_isaac_asset_root_path()

## [1.0.1] - 2024-10-24
### Changed
- Updated dependencies and imports after renaming

## [1.0.0] - 2024-10-16
### Changed
- Extension renamed to isaacsim.storage.native

## [0.3.2] - 2024-09-23
### Changed
- Updated to 4.5 asset path
- Updated omni.isaac.version dependency to isaacsim.core.version

## [0.3.1] - 2024-08-05
### Changed
- Updated to 4.2 asset path

## [0.3.0] - 2024-06-30
### Changed
- Set Cloud assets path as default
- Updated to 4.1 asset path

## [0.2.0] - 2024-06-06
### Changed
- Updated get root path functions to throw a runtime error if the root path is not found

## [0.1.2] - 2024-05-20
### Changed
- Updated to 4.0 asset path

## [0.1.1] - 2024-04-24
### Changed
- Reduced timeout.

## [0.1.0] - 2024-02-02
### Added
- Added first version of nucleus.
