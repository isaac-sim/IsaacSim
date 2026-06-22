# Changelog

## [1.4.4] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.4.3] - 2026-06-03
### Fixed
- `waitForRequestWithTimeout` now treats `timeoutMs = 0` as "wait indefinitely", matching the documented OGN-node default (`"If 0, waits indefinitely"`). Previously only `g_kUcxInfiniteTimeout` (`UINT32_MAX`) was treated as the infinite sentinel; `timeoutMs = 0` fell through the polling loop on the first iteration and returned "Request timed out", causing intermittent send-side timeouts in any UCX OGN node using the default value.
- Added a `TEST_CASE` covering `timeoutMs = 0` explicitly; renamed the existing `"infinite timeout (zero)"` case to `"infinite timeout (UINT32_MAX sentinel)"` since it actually passed `UINT32_MAX`.

## [1.4.2] - 2026-05-14
### Changed
- Add return type and docstring annotations to Python binding tests to satisfy ruff lint rules.

## [1.4.1] - 2026-05-05
### Fixed
- Set UCX_MODULE_DIR via dladdr so the UCS module loader finds libuct_cuda.so.0 and libucm_cuda.so.0 in the ucx/ subdirectory alongside the library
- Pre-load libucm_cuda.so.0 with RTLD_GLOBAL before ucxx::createContext() to make CUDA symbols available without requiring ucx/ in LD_LIBRARY_PATH at launch
- Explicitly request cuda_copy,sm,self,tcp TLS so the CUDA transport is active for GPU buffer rendezvous sends

## [1.4.0] - 2026-04-20
### Added
- Python bindings exposing `UCXListener` and `UCXListenerRegistry` to Python via `isaacsim.ucx.core`

## [1.3.2] - 2026-04-17
### Fixed
- Clear endpoint close callback during shutdown to prevent dangling pointer if external code holds stale endpoint references

## [1.3.1] - 2026-03-02
### Changed
- Add Overview.md, public python_api.md and update docstrings

## [1.3.0] - 2025-12-12
### Added
- Add UCXListenerRegistry::tryRemoveListener for reference-counted listener cleanup

## [1.2.2] - 2025-12-07
### Changed
- Fix issues found by clang tidy

## [1.2.1] - 2025-11-28
### Changed
- Regenerate pip prebundle

## [1.2.0] - 2025-11-25
### Added
- Added `UCXListener::tagSendWithRequest` for better monitoring of send requests.

## [1.1.0] - 2025-11-24
### Added
- Added `UcxUtils.h`.
- Added UCX Python dependencies.

### Changed
- Refactored UCXListener's tag messaging functions.

## [1.0.0] - 2025-10-15
### Added
- **UCX Communication Wrappers**
  - **UCXConnection**: High-performance point-to-point communication
    - Tagged send/receive operations for efficient message routing
    - Multi-buffer operations supporting CPU and CUDA memory transfers
    - Thread-safe connection lifecycle management

  - **UCXListener**: Server-side listener for incoming connections
    - Configurable port binding with timeout support
    - Thread-safe client connection tracking

  - **UCXListenerRegistry**: Centralized listener management
    - Singleton pattern ensuring one listener per port
    - Thread-safe listener creation and retrieval

- **Plugin Interface**: Carbonite plugin that publishes simulation clock and physics steps

- **Testing**: Unit tests for core components using doctest framework
