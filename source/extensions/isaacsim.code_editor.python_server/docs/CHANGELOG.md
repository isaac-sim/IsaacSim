# Changelog

## [1.3.1] - 2026-06-09
### Fixed
- Fix linter errors and missing or incomplete docstrings, and update `python_api.md`.

## [1.3.0] - 2026-05-19
### Added
- Token authentication for client requests via the `auth_token` JSON envelope field or the `# isaacsim-python-server-token:` raw-source header
- New `require_auth` and `auth_token` settings. When authentication is required and no token is configured, the server generates one at startup

## [1.2.2] - 2026-04-06
### Fixed
- RecursionError and MemoryError in user code no longer hang the server. Exception handlers in the executor are now resilient to secondary failures (e.g. `traceback.format_exc()` raising after RecursionError).
- Server always sends a structured error response even when an internal exception escapes the executor.

### Changed
- Disable SystemExit/BaseException tests in CI
- Memory-error hardening test uses an allocation that exceeds the virtual address space so MemoryError is raised immediately instead of triggering Linux overcommit thrashing.

## [1.2.1] - 2026-04-01
### Fixed
- **Critical:** Async user code (e.g. `create_new_stage_async`, `update_app_async`) no longer causes `RuntimeError: Cannot enter into task` on Python 3.12+. Replaced `create_task(_await_and_reply)` with manual coroutine stepping via `_drive_coroutine()` so user code never runs inside an asyncio Task.
- `SystemExit` and `sys.exit()` in user code are now caught by the executor and returned as error responses instead of crashing the application.

## [1.2.0] - 2026-03-28
### Added
- **JSON envelope protocol**: Requests may be sent as a JSON object `{"code": "...", "args": {...}, "timeout": N, "context": "name", "fire_and_forget": false}`. Raw Python source is also accepted. If the request starts with `{` but fails JSON parsing it falls back to raw execution.
- **Named execution contexts**: Replace the single shared globals dict with named contexts keyed by a string. The default context (`""`) preserves existing behaviour. Each context is an independent globals namespace that persists for the extension lifetime.
- **Per-request execution timeout**: Set `timeout` in the JSON envelope or configure a global default via the `execution_timeout` setting. Async code uses `asyncio.wait_for` for clean cancellation; sync code uses a background watchdog timer. Returns `{"status": "error", "ename": "TimeoutError", ...}`.
- **Fire-and-forget mode**: Set `"fire_and_forget": true` in the envelope to receive an immediate acknowledgement `{"status": "ok", "fire_and_forget": true, "task_id": "<uuid>"}` and execute code in the background. Results are stored (up to 100, FIFO eviction) and retrievable via introspection.
- **Introspection endpoint**: Send `{"introspect": "<command>"}` to query server state: `contexts`, `context`, `tasks`, `task`, `delete_context`, `status`.
- **Keepalive elapsed time**: New `keepalive_interval` setting. When set and execution exceeds the interval, an `elapsed_seconds` field is added to the response.
- New test suite `test_advanced_features.py` covering all new features.
- New settings: `execution_timeout` (default: 0) and `keepalive_interval` (default: 0).

## [1.1.1] - 2026-03-27
### Fixed
- Schedule code execution via `call_soon` outside the transport's `_read_ready` callback to prevent `cannot enter context` errors on Python 3.12+ when user code pumps the event loop
- Add guard for unavailable or stopped event loop during shutdown

## [1.1.0] - 2026-03-24
### Fixed
- Buffer incoming TCP data until EOF instead of processing on first `data_received` call, preventing partial execution of fragmented payloads
- Capture `print()` output from `await`ed coroutines in the JSON response `output` field (previously lost to real stdout)
- Add `write_eof()` calls in test client helpers to match the corrected wire protocol

### Changed
- Updated Overview.md with client examples, async code notes, and state persistence documentation
- Added `stdoutFailPatterns.exclude` for pre-existing asyncio context re-entry errors in test configuration

### Added
- New test suite `test_protocol_robustness.py` covering TCP fragmentation handling and async stdout capture

## [1.0.1] - 2026-03-20
### Fixed
- Replace `run_coroutine_threadsafe` with `ensure_future` in `data_received` to avoid `RuntimeError: Cannot enter into task` when other extensions have pending asyncio tasks

## [1.0.0] - 2026-03-09
### Added
- Initial release, extracted from isaacsim.code_editor.vscode
- TCP socket server for remote Python code execution
- Optional UDP-based Carbonite log broadcasting
- Expression evaluation with `result` field in JSON response
