# Changelog

## [1.0.1] - 2026-04-08
### Fixed
- Fix class name exposed in __init__.py file

## [1.0.0] - 2026-04-07
### Added
- Initial release as **isaacsim.node.examples** (renamed from `isaacsim.bridge.core`).
- TCP/IP sample OmniGraph nodes (no extra IPC libraries): **SimpleSendSimulationClockCpp/Py** (client, int64 LE) and **SimpleReceiveExternalStepCpp/Py** (server, uint32 LE, non-blocking recv).
- **SimpleSendSimulationClockCpp/Py:** `timeNanoseconds` input removed; `simulationTime` (`double`, seconds) matches **Isaac Read Simulation Time** for direct wiring. Wire payload is int64 LE nanoseconds after `llround`/`round` (no separate offset input).
- Tutorial TCP helper scripts merged into ``python/scripts/tcp_tutorial_playback_bridge.py`` (playback tick: step in / clock out per frame); removed separate clock listener and one-shot step client scripts.
- C++ headers live under `plugins/isaacsim.node.examples/` (no top-level `include/`).
