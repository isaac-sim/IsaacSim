# Changelog

## [Unreleased]

## [7.0.0a1] - 2026-09-16

### Added

- An `H`-key debug HUD shows the visible viewport's frame rate.
- A Foundation-integrated Python `Viewport` provides declarative camera configuration, automatic camera-pose
  publication in Foundation's native xyzw quaternion order, deterministic `close()`, and context-manager support.
- `author_viewport()` authors the standard camera, low dynamic range (LDR) RenderVar, and RenderProduct for Python
  applications.
- The keyword-based Python interface accepts and retains a Foundation OVStage directly.
- The viewport supports Windows and aarch64 systems, with SDL3 providing window, input, presentation, and HUD support.
- The viewport provides continuous keyboard movement, FPS-style mouse look, absolute drag input, and display-scale-aware
  rendering.
- Visible viewports present directly through OpenGL without GPU readback, CPU frame copies, or SDL texture uploads.
- Remote material images and dome-light HDRs are cached through the OmniClient runtime bundled with OVStage.
- The OVStage scene mirror supports authored and resolved asset tokens, populated hierarchies, scene-graph instances,
  and live local or Fabric world-transform updates.
- Eligible transform-only updates apply directly to OVGL's resident meshes, bypassing private mirror writes, seals, and
  hierarchy recomputation while retaining full synchronization as a correctness fallback.
- Optional versioned OVStage transform journals avoid match-all scene queries for physics-driven frames while retaining
  full synchronization for missing, incompatible, or discontinuous journals.

### Changed

- The public C++ viewport configuration type is `ViewportConfiguration`; Python exposes it as `ViewportConfig`.
- The `isaacsim-ovgl-viewport` distribution declares `isaacsim-common` and `isaacsim-foundation` runtime dependencies
  and uses `scikit-build-core>=1.0.3` for source builds.
