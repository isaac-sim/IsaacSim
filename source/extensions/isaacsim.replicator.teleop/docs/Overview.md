# Overview

The isaacsim.replicator.teleop extension provides the runtime for VR-driven teleoperation of robots in Isaac Sim. It manages OpenXR session connectivity, coordinate-frame conversion, visual frame markers, unified teleop profiles, a set of controllers that translate VR headset and controller poses into robot joint targets, rigid-body velocities, or base movements, and {func}`build_teleop_recorder <isaacsim.replicator.teleop.build_teleop_recorder>` for assembling an {class}`EpisodeRecorder <isaacsim.replicator.episode_recorder.EpisodeRecorder>` that captures simulation state plus VR controller inputs to HDF5 for offline replay and synthetic data generation.

The extension also provides stateless helpers for scripted Replicator validation;
they are not part of the live teleop path. `load_floating_grippers_async` loads
config-defined xArm and Dex3 grippers,
`setup_floating_gripper_controllers_async` returns an
opaque context passed to the controller helper functions, and
`move_floating_grippers_to_targets_async` executes tolerance-gated pose motion.
Generic stages, lights, geometry, rigid bodies, asset references, and cameras are
authored with experimental core APIs. Render products, annotators, and RGB/depth
capture are managed directly with Replicator.

Debug input, scripted motion, profiles, markers, visual cues, floating/grasp/
locomotion controllers, and the built-in non-PINK IK solvers are available on
Windows and Linux without the Isaac Teleop package. Live OpenXR input and MCAP
replay require the Linux-only Isaac Teleop prebundle, while PINK is exposed only
when its optional backend is installed. Use
{func}`get_teleop_capabilities <isaacsim.replicator.teleop.get_teleop_capabilities>`
to query these features without importing native modules.

## Floating-gripper scenes

The xArm, Robotiq 2F-85, and Robotiq Hand-E scenario stages pair with the
matching `floating_<name>.yaml` profiles. Their reusable asset layers live under
`<assets_root>/Isaac/Samples/Replicator/Teleop/` and reference vendor assets
under `Isaac/Robots`; keep those dependencies reachable when moving a stage.
The scenarios use right-side floating and grasp control with VR-origin
locomotion. See the Teleop tutorial's **Scenario stages** section for paths,
stage composition, object sizes, and setup steps.

## Key Components

### {class}`TeleopManager <isaacsim.replicator.teleop.TeleopManager>`

**Central orchestrator for the teleop session.** Manages the input-provider lifecycle, distributes controller poses and input signals to downstream controllers on eligible Kit app updates, and exposes a command bus (`CONNECT`, `START`, `STOP`, `RESET`, `DISCONNECT`) that can be driven from the UI or programmatically via {func}`dispatch_command <isaacsim.replicator.teleop.dispatch_command>`. Live OpenXR, synthetic debug markers, and headless MCAP replay all produce the same finalized {class}`TeleopFrame <isaacsim.replicator.teleop.TeleopFrame>` contract. The manager exposes `add_input_frame_observer`, plus compatibility `add_controller_inputs_observer` / `add_head_observer` hooks used by teleop {class}`Recordable <isaacsim.replicator.episode_recorder.Recordable>` plugins and {class}`VRRecordingButton <isaacsim.replicator.teleop.VRRecordingButton>`.

### {class}`TeleopFrame <isaacsim.replicator.teleop.TeleopFrame>` and input providers

**Typed boundary between input transport and simulation control.** Every frame retains the original controller/head snapshots. Live and MCAP frames distinguish source, coordinate-converted local, and tracking-space-composed world poses; debug frames fill world-space controller poses directly. {class}`LiveTeleopFrameProvider <isaacsim.replicator.teleop.LiveTeleopFrameProvider>` owns the OpenXR/DeviceIO lifecycle and can optionally record raw device data to MCAP. {class}`McapTeleopFrameProvider <isaacsim.replicator.teleop.McapTeleopFrameProvider>` replays the same controller/head channels without OpenXR. `TeleopManager` remains responsible for coordinate conversion, markers, controllers, timeline policy, and USD/physics writes. The pinned Isaac Teleop tracker API does not expose source sample timestamps, so built-in frames set `source_time_ns` to `None` while always providing the local `monotonic_time_ns`.

MCAP replay is caller-paced: one input frame is consumed per Kit app update while the timeline is playing. Stopping the timeline pauses consumption, and the teleop `RESET` command reopens the MCAP at its beginning. The bundled Isaac Teleop API does not expose reliable EOF, seek, loop, or timestamp-paced replay.

Live MCAP recording is session-scoped and captures every DeviceIO update, including updates acquired while the Kit timeline is stopped. The low-level file does not encode Isaac Sim timeline transitions, so replay cannot automatically reproduce those inactive intervals. Trim/align the input or record a separate state-event sidecar when timeline-exact action re-simulation is required.

### {class}`RobotIKController <isaacsim.replicator.teleop.RobotIKController>`

**Inverse-kinematics controller for articulated robot arms.** Takes a 6-DOF VR target pose and computes joint position targets. Supports per-side (left/right) configuration and four solver back-ends selectable via {class}`IKSolverType <isaacsim.replicator.teleop.IKSolverType>`:

- **Position-based** — single-iteration Jacobian differential IK
- **Velocity-based** — velocity-space IK with position integration
- **Levenberg–Marquardt** — multi-iteration LM solver per frame
- **PINK** — Pinocchio-based QP IK with a teleop-specific minimal USD-to-URDF export and selectable `daqp` / `osqp` QP back-ends

### {class}`FloatingRigidBodyController <isaacsim.replicator.teleop.FloatingRigidBodyController>`

**PD velocity controller for free rigid bodies** not attached to any articulation. Per-side configuration with tunable position/orientation gains and rotation offsets.

### {class}`GraspController <isaacsim.replicator.teleop.GraspController>`

**Maps controller analog input to gripper joint targets.** Uses {class}`GraspConfig <isaacsim.replicator.teleop.GraspConfig>` and {class}`JointMapping <isaacsim.replicator.teleop.JointMapping>` loaded from YAML presets. Each configured side selects one drive mode:

- `trigger` (default) sends the controller trigger activation to every configured joint.
- `retargeted` runs a named retargeter and sends explicit per-joint targets. The current `trihand` retargeter uses trigger for the index finger, squeeze for the middle finger, and their combination for the thumb.

Retargeting is input- and profile-driven; it is not skeletal hand-tracking retargeting. Live OpenXR, debug controls, and MCAP replay all provide the same controller snapshot, so they execute the same mapping:

```text
controller trigger/squeeze
  -> TriHand semantic activations
  -> profile joint_aliases
  -> grasp-config target ranges
  -> articulation targets or USD DriveAPI
```

The seven supported semantic aliases are `thumb_rotation`, `thumb_proximal`, `thumb_distal`, `index_proximal`, `index_distal`, `middle_proximal`, and `middle_distal`. The profile maps them to robot-specific USD joint names, while the grasp config remains the source of open/closed target ranges. Revolute-joint targets are defined in degrees and prismatic-joint targets in stage linear units; the controller converts angular targets to radians only when using the articulation tensor backend.

Profile resolution and controller configuration validate the complete chain: semantic names must be supported, alias targets must exist in the selected grasp config, and those targets must resolve to controllable USD joints below the grasp prim. The built-in `floating_xarm_dex3_retargeted.yaml` profile demonstrates the complete Dex3 mapping. Debug mode and the automated synthetic/real-asset tests do not require a running `isaacteleop` or CloudXR process. MCAP replay does require the Isaac Teleop Python package, but not a headset or live CloudXR process.

### {class}`LocomotionController <isaacsim.replicator.teleop.controllers.LocomotionController>`

**Movement driven by VR thumbstick input.** Left thumbstick translates in the world ground plane (using the prim's heading projected onto XY), right thumbstick rotates (yaw), and the right primary/secondary buttons move vertically along world Z. Movement axes are always orthogonal regardless of the target prim's local frame orientation. Configurable speed multipliers. Two drive modes are available through {class}`LocomotionDriveMode <isaacsim.replicator.teleop.LocomotionDriveMode>`:

- **Teleport** (kinematic) — writes the base world pose each frame, carrying every parented child even when it is not physically jointed. No physics is involved.
- **Velocity** (physics) — commands a linear/angular velocity on a dynamic (non-kinematic) rigid-body base so PhysX integrates the motion with real contacts; only physically jointed payloads follow. A plain `Xform` or kinematic body cannot be velocity-driven.

The default `AUTO` mode selects Velocity for a dynamic rigid-body target and Teleport otherwise. Two workflows are supported depending on the target prim:

- **Robot base locomotion** — the target prim is a robot base link. Thumbstick input moves the robot, and the attached arm/grippers follow. Toggling *Carry Tracking Space* (left primary button) also moves the VR origin so the user can navigate between work areas while remaining anchored to the robot.
- **VR origin locomotion** — the target prim is the built-in tracking-space origin marker (`/Teleop/Markers/TrackingOrigin`). Because the prim being moved *is* the VR origin, carry is implicit: every movement directly shifts the VR workspace. This is the primary workflow for {class}`FloatingRigidBodyController <isaacsim.replicator.teleop.FloatingRigidBodyController>` setups where grippers have no physical base — moving the VR origin repositions the grippers in the scene.

### {func}`build_teleop_recorder <isaacsim.replicator.teleop.build_teleop_recorder>`

**Factory for a teleop-ready {class}`EpisodeRecorder <isaacsim.replicator.episode_recorder.EpisodeRecorder>`.** Registers scene recordables under `state/<name>` (articulations, plain Xforms, rigid bodies) plus per-side VR input recordables:

- `teleop/<side>/trigger`, `squeeze`, `thumbstick_x`, `thumbstick_y` — analog axes in `[0, 1]` or `[-1, 1]` (`float32`).
- `teleop/<side>/primary_click`, `secondary_click`, `thumbstick_click` — button booleans (`uint8`).
- `teleop/<side>/aim_position` (`float32[3]`) and `aim_orientation` (`float32[4]`, `wxyz`) — OpenXR controller aim pose when `record_aim_pose=True`.
- `teleop/head/position` (`float32[3]`) and `teleop/head/orientation` (`float32[4]`, `wxyz`) — headset pose when `record_head_pose=True` and the manager implements `add_head_observer`.

The returned recorder uses the same session lifecycle (`open_session` / `close_session`), timeline-driven episodes, scene-level stage snapshot linking ({func}`export_stage_snapshot <isaacsim.replicator.episode_recorder.export_stage_snapshot>` / {meth}`EpisodeRecorder.export_stage_snapshot <isaacsim.replicator.episode_recorder.EpisodeRecorder.export_stage_snapshot>` — writes `<output_dir>/stage_snapshot.usd` + sidecar JSON once per scene, auto-linked on `open_session` via the HDF5 `stage_snapshot` attr when `link_stage_snapshot=True`), and the shared `EPISODE_CMD_EVENT` bus consumed by {func}`dispatch_episode_command <isaacsim.replicator.episode_recorder.dispatch_episode_command>`, {class}`VRRecordingButton <isaacsim.replicator.teleop.VRRecordingButton>`, and the standalone Episode Recorder window (`isaacsim.replicator.episode_recorder.ui`).

**HDF5 layout (typical teleop session).** Under each `episodes/episode_NNNNN/` group, channels live at `<recordable_group>/<channel_name>` (see {class}`SessionStorage <isaacsim.replicator.episode_recorder.SessionStorage>`). For {func}`build_teleop_recorder <isaacsim.replicator.teleop.build_teleop_recorder>` the groups are:

```text
<file>.hdf5                             # one file per open_session()
├── @schema_version, @stage_snapshot, manifest/, ...
└── episodes/
    ├── episode_00000/                  # @episode_index, @started_at, @ended_at, ...
    │   ├── meta/time/
    │   │   ├── sim_time            (N,)  float64
    │   │   ├── physics_step        (N,)  int64
    │   │   └── wall_time           (N,)  float64
    │   ├── state/<name>/                  # one group per selected articulation / xform / rigid body
    │   │   ├── positions (L, 3) ...                  # articulation: per-link world pose
    │   │   ├── orientations (L, 4) ...               # articulation: per-link wxyz quaternion
    │   │   ├── position / orientation               # xform or rigid body (wxyz)
    │   ├── teleop/left/ ...
    │   ├── teleop/right/ ...
    │   └── teleop/head/ ...               # when head pose recording is enabled
    ├── episode_00001/ ...
    └── episode_00002/ ...
```

{meth}`EpisodeReplayer.list_episodes <isaacsim.replicator.episode_recorder.EpisodeReplayer.list_episodes>` iterates these groups for per-episode playback.

**Recorded data vs. replayed data.** The `state/*` groups hold per-prim world poses — articulations as `(L, 3)` / `(L, 4)` link pose batches, rigid bodies and plain Xforms as `(3,)` / `(4,)` single poses — and are the *only* data that {class}`EpisodeReplayer <isaacsim.replicator.episode_recorder.EpisodeReplayer>` applies on replay. Reads and writes go exclusively through {class}`XformPrim <isaacsim.core.experimental.prims.XformPrim>`, so no physics-tensor backend is involved. The teleop input channels (`teleop/<side>/trigger`, aim poses, head pose, etc.) are recorded as extra HDF5 datasets for offline analysis and policy learning, but they are **ignored by the replayer** — replay never re-dispatches trigger commands and never runs the teleop controllers (IK, Grasp, Floating, Locomotion). The gripper opening / closing that replays correctly is purely the link's recorded world pose being re-authored onto the anonymous sublayer. HDF5 pose replay is independent of MCAP input replay: an Episode Recorder HDF5 file cannot be opened by {class}`McapTeleopFrameProvider <isaacsim.replicator.teleop.McapTeleopFrameProvider>`.

**Visual replay.** {meth}`EpisodeReplayer.start_replay <isaacsim.replicator.episode_recorder.EpisodeReplayer.start_replay>` advances recorded frames from `omni.kit.app` updates, applying one recorded frame on every app update. The replayer prefetches episode data before playback and batches pose writes across compatible prims so replay stays fast while the UI remains responsive during loading and progress reporting. When timeline seeking is enabled, replay seeks — but never plays — the Kit timeline to the recorded `sim_time`, so stage-authored USD animations evaluate in lockstep without stepping physics or waking up the teleop controllers. All pose writes land in an anonymous USD sublayer so the root stage is never mutated. {meth}`stop_replay <isaacsim.replicator.episode_recorder.EpisodeReplayer.stop_replay>` pops that sublayer, visibly returning every prim to its pre-replay pose in one step, without closing the HDF5 session — the user can start another replay immediately.

### {class}`VRRecordingButton <isaacsim.replicator.teleop.VRRecordingButton>` and {class}`VRButton <isaacsim.replicator.teleop.VRButton>`

**Rising-edge VR controller binding for episode commands.** Subscribes to the teleop manager's controller-input stream and dispatches `EPISODE_CMD_EVENT` (`start`, `end`, or `toggle`) on the Kit event bus when the configured button transitions from released to pressed. The default mapping (`VRButton.LEFT_SECONDARY` → `toggle`) lets an operator drive recording with the Meta Quest left-**Y** button. The binding has no direct dependency on the episode-recorder extension, so attaching / detaching the button is safe even when no recorder is active.

`TeleopManager` **auto-attaches** a toggle-mapped instance on construction, so the left-Y button drives the standalone Episode Recorder window's active session out of the box; no UI or script wiring is required for the common live/debug case. The automatic binding is temporarily detached during MCAP replay so a recorded Y-button edge cannot toggle a new HDF5 recording, and is restored on disconnect.

### {func}`install_teleop_session_injector <isaacsim.replicator.teleop.install_teleop_session_injector>`

**Plugs teleop channels into sessions opened from any Episode Recorder UI.** Registers a {data}`SessionInjector <isaacsim.replicator.episode_recorder.SessionInjector>` with `isaacsim.replicator.episode_recorder` that appends `TeleopControllerRecordable` (left + right) and optionally `TeleopHeadRecordable` to every session opened through {func}`apply_session_injectors <isaacsim.replicator.episode_recorder.apply_session_injectors>` (called by the standalone Episode Recorder window). `TeleopManager.__init__` invokes this automatically; `TeleopManager.destroy` cleans up. Aim / head-pose capture is controlled by carb settings `/persistent/exts/isaacsim.replicator.teleop/record/{record_aim_pose,record_head_pose}` (both default `True`).

Live and MCAP sources populate the legacy raw aim/head pose channels. Debug mode supplies synthetic controller analog/button values, but its marker poses live only in the typed `TeleopFrame`; legacy HDF5 aim/head recordables therefore write their invalid zero defaults in debug mode.

### {class}`MarkersManager <isaacsim.replicator.teleop.MarkersManager>`

**Visual frame markers** created in an anonymous session sublayer under `/Teleop/Markers/`. Left, right, and head markers are children of the origin marker (`TrackingOrigin`), mirroring the VR play-space model. Live input writes finalized world poses so the displayed frames use the same resolved transform as Kit XR and robot targets. With a custom scene anchor, the origin is a live visualization proxy; without one, it is the writable built-in tracking space used by locomotion.

### {class}`VisualCuesManager <isaacsim.replicator.teleop.VisualCuesManager>`

**Drop-line visual cues** for 2D teleoperation depth perception. Each side renders one vertical cylinder from a tracked world position down to a configurable reference *Z*. The default 0.5-opacity preview material is emissive for stable visibility and the cylinder is marked not to cast shadows; it does not create a light in the scene. Cues live in an anonymous session sublayer under `/Teleop/VisualCues/`, auto-link to {meth}`TeleopManager.get_input_world_position <isaacsim.replicator.teleop.TeleopManager.get_input_world_position>` (VR controller or debug marker input) unless a manual prim override is set, and are not automatically included by the Episode Recorder's default `/World` discovery. Selecting `/Teleop` or the cue prims explicitly can include them in a recording.

### {class}`XrAnchorManager <isaacsim.replicator.teleop.XrAnchorManager>`

**Canonical XR/teleop anchor** generated at `/World/XRAnchor` in an anonymous session layer. Finalized teleop poses and frame markers always consume the resolved position and yaw; an active Kit XR profile consumes the same transform for stereo rendering. The Session panel's **Custom Anchor** is read-only and can therefore use arbitrary authored xform stacks without normalization. **Fixed** holds its initial absolute yaw; follow modes track absolute yaw, with optional smoothing. Offset uses the resolved anchor's local axes, and fixed height applies to the shared transform. Cleanup removes the generated layer and restores Kit's prior XR profile settings.

The XR dependency is runtime-optional. In the 2D application, Isaac Teleop owns a headless OpenXR session and Kit XR profile settings are not a Connect prerequisite. In the stereo VR experience, Kit owns the OpenXR session and `isaacsim.kit.xr.teleop.bridge` exposes its instance, session, stage-space, and procedure-address handles to `DeviceIOSession`. The bridge does not transfer a USD anchor. Teleop resolves the scene anchor itself and publishes `/World/XRAnchor` back to Kit XR. If stereo XR is active but the bridge handles or profile anchor cannot be verified, Connect rolls back instead of starting a mismatched session.

### CloudXR readiness helpers

Live VR teleop expects the Isaac Teleop CloudXR runtime to be started **externally** (for example ``python -m isaacteleop.cloudxr --accept-eula`` in a separate terminal). The external process cannot update the environment of an already-running Isaac Sim process, so :func:`prepare_live_cloudxr_env <isaacsim.replicator.teleop.prepare_live_cloudxr_env>` checks the ``runtime_started`` sentinel under ``~/.cloudxr/run/`` and loads the launcher's ``cloudxr.env`` into ``os.environ`` before a live :meth:`TeleopManager.connect <isaacsim.replicator.teleop.TeleopManager.connect>`. If the runtime is missing, connect fails with terminal hints pointing to ``isaacsim.replicator.teleop/Overview.md``.

### Coordinate Utilities

{func}`transform_pose <isaacsim.replicator.teleop.transform_pose>` and {func}`transform_pose_openxr_to_isaacsim <isaacsim.replicator.teleop.transform_pose_openxr_to_isaacsim>` convert OpenXR Y-up poses to Isaac Sim Z-up coordinates. The {class}`CoordinateSystem <isaacsim.replicator.teleop.CoordinateSystem>` enum selects the active conversion.

## Integration

Live input requires the Isaac Teleop CloudXR runtime to be installed from PyPI and running with the headset client connected before {meth}`TeleopManager.connect <isaacsim.replicator.teleop.TeleopManager.connect>` is called with ``input_mode="live"``. MCAP replay requires the Isaac Teleop package but is headless; debug input requires neither the package nor CloudXR. Install the live/replay runtime with:

```bash
python -m pip install "isaacteleop[cloudxr,retargeters]==1.4.145"
```

Start CloudXR in a **separate terminal** before connecting live teleop:

```bash
python -m isaacteleop.cloudxr --accept-eula
```

When launching Isaac Sim from a terminal, you can optionally source the env file first (`details <https://nvidia.github.io/IsaacTeleop/main/getting_started/quick_start.html#load-cloudxr-environment-variables>`_):

```bash
source ~/.cloudxr/run/cloudxr.env
```

Open the [Isaac Teleop Web Client](https://nvidia.github.io/IsaacTeleop/client/) in the headset browser, connect, then click **Connect** in the Isaac Sim Teleop window **Session** panel. If the latest client has connection issues, use the matching URL printed by the CloudXR launcher.

## Functionality

### Procedural Teleop Scenarios

Create generic scene content with `isaacsim.core.experimental.objects`,
`isaacsim.core.experimental.prims`, and `isaacsim.core.experimental.utils`.
Use `get_supported_floating_grippers()` to discover the current gripper definitions
and `load_floating_grippers_async()` to instantiate side assignments without
duplicating asset paths, controller gains, frame offsets, or grasp configuration.
The definitions live under `data/floating_grippers`; adding a gripper requires
only a YAML file when its asset follows the existing root-attachment contract.
Each definition's `tcp` mapping names a parent link and child Xform and stores
the TCP translation in that link's local frame. Loading a scripted gripper
creates that Xform and returns its path as `paths["tcp"]`. Attach TCP-relative
cameras beneath this Xform and aim them at their local origin.
Use `setup_floating_gripper_controllers_async()` to obtain an opaque controller
context, then pass it to `move_floating_grippers_to_targets_async()`,
`set_floating_gripper_grasp_async()`, and
`stop_floating_gripper_controllers()` to release the scripted teleop session.
These are stateless functions; the context only carries controller and marker
state. Pose values are plain
`(position, orientation_wxyz)` tuples created with `make_pose()`. Scripted motion
linearly interpolates positions, slerps orientations, and advances only after
every controlled pose reaches its configured tolerance.

Author world and gripper-relative cameras with {class}`Camera
<isaacsim.core.experimental.objects.Camera>` and compute their orientations with
{func}`look_at_quaternion
<isaacsim.core.experimental.utils.transform.look_at_quaternion>`. For a
gripper-relative camera, express the eye and target in the gripper-parent frame
and pass both to `Camera.set_local_poses()`.
Use `omni.replicator.core` render products and annotators directly to capture an
RGB PNG, raw floating-point depth NPY, and normalized depth-preview PNG for each
named camera and phase. Detach annotators and destroy render products during
cleanup.

### Portable gripper definition paths

`resolve_floating_gripper_asset_path_async(asset_path, *, assets_root=None,
base_dir=None, verify=True)` supports four forms, in this precedence order:

1. A URI such as `omniverse://server/gripper.usda`, `https://host/gripper.usda`,
   or `file:///data/gripper.usda`, used as authored.
2. An existing absolute local file, including native Windows drive and UNC paths.
3. An existing file relative to the definition's directory, such as
   `overlays/gripper.usda`. Scripted calls supply that directory as `base_dir`;
   the YAML loader and curated scene builder resolve siblings automatically.
4. An asset-root fragment such as `/Isaac/Robots/Ufactory/xarm_gripper/xarm_gripper.usd`,
   appended to the configured root using the public storage API.

Only the last form requires asset-root discovery. `verify=True` checks whether
the resulting location is accessible; it does not validate the USD's transitive
dependencies. `verify=False` skips that accessibility probe, but still checks
local file existence to distinguish local files from asset-root fragments.

Missing local files fall back to the assets root because `/Isaac/...` also looks
like an absolute POSIX path. Use a `file://` URI when you require an explicit local
location with no fallback. Without `base_dir`, relative values are asset-root
fragments, not paths relative to the process working directory.

A gripper definition's `grasp_controller.config_path` can likewise name a local
YAML map relative to its own directory. Existing `builtin://` and absolute map
paths retain their meaning. Keep the definition, overlay, and map together when
moving the directory, and keep the overlay's own USD references portable.
This support does not add a custom-directory registry, hardware wizard, or
automatic profile-stage loading. It also does not change controller prim paths
in existing teleop profiles.

### Teleop Profiles

{class}`TeleopProfile <isaacsim.replicator.teleop.TeleopProfile>` captures the complete state of a teleop session — session settings, floating/IK/grasp/locomotion controller configurations, visual cue settings, and desired enabled states — in a single YAML file. The on-disk format mirrors the dataclass hierarchy, so adding or removing fields automatically updates the file format with no manual parsers or version checks. Profiles store the user's **intent** (desired enabled state) rather than runtime state, so a profile loaded before the stage is ready preserves its enable flags for later activation.

The profile hierarchy consists of:

- {class}`TeleopSettingsProfile <isaacsim.replicator.teleop.TeleopSettingsProfile>` — coordinate system, tracking space, marker scale, XR anchor offset/rotation/smoothing
- {class}`BimanualControllerProfile <isaacsim.replicator.teleop.BimanualControllerProfile>` — left/right {class}`ControllerSideProfile <isaacsim.replicator.teleop.ControllerSideProfile>` (enabled flag + settings dict), used by both Floating and IK controllers
- {class}`GraspControllerProfile <isaacsim.replicator.teleop.GraspControllerProfile>` — left/right {class}`GraspSideProfile <isaacsim.replicator.teleop.GraspSideProfile>` (enabled flag, prim path, config path, drive mode, retargeter kind, and semantic joint aliases)
- {class}`LocomotionProfile <isaacsim.replicator.teleop.LocomotionProfile>` — enabled flag + settings dict (prim path, drive mode, speed multipliers)
- {class}`VisualCuesProfile <isaacsim.replicator.teleop.VisualCuesProfile>` — reference *Z*, opacity, size, and left/right {class}`VisualCueSideProfile <isaacsim.replicator.teleop.VisualCueSideProfile>` (enabled flag, optional prim override)

```python
from isaacsim.replicator.teleop import (
    TeleopProfile,
    load_teleop_profile,
    save_teleop_profile,
    scan_teleop_profiles,
    get_builtin_teleop_profiles_dir,
)

# List built-in profiles
profiles = scan_teleop_profiles(get_builtin_teleop_profiles_dir())

# Load a profile
profile, errors = load_teleop_profile(profiles[0][1])

# Save a profile
save_teleop_profile("/path/to/my_profile.yaml", profile)
```

### Profile Validation

{func}`resolve_teleop_profile <isaacsim.replicator.teleop.resolve_teleop_profile>` validates a {class}`TeleopProfile` against the current USD stage, checking that referenced prims exist, carry the expected APIs (RigidBodyAPI, ArticulationRootAPI, Xformable), and that enum values and grasp configs are well-formed. Retargeted grasp validation also checks the retargeter kind and resolves every semantic alias through the grasp config to a controllable USD joint. It returns a {class}`TeleopResolutionReport <isaacsim.replicator.teleop.TeleopResolutionReport>` with structured {class}`TeleopResolverIssue <isaacsim.replicator.teleop.TeleopResolverIssue>` entries at error or warning severity. Validation is always explicit and user-triggered — opening the window or loading persistent settings does not produce terminal output.

### Programmatic Session Control

The teleop session can be controlled from scripts without the UI:

```python
from isaacsim.replicator.teleop import TeleopCommand, TeleopManager, dispatch_command

# Via command bus (fire-and-forget)
dispatch_command("connect")

# Via manager instance (complete live-session lifecycle)
manager = TeleopManager()
manager.execute_command(TeleopCommand.CONNECT)
```

`manager.connect()` is the lower-level input-transport API: it intentionally does not create markers, resolve tracking space, or acquire Kit's XR profile anchor. Use it for headless MCAP re-simulation or custom providers. For a headless re-simulation from raw operator input, call `manager.connect(input_mode="mcap_replay", mcap_path="/path/to/input.mcap")`, attach and configure controllers on that manager, and play the Kit timeline. Loading a profile YAML only returns a dataclass; it does not apply the profile to a standalone manager. To capture raw controller/head input from a live connection, pass `mcap_recording_path`; recording spans the whole transport connection and closes on `manager.disconnect()`. Existing files are rejected unless `mcap_recording_overwrite=True` is explicit. MCAP uses the default Isaac Teleop channel bases `controllers` and `head`. It complements rather than replaces episode-scoped HDF5 simulation recording.

### Episode Recording

Two workflows produce the same HDF5 format, driven by the same
{data}`EPISODE_CMD_EVENT
<isaacsim.replicator.episode_recorder.EPISODE_CMD_EVENT>` carb bus:

1. **Standalone Episode Recorder window**
   (`isaacsim.replicator.episode_recorder.ui`, *Tools > Replicator > Episode
   Recorder*). The window discovers recordable targets and opens / closes
   sessions. While a {class}`TeleopManager` is alive, its
   {func}`install_teleop_session_injector
   <isaacsim.replicator.teleop.install_teleop_session_injector>` hook
   automatically appends teleop controller / aim-pose / head-pose channels
   to each session the window opens. The auto-attached
   {class}`VRRecordingButton <isaacsim.replicator.teleop.VRRecordingButton>`
   toggles those sessions from the Meta Quest left-**Y** button.
2. **Scripted recording via
   {func}`build_teleop_recorder <isaacsim.replicator.teleop.build_teleop_recorder>`**,
   which returns an {class}`EpisodeRecorder
   <isaacsim.replicator.episode_recorder.EpisodeRecorder>` preconfigured
   with scene + teleop recordables. Episodes auto-start on timeline PLAY
   and auto-end on timeline STOP; the same lifecycle can be driven
   manually through
   {func}`dispatch_episode_command
   <isaacsim.replicator.episode_recorder.dispatch_episode_command>`
   (`"start"`, `"end"`, `"toggle"`) or the VR button.

```python
from isaacsim.replicator.teleop import TeleopManager, build_teleop_recorder

manager = TeleopManager()      # auto-attaches the VR recording button and
                               # installs the teleop session injector
manager.connect()  # Transport-only is intentional for this headless recorder.

recorder = build_teleop_recorder(
    "/tmp/teleop_demos",
    teleop_manager=manager,
    articulations={"robot": "/World/Robot"},
    xforms={"cube": "/World/Cube"},
)
recorder.open_session()

# ... drive teleop; each timeline Play / Stop, or each Meta Quest Y press,
# opens / closes an episode.

recorder.close_session()
```

Recorded files are consumed by {class}`EpisodeReplayer
<isaacsim.replicator.episode_recorder.EpisodeReplayer>` either for live
timeline-synced playback (the Episode Recorder window's Replay section) or
for offline synthetic data generation via an `apply_frame` /
`rep.orchestrator.step_async` loop.
