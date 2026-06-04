# Overview

The isaacsim.replicator.teleop extension provides the runtime for VR-driven teleoperation of robots in Isaac Sim. It manages OpenXR session connectivity, coordinate-frame conversion, visual frame markers, unified teleop profiles, a set of controllers that translate VR headset and controller poses into robot joint targets, rigid-body velocities, or base movements, and {func}`build_teleop_recorder <isaacsim.replicator.teleop.build_teleop_recorder>` for assembling an {class}`EpisodeRecorder <isaacsim.replicator.episode_recorder.EpisodeRecorder>` that captures simulation state plus VR controller inputs to HDF5 for offline replay and synthetic data generation.

## Key Components

### {class}`TeleopManager <isaacsim.replicator.teleop.TeleopManager>`

**Central orchestrator for the teleop session.** Manages the OpenXR connection lifecycle, distributes VR controller poses and input signals to all downstream controllers each simulation step, and exposes a command bus (`CONNECT`, `START`, `STOP`, `RESET`, `DISCONNECT`) that can be driven from the UI or programmatically via {func}`dispatch_command <isaacsim.replicator.teleop.dispatch_command>`. The manager also exposes `add_controller_inputs_observer` / `add_head_observer` hooks that feed per-frame controller and headset snapshots to downstream consumers such as teleop {class}`Recordable <isaacsim.replicator.episode_recorder.Recordable>` plugins (wired by {func}`build_teleop_recorder <isaacsim.replicator.teleop.build_teleop_recorder>`) and {class}`VRRecordingButton <isaacsim.replicator.teleop.VRRecordingButton>`.

### {class}`RobotIKController <isaacsim.replicator.teleop.RobotIKController>`

**Inverse-kinematics controller for articulated robot arms.** Takes a 6-DOF VR target pose and computes joint position targets. Supports per-side (left/right) configuration and four solver back-ends selectable via {class}`IKSolverType <isaacsim.replicator.teleop.IKSolverType>`:

- **Position-based** — single-iteration Jacobian differential IK
- **Velocity-based** — velocity-space IK with position integration
- **Levenberg–Marquardt** — multi-iteration LM solver per frame
- **PINK** — Pinocchio-based QP IK with a teleop-specific minimal USD-to-URDF export and selectable `daqp` / `osqp` QP back-ends

### {class}`FloatingRigidBodyController <isaacsim.replicator.teleop.FloatingRigidBodyController>`

**PD velocity controller for free rigid bodies** not attached to any articulation. Per-side configuration with tunable position/orientation gains and rotation offsets.

### {class}`GraspController <isaacsim.replicator.teleop.GraspController>`

**Maps VR trigger analog values to gripper joint targets** (0 = open, 1 = fully closed). Uses {class}`GraspConfig <isaacsim.replicator.teleop.GraspConfig>` and {class}`JointMapping <isaacsim.replicator.teleop.JointMapping>` loaded from YAML presets.

### {class}`LocomotionController <isaacsim.replicator.teleop.controllers.LocomotionController>`

**Kinematic movement driven by VR thumbstick input.** Left thumbstick translates in the world ground plane (using the prim's heading projected onto XY), right thumbstick rotates (yaw), and the right primary/secondary buttons move vertically along world Z. Movement axes are always orthogonal regardless of the target prim's local frame orientation. Configurable speed multipliers. Two workflows are supported depending on the target prim:

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

**Recorded data vs. replayed data.** The `state/*` groups hold per-prim world poses — articulations as `(L, 3)` / `(L, 4)` link pose batches, rigid bodies and plain Xforms as `(3,)` / `(4,)` single poses — and are the *only* data that {class}`EpisodeReplayer <isaacsim.replicator.episode_recorder.EpisodeReplayer>` applies on replay. Reads and writes go exclusively through {class}`XformPrim <isaacsim.core.experimental.prims.XformPrim>`, so no physics-tensor backend is involved. The teleop input channels (`teleop/<side>/trigger`, aim poses, head pose, etc.) are recorded as extra HDF5 datasets for offline analysis and policy learning, but they are **ignored by the replayer** — replay never re-dispatches trigger commands and never runs the teleop controllers (IK, Grasp, Floating, Locomotion). The gripper opening / closing that replays correctly is purely the link's recorded world pose being re-authored onto the anonymous sublayer.

**Visual replay.** {meth}`EpisodeReplayer.start_replay <isaacsim.replicator.episode_recorder.EpisodeReplayer.start_replay>` advances recorded frames from `omni.kit.app` updates, applying one recorded frame on every app update. The replayer prefetches episode data before playback and batches pose writes across compatible prims so replay stays fast while the UI remains responsive during loading and progress reporting. When timeline seeking is enabled, replay seeks — but never plays — the Kit timeline to the recorded `sim_time`, so stage-authored USD animations evaluate in lockstep without stepping physics or waking up the teleop controllers. All pose writes land in an anonymous USD sublayer so the root stage is never mutated. {meth}`stop_replay <isaacsim.replicator.episode_recorder.EpisodeReplayer.stop_replay>` pops that sublayer, visibly returning every prim to its pre-replay pose in one step, without closing the HDF5 session — the user can start another replay immediately.

### {class}`VRRecordingButton <isaacsim.replicator.teleop.VRRecordingButton>` and {class}`VRButton <isaacsim.replicator.teleop.VRButton>`

**Rising-edge VR controller binding for episode commands.** Subscribes to the teleop manager's controller-input stream and dispatches `EPISODE_CMD_EVENT` (`start`, `end`, or `toggle`) on the Kit event bus when the configured button transitions from released to pressed. The default mapping (`VRButton.LEFT_SECONDARY` → `toggle`) lets an operator drive recording with the Meta Quest left-**Y** button. The binding has no direct dependency on the episode-recorder extension, so attaching / detaching the button is safe even when no recorder is active.

`TeleopManager` **auto-attaches** a toggle-mapped instance on construction, so the left-Y button drives the standalone Episode Recorder window's active session out of the box; no UI or script wiring is required for the common case.

### {func}`install_teleop_session_injector <isaacsim.replicator.teleop.install_teleop_session_injector>`

**Plugs teleop channels into sessions opened from any Episode Recorder UI.** Registers a {data}`SessionInjector <isaacsim.replicator.episode_recorder.SessionInjector>` with `isaacsim.replicator.episode_recorder` that appends `TeleopControllerRecordable` (left + right) and optionally `TeleopHeadRecordable` to every session opened through {func}`apply_session_injectors <isaacsim.replicator.episode_recorder.apply_session_injectors>` (called by the standalone Episode Recorder window). `TeleopManager.__init__` invokes this automatically; `TeleopManager.destroy` cleans up. Aim / head-pose capture is controlled by carb settings `/persistent/exts/isaacsim.replicator.teleop/record/{record_aim_pose,record_head_pose}` (both default `True`).

### {class}`MarkersManager <isaacsim.replicator.teleop.MarkersManager>`

**Visual frame markers** created in an anonymous session sublayer under `/Teleop/Markers/`. Left, right, and head markers are children of the origin marker (`TrackingOrigin`), mirroring the VR play-space model. Moving the origin — via locomotion or {func}`move_tracking_space_to <isaacsim.replicator.teleop.MarkersManager.move_tracking_space_to>` — automatically repositions all child markers through USD transform inheritance. The origin marker also serves as the default tracking space for VR pose offsetting.

### {class}`XrAnchorManager <isaacsim.replicator.teleop.XrAnchorManager>`

**Headset camera anchor** at `/World/XRAnchor`, bound to the active VR profile so the headset and controllers follow it live. Supports a position offset, three rotation-tracking modes ({class}`AnchorRotationMode <isaacsim.replicator.teleop.AnchorRotationMode>`), and fixed-height lock. The Session panel's **Custom Anchor** field re-targets the anchor to any scene Xform; **Clear** reverts to the built-in origin marker.

### Coordinate Utilities

{func}`transform_pose <isaacsim.replicator.teleop.transform_pose>` and {func}`transform_pose_openxr_to_isaacsim <isaacsim.replicator.teleop.transform_pose_openxr_to_isaacsim>` convert OpenXR Y-up poses to Isaac Sim Z-up coordinates. The {class}`CoordinateSystem <isaacsim.replicator.teleop.CoordinateSystem>` enum selects the active conversion.

## Integration

The teleop session requires the Isaac Teleop CloudXR runtime to be installed from PyPI and running
with the headset client connected before {func}`TeleopManager.connect <isaacsim.replicator.teleop.TeleopManager.connect>`
is called. Install the runtime with:

```bash
python -m pip install "isaacteleop[cloudxr,retargeters]~=1.0.0"
```

Start the runtime in another terminal with:

```bash
python -m isaacteleop.cloudxr
```

Keep that process running while using teleop. Then open the hosted
[Isaac Teleop Web Client](https://nvidia.github.io/IsaacTeleop/client/) from the headset browser and
follow the displayed connection steps. Connect from Isaac Sim after the CloudXR runtime is running
and the headset client is connected.

## Functionality

### Teleop Profiles

{class}`TeleopProfile <isaacsim.replicator.teleop.TeleopProfile>` captures the complete state of a teleop session — session settings, floating/IK/grasp/locomotion controller configurations, and desired enabled states — in a single YAML file. The on-disk format mirrors the dataclass hierarchy, so adding or removing fields automatically updates the file format with no manual parsers or version checks. Profiles store the user's **intent** (desired enabled state) rather than runtime state, so a profile loaded before the stage is ready preserves its enable flags for later activation.

The profile hierarchy consists of:

- {class}`TeleopSettingsProfile <isaacsim.replicator.teleop.TeleopSettingsProfile>` — coordinate system, tracking space, marker scale, XR anchor offset/rotation/smoothing
- {class}`BimanualControllerProfile <isaacsim.replicator.teleop.BimanualControllerProfile>` — left/right {class}`ControllerSideProfile <isaacsim.replicator.teleop.ControllerSideProfile>` (enabled flag + settings dict), used by both Floating and IK controllers
- {class}`GraspControllerProfile <isaacsim.replicator.teleop.GraspControllerProfile>` — left/right {class}`GraspSideProfile <isaacsim.replicator.teleop.GraspSideProfile>` (enabled flag, prim path, config path)
- {class}`LocomotionProfile <isaacsim.replicator.teleop.LocomotionProfile>` — enabled flag + settings dict (prim path, speed multipliers)

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

{func}`resolve_teleop_profile <isaacsim.replicator.teleop.resolve_teleop_profile>` validates a {class}`TeleopProfile` against the current USD stage, checking that referenced prims exist, carry the expected APIs (RigidBodyAPI, ArticulationRootAPI, Xformable), and that enum values and grasp configs are well-formed. It returns a {class}`TeleopResolutionReport <isaacsim.replicator.teleop.TeleopResolutionReport>` with structured {class}`TeleopResolverIssue <isaacsim.replicator.teleop.TeleopResolverIssue>` entries at error or warning severity. Validation is always explicit and user-triggered — opening the window or loading persistent settings does not produce terminal output.

### Programmatic Session Control

The teleop session can be controlled from scripts without the UI:

```python
from isaacsim.replicator.teleop import TeleopManager, dispatch_command

# Via command bus (fire-and-forget)
dispatch_command("connect")

# Via manager instance (direct control)
manager = TeleopManager()
manager.connect()
```

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
manager.connect()

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
