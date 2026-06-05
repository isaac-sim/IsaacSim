# Overview

The `isaacsim.replicator.episode_recorder` extension provides a manifest-first
HDF5 recorder / replayer for capturing simulation state per-episode and
replaying it back onto a live USD stage.

The recorder is **plugin-based** via the {class}`Recordable
<isaacsim.replicator.episode_recorder.Recordable>` protocol: every recorded
channel (articulation DOFs, rigid-body velocities, xform world poses, camera
intrinsics, arbitrary USD attributes, simulation time, teleop inputs, ...)
comes from a `Recordable` that declares its schema, samples data each tick,
applies it to the stage on replay, and round-trips through a JSON manifest
stored inside the HDF5 file.

## Key Components

- {class}`EpisodeRecorder
  <isaacsim.replicator.episode_recorder.EpisodeRecorder>` — orchestrator that
  owns the HDF5 session, the session / episode lifecycle, and the sampling
  tick subscription. World-pose I/O routes through a selectable
  ``pose_backend`` (``"usd"`` / ``"usdrt"`` / ``"fabric"``); the non-USD
  backends require Fabric Scene Delegate.
- {class}`EpisodeReplayer
  <isaacsim.replicator.episode_recorder.EpisodeReplayer>` — reads an HDF5
  session, rehydrates each `Recordable` from the manifest, and applies
  per-frame state to the live stage. Pose batches write in parents-first
  ancestry tiers so nested xforms / articulations don't lag a frame behind
  a moving parent.
- {class}`SessionStorage
  <isaacsim.replicator.episode_recorder.SessionStorage>` /
  {class}`SessionReader
  <isaacsim.replicator.episode_recorder.SessionReader>` — write and read
  halves of the HDF5 V2 layout.
- {class}`TimelineDrivenEpisodeController
  <isaacsim.replicator.episode_recorder.TimelineDrivenEpisodeController>` —
  drives recorder episodes from Kit timeline PLAY / STOP events. The
  PLAY → start coupling is gated by a runtime-togglable
  ``auto_start_on_play`` flag so UIs can offer an explicit manual-start
  workflow; STOP always drains an active episode regardless of the flag.
- {data}`EPISODE_CMD_EVENT
  <isaacsim.replicator.episode_recorder.EPISODE_CMD_EVENT>` /
  {func}`dispatch_episode_command
  <isaacsim.replicator.episode_recorder.dispatch_episode_command>` — carb
  event-bus commands for `start` / `end` / `toggle` / `pause` / `resume` /
  `step` / `open_session` / `close_session`.

## Built-in recordables

- {class}`SimTimeRecordable
  <isaacsim.replicator.episode_recorder.SimTimeRecordable>` — auto-attached
  `sim_time`, `physics_step`, `wall_time` channels.
- {class}`ArticulationRecordable
  <isaacsim.replicator.episode_recorder.ArticulationRecordable>` — world pose of
  every `UsdGeom.Xformable` link under an articulation root, stored as a single
  `(L, 3)` / `(L, 4)` tensor; pure-USD, no physics-tensor reads.
- {class}`RigidBodyRecordable
  <isaacsim.replicator.episode_recorder.RigidBodyRecordable>` — world
  `position` + `wxyz orientation` of a single rigid body (pure-USD).
- {class}`XformRecordable
  <isaacsim.replicator.episode_recorder.XformRecordable>` — world or local
  `position` + `wxyz orientation` of a plain USD Xform prim.
- {class}`CameraRecordable
  <isaacsim.replicator.episode_recorder.CameraRecordable>` — camera world
  pose and intrinsics.
- {class}`AttributeRecordable
  <isaacsim.replicator.episode_recorder.AttributeRecordable>` — generic USD
  attribute capture.

Custom recordables are registered with the
{func}`register_recordable
<isaacsim.replicator.episode_recorder.register_recordable>` decorator so the
replayer can rehydrate them directly from a manifest entry.

## Target discovery

The {mod}`isaacsim.replicator.episode_recorder.target_discovery` submodule
walks a USD stage to auto-discover recordable targets (articulations, rigid
bodies, loose xforms) under a given root prim path — ideal for UIs and
standalone scripts that want to "point at `/World` and record everything
below".

## Stage snapshots

{func}`export_stage_snapshot
<isaacsim.replicator.episode_recorder.export_stage_snapshot>` writes a
flattened USD of the current stage (`<output_dir>/stage_snapshot.usd`) plus a
sidecar JSON describing the export. When a session is opened with
`link_stage_snapshot=True` (the default) and the file exists in the output
directory, its basename is stamped into the HDF5 `stage_snapshot` root
attribute so the replayer can resolve it on any machine without a separate
stage author step.

## Session injectors

{func}`register_session_injector
<isaacsim.replicator.episode_recorder.register_session_injector>` installs a
process-wide callback that fires when
{func}`apply_session_injectors
<isaacsim.replicator.episode_recorder.apply_session_injectors>` is invoked
with an {class}`EpisodeRecorder
<isaacsim.replicator.episode_recorder.EpisodeRecorder>`. UIs (for example the
standalone Episode Recorder window) call `apply_session_injectors` just
before opening a session so that other extensions can attach their own
{class}`Recordable <isaacsim.replicator.episode_recorder.Recordable>`
instances without any direct dependency on the UI.

`isaacsim.replicator.teleop` uses this hook
({func}`install_teleop_session_injector
<isaacsim.replicator.teleop.install_teleop_session_injector>`) to
automatically append `teleop/left`, `teleop/right`, and optional `teleop/head`
channels to any session opened while a live `TeleopManager` is alive.

```python
from isaacsim.replicator.episode_recorder import (
    register_session_injector,
)

def add_my_channels(recorder) -> None:
    recorder.add(MyRecordable(group="my/channel"))

handle = register_session_injector(add_my_channels)
# ... later ...
handle()  # unregister
```

## UI

Recording and replay from the desktop is provided by the dedicated
`isaacsim.replicator.episode_recorder.ui` extension
(*Tools > Replicator > Episode Recorder*). This library is UI-agnostic and
can be driven entirely from scripts or from the
{data}`EPISODE_CMD_EVENT
<isaacsim.replicator.episode_recorder.EPISODE_CMD_EVENT>` carb bus.
