# Overview

The `isaacsim.replicator.episode_recorder.ui` extension adds a dockable
**Episode Recorder** window under the *Tools > Replicator* menu. It drives
the manifest-first HDF5 recorder / replayer provided by
`isaacsim.replicator.episode_recorder` with no teleop coupling, so it works
for any stage whose prims expose an articulation, rigid body, or xform.

## Functionality

- **USD Root + Discover** — walks the stage under a root prim path and lists
  discovered articulations, rigid bodies, and loose xforms with individual
  opt-in checkboxes.
- **Session options** — output directory, file prefix, and a one-shot
  *Export Scene* button that writes `stage_snapshot.usd` (+ sidecar JSON)
  beside future recordings so sessions auto-link a flattened reference stage.
- **Session control** — *Open Session* / *Close Session* toggle, plus a
  *Start* / *End* button that wires into
  {class}`TimelineDrivenEpisodeController
  <isaacsim.replicator.episode_recorder.TimelineDrivenEpisodeController>` so
  episodes follow Kit's timeline play / stop state. An *Auto-start recording
  on timeline Play* checkbox toggles whether timeline PLAY automatically
  starts a new episode; when unchecked, recording must be started explicitly
  via the *Start* button (which also works at any point while the timeline
  is already playing). Timeline STOP always drains an active episode so the
  HDF5 file stays consistent regardless of the checkbox state.
- **Replay** — file picker + episode combo, *Latest* shortcut, and playback
  controls (*Start Replay*, *Pause Replay*, *Step Forward*, *Step Backward*,
  and *Stop Replay*). The replay drives {meth}`EpisodeReplayer.start_replay
  <isaacsim.replicator.episode_recorder.EpisodeReplayer.start_replay>`, which
  applies one recorded frame on every app update. If *Seek timeline* is checked,
  replay also seeks (never plays) the Kit timeline to the recorded ``sim_time``
  so stage-authored USD animations play back in sync without stepping physics.
  The progress label and terminal log are throttled so long episodes do not
  flood the UI thread.
  Pressing *Stop Replay* (or hitting the last frame in non-loop mode) pops the
  anonymous USD sublayer the replayer wrote into, visibly reverting the stage
  to its pre-replay state.

## Extending the UI from another extension

The panel uses the built-in recordables
({class}`ArticulationRecordable
<isaacsim.replicator.episode_recorder.ArticulationRecordable>`,
{class}`RigidBodyRecordable
<isaacsim.replicator.episode_recorder.RigidBodyRecordable>`,
{class}`XformRecordable
<isaacsim.replicator.episode_recorder.XformRecordable>`) and then invokes
{func}`apply_session_injectors
<isaacsim.replicator.episode_recorder.apply_session_injectors>` to let other
extensions contribute channels. Register a callback with
{func}`register_session_injector
<isaacsim.replicator.episode_recorder.register_session_injector>` and it
will fire for every session opened from the window:

```python
from isaacsim.replicator.episode_recorder import (
    register_session_injector,
    SessionInjector,
)

def add_my_channels(recorder) -> None:
    recorder.add(MyRecordable(group="my/channel"))

handle = register_session_injector(add_my_channels)
# ... later ...
handle()  # unregister
```

`isaacsim.replicator.teleop` uses this mechanism to attach its controller /
head-pose channels to sessions opened from the Episode Recorder window
whenever a live `TeleopManager` is active.

## Command-bus integration

Start / end / toggle events are dispatched through the shared
{data}`EPISODE_CMD_EVENT
<isaacsim.replicator.episode_recorder.EPISODE_CMD_EVENT>` carb event, so
external callers (e.g. a VR button, a remote script, a keyboard shortcut)
can drive the same session without the UI in focus.
