# Public API for module isaacsim.replicator.episode_recorder:

## Classes

- class ArticulationRecordable(Recordable)
  - TYPE_ID: str
  - def __init__(self)
  - [property] def num_links(self) -> int
  - [property] def link_paths(self) -> list[str]
  - def describe_channels(self) -> dict[str, ChannelDescriptor]
  - def on_session_open(self, stage: Any)
  - def on_session_close(self)
  - def pose_paths(self) -> list[str] | None
  - def consume_pose_batch(self, positions: np.ndarray, orientations: np.ndarray) -> dict[str, np.ndarray]
  - def sample(self) -> dict[str, np.ndarray]
  - def apply(self, frame: Mapping[str, np.ndarray])
  - def to_manifest(self) -> dict[str, Any]
  - class def from_manifest(cls, entry: Mapping[str, Any]) -> ArticulationRecordable

- class AttributeRecordable(Recordable)
  - TYPE_ID: str
  - def __init__(self)
  - def describe_channels(self) -> dict[str, ChannelDescriptor]
  - def on_session_open(self, stage: Any)
  - def on_session_close(self)
  - def sample(self) -> dict[str, np.ndarray]
  - def apply(self, frame: Mapping[str, np.ndarray])
  - def to_manifest(self) -> dict[str, Any]
  - class def from_manifest(cls, entry: Mapping[str, Any]) -> AttributeRecordable

- class CameraRecordable(_PoseBase)
  - TYPE_ID: str
  - def __init__(self)
  - def describe_channels(self) -> dict[str, ChannelDescriptor]
  - def on_session_open(self, stage: Any)
  - def on_session_close(self)
  - def pose_paths(self) -> list[str] | None
  - def consume_pose_batch(self, positions: np.ndarray, orientations: np.ndarray) -> dict[str, np.ndarray]
  - def sample(self) -> dict[str, np.ndarray]
  - def apply(self, frame: Mapping[str, np.ndarray])
  - def to_manifest(self) -> dict[str, Any]
  - class def from_manifest(cls, entry: Mapping[str, Any]) -> CameraRecordable

- class ChannelDescriptor
  - shape: tuple[int, Ellipsis]
  - dtype: str
  - units: str | None
  - space: str | None
  - quaternion_order: str | None
  - attrs: Mapping[str, Any]

- class EpisodeRecorder
  - def __init__(self, output_dir: str)
  - [property] def session_id(self) -> str
  - [property] def output_dir(self) -> str
  - [property] def hdf5_path(self) -> str | None
  - [property] def is_session_open(self) -> bool
  - [property] def is_recording(self) -> bool
  - [property] def is_paused(self) -> bool
  - [property] def current_episode_frames(self) -> int
  - [property] def events(self) -> SessionEvents
  - [property] def state(self) -> str
  - [property] def pose_backend(self) -> PoseBackend
  - def add(self, recordable: Recordable)
  - def recordables(self) -> list[Recordable]
  - def open_session(self, output_path: str | None = None) -> str
  - def close_session(self)
  - def destroy(self)
  - def start_episode(self, metadata: dict[str, Any] | None = None) -> int
  - def end_episode(self)
  - def pause(self)
  - def resume(self)
  - def export_stage_snapshot(self, output_dir: str | None = None) -> str

- class EpisodeReplayer
  - def __init__(self, hdf5_path: str)
  - [property] def hdf5_path(self) -> str
  - [property] def policy(self) -> ReplayPolicy
  - [property] def is_replaying(self) -> bool
  - [property] def is_paused(self) -> bool
  - [property] def current_frame(self) -> int
  - [property] def total_frames(self) -> int
  - [property] def prepared_recordables(self) -> list[Recordable]
  - [property] def pose_batch_size(self) -> int
  - [property] def pose_batch_tier_count(self) -> int
  - [property] def pose_backend(self) -> PoseBackend
  - def list_episodes(self) -> list[str]
  - def num_frames(self, episode: int | str) -> int
  - def episode_attrs(self, episode: int | str) -> dict[str, Any]
  - def manifest(self) -> SessionManifest
  - def session_metadata(self) -> dict[str, Any]
  - def prepare_episode(self, episode: int | str)
  - async def prepare_episode_async(self, episode: int | str)
  - def apply_frame(self, frame_index: int)
  - def replay_episode(self, episode: int | str = 0)
  - def start_replay(self)
  - async def start_replay_async(self)
  - def stop_replay(self)
  - def pause_replay(self)
  - def resume_replay(self)
  - def step_frame(self, delta: int = 1) -> int | None
  - def close(self)

- class Recordable(ABC)
  - TYPE_ID: str
  - def __init__(self)
  - def describe_channels(self) -> dict[str, ChannelDescriptor]
  - def on_session_open(self, stage: Any)
  - def on_session_close(self)
  - def on_episode_start(self)
  - def on_episode_end(self)
  - def sample(self) -> dict[str, np.ndarray | float | int]
  - def pose_paths(self) -> list[str] | None
  - def consume_pose_batch(self, positions: np.ndarray, orientations: np.ndarray) -> dict[str, np.ndarray | float | int]
  - def apply(self, frame: Mapping[str, np.ndarray])
  - def to_manifest(self) -> dict[str, Any]
  - class def from_manifest(cls, entry: Mapping[str, Any]) -> Recordable

- class ReplayPolicy
  - strictness: str

- class RigidBodyRecordable(_PoseBase)
  - TYPE_ID: str
  - def __init__(self)
  - def to_manifest(self) -> dict[str, Any]
  - class def from_manifest(cls, entry: Mapping[str, Any]) -> RigidBodyRecordable

- class SamplingConfig
  - mode: str
  - decimation: int

- class SessionEvents
  - def __init__(self)
  - def add_session_opened(self, cb: Callable[[], None]) -> Callable[[], None]
  - def add_session_closed(self, cb: Callable[[], None]) -> Callable[[], None]
  - def add_episode_started(self, cb: Callable[[int], None]) -> Callable[[], None]
  - def add_episode_ended(self, cb: Callable[[int, bool | None, int], None]) -> Callable[[], None]
  - def add_paused(self, cb: Callable[[], None]) -> Callable[[], None]
  - def add_resumed(self, cb: Callable[[], None]) -> Callable[[], None]

- class SessionManifest
  - tracks: list[dict[str, Any]]
  - session: dict[str, Any]
  - sampling: dict[str, Any]
  - coord_conventions: dict[str, Any]
  - def track_groups(self) -> list[str]

- class SessionReader
  - def __init__(self, h5_path: str)
  - [property] def path(self) -> str
  - def manifest(self) -> SessionManifest
  - def list_episodes(self) -> list[str]
  - def normalize_episode(self, episode: int | str) -> str
  - def num_frames(self, episode: int | str) -> int
  - def episode_attrs(self, episode: int | str) -> dict[str, Any]
  - def read_frame(self, episode: int | str, recordable_group: str, frame_index: int) -> dict[str, np.ndarray]
  - def read_channel(self, episode: int | str, recordable_group: str, channel: str) -> np.ndarray
  - def read_group_all_frames(self, episode: int | str, recordable_group: str) -> dict[str, np.ndarray]
  - def close(self)

- class SessionStorage
  - def __init__(self, h5_path: str)
  - [property] def path(self) -> str
  - [property] def is_open(self) -> bool
  - [property] def current_episode_frames(self) -> int
  - [property] def num_episodes_finalized(self) -> int
  - def open(self)
  - def write_manifest(self, manifest: SessionManifest)
  - def set_root_attr(self, key: str, value: Any)
  - def close(self)
  - def begin_episode(self, channel_schemas: Mapping[str, Mapping[str, ChannelDescriptor]]) -> int
  - def append_frame(self, recordable_group: str, frame: Mapping[str, np.ndarray | float | int])
  - def advance_episode_frame(self)
  - def flush(self)
  - def end_episode(self)

- class SimTimeRecordable(Recordable)
  - TYPE_ID: str
  - def __init__(self)
  - def describe_channels(self) -> dict[str, ChannelDescriptor]
  - def sample(self) -> dict[str, Any]
  - def apply(self, frame: Mapping[str, Any])
  - def to_manifest(self) -> dict[str, Any]
  - class def from_manifest(cls, entry: Mapping[str, Any]) -> SimTimeRecordable

- class TimelineDrivenEpisodeController
  - def __init__(self, recorder: EpisodeRecorder)
  - [property] def is_enabled(self) -> bool
  - [property] def auto_start_on_play(self) -> bool
  - def set_auto_start_on_play(self, value: bool)
  - def enable(self)
  - def disable(self)

- class XformRecordable(_PoseBase)
  - TYPE_ID: str
  - def __init__(self)
  - def to_manifest(self) -> dict[str, Any]
  - class def from_manifest(cls, entry: Mapping[str, Any]) -> XformRecordable

## Functions

- def apply_session_injectors(recorder: EpisodeRecorder)
- def build_manifest(recordable_entries: Iterable[dict[str, Any]]) -> SessionManifest
- def clear_session_injectors()
- def dispatch_episode_binding(action: str)
- def dispatch_episode_command(command: str, **payload: Any)
- def export_stage_snapshot(output_dir: str) -> str
- def get_registered(type_id: str) -> type[Recordable] | None
- def read_manifest(h5_file: Any) -> SessionManifest
- def register_recordable(cls: type[Recordable]) -> type[Recordable]
- def register_session_injector(fn: SessionInjector) -> Callable[[], None]
- def registered_session_injectors() -> tuple[SessionInjector, Ellipsis]
- def registered_types() -> list[str]
- def rehydrate(entry: Mapping[str, Any]) -> Recordable
- def require_h5py() -> Any
- def unregister_recordable(type_id: str)
- def unregister_session_injector(fn: SessionInjector) -> bool
- def write_manifest(h5_file: Any, manifest: SessionManifest)

## Variables

- DEFAULT_BUFFER_FRAMES: int
- EPISODE_BINDING_EVENT: str
- EPISODE_CMD_EVENT: str
- PoseBackend: Unknown
- STAGE_SNAPSHOT_BASENAME: str
- SCHEMA_VERSION: int
- SessionInjector: Unknown
- VALID_BINDING_ACTIONS: Unknown
- VALID_COMMANDS: Unknown
