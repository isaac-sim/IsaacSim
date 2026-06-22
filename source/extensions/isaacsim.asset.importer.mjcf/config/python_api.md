# Public API for module isaacsim.asset.importer.mjcf:

## Classes

- class MJCFImporterConfig
  - mjcf_path: str | None
  - usd_path: str | None
  - import_scene: bool
  - merge_mesh: bool
  - debug_mode: bool
  - collision_from_visuals: bool
  - collision_type: str
  - allow_self_collision: bool
  - robot_type: str
  - fix_base: bool | None
  - link_density: float | None
  - joint_drive_type: str | dict[str, str] | None
  - joint_target_type: str | dict[str, str] | None
  - override_gain_type: str | None
  - override_bias_type: str | None
  - override_gain_prm: list[float] | None
  - override_bias_prm: list[float] | None
  - run_asset_transformer: bool
  - run_multi_physics_conversion: bool

- class MJCFImporter
  - def __init__(self, config: MJCFImporterConfig | None = None)
  - [property] def config(self) -> MJCFImporterConfig
  - [config.setter] def config(self, config: MJCFImporterConfig)
  - def import_mjcf(self, config: MJCFImporterConfig | None = None) -> str
