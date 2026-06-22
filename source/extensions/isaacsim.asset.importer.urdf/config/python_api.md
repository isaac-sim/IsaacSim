# Public API for module isaacsim.asset.importer.urdf:

## Classes

- class URDFImporter
  - def __init__(self, config: URDFImporterConfig | None = None)
  - [property] def config(self) -> URDFImporterConfig
  - [config.setter] def config(self, config: URDFImporterConfig)
  - def import_urdf(self, config: URDFImporterConfig | None = None) -> str

- class URDFImporterConfig
  - urdf_path: str | None
  - usd_path: str | None
  - merge_fixed_joints: bool
  - merge_mesh: bool
  - debug_mode: bool
  - collision_from_visuals: bool
  - collision_type: str
  - allow_self_collision: bool
  - ros_package_paths: list[dict[str, str]]
  - robot_type: str
  - fix_base: bool | None
  - link_density: float | None
  - joint_drive_type: str | dict[str, str] | None
  - joint_target_type: str | dict[str, str] | None
  - override_joint_stiffness: float | dict[str, float] | None
  - override_joint_damping: float | dict[str, float] | None
  - run_asset_transformer: bool
  - run_multi_physics_conversion: bool
