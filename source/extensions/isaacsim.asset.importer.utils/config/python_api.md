# Public API for module isaacsim.asset.importer.utils:

# Public API for module isaacsim.asset.importer.utils.impl.importer_utils:

## Classes

- class PhysxAttr(Enum)
  - JOINT_ARMATURE: Tuple
  - JOINT_FRICTION: Tuple
  - JOINT_MAX_VELOCITY: Tuple
  - ARTICULATION_SELF_COLLISION: Tuple
  - [property] def name(self) -> str
  - [property] def type(self) -> Sdf.ValueTypeName

- class PhysxMimicAttr(Enum)
  - GEARING: Tuple
  - OFFSET: Tuple
  - REFERENCE_JOINT_AXIS: Tuple
  - [property] def type(self) -> Sdf.ValueTypeName
  - def format(self, axis: str) -> str

- class PhysxMimicRel(Enum)
  - REFERENCE_JOINT: str
  - def format(self, axis: str) -> str

- class PhysxSchema(str, Enum)
  - JOINT_API: str
  - ARTICULATION_API: str
  - MIMIC_JOINT_API: str
  - JOINT_STATE_API: str

## Functions

- def collision_from_visuals(stage: Usd.Stage, collision_type: str) -> int
- def enable_self_collision(usd_stage: Usd.Stage, enabled: bool = True) -> int
- def run_asset_transformer_profile(input_stage_path: str, output_package_root: str, profile_json_path: str)
- def delete_scope(stage: Usd.Stage, prim_path: str)
- def add_joint_schemas(stage: Usd.Stage)
- def add_rigid_body_schemas(stage: Usd.Stage)
- def remove_custom_scopes(stage: Usd.Stage)
- def parse_robot_name(path: str, *, expected_extension: str) -> str
- def create_robot_schema(stage: Usd.Stage, robot_type: str = 'Default', *, prim_path: str | None = None, add_sites: bool = True, sites_last: bool = False) -> tuple[Usd.Prim | None, Usd.Prim | None]

## Variables

- USD_GEOMETRY_TYPES: Set
- MESH_APPROXIMATION_MAP: Dict
- PHYSICS_AXIS_MAP: Dict
- ROBOT_TYPE_TOKENS: list[str]

# Public API for module isaacsim.asset.importer.utils.impl.asset_utils:

## Functions

- def apply_fix_base(stage: Usd.Stage)
- def fix_articulation_root_for_fixed_base(stage: Usd.Stage) -> int
- def apply_link_density(stage: Usd.Stage, density: float)
- def apply_joint_drives(stage: Usd.Stage, drive_type: str | dict[str, str] | None = None, target_type: str | dict[str, str] | None = None, stiffness: float | dict[str, float] | None = None, damping: float | dict[str, float] | None = None)
- def apply_mjc_actuator_gains(stage: Usd.Stage, gain_type: str, bias_type: str, gain_prm: list[float], bias_prm: list[float]) -> int


# Public API for module isaacsim.asset.importer.utils.impl.merge_mesh_utils:

## Functions

- def clean_mesh_operation(stage: Usd.Stage)
- def generate_mesh_uv_normals_operation(stage: Usd.Stage)
- def merge_meshes_operation(stage: Usd.Stage) -> int
- def merge_mesh(stage: Usd.Stage, meshes: list[str])


# Public API for module isaacsim.asset.importer.utils.impl.stage_utils:

## Functions

- def save_stage(stage: Usd.Stage, usd_path: str) -> bool
- def open_stage(usd_path: str) -> Usd.Stage
- def get_stage_id(stage: Usd.Stage) -> int
