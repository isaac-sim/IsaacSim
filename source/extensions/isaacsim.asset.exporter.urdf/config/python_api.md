# Public API for module isaacsim.asset.exporter.urdf:

## Classes

- class UsdToUrdfConverter
  - def __init__(self, stage: Usd.Stage | str | os.PathLike, root_prim_path: str | None = None, mesh_dir_name: str = 'meshes', mesh_path_prefix: str = './', visualize_collision_meshes: bool = False, variant_selections: dict[str, str] | None = None)
  - def convert(self, output_path: str | None = None) -> str
