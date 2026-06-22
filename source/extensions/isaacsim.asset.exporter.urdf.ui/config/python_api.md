# Public API for module isaacsim.asset.exporter.urdf.ui:

## Classes

- class UsdToUrdfConverter
  - def __init__(self, stage: Usd.Stage | str | os.PathLike, root_prim_path: str | None = None, mesh_dir_name: str = 'meshes', mesh_path_prefix: str = './', visualize_collision_meshes: bool = False, variant_selections: dict[str, str] | None = None)
  - def convert(self, output_path: str | None = None) -> str

- class OptionWidget
  - def __init__(self)
  - [property] def mesh_dir_name(self) -> str
  - [property] def mesh_path_prefix(self) -> str
  - [property] def root_prim_path(self) -> str | None
  - [property] def visualize_collision_meshes(self) -> bool
  - [property] def package_name(self) -> str
  - [property] def use_physx_inertia(self) -> bool
  - def cleanup(self)
  - def build(self)

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

- class UrdfExporterDelegate(ExportOptionsDelegate)
  - def __init__(self)
  - def export(self, filename: str, dirname: str, extension: str = '', selections: list[str] | None = None)
  - def cleanup(self)

## Functions

- def get_instance() -> Extension | None

## Variables

- EXTENSION_TITLE: str
