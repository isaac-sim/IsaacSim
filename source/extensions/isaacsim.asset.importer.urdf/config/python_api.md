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
  - merge_mesh: bool
  - debug_mode: bool
  - collision_from_visuals: bool
  - collision_type: str
  - allow_self_collision: bool
  - ros_package_paths: list[dict[str, str]]

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)
