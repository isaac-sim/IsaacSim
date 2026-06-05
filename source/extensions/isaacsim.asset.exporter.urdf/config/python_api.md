# Public API for module isaacsim.asset.exporter.urdf:

## Classes

- class UrdfExporter
  - def __init__(self)
  - def cleanup(self)
  - def build_exporter_options(self)

- class UrdfExporterDelegate(ExportOptionsDelegate)
  - def __init__(self)
  - def export(self, filename: str, dirname: str, extension: str = '', selections: list[str] | None = None)
  - def cleanup(self)
