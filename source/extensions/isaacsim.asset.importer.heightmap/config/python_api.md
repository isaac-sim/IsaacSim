# Public API for module isaacsim.asset.importer.heightmap:

## Classes

- class HeightmapImporter
  - def __init__(self, stage: Usd.Stage | None = None)
  - [property] def terrain_path(self) -> str | None
  - def create_heightmap(self, image: Image.Image, cell_scale: float, create_ground_plane: bool = True, create_lighting: bool = True, configure_stage_metadata: bool = True, overwrite_existing: bool = True) -> int
