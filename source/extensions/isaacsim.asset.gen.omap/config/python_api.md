# Public API for module isaacsim.asset.gen.omap:

## Functions

- def update_location(om: object, start_location: tuple[float, float, float], lower_bound: tuple[float, float, float], upper_bound: tuple[float, float, float])
- def compute_coordinates(om: object, cell_size: float) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float], np.ndarray]
- def generate_image(om: object, occupied_col: list[int], unknown_col: list[int], freespace_col: list[int]) -> list[int]
