# Public API for module isaacsim.core.experimental.objects:

## Classes

- class Camera(XformPrim)
  - def __init__(self, paths: str | list[str])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - def set_focal_lengths(self, focal_lengths: float | list | np.ndarray | wp.array)
  - def get_focal_lengths(self) -> wp.array
  - def set_focus_distances(self, focus_distances: float | list | np.ndarray | wp.array)
  - def get_focus_distances(self) -> wp.array
  - def set_stereo_roles(self, roles: Literal['mono', 'left', 'right'] | list[Literal['mono', 'left', 'right']])
  - def get_stereo_roles(self) -> list[Literal[mono, left, right]]
  - def set_fstops(self, fstops: float | list | np.ndarray | wp.array)
  - def get_fstops(self) -> wp.array
  - def set_apertures(self, horizontal_apertures: float | list | np.ndarray | wp.array = None, vertical_apertures: float | list | np.ndarray | wp.array = None)
  - def get_apertures(self) -> tuple[wp.array, wp.array]
  - def set_aperture_offsets(self, horizontal_offsets: float | list | np.ndarray | wp.array = None, vertical_offsets: float | list | np.ndarray | wp.array = None)
  - def get_aperture_offsets(self) -> tuple[wp.array, wp.array]
  - def set_projections(self, projections: Literal['perspective', 'orthographic'] | list[Literal['perspective', 'orthographic']])
  - def get_projections(self) -> list[Literal[perspective, orthographic]]
  - def set_clipping_ranges(self, near_distances: float | list | np.ndarray | wp.array = None, far_distances: float | list | np.ndarray | wp.array = None)
  - def get_clipping_ranges(self) -> tuple[wp.array, wp.array]
  - def set_shutter_times(self, open_times: float | list | np.ndarray | wp.array = None, close_times: float | list | np.ndarray | wp.array = None)
  - def get_shutter_times(self) -> tuple[wp.array, wp.array]
  - def enforce_square_pixels(self, resolutions: list | np.ndarray | wp.array)

- class GroundPlane(XformPrim)
  - def __init__(self, paths: str | list[str])
  - [property] def planes(self) -> Plane
  - [property] def meshes(self) -> Mesh
  - def set_offsets(self, contact_offsets: float | list | np.ndarray | wp.array = None, rest_offsets: float | list | np.ndarray | wp.array = None)
  - def get_offsets(self) -> tuple[wp.array, wp.array]
  - def set_torsional_patch_radii(self, radii: float | list | np.ndarray | wp.array)
  - def get_torsional_patch_radii(self) -> wp.array
  - def set_enabled_collisions(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_enabled_collisions(self) -> wp.array
  - def apply_physics_materials(self, materials: type['PhysicsMaterial'] | list[type['PhysicsMaterial']])
  - def get_applied_physics_materials(self) -> list[type['PhysicsMaterial'] | None]
  - def apply_visual_templates(self, templates: TEMPLATE | list[TEMPLATE])

- class CylinderLight(Light)
  - def __init__(self, paths: str | list[str])
  - def set_lengths(self, lengths: float | list | np.ndarray | wp.array)
  - def get_lengths(self) -> wp.array
  - def set_radii(self, radii: float | list | np.ndarray | wp.array)
  - def get_radii(self) -> wp.array
  - def set_enabled_treat_as_lines(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_enabled_treat_as_lines(self) -> wp.array
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class DiskLight(Light)
  - def __init__(self, paths: str | list[str])
  - def set_radii(self, radii: float | list | np.ndarray | wp.array)
  - def get_radii(self) -> wp.array
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class DistantLight(Light)
  - def __init__(self, paths: str | list[str])
  - def set_angles(self, angles: float | list | np.ndarray | wp.array)
  - def get_angles(self) -> wp.array
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class DomeLight(Light)
  - def __init__(self, paths: str | list[str])
  - def set_guide_radii(self, radii: float | list | np.ndarray | wp.array)
  - def get_guide_radii(self) -> wp.array
  - def set_texture_files(self, texture_files: str | list[str])
  - def get_texture_files(self) -> list[str]
  - def set_texture_formats(self, texture_formats: Literal['automatic', 'latlong', 'mirroredBall', 'angular', 'cubeMapVerticalCross'] | list[Literal['automatic', 'latlong', 'mirroredBall', 'angular', 'cubeMapVerticalCross']])
  - def get_texture_formats(self) -> list[Literal[automatic, latlong, mirroredBall, angular, cubeMapVerticalCross]]
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class Light(XformPrim, ABC)
  - def __init__(self, paths: str | list[str])
  - [property] def lights(self) -> list[UsdLux.Light]
  - def set_intensities(self, intensities: float | list | np.ndarray | wp.array)
  - def get_intensities(self) -> wp.array
  - def set_exposures(self, exposures: float | list | np.ndarray | wp.array)
  - def get_exposures(self) -> wp.array
  - def set_multipliers(self, diffuse_multipliers: float | list | np.ndarray | wp.array = None, specular_multipliers: float | list | np.ndarray | wp.array = None)
  - def get_multipliers(self) -> tuple[wp.array, wp.array]
  - def set_enabled_normalizations(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_enabled_normalizations(self) -> wp.array
  - def set_enabled_color_temperatures(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_enabled_color_temperatures(self) -> wp.array
  - def set_color_temperatures(self, color_temperatures: float | list | np.ndarray | wp.array)
  - def get_color_temperatures(self) -> wp.array
  - def set_colors(self, colors: list | np.ndarray | wp.array)
  - def get_colors(self) -> wp.array
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - static def fetch_instances(paths: str | Usd.Prim | list[str | Usd.Prim]) -> list[Light | None]

- class RectLight(Light)
  - def __init__(self, paths: str | list[str])
  - def set_widths(self, widths: float | list | np.ndarray | wp.array)
  - def get_widths(self) -> wp.array
  - def set_heights(self, heights: float | list | np.ndarray | wp.array)
  - def get_heights(self) -> wp.array
  - def set_texture_files(self, texture_files: str | list[str])
  - def get_texture_files(self) -> list[str | None]
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class SphereLight(Light)
  - def __init__(self, paths: str | list[str])
  - def set_radii(self, radii: float | list | np.ndarray | wp.array)
  - def get_radii(self) -> wp.array
  - def set_enabled_treat_as_points(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_enabled_treat_as_points(self) -> wp.array
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class Mesh(XformPrim)
  - def __init__(self, paths: str | list[str])
  - [property] def geoms(self) -> list[UsdGeom.Mesh]
  - [property] def num_faces(self) -> list[int]
  - static def update_extents(geoms: list[UsdGeom.Mesh])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - static def fetch_instances(paths: str | Usd.Prim | list[str | Usd.Prim]) -> list[Mesh | None]
  - def set_display_colors(self, colors: str | list | np.ndarray | wp.array)
  - def set_points(self, points: list[list | np.ndarray | wp.array])
  - def get_points(self) -> list[wp.array]
  - def set_normals(self, normals: list[list | np.ndarray | wp.array])
  - def get_normals(self) -> list[wp.array]
  - def set_face_specs(self, vertex_indices: list[list | np.ndarray | wp.array] | None = None, vertex_counts: list[list | np.ndarray | wp.array] | None = None, varying_linear_interpolations: list[Literal['none', 'cornersOnly', 'cornersPlus1', 'cornersPlus2', 'boundaries', 'all']] | None = None, hole_indices: list[list | np.ndarray | wp.array] | None = None)
  - def get_face_specs(self) -> tuple[list[wp.array], list[wp.array], list[Literal[none, cornersOnly, cornersPlus1, cornersPlus2, boundaries, all]], list[wp.array]]
  - def set_crease_specs(self, crease_indices: list[list | np.ndarray | wp.array], crease_lengths: list[list | np.ndarray | wp.array], crease_sharpnesses: list[list | np.ndarray | wp.array])
  - def get_crease_specs(self) -> tuple[list[wp.array], list[wp.array], list[wp.array]]
  - def set_corner_specs(self, corner_indices: list[list | np.ndarray | wp.array], corner_sharpnesses: list[list | np.ndarray | wp.array])
  - def get_corner_specs(self) -> tuple[list[wp.array], list[wp.array]]
  - def set_subdivision_specs(self, subdivision_schemes: list[Literal['catmullClark', 'loop', 'bilinear', 'none']] | None = None, interpolate_boundaries: list[Literal['none', 'edgeOnly', 'edgeAndCorner']] | None = None, triangle_subdivision_rules: list[Literal['catmullClark', 'smooth']] | None = None)
  - def get_subdivision_specs(self) -> tuple[list[Literal[catmullClark, loop, bilinear, none]], list[Literal[none, edgeOnly, edgeAndCorner]], list[Literal[catmullClark, smooth]]]

- class Capsule(Shape)
  - def __init__(self, paths: str | list[str])
  - static def update_extents(geoms: list[UsdGeom.Capsule])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - def set_radii(self, radii: float | list | np.ndarray | wp.array)
  - def get_radii(self) -> wp.array
  - def set_heights(self, heights: float | list | np.ndarray | wp.array)
  - def get_heights(self) -> wp.array
  - def set_axes(self, axes: Literal['X', 'Y', 'Z'] | list[Literal['X', 'Y', 'Z']])
  - def get_axes(self) -> list[Literal[X, Y, Z]]

- class Cone(Shape)
  - def __init__(self, paths: str | list[str])
  - static def update_extents(geoms: list[UsdGeom.Cone])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - def set_radii(self, radii: float | list | np.ndarray | wp.array)
  - def get_radii(self) -> wp.array
  - def set_heights(self, heights: float | list | np.ndarray | wp.array)
  - def get_heights(self) -> wp.array
  - def set_axes(self, axes: Literal['X', 'Y', 'Z'] | list[Literal['X', 'Y', 'Z']])
  - def get_axes(self) -> list[Literal[X, Y, Z]]

- class Cube(Shape)
  - def __init__(self, paths: str | list[str])
  - static def update_extents(geoms: list[UsdGeom.Cube])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - def set_sizes(self, sizes: float | list | np.ndarray | wp.array)
  - def get_sizes(self) -> wp.array

- class Cylinder(Shape)
  - def __init__(self, paths: str | list[str])
  - static def update_extents(geoms: list[UsdGeom.Cylinder])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - def set_radii(self, radii: float | list | np.ndarray | wp.array)
  - def get_radii(self) -> wp.array
  - def set_heights(self, heights: float | list | np.ndarray | wp.array)
  - def get_heights(self) -> wp.array
  - def set_axes(self, axes: Literal['X', 'Y', 'Z'] | list[Literal['X', 'Y', 'Z']])
  - def get_axes(self) -> list[Literal[X, Y, Z]]

- class Plane(Shape)
  - def __init__(self, paths: str | list[str])
  - static def update_extents(geoms: list[UsdGeom.Plane])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - def set_widths(self, widths: float | list | np.ndarray | wp.array)
  - def get_widths(self) -> wp.array
  - def set_lengths(self, lengths: float | list | np.ndarray | wp.array)
  - def get_lengths(self) -> wp.array
  - def set_axes(self, axes: Literal['X', 'Y', 'Z'] | list[Literal['X', 'Y', 'Z']])
  - def get_axes(self) -> list[Literal[X, Y, Z]]

- class Shape(XformPrim, ABC)
  - def __init__(self, paths: str | list[str])
  - [property] def geoms(self) -> list[UsdGeom.Gprim]
  - static def update_extents(geoms: list[UsdGeom.Gprim])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - static def fetch_instances(paths: str | Usd.Prim | list[str | Usd.Prim]) -> list[Shape | None]
  - def set_display_colors(self, colors: str | list | np.ndarray | wp.array)

- class Sphere(Shape)
  - def __init__(self, paths: str | list[str])
  - static def update_extents(geoms: list[UsdGeom.Sphere])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - def set_radii(self, radii: float | list | np.ndarray | wp.array)
  - def get_radii(self) -> wp.array
