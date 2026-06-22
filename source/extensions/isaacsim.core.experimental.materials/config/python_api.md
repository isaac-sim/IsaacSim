# Public API for module isaacsim.core.experimental.materials:

## Classes

- class NonVisualMaterial(Prim)
  - def __init__(self, paths: str | list[str])
  - [property] def materials(self) -> list[UsdShade.Material]
  - static def encode_material_ids(prims: str | Usd.Prim | UsdShade.Material | list[str | Usd.Prim | UsdShade.Material] | NonVisualMaterial) -> wp.array
  - static def decode_material_ids(ids: int | list | np.ndarray | wp.array) -> list[tuple[str, str, str]]
  - def set_bases(self, bases: str | list[str])
  - def get_bases(self) -> list[str]
  - def set_coatings(self, coatings: str | list[str])
  - def get_coatings(self) -> list[str]
  - def set_attributes(self, attributes: str | list[str])
  - def get_attributes(self) -> list[str]

- class PhysicsMaterial(Prim, ABC)
  - def __init__(self, paths: str | list[str])
  - [property] def materials(self) -> list[UsdShade.Material]
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - static def fetch_instances(paths: str | Usd.Prim | list[str | Usd.Prim]) -> list[PhysicsMaterial | None]

- class RigidBodyMaterial(PhysicsMaterial)
  - def __init__(self, paths: str | list[str])
  - def set_friction_coefficients(self, static_frictions: float | list | np.ndarray | wp.array = None, dynamic_frictions: float | list | np.ndarray | wp.array = None)
  - def get_friction_coefficients(self) -> tuple[wp.array, wp.array]
  - def set_restitution_coefficients(self, restitutions: float | list | np.ndarray | wp.array)
  - def get_restitution_coefficients(self) -> wp.array
  - def set_densities(self, densities: float | list | np.ndarray | wp.array)
  - def get_densities(self) -> wp.array
  - def set_combine_modes(self, frictions: Literal['average', 'max', 'min', 'multiply'] | list[Literal['average', 'max', 'min', 'multiply']] | None = None, restitutions: Literal['average', 'max', 'min', 'multiply'] | list[Literal['average', 'max', 'min', 'multiply']] | None = None, dampings: Literal['average', 'max', 'min', 'multiply'] | list[Literal['average', 'max', 'min', 'multiply']] | None = None)
  - def get_combine_modes(self) -> tuple[list[Literal[average, max, min, multiply]], list[Literal[average, max, min, multiply]], list[Literal[average, max, min, multiply]]]
  - def set_enabled_compliant_contacts(self, enabled: bool | list | np.ndarray | wp.array)
  - def get_enabled_compliant_contacts(self) -> wp.array
  - def set_compliant_contact_gains(self, stiffnesses: float | list | np.ndarray | wp.array | None = None, dampings: float | list | np.ndarray | wp.array | None = None)
  - def get_compliant_contact_gains(self) -> tuple[wp.array, wp.array]
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class SurfaceDeformableMaterial(PhysicsMaterial)
  - def __init__(self, paths: str | list[str])
  - def set_friction_coefficients(self, static_frictions: float | list | np.ndarray | wp.array = None, dynamic_frictions: float | list | np.ndarray | wp.array = None)
  - def get_friction_coefficients(self) -> tuple[wp.array, wp.array]
  - def set_youngs_moduli(self, youngs_moduli: float | list | np.ndarray | wp.array)
  - def get_youngs_moduli(self) -> wp.array
  - def set_poissons_ratios(self, poissons_ratios: float | list | np.ndarray | wp.array)
  - def get_poissons_ratios(self) -> wp.array
  - def set_densities(self, densities: float | list | np.ndarray | wp.array)
  - def get_densities(self) -> wp.array
  - def set_surface_thicknesses(self, surface_thicknesses: float | list | np.ndarray | wp.array)
  - def get_surface_thicknesses(self) -> wp.array
  - def set_surface_stiffnesses(self, stretch_stiffnesses: float | list | np.ndarray | wp.array | None = None, shear_stiffnesses: float | list | np.ndarray | wp.array | None = None, bend_stiffnesses: float | list | np.ndarray | wp.array | None = None)
  - def get_surface_stiffnesses(self) -> tuple[wp.array, wp.array, wp.array]
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class VolumeDeformableMaterial(PhysicsMaterial)
  - def __init__(self, paths: str | list[str])
  - def set_friction_coefficients(self, static_frictions: float | list | np.ndarray | wp.array = None, dynamic_frictions: float | list | np.ndarray | wp.array = None)
  - def get_friction_coefficients(self) -> tuple[wp.array, wp.array]
  - def set_youngs_moduli(self, youngs_moduli: float | list | np.ndarray | wp.array)
  - def get_youngs_moduli(self) -> wp.array
  - def set_poissons_ratios(self, poissons_ratios: float | list | np.ndarray | wp.array)
  - def get_poissons_ratios(self) -> wp.array
  - def set_densities(self, densities: float | list | np.ndarray | wp.array)
  - def get_densities(self) -> wp.array
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class OmniGlassMaterial(VisualMaterial)
  - def __init__(self, paths: str | list[str])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class OmniPbrMaterial(VisualMaterial)
  - def __init__(self, paths: str | list[str])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class PreviewSurfaceMaterial(VisualMaterial)
  - def __init__(self, paths: str | list[str])
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array

- class VisualMaterial(Prim, ABC)
  - def __init__(self, paths: str | list[str])
  - [property] def materials(self) -> list[UsdShade.Material]
  - [property] def shaders(self) -> list[UsdShade.Shader]
  - def set_input_values(self, name: str, values: str | bool | int | float | list | np.ndarray | wp.array)
  - def get_input_values(self, name: str) -> wp.array
  - static def are_of_type(paths: str | Usd.Prim | list[str | Usd.Prim]) -> wp.array
  - static def fetch_instances(paths: str | Usd.Prim | list[str | Usd.Prim]) -> list[VisualMaterial | None]
