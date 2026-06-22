# Public API for module isaacsim.core.cloner:

## Classes

- class Cloner
  - def __init__(self, stage: Usd.Stage = None)
  - def define_base_env(self, base_env_path: str)
  - def generate_paths(self, root_path: str, num_paths: int) -> list[str]
  - def replicate_physics(self, source_prim_path: str, prim_paths: list, base_env_path: str, root_path: str, enable_env_ids: bool = False, clone_in_fabric: bool = False)
  - def disable_change_listener(self)
  - def enable_change_listener(self)
  - def clone(self, source_prim_path: str, prim_paths: list[str], positions: np.ndarray | 'torch.Tensor' = None, orientations: np.ndarray | 'torch.Tensor' = None, replicate_physics: bool = False, base_env_path: str = None, root_path: str = None, copy_from_source: bool = False, unregister_physics_replication: bool = False, enable_env_ids: bool = False, clone_in_fabric: bool = False)
  - def filter_collisions(self, physicsscene_path: str, collision_root_path: str, prim_paths: list[str], global_paths: list[str] | None = None)

- class GridCloner(Cloner)
  - def __init__(self, spacing: float, num_per_row: int = -1, stage: Usd.Stage = None)
  - def get_clone_transforms(self, num_clones: int, position_offsets: np.ndarray = None, orientation_offsets: np.ndarray = None) -> tuple[list, list]
  - def clone(self, source_prim_path: str, prim_paths: list[str], position_offsets: np.ndarray = None, orientation_offsets: np.ndarray = None, replicate_physics: bool = False, base_env_path: str = None, root_path: str = None, copy_from_source: bool = False, enable_env_ids: bool = False, clone_in_fabric: bool = False) -> list
