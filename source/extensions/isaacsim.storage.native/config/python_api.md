# Public API for module isaacsim.storage.native:

## Classes

- class Version(namedtuple('Version', 'major minor patch'))

## Functions

- def get_assets_root_path() -> str
- async def get_assets_root_path_async() -> str
- def path_join(base: str, name: str) -> str
- def is_local_path(path: str) -> bool
- def find_files_recursive(abs_path: list[str], filter_fn: Callable[[str], bool] = lambda a: True) -> list[str]
- def find_filtered_files(abs_paths: list[str], max_depth: int | None = None, filepath_excludes: list[str] | None = None, filter_patterns: list[str] | None = None, match_all: bool = False) -> set[str]
- def get_stage_references(stage_path: str, resolve_relatives: bool = True) -> list[str]
- def is_absolute_path(path: str) -> bool
- def is_valid_usd_file(item: str, excludes: list) -> bool
- def is_mdl_file(item: str) -> bool
- async def find_absolute_paths_in_usds(base_path: str) -> dict[str, list[str]]
- def is_path_external(path: str, base_path: str) -> bool
- async def find_external_references(base_path: str) -> dict[str, list[str]]
- async def count_asset_references(base_path: str) -> dict[str, int]
- def find_missing_references(base_path: str)
- async def path_exists(path: str) -> bool
- def layer_has_missing_references(layer_identifier: str) -> bool
- def prim_spec_has_missing_references(prim_spec: object) -> bool
- def prim_has_missing_references(prim: object) -> bool
- def path_relative(path: str, start: str) -> str
- def path_dirname(path: str) -> str
- async def resolve_asset_path_async(original_path: str) -> str | None
- def resolve_asset_path(original_path: str) -> str | None
- async def find_filtered_files_async(root_path: str, filter_patterns: list[str] | None = None, match_all: bool = False, filepath_excludes: list[str] | None = None, max_depth: int | None = None) -> set[str]
- def get_url_root(url: str) -> str
- def create_folder(server: str, path: str) -> bool
- def delete_folder(server: str, path: str) -> bool
- async def download_assets_async(src: str, dst: str, progress_callback: object, concurrency: int = 10, copy_behaviour: omni.client.CopyBehavior = CopyBehavior.OVERWRITE, copy_after_delete: bool = True, timeout: float = 300.0) -> omni.client.Result
- def check_server(server: str, path: str, timeout: float = 10.0) -> bool
- async def check_server_async(server: str, path: str, timeout: float = 10.0) -> bool
- def build_server_list() -> list
- def find_nucleus_server(suffix: str) -> tuple[bool, str]
- def get_server_path(suffix: str = '') -> str | None
- async def get_server_path_async(suffix: str = '') -> str | None
- def verify_asset_root_path(path: str) -> tuple[omni.client.Result, str]
- def get_full_asset_path(path: str) -> str | None
- async def get_full_asset_path_async(path: str) -> str | None
- def get_nvidia_asset_root_path() -> str | None
- def get_isaac_asset_root_path() -> str | None
- def get_assets_server() -> str | None
- async def is_dir_async(path: str) -> bool
- def is_dir(path: str) -> bool
- async def is_file_async(path: str) -> bool
- def is_file(path: str) -> bool
- async def recursive_list_folder(path: str) -> list
- async def list_folder(path: str) -> tuple[list, list]
