# Public API for module isaacsim.replicator.writers:

## Classes

- class DataVisualizationWriter(Writer)
  - BB_2D_TIGHT: str
  - BB_2D_LOOSE: str
  - BB_3D: str
  - SUPPORTED_BACKGROUNDS: List
  - def __init__(self, output_dir: str, bounding_box_2d_tight: bool = False, bounding_box_2d_tight_params: dict = None, bounding_box_2d_loose: bool = False, bounding_box_2d_loose_params: dict = None, bounding_box_3d: bool = False, bounding_box_3d_params: dict = None, frame_padding: int = 4)
  - def write(self, data: dict)
  - def detach(self)

- class DOPEWriter(Writer)
  - def __init__(self, output_dir: str, class_name_to_index_map: Dict, semantic_types: List[str] = None, image_output_format: str = 'png', use_s3: bool = False, bucket_name: str = '', endpoint_url: str = '', s3_region: str = 'us-east-1')
  - def register_pose_annotator(config_data: dict)
  - def setup_writer(config_data: dict, writer_config: dict)
  - def write(self, data: dict)
  - def is_last_frame_valid(self) -> bool

- class PoseWriter(Writer)
  - RGB_ANNOT_NAME: str
  - BB3D_ANNOT_NAME: str
  - CAM_PARAMS_ANNOT_NAME: str
  - SUPPORTED_FORMATS: Unknown
  - CUBOID_KEYPOINTS_ORDER_DEFAULT: List
  - CUBOID_KEYPOINT_ORDER_DOPE: List
  - CUBOID_KEYPOINT_COLORS: List
  - CUBOID_EDGE_COLORS: Dict
  - def __init__(self, output_dir: str = None, use_subfolders: bool = False, visibility_threshold: float = 0.0, skip_empty_frames: bool = True, write_debug_images: bool = False, frame_padding: int = 6, format: str = None, use_s3: bool = False, s3_bucket: str = None, s3_endpoint_url: str = None, s3_region: str = None, backend: BaseBackend = None, image_output_format: str = 'png')
  - def write(self, data: dict)
  - def get_current_frame_id(self)
  - def detach(self)

- class PytorchListener
  - def __init__(self)
  - def write_data(self, data: dict)
  - def get_rgb_data(self) -> torch.Tensor | None

- class PytorchWriter(Writer)
  - def __init__(self, listener: PytorchListener, output_dir: str = None, tiled_sensor: bool = False, device: str = 'cuda')
  - def write(self, data: dict)

- class YCBVideoWriter(Writer)
  - def __init__(self, output_dir: str, num_frames: int, semantic_types: List[str] = None, rgb: bool = False, bounding_box_2d_tight: bool = False, semantic_segmentation: bool = False, distance_to_image_plane: bool = False, image_output_format: str = 'png', pose: bool = False, class_name_to_index_map: Dict = None, factor_depth: int = 10000, intrinsic_matrix: np.ndarray = None)
  - def register_pose_annotator(config_data: dict)
  - def setup_writer(config_data: dict, writer_config: dict)
  - def write(self, data: dict)
  - def save_mesh_vertices(mesh_prim: UsdGeom.Mesh, coord_prim: Usd.Prim, model_name: str, output_folder: str)
  - def is_last_frame_valid(self) -> bool
