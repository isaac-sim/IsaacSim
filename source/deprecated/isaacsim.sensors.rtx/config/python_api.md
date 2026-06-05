# Public API for module isaacsim.sensors.rtx:

## Classes

- class IsaacSensorCreateRtxLidar(IsaacSensorCreateRtxSensor)
  - def __init__(self, **kwargs: Any)
  - def do(self) -> Usd.Prim

- class IsaacSensorCreateRtxIDS(IsaacSensorCreateRtxSensor)
  - def __init__(self, **kwargs: Any)

- class IsaacSensorCreateRtxRadar(IsaacSensorCreateRtxSensor)
  - def do(self) -> Usd.Prim | None

- class LidarRtx(BaseSensor)
  - static def make_add_remove_deprecated_attr(deprecated_attr: str) -> list[Callable]
  - def __init__(self, prim_path: str, name: str = 'lidar_rtx', position: np.ndarray | None = None, translation: np.ndarray | None = None, orientation: np.ndarray | None = None, config_file_name: str | None = None, **kwargs)
  - def get_render_product_path(self) -> str | None
  - def get_current_frame(self) -> dict
  - def get_annotators(self) -> dict
  - def attach_annotator(self, annotator_name: Literal[IsaacComputeRTXLidarFlatScan, IsaacExtractRTXSensorPointCloudNoAccumulator, IsaacCreateRTXLidarScanBuffer, StableIdMap, GenericModelOutput], **kwargs)
  - def detach_annotator(self, annotator_name: str)
  - def detach_all_annotators(self)
  - def get_writers(self) -> dict
  - def attach_writer(self, writer_name: str, **kwargs)
  - def detach_writer(self, writer_name: str)
  - def detach_all_writers(self)
  - def initialize(self, physics_sim_view: Any = None)
  - def post_reset(self)
  - def resume(self)
  - def pause(self)
  - def is_paused(self) -> bool
  - def get_horizontal_resolution(self) -> float | None
  - def get_horizontal_fov(self) -> float | None
  - def get_num_rows(self) -> int | None
  - def get_num_cols(self) -> int | None
  - def get_rotation_frequency(self) -> float | None
  - def get_depth_range(self) -> tuple[float, float] | None
  - def get_azimuth_range(self) -> tuple[float, float] | None
  - def enable_visualization(self)
  - def disable_visualization(self)
  - def add_point_cloud_data_to_frame(self)
  - def add_linear_depth_data_to_frame(self)
  - def add_intensities_data_to_frame(self)
  - def add_azimuth_range_to_frame(self)
  - def add_horizontal_resolution_to_frame(self)
  - def add_range_data_to_frame(self)
  - def add_azimuth_data_to_frame(self)
  - def add_elevation_data_to_frame(self)
  - def remove_point_cloud_data_to_frame(self)
  - def remove_linear_depth_data_to_frame(self)
  - def remove_intensities_data_to_frame(self)
  - def remove_azimuth_range_to_frame(self)
  - def remove_horizontal_resolution_to_frame(self)
  - def remove_range_data_to_frame(self)
  - def remove_azimuth_data_to_frame(self)
  - def remove_elevation_data_to_frame(self)
  - static def decode_stable_id_mapping(stable_id_mapping_raw: bytes) -> dict
  - static def get_object_ids(obj_ids: np.ndarray) -> list[int]

# Public API for module isaacsim.sensors.rtx.generic_model_output:

No public API

# Public API for module isaacsim.sensors.rtx.sensor_checker:

No public API
