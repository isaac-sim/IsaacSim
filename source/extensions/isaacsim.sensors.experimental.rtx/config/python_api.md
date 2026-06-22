# Public API for module isaacsim.sensors.experimental.rtx:

## Classes

- class Acoustic(_SensorAuthoring)
  - static def create(path: str) -> Acoustic

- class AcousticSensor(_SensorRuntime)
  - [property] def acoustic(self) -> Acoustic

- class CameraSensor(_SensorRuntime)
  - def __init__(self, path: str | RtxCamera)
  - [property] def camera(self) -> Any
  - [property] def resolution(self) -> tuple[int, int]
  - def attach_annotators(self, annotators: str | list[str]) -> dict[str, Any]
  - def get_data(self, annotator: str) -> tuple[wp.array | None, dict[str, Any]]

- class Lidar(_SensorAuthoring)
  - def __init__(self, path: str)
  - static def create(path: str) -> Lidar

- class LidarSensor(_SensorRuntime)
  - [property] def lidar(self) -> Lidar

- class Radar(_SensorAuthoring)
  - static def create(path: str) -> Radar

- class RadarSensor(_SensorRuntime)
  - [property] def radar(self) -> Radar

- class RtxCamera(_SensorAuthoring)
  - def __init__(self, path: str)
  - static def create(path: str) -> RtxCamera
  - [property] def camera(self) -> Camera

- class SingleViewDepthCameraSensor(CameraSensor)
  - def __init__(self, path: str | RtxCamera)
  - def set_sensor_baseline(self, baseline: float)
  - def get_sensor_baseline(self) -> float
  - def set_sensor_disparity_confidence(self, confidence_threshold: float)
  - def get_sensor_disparity_confidence(self) -> float
  - def set_sensor_maximum_disparity(self, maximum_disparity: float)
  - def get_sensor_maximum_disparity(self) -> float
  - def set_enabled_post_processing(self, enabled: bool)
  - def get_enabled_post_processing(self) -> bool
  - def set_sensor_focal_length(self, focal_length: float)
  - def get_sensor_focal_length(self) -> float
  - def set_sensor_distance_cutoffs(self, minimum_distance: float = None, maximum_distance: float = None)
  - def get_sensor_distance_cutoffs(self) -> tuple[float, float]
  - def set_sensor_disparity_noise_downscale(self, downscale: float)
  - def get_sensor_disparity_noise_downscale(self) -> float
  - def set_sensor_noise_parameters(self, noise_mean: float = None, noise_sigma: float = None)
  - def get_sensor_noise_parameters(self) -> tuple[float, float]
  - def set_enabled_outlier_removal(self, enabled: bool)
  - def get_enabled_outlier_removal(self) -> bool
  - def set_sensor_output_mode(self, mode: int)
  - def get_sensor_output_mode(self) -> int
  - def set_sensor_size(self, size: float)
  - def get_sensor_size(self) -> float
  - static def add_template_render_product(parent_prim_path: str, camera_prim_path: str, **kwargs: Any) -> Usd.Prim

- class StructuredLightCamera(RtxCamera)
  - def __init__(self, path: str, projector_light_patterns: list[str | Path], projector_direction_texture: str | Path)
  - def destroy(self)
  - def post_reset(self)
  - def get_active_pattern_index(self) -> int
  - def set_active_pattern_manual(self, pattern_index: int)
  - def get_num_patterns(self) -> int
  - def get_projector_prim_path(self) -> str
  - def get_rect_light_prims(self) -> list[Usd.Prim]
  - def get_projector_direction_texture(self) -> str | Path
  - def get_projector_timestamps(self) -> list[tuple[int, int]]
  - def set_projector_timestamps(self, timestamps: list[tuple[int, int]])
  - def get_projector_cycle_period(self) -> tuple[int, int]
  - def set_projector_cycle_period(self, period: tuple[int, int] | None)

- class TiledCameraSensor
  - def __init__(self, paths: str | list[str] | Camera)
  - [property] def annotators(self) -> list[str]
  - [property] def camera(self) -> Camera
  - [property] def resolution(self) -> tuple[int, int]
  - [property] def tiled_resolution(self) -> tuple[int, int]
  - [property] def render_product(self) -> UsdRender.Product
  - def attach_annotators(self, annotators: str | list[str]) -> dict[str, Any]
  - def detach_annotators(self, annotators: str | list[str])
  - def get_data(self, annotator: str) -> tuple[wp.array | None, dict[str, Any]]

## Functions

- def register_annotator_spec(name: str, spec: dict)
- def register_writer_spec(name: str, spec: dict)
- def unregister_annotator_spec(name: str)
- def unregister_writer_spec(name: str)
- def draw_annotator_data_to_image() -> np.ndarray
- def get_camera_metadata(config_path: str) -> dict[str, Any]
- def parse_generic_model_output_data(data: wp.array) -> generic_model_output.GenericModelOutput
- def parse_object_ids(obj_ids: np.ndarray) -> list[int]
- def parse_stable_id_map_data(data: wp.array) -> dict

## Variables

- SUPPORTED_ACOUSTIC_CONFIGS: dict[str, set[str]]
- SUPPORTED_ACOUSTIC_VARIANT_SET_NAME: str
- SUPPORTED_CAMERA_CONFIGS: dict[str, dict[str, Any]]
- SUPPORTED_CAMERA_VARIANT_SET_NAME: str
- SUPPORTED_LIDAR_CONFIGS: dict[str, set[str] | list[dict[str, str]]]
- SUPPORTED_LIDAR_VARIANT_SET_NAME: str
- SUPPORTED_RADAR_CONFIGS: Dict
- SUPPORTED_RADAR_VARIANT_SET_NAME: str

# Public API for module isaacsim.sensors.experimental.rtx.generic_model_output:

## Classes

- class AccessType
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - READ: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.AccessType
  - RECORD_BASIC: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.AccessType
  - RECORD_FULL: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.AccessType

- class AuxType
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - BASIC: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.AuxType
  - EXTRA: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.AuxType
  - FULL: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.AuxType
  - NONE: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.AuxType

- class CoordsType
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - CARTESIAN: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.CoordsType
  - NOT_APPLICABLE: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.CoordsType
  - SPHERICAL: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.CoordsType

- class ElementFlags
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - FLAG_1: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.ElementFlags
  - FLAG_2: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.ElementFlags
  - FLAG_3: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.ElementFlags
  - FLAG_4: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.ElementFlags
  - FLAG_5: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.ElementFlags
  - FLAG_6: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.ElementFlags
  - FLAG_7: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.ElementFlags
  - VALID: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.ElementFlags

- class FrameAtTime
  - def __init__(self)
  - [property] def orientation(self) -> typing.List[float]
  - [orientation.setter] def orientation(self, arg1: typing.List[float])
  - [property] def posM(self) -> typing.List[float]
  - [posM.setter] def posM(self, arg1: typing.List[float])
  - [property] def timestampNs(self) -> int
  - [timestampNs.setter] def timestampNs(self, arg0: int)

- class FrameOfReference
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - CUSTOM: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.FrameOfReference
  - PARENT: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.FrameOfReference
  - SENSOR: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.FrameOfReference
  - WORLD: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.FrameOfReference

- class GMOIOConfig
  - def __init__(self)
  - [property] def accessType(self) -> AccessType
  - [accessType.setter] def accessType(self, arg0: AccessType)
  - [property] def clientName(self) -> str
  - [clientName.setter] def clientName(self, arg1: str)
  - [property] def fileName(self) -> str
  - [fileName.setter] def fileName(self, arg1: str)
  - [property] def groupName(self) -> str
  - [groupName.setter] def groupName(self, arg1: str)
  - [property] def loop(self) -> bool
  - [loop.setter] def loop(self, arg0: bool)
  - [property] def maxPoints(self) -> int
  - [maxPoints.setter] def maxPoints(self, arg0: int)
  - [property] def onlyValid(self) -> bool
  - [onlyValid.setter] def onlyValid(self, arg0: bool)

- class GenericModelOutput
  - def __init__(self)
  - [property] def auxType(self) -> AuxType
  - [auxType.setter] def auxType(self, arg0: AuxType)
  - [property] def azimuthOffset(self) -> float
  - [property] def channelId(self) -> numpy.ndarray
  - [property] def cycleCnt(self) -> int
  - [property] def echoId(self) -> numpy.ndarray
  - [property] def elementsCoordsType(self) -> CoordsType
  - [elementsCoordsType.setter] def elementsCoordsType(self, arg0: CoordsType)
  - [property] def emitterId(self) -> numpy.ndarray
  - [property] def filledAuxMembers(self) -> LidarAuxHas
  - [property] def flags(self) -> numpy.ndarray
  - [property] def frameEnd(self) -> FrameAtTime
  - [frameEnd.setter] def frameEnd(self, arg0: FrameAtTime)
  - [property] def frameId(self) -> int
  - [frameId.setter] def frameId(self, arg0: int)
  - [property] def frameOfReference(self) -> FrameOfReference
  - [frameOfReference.setter] def frameOfReference(self, arg0: FrameOfReference)
  - [property] def frameStart(self) -> FrameAtTime
  - [frameStart.setter] def frameStart(self, arg0: FrameAtTime)
  - [property] def hitNormals(self) -> numpy.ndarray
  - [property] def idsFilledAuxMembers(self) -> IDSAuxHas
  - [property] def idsVelocities(self) -> numpy.ndarray
  - [property] def magicNumber(self) -> int
  - [magicNumber.setter] def magicNumber(self, arg0: int)
  - [property] def majorVersion(self) -> int
  - [majorVersion.setter] def majorVersion(self, arg0: int)
  - [property] def matId(self) -> numpy.ndarray
  - [property] def materialId(self) -> numpy.ndarray
  - [property] def maxAzRad(self) -> float
  - [property] def maxElRad(self) -> float
  - [property] def maxRangeM(self) -> float
  - [property] def maxVelMps(self) -> float
  - [property] def minAzRad(self) -> float
  - [property] def minElRad(self) -> float
  - [property] def minVelMps(self) -> float
  - [property] def minorVersion(self) -> int
  - [minorVersion.setter] def minorVersion(self, arg0: int)
  - [property] def modality(self) -> Modality
  - [modality.setter] def modality(self, arg0: Modality)
  - [property] def modelToAppTransform(self) -> numpy.ndarray
  - [property] def motionCompensationState(self) -> MotionCompensationState
  - [motionCompensationState.setter] def motionCompensationState(self, arg0: MotionCompensationState)
  - [property] def numCols(self) -> int
  - [property] def numElements(self) -> int
  - [numElements.setter] def numElements(self, arg0: int)
  - [property] def numRows(self) -> int
  - [property] def numSamplesPerSgw(self) -> int
  - [property] def numSgws(self) -> int
  - [property] def objId(self) -> numpy.ndarray
  - [property] def objectId(self) -> numpy.ndarray
  - [property] def originX(self) -> numpy.ndarray
  - [property] def originY(self) -> numpy.ndarray
  - [property] def originZ(self) -> numpy.ndarray
  - [property] def outputType(self) -> OutputType
  - [outputType.setter] def outputType(self, arg0: OutputType)
  - [property] def patchVersion(self) -> int
  - [patchVersion.setter] def patchVersion(self, arg0: int)
  - [property] def rv_ms(self) -> numpy.ndarray
  - [property] def scalar(self) -> numpy.ndarray
  - [property] def scanComplete(self) -> int
  - [property] def scanIdx(self) -> int
  - [property] def sensorID(self) -> int
  - [property] def sizeInBytes(self) -> int
  - [sizeInBytes.setter] def sizeInBytes(self, arg0: int)
  - [property] def tickId(self) -> numpy.ndarray
  - [property] def tickStates(self) -> numpy.ndarray
  - [property] def timeOffSetNs(self) -> numpy.ndarray
  - [property] def timeOffsetNs(self) -> numpy.ndarray
  - [property] def timestampNs(self) -> int
  - [timestampNs.setter] def timestampNs(self, arg0: int)
  - [property] def velocities(self) -> numpy.ndarray
  - [property] def x(self) -> numpy.ndarray
  - [property] def y(self) -> numpy.ndarray
  - [property] def z(self) -> numpy.ndarray

- class GenericModelOutputIOFactory
  - def __init__(self)
  - static def createInstance() -> IGenericModelOutputIO

- class IDSAuxHas
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - NONE: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.IDSAuxHas
  - VELOCITIES: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.IDSAuxHas

- class IGenericModelOutputIO
  - def __init__(self)
  - def addPacket(self, arg0: capsule, arg1: int)
  - def init(self, arg0: GMOIOConfig)
  - def readModelOutput(self, clientName: str = '', frameId: int = -1) -> GenericModelOutput
  - def writeModelOutput(self, arg0: GenericModelOutput)

- class LidarAuxHas
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - CHANNEL_ID: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.LidarAuxHas
  - ECHO_ID: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.LidarAuxHas
  - EMITTER_ID: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.LidarAuxHas
  - HIT_NORMALS: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.LidarAuxHas
  - MAT_ID: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.LidarAuxHas
  - NONE: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.LidarAuxHas
  - OBJ_ID: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.LidarAuxHas
  - TICK_ID: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.LidarAuxHas
  - TICK_STATES: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.LidarAuxHas
  - VELOCITIES: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.LidarAuxHas

- class Modality
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - ACOUSTIC: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.Modality
  - IDS: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.Modality
  - LIDAR: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.Modality
  - RADAR: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.Modality
  - UNDEFINED: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.Modality

- class MotionCompensationState
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - COMPENSATED: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.MotionCompensationState
  - NONCOMPENSATED: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.MotionCompensationState
  - NOT_APPLICABLE: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.MotionCompensationState

- class OutputType
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - POINTCLOUD: isaacsim.sensors.experimental.rtx.generic_model_output._rtx_sensors_gmo.OutputType

## Functions

- def getMagicNumberGMO() -> int
- def getModelOutputFromBuffer(*args, **kwargs) -> typing.Any

# Public API for module isaacsim.sensors.experimental.rtx.sensor_checker:

## Classes

- class AOVInfo
  - def __init__(self)
  - def fillOpaqueBufferMetadata(self, arg0: int)
  - def fillStructureMetadata(self, arg0: str, arg1: int)
  - def fillTensorMetadata(self, arg0: int, arg1: typing.List[str], arg2: typing.List[int], arg3: TensorMetadataDataType)
  - [property] def aovBuffer(self) -> capsule
  - [aovBuffer.setter] def aovBuffer(self, arg1: buffer)
  - [property] def aovMetadata(self) -> capsule
  - [aovMetadata.setter] def aovMetadata(self, arg1: buffer)
  - [property] def aovName(self) -> str
  - [aovName.setter] def aovName(self, arg1: str)

- class ModelInfo
  - def __init__(self)
  - [property] def marketName(self) -> str
  - [marketName.setter] def marketName(self, arg1: str)
  - [property] def modelName(self) -> str
  - [modelName.setter] def modelName(self, arg1: str)
  - [property] def modelVendor(self) -> str
  - [modelVendor.setter] def modelVendor(self, arg1: str)
  - [property] def modelVersion(self) -> str
  - [modelVersion.setter] def modelVersion(self, arg1: str)
  - [property] def schemaVersion(self) -> str
  - [schemaVersion.setter] def schemaVersion(self, arg1: str)

- class Parameters
  - def __init__(self)
  - [property] def dataTypes(self) -> typing.List[SensorCheckerParamType]
  - [property] def numParams(self) -> int
  - [property] def paramNames(self) -> typing.List[str]
  - [property] def paramValues(self) -> typing.List[object]
  - [property] def paramVectorLengths(self) -> typing.List[int]

- class SensorCheckerParamType
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - BOOL: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - DOUBLE: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - DOUBLE2: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - DOUBLE3: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - DOUBLE4: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - DOUBLE4x4: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - FLOAT: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - FLOAT2: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - FLOAT3: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - FLOAT4: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - FLOAT4x4: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - INT: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - INT2: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - INT3: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - INT4: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType
  - STRING: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.SensorCheckerParamType

- class SensorCheckerUtil
  - def __init__(self)
  - def getValidatedParams(self) -> Parameters
  - def init(self, modelInfo: ModelInfo, checkerImplPath: str = None) -> str
  - def validateAOV(self, arg0: AOVInfo) -> str
  - def validateParams(self, arg0: Parameters) -> str
  - def validateParams(self, arg0: str, arg1: str) -> str
  - def validateParams(self, arg0: object) -> str

- class TensorMetadataDataType
  - def __init__(self, value: int)
  - [property] def name(self) -> str
  - [property] def value(self) -> int
  - FLOAT16: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
  - FLOAT32: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
  - FLOAT64: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
  - INT16: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
  - INT32: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
  - INT64: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
  - INT8: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
  - UINT16: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
  - UINT32: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
  - UINT64: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
  - UINT8: isaacsim.sensors.experimental.rtx.sensor_checker._rtx_sensors_checker.TensorMetadataDataType
