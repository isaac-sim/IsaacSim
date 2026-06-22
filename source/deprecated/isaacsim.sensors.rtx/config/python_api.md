# Public API for module isaacsim.sensors.rtx:

## Classes

- class IsaacSensorCreateRtxSensor(omni.kit.commands.Command)
  - def __init__(self, path: str | None = None, parent: str | None = None, config: str | None = None, usd_path: str | None = None, translation: Gf.Vec3d | None = Gf.Vec3d(0, 0, 0), orientation: Gf.Quatd | None = Gf.Quatd(1, 0, 0, 0), visibility: bool = False, variant: str | dict[str, str] | None = None, force_camera_prim: bool = False, **kwargs: Any)
  - def do(self) -> Usd.Prim
  - def undo(self)

- class IsaacSensorCreateRtxLidar(IsaacSensorCreateRtxSensor)
  - def __init__(self, **kwargs: Any)
  - def do(self) -> Usd.Prim

- class IsaacSensorCreateRtxRadar(IsaacSensorCreateRtxSensor)
  - def do(self) -> Usd.Prim | None

- class IsaacSensorCreateRtxIDS(IsaacSensorCreateRtxSensor)
  - def __init__(self, **kwargs: Any)

- class IsaacSensorCreateRtxUltrasonic(IsaacSensorCreateRtxSensor)

- class Extension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

- class LidarRtx(BaseSensor)
  - static def make_add_remove_deprecated_attr(deprecated_attr: str) -> list[Callable]
  - def __init__(self, prim_path: str, name: str = 'lidar_rtx', position: np.ndarray | None = None, translation: np.ndarray | None = None, orientation: np.ndarray | None = None, config_file_name: str | None = None, **kwargs: Any)
  - def get_render_product_path(self) -> str | None
  - def get_current_frame(self) -> dict
  - def get_annotators(self) -> dict
  - def attach_annotator(self, annotator_name: Literal[IsaacComputeRTXLidarFlatScan, IsaacExtractRTXSensorPointCloudNoAccumulator, IsaacCreateRTXLidarScanBuffer, StableIdMap, GenericModelOutput], **kwargs: object)
  - def detach_annotator(self, annotator_name: str)
  - def detach_all_annotators(self)
  - def get_writers(self) -> dict
  - def attach_writer(self, writer_name: str, **kwargs: object)
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

## Functions

- def delete_prim(prim_path: str)
- def add_reference_to_stage(usd_path: str, prim_path: str, prim_type: str = 'Xform') -> Usd.Prim
- def get_next_free_path(path: str, parent: str = None) -> str
- def reset_and_set_xform_ops(prim: Usd.Prim, translation: Gf.Vec3d, orientation: Gf.Quatd, scale: Gf.Vec3d = Gf.Vec3d([1.0, 1.0, 1.0]))
- def get_assets_root_path() -> str
- def register_annotator_from_node_with_telemetry(*args: Any, **kwargs: Any)
- def register_node_writer_with_telemetry(*args: Any, **kwargs: Any)
- def get_prim_at_path(prim_path: str, fabric: bool = False) -> Usd.Prim | usdrt.Usd._Usd.Prim | None
- def get_gmo_data(dataPtr: int | np.ndarray) -> gmo_utils.GenericModelOutput
- def apply_nonvisual_material(prim: Usd.Prim, base: str | int, coating: str | int = 'none', attribute: str | int = 'none') -> bool
- def get_material_id(prim: Usd.Prim) -> int
- def decode_material_id(material_id: int) -> tuple[str, str, str]

## Variables

- SUPPORTED_LIDAR_CONFIGS: dict[str, set[str] | list[dict[str, str]]]
- SUPPORTED_LIDAR_VARIANT_SET_NAME: str
- EXTENSION_NAME: str
- NONE_BASE: Dict
- METALS_BASE: Dict
- POLYMERS_BASE: Dict
- GLASS_BASE: Dict
- OTHER_BASE: Dict
- BASE_MATERIALS: Dict
- COATINGS: Dict
- ATTRIBUTES: Dict
- ATTR_PREFIX: Unknown
- ATTR_BASE: Unknown
- ATTR_COATING: Unknown
- ATTR_ATTRIBUTE: Unknown

# Public API for module isaacsim.sensors.rtx.generic_model_output:

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

# Public API for module isaacsim.sensors.rtx.generic_model_output:

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

# Public API for module isaacsim.sensors.rtx.sensor_checker:

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
