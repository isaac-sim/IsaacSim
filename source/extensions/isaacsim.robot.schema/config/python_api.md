# Public API for module omni.isaac.RangeSensorSchema:

## Classes

- class RangeSensor
  - def __init__(self, prim: Usd.Prim)
  - class def Define(cls, stage: Usd.Stage, path: str) -> RangeSensor
  - def GetPrim(self) -> Usd.Prim
  - def GetPath(self) -> Sdf.Path
  - def GetEnabledAttr(self) -> Usd.Attribute
  - def CreateEnabledAttr(self, value: Any = None) -> Usd.Attribute
  - def GetDrawPointsAttr(self) -> Usd.Attribute
  - def CreateDrawPointsAttr(self, value: Any = None) -> Usd.Attribute
  - def GetDrawLinesAttr(self) -> Usd.Attribute
  - def CreateDrawLinesAttr(self, value: Any = None) -> Usd.Attribute
  - def GetMinRangeAttr(self) -> Usd.Attribute
  - def CreateMinRangeAttr(self, value: Any = None) -> Usd.Attribute
  - def GetMaxRangeAttr(self) -> Usd.Attribute
  - def CreateMaxRangeAttr(self, value: Any = None) -> Usd.Attribute

- class Lidar(RangeSensor)
  - def GetYawOffsetAttr(self) -> Usd.Attribute
  - def CreateYawOffsetAttr(self, value: Any = None) -> Usd.Attribute
  - def GetRotationRateAttr(self) -> Usd.Attribute
  - def CreateRotationRateAttr(self, value: Any = None) -> Usd.Attribute
  - def GetHighLodAttr(self) -> Usd.Attribute
  - def CreateHighLodAttr(self, value: Any = None) -> Usd.Attribute
  - def GetHorizontalFovAttr(self) -> Usd.Attribute
  - def CreateHorizontalFovAttr(self, value: Any = None) -> Usd.Attribute
  - def GetVerticalFovAttr(self) -> Usd.Attribute
  - def CreateVerticalFovAttr(self, value: Any = None) -> Usd.Attribute
  - def GetHorizontalResolutionAttr(self) -> Usd.Attribute
  - def CreateHorizontalResolutionAttr(self, value: Any = None) -> Usd.Attribute
  - def GetVerticalResolutionAttr(self) -> Usd.Attribute
  - def CreateVerticalResolutionAttr(self, value: Any = None) -> Usd.Attribute
  - def GetEnableSemanticsAttr(self) -> Usd.Attribute
  - def CreateEnableSemanticsAttr(self, value: Any = None) -> Usd.Attribute

- class Generic(RangeSensor)
  - def GetSamplingRateAttr(self) -> Usd.Attribute
  - def CreateSamplingRateAttr(self, value: Any = None) -> Usd.Attribute
  - def GetStreamingAttr(self) -> Usd.Attribute
  - def CreateStreamingAttr(self, value: Any = None) -> Usd.Attribute

# Public API for module omni.isaac.IsaacSensorSchema:

## Classes

- class IsaacBaseSensor
  - def __init__(self, prim: Usd.Prim)
  - class def Define(cls, stage: Usd.Stage, path: str) -> IsaacBaseSensor
  - def GetPrim(self) -> Usd.Prim
  - def GetPath(self) -> Sdf.Path
  - def GetEnabledAttr(self) -> Usd.Attribute
  - def CreateEnabledAttr(self, value: Any = None) -> Usd.Attribute
  - def GetSensorPeriodAttr(self) -> Usd.Attribute
  - def CreateSensorPeriodAttr(self, value: Any = None) -> Usd.Attribute

- class IsaacContactSensor(IsaacBaseSensor)
  - def GetThresholdAttr(self) -> Usd.Attribute
  - def CreateThresholdAttr(self, value: Any = None) -> Usd.Attribute
  - def GetRadiusAttr(self) -> Usd.Attribute
  - def CreateRadiusAttr(self, value: Any = None) -> Usd.Attribute
  - def GetColorAttr(self) -> Usd.Attribute
  - def CreateColorAttr(self, value: Any = None) -> Usd.Attribute

- class IsaacImuSensor(IsaacBaseSensor)
  - def GetLinearAccelerationFilterWidthAttr(self) -> Usd.Attribute
  - def CreateLinearAccelerationFilterWidthAttr(self, value: Any = None) -> Usd.Attribute
  - def GetAngularVelocityFilterWidthAttr(self) -> Usd.Attribute
  - def CreateAngularVelocityFilterWidthAttr(self, value: Any = None) -> Usd.Attribute
  - def GetOrientationFilterWidthAttr(self) -> Usd.Attribute
  - def CreateOrientationFilterWidthAttr(self, value: Any = None) -> Usd.Attribute

- class IsaacLightBeamSensor(IsaacBaseSensor)
  - def __init__(self, prim: Usd.Prim)
  - def GetNumRaysAttr(self) -> Usd.Attribute
  - def CreateNumRaysAttr(self, value: Any = None) -> Usd.Attribute
  - def GetCurtainLengthAttr(self) -> Usd.Attribute
  - def CreateCurtainLengthAttr(self, value: Any = None) -> Usd.Attribute
  - def GetForwardAxisAttr(self) -> Usd.Attribute
  - def CreateForwardAxisAttr(self, value: Any = None) -> Usd.Attribute
  - def GetCurtainAxisAttr(self) -> Usd.Attribute
  - def CreateCurtainAxisAttr(self, value: Any = None) -> Usd.Attribute
  - def GetMinRangeAttr(self) -> Usd.Attribute
  - def CreateMinRangeAttr(self, value: Any = None) -> Usd.Attribute
  - def GetMaxRangeAttr(self) -> Usd.Attribute
  - def CreateMaxRangeAttr(self, value: Any = None) -> Usd.Attribute

- class IsaacRaycastSensor(IsaacBaseSensor)
  - def GetNumRaysAttr(self) -> Usd.Attribute
  - def CreateNumRaysAttr(self, value: Any = None) -> Usd.Attribute
  - def GetMinRangeAttr(self) -> Usd.Attribute
  - def CreateMinRangeAttr(self, value: Any = None) -> Usd.Attribute
  - def GetMaxRangeAttr(self) -> Usd.Attribute
  - def CreateMaxRangeAttr(self, value: Any = None) -> Usd.Attribute
  - def GetRayOriginsAttr(self) -> Usd.Attribute
  - def CreateRayOriginsAttr(self, value: Any = None) -> Usd.Attribute
  - def GetRayDirectionsAttr(self) -> Usd.Attribute
  - def CreateRayDirectionsAttr(self, value: Any = None) -> Usd.Attribute
  - def GetRayTimeOffsetsAttr(self) -> Usd.Attribute
  - def CreateRayTimeOffsetsAttr(self, value: Any = None) -> Usd.Attribute
  - def GetOutputFrameOfReferenceAttr(self) -> Usd.Attribute
  - def CreateOutputFrameOfReferenceAttr(self, value: Any = None) -> Usd.Attribute
  - def GetReportHitPrimPathsAttr(self) -> Usd.Attribute
  - def CreateReportHitPrimPathsAttr(self, value: Any = None) -> Usd.Attribute

- class IsaacRtxLidarSensorAPI(_APISchemaWrapper)

- class IsaacRtxRadarSensorAPI(_APISchemaWrapper)

# Public API for module usd.schema.isaac:

## Variables

- logger: Unknown
- ext_path: Unknown

# Public API for module usd.schema.isaac.robot_schema:

## Classes

- class Classes(Enum)
  - ROBOT_API: str
  - LINK_API: str
  - REFERENCE_POINT_API: str
  - SITE_API: str
  - JOINT_API: str
  - SURFACE_GRIPPER: str
  - ATTACHMENT_POINT_API: str
  - NAMED_POSE: str

- class DofOffsetOpOrder(Enum)
  - TRANS_X: str
  - TRANS_Y: str
  - TRANS_Z: str
  - ROT_X: str
  - ROT_Y: str
  - ROT_Z: str

- class Attributes(Enum)
  - DESCRIPTION: Tuple
  - NAMESPACE: Tuple
  - ROBOT_TYPE: Tuple
  - LICENSE: Tuple
  - VERSION: Tuple
  - SOURCE: Tuple
  - CHANGELOG: Tuple
  - NAME_OVERRIDE: Tuple
  - REFERENCE_DESCRIPTION: Tuple
  - FORWARD_AXIS: Tuple
  - JOINT_NAME_OVERRIDE: Tuple
  - DOF_OFFSET_OP_ORDER: Tuple
  - ACTUATOR: Tuple
  - RETRY_INTERVAL: Tuple
  - STATUS: Tuple
  - SHEAR_FORCE_LIMIT: Tuple
  - COAXIAL_FORCE_LIMIT: Tuple
  - MAX_GRIP_DISTANCE: Tuple
  - CLEARANCE_OFFSET: Tuple
  - POSE_VALID: Tuple
  - POSE_JOINT_VALUES: Tuple
  - POSE_JOINT_FIXED: Tuple
  - [property] def name(self) -> str
  - [property] def display_name(self) -> str
  - [property] def type(self) -> pxr.Sdf.ValueTypeName

- class Relations(Enum)
  - ROBOT_LINKS: Tuple
  - ROBOT_JOINTS: Tuple
  - NAMED_POSES: Tuple
  - ATTACHMENT_POINTS: Tuple
  - GRIPPED_OBJECTS: Tuple
  - POSE_START_LINK: Tuple
  - POSE_END_LINK: Tuple
  - POSE_JOINTS: Tuple
  - [property] def name(self) -> str
  - [property] def display_name(self) -> str

## Functions

- def get_allowed_tokens(attribute: Attributes) -> tuple[str, Ellipsis]
- def ApplyRobotAPI(prim: pxr.Usd.Prim)
- def ApplyLinkAPI(prim: pxr.Usd.Prim)
- def ApplySiteAPI(prim: pxr.Usd.Prim)
- def ApplyReferencePointAPI(prim: pxr.Usd.Prim)
- def ApplyJointAPI(prim: pxr.Usd.Prim)
- def CreateSurfaceGripper(stage: pxr.Usd.Stage, prim_path: str) -> pxr.Usd.Prim
- def ApplyAttachmentPointAPI(prim: pxr.Usd.Prim)
- def CreateNamedPose(stage: pxr.Usd.Stage, prim_path: str) -> pxr.Usd.Prim

## Variables

- logger: Unknown
