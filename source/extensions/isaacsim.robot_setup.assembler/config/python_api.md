# Public API for module isaacsim.robot_setup.assembler:

## Classes

- class RobotAssembler
  - def __init__(self)
  - def reset(self)
  - def is_root_joint(self, prim: object) -> bool
  - def get_articulation_root_api_path(self, prim_path: str) -> str
  - def mask_collisions(self, prim_path_a: str, prim_path_b: str) -> Usd.Relationship
  - def begin_assembly(self, stage: object, base_prim_path: str, base_mount_path: str, attachment_prim_path: str, attachment_mount_path: str, variant_set: str, variant_name: str)
  - def cancel_assembly(self)
  - def assemble(self)
  - def finish_assemble(self)

- class AssembledRobot
  - def __init__(self, assembled_robots: AssembledBodies)
  - [property] def base_path(self) -> str
  - [property] def attach_path(self) -> str
  - [property] def fixed_joint(self) -> UsdPhysics.FixedJoint
  - [property] def root_joints(self) -> list[UsdPhysics.Joint]
  - [property] def collision_mask(self) -> Usd.Relationship

- class AssembledBodies
  - def __init__(self, base_path: str, attach_path: str, fixed_joint: UsdPhysics.FixedJoint, root_joints: list[UsdPhysics.Joint], attach_body_articulation_root: Usd.Prim, collision_mask: object = None)
  - [property] def base_path(self) -> str
  - [property] def attach_path(self) -> str
  - [property] def fixed_joint(self) -> UsdPhysics.FixedJoint
  - [property] def attach_body_articulation_root(self) -> Usd.Prim
  - [property] def root_joints(self) -> list[UsdPhysics.Joint]
  - [property] def collision_mask(self) -> Usd.Relationship
