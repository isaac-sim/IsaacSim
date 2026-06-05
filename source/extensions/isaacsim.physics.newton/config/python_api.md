# Public API for module isaacsim.physics.newton:

## Classes

- class NewtonConfig
  - num_substeps: int
  - debug_mode: bool
  - use_cuda_graph: bool
  - time_step_app: bool
  - physics_frequency: float
  - update_fabric: bool
  - disable_physx_fabric_tracker: bool
  - collapse_fixed_joints: bool
  - fix_missing_xform_ops: bool
  - contact_ke: float
  - contact_kd: float
  - contact_kf: float
  - contact_mu: float
  - contact_ka: float
  - restitution: float
  - contact_margin: float
  - soft_contact_margin: float
  - joint_limit_ke: float
  - joint_limit_kd: float
  - armature: float
  - joint_damping: float
  - pd_scale: float
  - solver_cfg: NewtonSolverConfig

- class NewtonStage
  - def __init__(self, cfg: NewtonConfig | None = None, device: str | None = None)
  - def init(self)
  - def on_timeline_event(self, e: omni.timeline.TimelineEventType)
  - def on_update(self, event: carb.events.IEvent, dt: float)
  - def step_sim(self, dt: float)
  - def update_fabric(self)
  - def on_detach(self)
  - def on_attach(self, stage_id: int, meters_per_unit: float)
  - def on_resume(self, currentTime: float)
  - def on_change(self, path: str)
  - def update_fabric_attrs(self)
  - def initialize_newton(self, device: str | None)
  - def simulate(self, num_substeps: int | None = None, dt: float | None = None)

- class XPBDSolverConfig(NewtonSolverConfig)
  - solver_type: Literal[xpbd]
  - iterations: int
  - soft_body_relaxation: float
  - soft_contact_relaxation: float
  - joint_linear_relaxation: float
  - joint_angular_relaxation: float
  - joint_linear_compliance: float
  - joint_angular_compliance: float
  - rigid_contact_relaxation: float
  - rigid_contact_con_weighting: bool
  - angular_damping: float
  - enable_restitution: bool

- class MuJoCoSolverConfig(NewtonSolverConfig)
  - solver_type: Literal[mujoco]
  - njmax: int
  - nconmax: int | None
  - iterations: int
  - ls_iterations: int
  - solver: str
  - integrator: str
  - cone: str
  - impratio: float
  - use_mujoco_cpu: bool
  - disable_contacts: bool
  - update_data_interval: int
  - save_to_mjcf: str | None
  - ls_parallel: bool
  - use_mujoco_contacts: bool
  - tolerance: float
  - ls_tolerance: float
  - include_sites: bool

## Functions

- def acquire_physics_interface() -> NewtonPhysicsInterface | None
- def acquire_stage() -> NewtonStage | None
- def get_active_physics_engine() -> str
- def get_available_physics_engines(verbose: bool = False) -> list[tuple[str, bool]]
