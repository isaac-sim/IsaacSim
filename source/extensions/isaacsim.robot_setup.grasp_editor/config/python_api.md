# Public API for module isaacsim.robot_setup.grasp_editor:

## Classes

- class GraspSpec
  - def __init__(self, imported_data: dict)
  - def get_grasp_names(self) -> list[str]
  - def get_grasp_dict_by_name(self, name: str) -> dict
  - def get_grasp_dicts(self) -> dict
  - def compute_gripper_pose_from_rigid_body_pose(self, grasp_name: str, rb_trans: np.array, rb_quat: np.array) -> tuple[np.array, np.array]
  - def compute_rigid_body_pose_from_gripper_pose(self, grasp_name: str, gripper_trans: np.array, gripper_quat: np.array) -> tuple[np.array, np.array]

## Functions

- def import_grasps_from_file(file_path: str) -> GraspSpec
