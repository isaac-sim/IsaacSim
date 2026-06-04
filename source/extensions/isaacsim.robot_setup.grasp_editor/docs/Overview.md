# Overview

The isaacsim.robot_setup.grasp_editor extension provides functionality for importing and managing robot grasps from YAML files. It enables robotic manipulation tasks by offering tools to parse grasp specifications and perform coordinate transformations between gripper and object poses.

## Functionality

This extension processes isaac_grasp YAML files to create structured grasp data that can be used in Isaac Sim robotic simulations. It provides pose transformation capabilities that are essential for robotic grasping tasks where precise positioning and orientation are required.

### Grasp Import and Management

The extension loads grasp specifications from YAML files using the {func}`import_grasps_from_file <isaacsim.robot_setup.grasp_editor.import_grasps_from_file>` function, which returns a {class}`GraspSpec <isaacsim.robot_setup.grasp_editor.GraspSpec>` object containing all parsed grasp data. Each grasp specification includes position, orientation, confidence values, and joint configurations that define how a gripper should approach and grasp an object.

### Coordinate Frame Transformations

The {class}`GraspSpec <isaacsim.robot_setup.grasp_editor.GraspSpec>` class provides bidirectional pose transformation methods:

- `compute_gripper_pose_from_rigid_body_pose`: Calculates the required gripper position and orientation given an object's pose in the world
- `compute_rigid_body_pose_from_gripper_pose`: Determines the object's position and orientation given a gripper's pose

These transformations enable users to determine optimal gripper positioning for successful grasps or to predict object poses based on gripper configurations.

### Grasp Data Access

The extension provides methods to access specific grasp information:

- `get_grasp_names`: Returns all available grasp identifiers from the imported file
- `get_grasp_dict_by_name`: Retrieves detailed grasp data including confidence levels, poses, and joint configurations
- `get_grasp_dicts`: Provides access to all grasp specifications in a structured format

Each grasp contains confidence values, position and orientation data, joint configurations for grasping, and pregrasp positions for gripper preparation.
