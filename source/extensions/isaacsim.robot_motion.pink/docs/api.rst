Python API
==========

.. Summary

.. currentmodule:: isaacsim.robot_motion.pink

.. rubric:: *Configuration Loading*
.. autosummary::
    :nosignatures:

    load_pink_robot
    load_pink_supported_robot
    PinkRobot

.. rubric:: *Inverse Kinematics Controller*
.. autosummary::
    :nosignatures:

    PinkIKController

.. rubric:: *Transform Utilities*
.. autosummary::
    :nosignatures:

    impl.utils.isaac_sim_position_quaternion_to_se3
    impl.utils.se3_to_isaac_sim_position_quaternion
    impl.utils.map_joint_positions_to_pinocchio
    impl.utils.map_pinocchio_velocity_to_joint_state

|

.. API

Configuration Loading
^^^^^^^^^^^^^^^^^^^^^

.. autofunction:: isaacsim.robot_motion.pink.load_pink_robot

.. autofunction:: isaacsim.robot_motion.pink.load_pink_supported_robot

.. autoclass:: isaacsim.robot_motion.pink.PinkRobot
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Inverse Kinematics Controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.robot_motion.pink.PinkIKController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Transform Utilities
^^^^^^^^^^^^^^^^^^^

The transform utilities convert between Isaac Sim's (position, quaternion) representation and Pinocchio's SE3 transforms.

.. autofunction:: isaacsim.robot_motion.pink.impl.utils.isaac_sim_position_quaternion_to_se3

.. autofunction:: isaacsim.robot_motion.pink.impl.utils.se3_to_isaac_sim_position_quaternion

.. autofunction:: isaacsim.robot_motion.pink.impl.utils.map_joint_positions_to_pinocchio

.. autofunction:: isaacsim.robot_motion.pink.impl.utils.map_pinocchio_velocity_to_joint_state
