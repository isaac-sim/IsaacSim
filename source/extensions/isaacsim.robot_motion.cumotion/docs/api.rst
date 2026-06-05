Python API
==========

.. Summary

.. currentmodule:: isaacsim.robot_motion.cumotion

.. rubric:: *Configuration Loading*
.. autosummary::
    :nosignatures:

    load_cumotion_robot
    load_cumotion_supported_robot
    CumotionRobot

.. rubric:: *World Interface*
.. autosummary::
    :nosignatures:

    CumotionWorldInterface

.. rubric:: *Motion Policies*
.. autosummary::
    :nosignatures:

    RmpFlowController

.. rubric:: *Motion Planning*
.. autosummary::
    :nosignatures:

    GraphBasedMotionPlanner

.. rubric:: *Trajectory Generation*
.. autosummary::
    :nosignatures:

    TrajectoryGenerator
    TrajectoryOptimizer
    CumotionTrajectory

.. rubric:: *Transform Utilities*
.. autosummary::
    :nosignatures:

    impl.utils.isaac_sim_to_cumotion_pose
    impl.utils.isaac_sim_to_cumotion_translation
    impl.utils.isaac_sim_to_cumotion_rotation
    impl.utils.cumotion_to_isaac_sim_pose
    impl.utils.cumotion_to_isaac_sim_translation
    impl.utils.cumotion_to_isaac_sim_rotation

|

.. API

Configuration Loading
^^^^^^^^^^^^^^^^^^^^^

.. autofunction:: isaacsim.robot_motion.cumotion.load_cumotion_robot

.. autofunction:: isaacsim.robot_motion.cumotion.load_cumotion_supported_robot

.. autoclass:: isaacsim.robot_motion.cumotion.CumotionRobot
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

World Interface
^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.robot_motion.cumotion.CumotionWorldInterface
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Motion Policies
^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.robot_motion.cumotion.RmpFlowController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Motion Planning
^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.robot_motion.cumotion.GraphBasedMotionPlanner
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Trajectory Generation
^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.robot_motion.cumotion.TrajectoryGenerator
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.robot_motion.cumotion.TrajectoryOptimizer
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.robot_motion.cumotion.CumotionTrajectory
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Transform Utilities
^^^^^^^^^^^^^^^^^^^

The transform utilities provide coordinate conversion between Isaac Sim world frame and cuMotion robot base frame.

.. autofunction:: isaacsim.robot_motion.cumotion.impl.utils.isaac_sim_to_cumotion_pose

.. autofunction:: isaacsim.robot_motion.cumotion.impl.utils.isaac_sim_to_cumotion_translation

.. autofunction:: isaacsim.robot_motion.cumotion.impl.utils.isaac_sim_to_cumotion_rotation

.. autofunction:: isaacsim.robot_motion.cumotion.impl.utils.cumotion_to_isaac_sim_pose

.. autofunction:: isaacsim.robot_motion.cumotion.impl.utils.cumotion_to_isaac_sim_translation

.. autofunction:: isaacsim.robot_motion.cumotion.impl.utils.cumotion_to_isaac_sim_rotation
