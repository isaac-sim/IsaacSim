Python API
==========

.. Summary

.. automodule:: lula
    :no-index:

.. currentmodule:: lula

.. rubric:: *Logging*
.. autosummary::
    :nosignatures:

    LogLevel

.. rubric:: *Rotations and Poses*
.. autosummary::
    :nosignatures:

    Rotation3
    Pose3

.. rubric:: *Robot Specification*
.. autosummary::
    :nosignatures:

    RobotDescription
    load_robot
    load_robot_from_memory

.. rubric:: *World Specification*
.. autosummary::
    :nosignatures:

    Obstacle
    create_obstacle
    World
    create_world
    WorldView

.. rubric:: *Kinematics*
.. autosummary::
    :nosignatures:

    CyclicCoordDescentIkConfig
    CyclicCoordDescentIkResults
    compute_ik_ccd

.. rubric:: *Path Specification*
.. autosummary::
    :nosignatures:

    CSpacePathSpec
    create_c_space_path_spec
    TaskSpacePathSpec
    create_task_space_path_spec
    CompositePathSpec
    create_composite_path_spec
    load_c_space_path_spec_from_file
    load_c_space_path_spec_from_memory
    export_c_space_path_spec_to_memory
    load_task_space_path_spec_from_file
    load_task_space_path_spec_from_memory
    export_task_space_path_spec_to_memory
    load_composite_path_spec_from_file
    load_composite_path_spec_from_memory
    export_composite_path_spec_to_memory

.. rubric:: *Path Generation*
.. autosummary::
    :nosignatures:

    CSpacePath
    LinearCSpacePath
    create_linear_c_space_path
    TaskSpacePath
    TaskSpacePathConversionConfig
    convert_composite_path_spec_to_c_space
    convert_task_space_path_spec_to_c_space

.. rubric:: *Trajectory Generation*
.. autosummary::
    :nosignatures:

    Trajectory
    CSpaceTrajectoryGenerator
    create_c_space_trajectory_generator

.. rubric:: *Collision Sphere Generation*
.. autosummary::
    :nosignatures:

    CollisionSphereGenerator
    create_collision_sphere_generator

.. rubric:: *Motion Planning*
.. autosummary::
    :nosignatures:

    MotionPlanner
    create_motion_planner

.. rubric:: *RmpFlow*
.. autosummary::
    :nosignatures:

    RmpFlowConfig
    create_rmpflow_config
    create_rmpflow_config_from_memory
    RmpFlow
    create_rmpflow

|

.. API

Logging
^^^^^^^

..
  autodoc does not provide a mechanism for controlling the ordering of class members derived
  from bound C++ (as opposed to python source).  Default ordering is alphabetical, so we have
  to document the log levels manually to ensure that they appear in the correct order.

.. autoclass:: lula.LogLevel

    .. py:data:: FATAL

        Logging level for nonrecoverable errors (minimum level, so always enabled).

    .. py:data:: ERROR

        Logging level for recoverable errors.

    .. py:data:: WARNING

        Logging level for warnings, indicating possible cause for concern.

    .. py:data:: INFO

        Logging level for informational messages.

    .. py:data:: VERBOSE

        Logging level for highly verbose informational messages.

.. autofunction:: lula.set_log_level

|

Rotations and Poses
^^^^^^^^^^^^^^^^^^^

.. autoclass:: lula.Rotation3
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: lula.Pose3
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Robot Specification
^^^^^^^^^^^^^^^^^^^

.. autoclass:: lula.RobotDescription
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.load_robot

.. autofunction:: lula.load_robot_from_memory

|

World Specification
^^^^^^^^^^^^^^^^^^^

.. autoclass:: lula.Obstacle
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_obstacle

.. autoclass:: lula.World
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_world

.. autoclass:: lula.WorldView
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Kinematics
^^^^^^^^^^

.. autoclass:: lula.Kinematics
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Inverse Kinematics
^^^^^^^^^^^^^^^^^^

.. autoclass:: lula.CyclicCoordDescentIkConfig
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: lula.CyclicCoordDescentIkResults
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.compute_ik_ccd

|

Path Specification
^^^^^^^^^^^^^^^^^^

.. autoclass:: lula.CSpacePathSpec
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_c_space_path_spec

.. autoclass:: lula.TaskSpacePathSpec
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_task_space_path_spec

.. autoclass:: lula.CompositePathSpec
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_composite_path_spec

.. autofunction:: lula.load_c_space_path_spec_from_file

.. autofunction:: lula.load_c_space_path_spec_from_memory

.. autofunction:: lula.export_c_space_path_spec_to_memory

.. autofunction:: lula.load_task_space_path_spec_from_file

.. autofunction:: lula.load_task_space_path_spec_from_memory

.. autofunction:: lula.export_task_space_path_spec_to_memory

.. autofunction:: lula.load_composite_path_spec_from_file

.. autofunction:: lula.load_composite_path_spec_from_memory

.. autofunction:: lula.export_composite_path_spec_to_memory

|

Path Generation
^^^^^^^^^^^^^^^

.. autoclass:: lula.CSpacePath
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: lula.LinearCSpacePath
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_linear_c_space_path

.. autoclass:: lula.TaskSpacePath
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: lula.TaskSpacePathConversionConfig
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.convert_composite_path_spec_to_c_space

.. autofunction:: lula.convert_task_space_path_spec_to_c_space

|

Trajectory Generation
^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: lula.Trajectory
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: lula.CSpaceTrajectoryGenerator
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_c_space_trajectory_generator

|

Collision Sphere Generation
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: lula.CollisionSphereGenerator
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_collision_sphere_generator

|

Motion Planning
^^^^^^^^^^^^^^^

.. autoclass:: lula.MotionPlanner
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_motion_planner

|

RmpFlow
^^^^^^^

.. autoclass:: lula.RmpFlowConfig
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_rmpflow_config

.. autofunction:: lula.create_rmpflow_config_from_memory

.. autoclass:: lula.RmpFlow
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: lula.create_rmpflow
