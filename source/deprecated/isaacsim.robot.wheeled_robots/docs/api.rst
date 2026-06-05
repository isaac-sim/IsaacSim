Python API
==========

.. Summary

.. currentmodule:: isaacsim.robot.wheeled_robots

.. rubric:: controllers
.. autosummary::
    :nosignatures:

    ~controllers.DifferentialController
    ~controllers.HolonomicController
    ~controllers.WheelBasePoseController

.. rubric:: robots
.. autosummary::
    :nosignatures:

    ~robots.WheeledRobot
    ~robots.HolonomicRobotUsdSetup

.. rubric:: *utilities* (controllers)
.. autosummary::
    :nosignatures:

    ~controllers.QuinticPolynomial
    ~controllers.quintic_polynomials_planner
    ~controllers.stanley_control
    ~controllers.pid_control

|

.. API

Controllers
^^^^^^^^^^^

.. autoclass:: isaacsim.robot.wheeled_robots.controllers.DifferentialController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.robot.wheeled_robots.controllers.HolonomicController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.robot.wheeled_robots.controllers.WheelBasePoseController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Robots
^^^^^^

.. autoclass:: isaacsim.robot.wheeled_robots.robots.WheeledRobot
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.robot.wheeled_robots.robots.HolonomicRobotUsdSetup
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Utilities (controllers)
^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.robot.wheeled_robots.controllers.QuinticPolynomial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: isaacsim.robot.wheeled_robots.controllers.quintic_polynomials_planner

.. autofunction:: isaacsim.robot.wheeled_robots.controllers.stanley_control

.. autofunction:: isaacsim.robot.wheeled_robots.controllers.pid_control
