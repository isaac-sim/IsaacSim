Python API
==========

.. Summary

.. currentmodule:: isaacsim.robot.experimental.wheeled_robots

.. note::
   Entries and autoclass/autofunction directives in all sections below are ordered by name.

.. rubric:: controllers
.. autosummary::
    :nosignatures:

    ~controllers.AckermannController
    ~controllers.DifferentialController
    ~controllers.HolonomicController

.. rubric:: robots
.. autosummary::
    :nosignatures:

    ~robots.HolonomicRobotUsdSetup
    ~robots.WheeledRobot

.. rubric:: *utilities* (controllers)
.. autosummary::
    :nosignatures:

    ~controllers.pid_control
    ~controllers.QuinticPolynomial
    ~controllers.quintic_polynomials_planner
    ~controllers.stanley_control

|

.. API

Controllers
^^^^^^^^^^^

.. autoclass:: isaacsim.robot.experimental.wheeled_robots.controllers.AckermannController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.robot.experimental.wheeled_robots.controllers.DifferentialController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.robot.experimental.wheeled_robots.controllers.HolonomicController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Robots
^^^^^^

.. autoclass:: isaacsim.robot.experimental.wheeled_robots.robots.HolonomicRobotUsdSetup
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.robot.experimental.wheeled_robots.robots.WheeledRobot
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Utilities (controllers)
^^^^^^^^^^^^^^^^^^^^^^^

.. autofunction:: isaacsim.robot.experimental.wheeled_robots.controllers.pid_control

.. autoclass:: isaacsim.robot.experimental.wheeled_robots.controllers.QuinticPolynomial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autofunction:: isaacsim.robot.experimental.wheeled_robots.controllers.quintic_polynomials_planner

.. autofunction:: isaacsim.robot.experimental.wheeled_robots.controllers.stanley_control
