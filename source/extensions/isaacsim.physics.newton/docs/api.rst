Python API
==========

.. Summary

The following table summarizes the available classes and functions.

.. currentmodule:: isaacsim.physics.newton

.. autosummary::
    :nosignatures:

    acquire_physics_interface
    acquire_stage
    configure_newton
    get_newton_config
    get_active_physics_engine
    get_available_physics_engines

Configuration Classes
^^^^^^^^^^^^^^^^^^^^^

.. currentmodule:: isaacsim.physics.newton

.. autosummary::
    :nosignatures:

    NewtonConfig
    CollisionConfig
    HydroelasticConfig
    XPBDSolverConfig
    MuJoCoSolverConfig

Tensor Interface
^^^^^^^^^^^^^^^^

The Newton tensor backend is provided by the separate
:ref:`isaacsim.physics.newton.tensors extension <ext_isaacsim_physics_newton_tensors>`.
Its generated Python API lists the simulation and view interfaces available when the extension is loaded.

.. API Details

Functions
^^^^^^^^^

.. autofunction:: isaacsim.physics.newton.acquire_physics_interface

.. autofunction:: isaacsim.physics.newton.acquire_stage

.. autofunction:: isaacsim.physics.newton.configure_newton

.. autofunction:: isaacsim.physics.newton.get_newton_config

.. autofunction:: isaacsim.physics.newton.get_active_physics_engine

.. autofunction:: isaacsim.physics.newton.get_available_physics_engines

Configuration Classes
^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.physics.newton.NewtonConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.physics.newton.CollisionConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.physics.newton.HydroelasticConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.physics.newton.XPBDSolverConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.physics.newton.MuJoCoSolverConfig
    :members:
    :undoc-members:
    :show-inheritance:
