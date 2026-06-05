Python API
==========

.. Summary

The following table summarizes the available classes and functions.

.. currentmodule:: isaacsim.physics.newton

.. autosummary::
    :nosignatures:

    acquire_physics_interface
    acquire_stage
    get_active_physics_engine
    get_available_physics_engines

Configuration Classes
^^^^^^^^^^^^^^^^^^^^^

.. currentmodule:: isaacsim.physics.newton

.. autosummary::
    :nosignatures:

    NewtonConfig
    XPBDSolverConfig
    MuJoCoSolverConfig

Tensor Interface
^^^^^^^^^^^^^^^^

The tensor interface provides NumPy, PyTorch, and Warp frontends for physics data access.

.. currentmodule:: isaacsim.physics.newton.tensors

.. autosummary::
    :nosignatures:

    create_simulation_view
    NewtonArticulationView
    NewtonRigidBodyView
    NewtonRigidContactView

.. API Details

Functions
^^^^^^^^^

.. autofunction:: isaacsim.physics.newton.acquire_physics_interface

.. autofunction:: isaacsim.physics.newton.acquire_stage

.. autofunction:: isaacsim.physics.newton.get_active_physics_engine

.. autofunction:: isaacsim.physics.newton.get_available_physics_engines

Configuration Classes
^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.physics.newton.NewtonConfig
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

Tensor Views
^^^^^^^^^^^^

.. autofunction:: isaacsim.physics.newton.tensors.create_simulation_view

.. autoclass:: isaacsim.physics.newton.tensors.NewtonArticulationView
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.physics.newton.tensors.NewtonRigidBodyView
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.physics.newton.tensors.NewtonRigidContactView
    :members:
    :undoc-members:
    :show-inheritance:
