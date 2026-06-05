Python API
==========

.. warning::

    **The API featured in this extension is experimental and subject to change without deprecation cycles.**
    Although we will try to maintain backward compatibility in the event of a change, it may not always be possible.

.. Summary

The following table summarizes the public API of the ``isaacsim.core.experimental.actuators`` extension.

.. currentmodule:: isaacsim.core.experimental.actuators

.. autosummary::
    :nosignatures:

    ArticulationActuators
    ActuatorConfig
    PDControlConfig
    PIDControlConfig
    NeuralControlConfig
    MaxEffortClampingConfig
    DCMotorClampingConfig
    PositionBasedClampingConfig
    DelayConfig
    add_actuator

.. API

Wrappers
^^^^^^^^

.. autoclass:: isaacsim.core.experimental.actuators.ArticulationActuators
    :members:
    :undoc-members:
    :show-inheritance:
    :special-members: __enter__, __exit__

Configuration dataclasses
^^^^^^^^^^^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.core.experimental.actuators.ActuatorConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.actuators.PDControlConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.actuators.PIDControlConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.actuators.NeuralControlConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.actuators.MaxEffortClampingConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.actuators.DCMotorClampingConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.actuators.PositionBasedClampingConfig
    :members:
    :undoc-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.actuators.DelayConfig
    :members:
    :undoc-members:
    :show-inheritance:

USD authoring
^^^^^^^^^^^^^

.. autofunction:: isaacsim.core.experimental.actuators.add_actuator
