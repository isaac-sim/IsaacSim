Python API
==========

.. Summary

.. currentmodule:: isaacsim.core.api

.. rubric:: controllers
.. autosummary::
    :nosignatures:

    ~controllers.ArticulationController
    ~controllers.BaseController
    ~controllers.BaseGripperController

.. rubric:: loggers
.. autosummary::
    :nosignatures:

    ~loggers.DataLogger

.. rubric:: materials
.. autosummary::
    :nosignatures:

    ~materials.VisualMaterial
    ~materials.PreviewSurface
    ~materials.OmniPBR
    ~materials.OmniGlass
    ~materials.PhysicsMaterial
    ~materials.ParticleMaterial
    ~materials.ParticleMaterialView
    ~materials.DeformableMaterial
    ~materials.DeformableMaterialView

.. rubric:: objects
.. autosummary::
    :nosignatures:

    ~objects.GroundPlane
    ~objects.VisualCapsule
    ~objects.VisualCone
    ~objects.VisualCuboid
    ~objects.VisualCylinder
    ~objects.VisualSphere
    ~objects.FixedCapsule
    ~objects.FixedCone
    ~objects.FixedCuboid
    ~objects.FixedCylinder
    ~objects.FixedSphere
    ~objects.DynamicCapsule
    ~objects.DynamicCone
    ~objects.DynamicCuboid
    ~objects.DynamicCylinder
    ~objects.DynamicSphere

.. rubric:: physics_context
.. autosummary::
    :nosignatures:

    ~physics_context.PhysicsContext

.. rubric:: robots
.. autosummary::
    :nosignatures:

    ~robots.Robot
    ~robots.RobotView

.. rubric:: scenes
.. autosummary::
    :nosignatures:

    ~scenes.Scene
    ~scenes.SceneRegistry

.. rubric:: sensors
.. autosummary::
    :nosignatures:

    ~sensors.BaseSensor
    ~sensors.RigidContactView

.. rubric:: simulation_context
.. autosummary::
    :nosignatures:

    ~simulation_context.SimulationContext

.. rubric:: world
.. autosummary::
    :nosignatures:

    ~world.World

.. rubric:: tasks
.. autosummary::
    :nosignatures:

    ~tasks.BaseTask
    ~tasks.FollowTarget
    ~tasks.PickPlace
    ~tasks.Stacking

|

.. API

Controllers
^^^^^^^^^^^

.. autoclass:: isaacsim.core.api.controllers.ArticulationController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.controllers.BaseController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.controllers.BaseGripperController
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Loggers
^^^^^^^

.. autoclass:: isaacsim.core.api.loggers.DataLogger
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Materials
^^^^^^^^^

.. autoclass:: isaacsim.core.api.materials.VisualMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.materials.PreviewSurface
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.materials.OmniPBR
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.materials.OmniGlass
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.materials.PhysicsMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.materials.ParticleMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.materials.ParticleMaterialView
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.materials.DeformableMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.materials.DeformableMaterialView
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Objects
^^^^^^^

Modules to create/encapsulate visual, fixed, and dynamic shapes (Capsule, Cone, Cuboid, Cylinder, Sphere) as well as ground planes

.. list-table::
    :header-rows: 1

    * - Type
      - Collider API
      - Rigid Body API
    * - Visual
      - No
      - No
    * - Fixed
      - Yes
      - No
    * - Dynamic
      - Yes
      - Yes

.. autoclass:: isaacsim.core.api.objects.GroundPlane
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.VisualCapsule
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.VisualCone
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.VisualCuboid
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.VisualCylinder
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.VisualSphere
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.FixedCapsule
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.FixedCone
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.FixedCuboid
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.FixedCylinder
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.FixedSphere
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.DynamicCapsule
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.DynamicCone
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.DynamicCuboid
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.DynamicCylinder
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.objects.DynamicSphere
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Physics Context
^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.core.api.physics_context.PhysicsContext
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Robots
^^^^^^

.. autoclass:: isaacsim.core.api.robots.Robot
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.robots.RobotView
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Scenes
^^^^^^

.. autoclass:: isaacsim.core.api.scenes.Scene
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.scenes.SceneRegistry
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Sensors
^^^^^^^

.. autoclass:: isaacsim.core.api.sensors.BaseSensor
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.sensors.RigidContactView
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

Simulation Context
^^^^^^^^^^^^^^^^^^

.. autoclass:: isaacsim.core.api.simulation_context.SimulationContext
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

|

World
^^^^^

.. autoclass:: isaacsim.core.api.world.World
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

Tasks
^^^^^

.. autoclass:: isaacsim.core.api.tasks.BaseTask
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.tasks.FollowTarget
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.tasks.PickPlace
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.api.tasks.Stacking
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:
