API
===

.. warning::

    **The API featured in this extension is experimental and subject to change without deprecation cycles.**
    Although we will try to maintain backward compatibility in the event of a change, it may not always be possible.

Python API
----------

.. Summary

The following table summarizes the available materials.

.. currentmodule:: isaacsim.core.experimental.materials

.. rubric:: physics materials
.. autosummary::
    :nosignatures:

    PhysicsMaterial
    RigidBodyMaterial
    SurfaceDeformableMaterial
    VolumeDeformableMaterial

.. rubric:: visual materials
.. autosummary::
    :nosignatures:

    OmniGlassMaterial
    OmniPbrMaterial
    PreviewSurfaceMaterial
    VisualMaterial

.. Details

.. On shaders, the shader parameters are encoded as inputs.
.. https://docs.omniverse.nvidia.com/materials-and-rendering/latest/materials_templates.html

Materials
^^^^^^^^^

Physics Materials
"""""""""""""""""

.. autoclass:: isaacsim.core.experimental.materials.PhysicsMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.materials.RigidBodyMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.materials.SurfaceDeformableMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.materials.VolumeDeformableMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

Visual Materials
""""""""""""""""

.. autoclass:: isaacsim.core.experimental.materials.OmniGlassMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.materials.OmniPbrMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.materials.PreviewSurfaceMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.materials.VisualMaterial
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:
