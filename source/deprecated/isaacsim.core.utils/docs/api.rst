Python API
==========

Articulation Utils
^^^^^^^^^^^^^^^^^^

Utils for programmatically interacting with Articulations on the Stage.

The utils can be used to:

* Modify Articulation Roots.
* Determine the base paths of every Articulation on the Stage.

.. automodule:: isaacsim.core.utils.articulations
    :members:
    :undoc-members:
    :show-inheritance:

|

Bounds Utils
^^^^^^^^^^^^

Utils for computing the Axis-Aligned Bounding Box (AABB) and the Oriented Bounding Box (OBB) of a prim.

* The AABB is the smallest cuboid that can completely contain the prim it represents.
  It is defined by the following 3D coordinates: :math:`(x_{min}, y_{min}, z_{min}, x_{max}, y_{max}, z_{max})`.
* Unlike the AABB, which is aligned with the coordinate axes, the OBB can be oriented at any angle in 3D space.

.. automodule:: isaacsim.core.utils.bounds
    :members:
    :undoc-members:
    :show-inheritance:

|

Carb Utils
^^^^^^^^^^

Carb settings is a generalized subsystem designed to provide a simple to use interface to Kit's various subsystems,
which can be automated, enumerated, serialized and so on.

The most common types of settings are:

* Persistent (saved between sessions): ``"/persistent/<setting>"``
  |br| (e.g., ``"/persistent/physics/numThreads"``)
* Application: ``"/app/<setting>"`` (e.g., ``"/app/viewport/grid/enabled"``)
* Extension: ``"/exts/<extension>/<setting>"`` (e.g., ``"/exts/omni.kit.debug.python/host"``)

.. automodule:: isaacsim.core.utils.carb
    :members:
    :undoc-members:
    :show-inheritance:

|

Collisions Utils
^^^^^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.collisions
    :members:
    :undoc-members:
    :show-inheritance:

|

Commands
^^^^^^^^

.. automodule:: isaacsim.core.utils.commands
    :members:
    :undoc-members:
    :show-inheritance:
    :imported-members:
    :exclude-members: do, undo

|

Constants Utils
^^^^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.constants
    :members:
    :undoc-members:
    :show-inheritance:

|

Distance Metrics Utils
^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.distance_metrics
    :members:
    :undoc-members:
    :show-inheritance:

|

Extensions Utils
^^^^^^^^^^^^^^^^

Utilities for enabling and disabling extensions from the Extension Manager and knowing their locations

.. automodule:: isaacsim.core.utils.extensions
    :members:
    :undoc-members:
    :show-inheritance:

|

Interoperability Utils
^^^^^^^^^^^^^^^^^^^^^^

Utilities for interoperability between different (ML) frameworks.
|br| Supported frameworks are:

* `Warp <https://nvidia.github.io/warp/index.html>`_
* `PyTorch <https://pytorch.org>`_
* `JAX <https://jax.readthedocs.io/>`_
* `TensorFlow <https://www.tensorflow.org>`_
* `NumPy <https://numpy.org>`_

.. automodule:: isaacsim.core.utils.interops
    :members:
    :undoc-members:
    :show-inheritance:

|

Math Utils
^^^^^^^^^^

.. automodule:: isaacsim.core.utils.math
    :members:
    :undoc-members:
    :show-inheritance:

.. automodule:: isaacsim.core.utils.bindings._isaac_utils.math
    :members:
    :undoc-members:
    :show-inheritance:
    :imported-members:

.. automodule:: isaacsim.core.utils.bindings._isaac_utils.transforms
    :members:
    :undoc-members:
    :show-inheritance:
    :imported-members:

|

Mesh Utils
^^^^^^^^^^

.. automodule:: isaacsim.core.utils.mesh
    :members:
    :undoc-members:
    :show-inheritance:

|

Physics Utils
^^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.physics
    :members:
    :undoc-members:
    :show-inheritance:

|

Prims Utils
^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.prims
    :members:
    :undoc-members:
    :show-inheritance:

|

Random Utils
^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.random
    :members:
    :undoc-members:
    :show-inheritance:

|

Render Product Utils
^^^^^^^^^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.render_product
    :members:
    :undoc-members:
    :show-inheritance:

|

Rotations Utils
^^^^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.rotations
    :members:
    :undoc-members:
    :show-inheritance:

|

Semantics Utils
^^^^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.semantics
    :members:
    :undoc-members:
    :show-inheritance:

|

Stage Utils
^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.stage
    :members:
    :undoc-members:
    :show-inheritance:

|

String Utils
^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.string
    :members:
    :undoc-members:
    :show-inheritance:

|

Transformations Utils
^^^^^^^^^^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.transformations
    :members:
    :undoc-members:
    :show-inheritance:

|

Types Utils
^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.types
    :members:
    :undoc-members:
    :show-inheritance:

|

Viewports Utils
^^^^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.viewports
    :members:
    :undoc-members:
    :show-inheritance:

|

XForms Utils
^^^^^^^^^^^^

.. automodule:: isaacsim.core.utils.xforms
    :members:
    :undoc-members:
    :show-inheritance:

|

NumPy Utils
^^^^^^^^^^^

Rotations
"""""""""

.. automodule:: isaacsim.core.utils.numpy.rotations
    :members:
    :undoc-members:
    :show-inheritance:

Maths
"""""

.. automodule:: isaacsim.core.utils.numpy.maths
    :members:
    :undoc-members:
    :show-inheritance:

Tensor
""""""

.. automodule:: isaacsim.core.utils.numpy.tensor
    :members:
    :undoc-members:
    :show-inheritance:

Transformations
"""""""""""""""

.. automodule:: isaacsim.core.utils.numpy.transformations
    :members:
    :undoc-members:
    :show-inheritance:

|

Torch Utils
^^^^^^^^^^^

Rotations
"""""""""

.. automodule:: isaacsim.core.utils.torch.rotations
    :members:
    :undoc-members:
    :show-inheritance:

Maths
"""""

.. automodule:: isaacsim.core.utils.torch.maths
    :members:
    :undoc-members:
    :show-inheritance:

Tensor
""""""

.. automodule:: isaacsim.core.utils.torch.tensor
    :members:
    :undoc-members:
    :show-inheritance:

Transformations
"""""""""""""""

.. automodule:: isaacsim.core.utils.torch.transformations
    :members:
    :undoc-members:
    :show-inheritance:

|

Warp Utils
^^^^^^^^^^

Rotations
"""""""""

.. automodule:: isaacsim.core.utils.torch.rotations
    :members:
    :undoc-members:
    :show-inheritance:

Tensor
""""""

.. automodule:: isaacsim.core.utils.torch.tensor
    :members:
    :undoc-members:
    :show-inheritance:

Transformations
"""""""""""""""

.. automodule:: isaacsim.core.utils.torch.transformations
    :members:
    :undoc-members:
    :show-inheritance:
