API
===

.. warning::

    **The API featured in this extension is experimental and subject to change without deprecation cycles.**
    Although we will try to maintain backward compatibility in the event of a change, it may not always be possible.

Python API
----------

.. Summary

The following table summarizes the available wrappers.

.. currentmodule:: isaacsim.core.experimental.prims

.. autosummary::
    :nosignatures:

    Articulation
    DeformablePrim
    GeomPrim
    Prim
    RigidPrim
    XformPrim

.. Details

Implementation details
^^^^^^^^^^^^^^^^^^^^^^

The implementation of the wrappers' properties and methods is done using one or more of the backends listed in the following table.
The docstring of such properties and methods will indicate which backends (in order of call) are supported.

.. |OpenUSD| replace:: `OpenUSD <https://www.nvidia.com/en-us/omniverse/usd>`__
.. |IFabricHierarchy| replace:: `Fabric Scene Delegate (FSD) and IFabricHierarchy <https://docs.omniverse.nvidia.com/kit/docs/usdrt/latest/docs/fabric_hierarchy.html>`__
.. |Fabric| replace:: `USD, Fabric, and USDRT <https://docs.omniverse.nvidia.com/kit/docs/usdrt/latest/docs/usd_fabric_usdrt.html>`__
.. |Omni Physics Tensors| replace:: `Omni Physics Tensors <https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physics.tensors/docs/index.html>`__

.. list-table::
    :header-rows: 1

    * - Backend
      - Description
      - Performance
      - Availability
    * - :guilabel:`usd`
      - System for authoring, composing, and reading hierarchically organized scene description (see |OpenUSD|).
        OpenUSD is foundational to NVIDIA Omniverse.
      - Standard
      - At any time
    * - :guilabel:`usdrt`
      - Omniverse API that mirrors the USD API but reads and writes data to and from Fabric instead of USD (see |IFabricHierarchy|).
      - Fast
      - At any time
    * - :guilabel:`fabric`
      - Omniverse library that enables high-performance creation, modification, and access of scene data (see |Fabric|).
      - Fast
      - At any time
    * - :guilabel:`tensor`
      - Interface for interacting with physics simulations in a data-oriented way (see |Omni Physics Tensors|).
      - Fastest
      - During simulation

.. note::

    The selection of a backend (when an implementation supports more than one) will be made according to its
    *availability* and according to the listed order. The *availability* refers to the state of the simulation
    in which a backend can be used after instantiating a class.

    A specific backend can be explicitly requested using the :func:`~isaacsim.core.experimental.utils.impl.backend.use_backend` context manager.

.. warning::

    If a backend is explicitly requested (using the :func:`~isaacsim.core.experimental.utils.impl.backend.use_backend` context manager)
    but is unavailable at the time of the request, resulting in a fallback to another backend, a warning is logged.

.. warning::

    The :guilabel:`usdrt` and :guilabel:`fabric` backends require Fabric Scene Delegate (FSD) to be enabled.
    FSD can be enabled in *apps/.kit* experience files by setting ``app.useFabricSceneDelegate = true``.

.. warning::

    The :guilabel:`tensor` backend requires the simulation to be running (in play). Calling a property or method
    implemented only using this backend will raise an :exc:`AssertionError` if the simulation is not running.
    If the implementation supports several backends, and the simulation is not running, the call will fallback
    to the next listed backend (typically :guilabel:`usd`).

.. API

Wrappers
^^^^^^^^

.. autoclass:: isaacsim.core.experimental.prims.Articulation
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.prims.DeformablePrim
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.prims.GeomPrim
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.prims.Prim
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:
    :special-members: __len__

.. autoclass:: isaacsim.core.experimental.prims.RigidPrim
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:

.. autoclass:: isaacsim.core.experimental.prims.XformPrim
    :members:
    :undoc-members:
    :inherited-members:
    :show-inheritance:
