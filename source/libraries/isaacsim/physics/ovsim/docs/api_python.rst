..
   SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
   SPDX-License-Identifier: Apache-2.0

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

.. _isaacsim-physics-ovsim-api-python:

==============================
Python guide and API reference
==============================

.. _isaacsim-physics-ovsim-api-python-authoring:

Authoring
=========

The authoring names are present for OV SIM interface compatibility, but this
physics-only adapter does not implement stage authoring. Every function in this
section currently raises ``RuntimeError``.

.. currentmodule:: isaacsim.physics.ovsim.control.authoring

.. autosummary::

    create_stage
    open_stage
    save_stage
    export_stage_to_string
    import_stage_from_string
    close_stage
    add_reference_to_stage
    define_prim
    move_prim
    remove_prim
    create_prim_attribute
    remove_prim_attribute
    set_parameter
    get_parameter

.. autofunction:: create_stage
.. autofunction:: open_stage
.. autofunction:: save_stage
.. autofunction:: export_stage_to_string
.. autofunction:: import_stage_from_string
.. autofunction:: close_stage
.. autofunction:: add_reference_to_stage
.. autofunction:: define_prim
.. autofunction:: move_prim
.. autofunction:: remove_prim
.. autofunction:: create_prim_attribute
.. autofunction:: remove_prim_attribute
.. autofunction:: set_parameter
.. autofunction:: get_parameter

.. _isaacsim-physics-ovsim-api-python-simulation:

Simulation
==========

Manual simulation requires an OVStage native handle and a registered physics
engine. Configure both before initialization:

.. code-block:: python

    from isaacsim.physics.ovsim.control import simulation

    simulation.set_parameter("physics", "ovstage-stage-ptr", stage_handle)
    simulation.set_parameter("physics", "physics-engine", "ovphysx")
    simulation.initialize()
    simulation.step()
    simulation.invalidate()

``play()``, ``pause()``, and ``stop()`` are compatibility no-ops; use
``initialize()``, ``step()``, and ``invalidate()`` for manual control.

.. currentmodule:: isaacsim.physics.ovsim.control.simulation

.. autosummary::

    initialize
    play
    pause
    stop
    step
    invalidate
    set_parameter
    get_parameter

.. autofunction:: initialize
.. autofunction:: play
.. autofunction:: pause
.. autofunction:: stop
.. autofunction:: step
.. autofunction:: invalidate
.. autofunction:: set_parameter
.. autofunction:: get_parameter

.. _isaacsim-physics-ovsim-api-python-data:

Data
====

Data operations resolve physics entity views using the active engine. The
optional ``timestamp`` parameter is accepted for OV SIM compatibility, but
time-sampled reads and writes are not yet applied.

.. currentmodule:: isaacsim.physics.ovsim.data

.. autosummary::

    read
    write

.. autofunction:: read
.. autofunction:: write
