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

.. _isaacsim-physics-engines-ovphysx-api-python:

==============================
Python guide and API reference
==============================

.. _isaacsim-physics-engines-ovphysx-registration:

Backend registration
====================

Import the module to load and register the OvPhysX backend. Then select it through the physics manager:

.. code-block:: python

    import isaacsim.physics_engines.ovphysx
    from isaacsim.physics.manager import PhysicsManager

    physics_manager = PhysicsManager.get_instance()
    if not physics_manager.switch_physics_engine("ovphysx"):
        raise RuntimeError("OvPhysX physics engine is unavailable.")

You do not need to call ``activate()`` or ``shutdown()`` for normal application use.

.. currentmodule:: isaacsim.physics_engines.ovphysx

.. automodule:: isaacsim.physics_engines.ovphysx
    :members:
    :undoc-members:
