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

.. _isaacsim-foundation-ovsim-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

Link against ``isaacsim::foundation-ovsim``. Headers are split across three
namespaces matching their include paths.

.. _isaacsim-foundation-ovsim-api-cpp-authoring:

Authoring
=========

``#include <isaacsim/foundation/ovsim/control/authoring/Authoring.hpp>``

Namespace: ``isaacsim::foundation::ovsim::control::authoring``

Stage lifecycle
---------------

.. code-block:: cpp

    bool created = createStage();
    bool opened  = openStage("/path/to/scene.usd");
    bool saved   = saveStage("/path/to/output.usd");
    std::string usdString = exportStageToString();
    bool imported = importStageFromString(usdString);
    bool closed  = closeStage();

References
----------

.. code-block:: cpp

    // Add a USD reference arc, defining the prim first if needed.
    bool ok = addReferenceToStage("/path/to/ref.usd", "/World/Prim", "Xform");

Prim authoring
--------------

.. code-block:: cpp

    bool ok      = definePrim("/World/Prim", "Xform");
    bool moved   = movePrim("/World/OldName", "/World/NewName");
    bool removed = removePrim("/World/Prim");

Attributes
----------

.. code-block:: cpp

    bool created = createPrimAttribute("/World/Prim", "mass", "float");
    bool removed = removePrimAttribute("/World/Prim", "mass");

Parameters
----------

The current authoring implementation accepts and ignores values passed to
``setParameter()``. Use ``getParameter()`` with the ``stage`` provider to query
the retained OpenUSD or OVStage stage identifier or pointer.

.. code-block:: cpp

    setParameter("provider", "parameterName", value);
    OutputParameterType value = getParameter("provider", "parameterName");

.. _isaacsim-foundation-ovsim-api-cpp-simulation:

Simulation
==========

``#include <isaacsim/foundation/ovsim/control/simulation/Simulation.hpp>``

Namespace: ``isaacsim::foundation::ovsim::control::simulation``

``play()``, ``pause()``, ``stop()``, and ``step()`` currently accept requests
without advancing simulation state. ``initialize()`` copies the active OpenUSD
stage into a retained OVStage snapshot, and ``invalidate()`` releases that
snapshot. Simulation provider parameters are not implemented and raise
``std::logic_error``.

.. code-block:: cpp

    // automatic mode
    play();
    pause();
    stop();
    // manual mode
    initialize();
    step();
    invalidate();

    setParameter("provider", "parameterName", value);
    OutputParameterType value = getParameter("provider", "parameterName");

.. _isaacsim-foundation-ovsim-api-cpp-data:

Data
====

``#include <isaacsim/foundation/ovsim/data/Data.hpp>``

Namespace: ``isaacsim::foundation::ovsim::data``

Types re-exported from the OV SIM interface:

- ``PathType`` --- list of absolute prim path strings.
- ``InputValueType`` --- value(s) to write.
- ``OutputValueType`` --- value(s) read back.

.. code-block:: cpp

    // Read an attribute from one or more prims.
    OutputValueType values = read(paths, "mass");

    // Write an attribute to one or more prims.
    write(paths, "mass", inputValues);

The optional timestamp parameter is reserved for time-sampled values. The
current implementation reads and writes default-time values.
