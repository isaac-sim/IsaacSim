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

.. _isaacsim-foundation-utils-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

All declarations live in the ``isaacsim::foundation::utils`` namespace.

.. code-block:: cpp

    #include <isaacsim/foundation/utils/Backend.hpp>
    #include <isaacsim/foundation/utils/Prim.hpp>
    #include <isaacsim/foundation/utils/Semantics.hpp>
    #include <isaacsim/foundation/utils/Stage.hpp>

    using namespace isaacsim::foundation::utils;

.. _isaacsim-foundation-utils-api-cpp-backend:

Backend
=======

``BackendGuard`` activates a named backend for the current thread within a
scope and restores the previous context on destruction.  Declare it as a named
local variable --- a temporary is destroyed immediately:

.. code-block:: cpp

    {
        BackendGuard guard("tensor");
        // getCurrentBackend({"usd", "tensor"}) returns "tensor" here
    }
    // previous backend restored

``getCurrentBackend`` returns the active backend, falling back to the first
entry of *supportedBackends* when none is set or when the active backend is not
in the list:

.. code-block:: cpp

    std::string backend = getCurrentBackend({"usd", "tensor"});

    bool set = isBackendSet();
    bool raiseUnsupported = shouldRaiseOnUnsupported();
    bool raiseFallback    = shouldRaiseOnFallback();

.. _isaacsim-foundation-utils-api-cpp-stage:

Stage
=====

``StageGuard`` temporarily overrides the thread-local active stage and
restores the previous one on destruction.  Declare it as a named local
variable --- a temporary is destroyed immediately:

.. code-block:: cpp

    // Create a new stage (becomes the default automatically)
    objects::Stage stage = objects::Stage("ovstage").createStage();

    // Manage the process-wide default stage explicitly
    setDefaultStage(stage);
    objects::Stage defaultStage = getDefaultStage(); // throws if none set

    // Temporarily override the thread-local active stage
    {
        StageGuard guard(otherStage);
        objects::Stage activeStage = getActiveStage();  // throws if neither is set (default or active)
        findMatchingPrimPaths("/World/Robot", /*traverse=*/true);
    }

.. _isaacsim-foundation-utils-api-cpp-prim:

Prim
====

All prim functions operate on the active stage returned by ``getActiveStage``.
Paths are absolute USD path strings (e.g. ``"/World/Cube"``).

Path search
-----------

``findMatchingPrimPaths`` matches a literal path or regex pattern against prims
on the active stage:

.. code-block:: cpp

    // All descendants of /World/Robot
    std::vector<std::string> paths =
        findMatchingPrimPaths("/World/Robot", /*traverse=*/true);

    // Prims whose path matches a pattern, segment by segment
    std::vector<std::string> matches =
        findMatchingPrimPaths(".*Cube.*", /*traverse=*/false);

BFS traversal
-------------

``getAllMatchingChildPrims`` collects every prim in a subtree that satisfies a
predicate.  ``getFirstMatchingChildPrim`` stops at the first match:

.. code-block:: cpp

    auto isMesh = [](const std::string& p) { return p.find("Mesh") != std::string::npos; };

    std::vector<std::string> meshes =
        getAllMatchingChildPrims("/World", isMesh,
                                /*includeSelf=*/false,
                                /*maxDepth=*/std::nullopt);

    std::optional<std::string> first =
        getFirstMatchingChildPrim("/World", isMesh);

``getFirstMatchingParentPrim`` walks up the parent chain and returns the first
ancestor that satisfies the predicate:

.. code-block:: cpp

    std::optional<std::string> root =
        getFirstMatchingParentPrim("/World/Robot/Arm/Link",
                                   [](const std::string& p) { return p == "/World/Robot"; });

.. _isaacsim-foundation-utils-api-cpp-semantics:

Semantics
=========

Semantic labels are stored as ``SemanticsLabelsAPI`` multiple-apply schemas
keyed by taxonomy (default: ``"class"``).

.. code-block:: cpp

    // Add labels
    addLabels("/World/Cube", "vehicle");
    addLabels("/World/Cube", std::vector<std::string>{"vehicle", "car"}, "type");

    // Query labels (optionally including descendants)
    std::unordered_map<std::string, std::vector<std::string>> labels =
        getLabels("/World/Cube", /*includeDescendants=*/false);
    // e.g. { "class": ["vehicle"], "type": ["vehicle", "car"] }

    // Remove specific labels
    removeLabels("/World/Cube", "vehicle");
    removeLabels("/World/Cube", std::vector<std::string>{"vehicle", "car"},
                 /*taxonomy=*/"type", /*includeDescendants=*/false);

    // Clear all labels; pass removeTaxonomies=true to also unapply the schema
    removeAllLabels("/World/Cube", /*removeTaxonomies=*/false,
                    /*includeDescendants=*/false);
