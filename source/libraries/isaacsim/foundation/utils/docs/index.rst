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

.. _isaacsim-foundation-utils:
.. _isaacsim-foundation-utils-overview:

=========================
isaacsim.foundation.utils
=========================

``isaacsim.foundation.utils`` provides shared utilities for stage management,
prim authoring, semantics labeling, and backend selection used across Isaac Sim
foundation modules.

.. _isaacsim-foundation-utils-backend-selection:

Backend selection
-----------------

A thread-local ``BackendGuard`` activates a named backend (e.g. ``"usd"``,
``"tensor"``) for the duration of a scope.  ``getCurrentBackend()`` returns
the active backend, falling back to the first entry of a caller-supplied
supported-backends list when none is set.

.. _isaacsim-foundation-utils-stage-prim-semantics:

Stage, prim, and semantics utilities
------------------------------------

Helper functions for common USD operations --- managing default stage IDs,
querying prim hierarchies, and reading or writing semantics attributes ---
are grouped into the ``stage``, ``prim``, and ``semantics`` sub-modules.
All utilities that do not accept an explicit stage instance operate on the
active stage for the calling thread, or the process-global default stage.

.. toctree::
    :maxdepth: 2

    api_cpp
    api_python
