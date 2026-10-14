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

.. _isaacsim-foundation-utils-api-python:

==============================
Python guide and API reference
==============================

.. currentmodule:: isaacsim.foundation.utils

.. _isaacsim-foundation-utils-python-summary:

Summary
========

* :ref:`Backend <isaacsim-foundation-utils-python-backend>`
* :ref:`Stage <isaacsim-foundation-utils-python-stage>`
* :ref:`Prim <isaacsim-foundation-utils-python-prim>`
* :ref:`Semantics <isaacsim-foundation-utils-python-semantics>`

.. _isaacsim-foundation-utils-python-backend:

Backend
========

Provides a context manager that activates a named backend for the
current thread and restores the previous state on exit.

.. autofunction:: isaacsim.foundation.utils.backend.use_backend

.. autofunction:: isaacsim.foundation.utils.backend.get_current_backend

.. autofunction:: isaacsim.foundation.utils.backend.is_backend_set

.. autofunction:: isaacsim.foundation.utils.backend.should_raise_on_unsupported

.. autofunction:: isaacsim.foundation.utils.backend.should_raise_on_fallback

.. _isaacsim-foundation-utils-python-stage:

Stage
=====

Utilities for managing the process-wide default stage and the thread-local
active stage used by prim and semantics functions.

.. autofunction:: isaacsim.foundation.utils.stage.set_default_stage

.. autofunction:: isaacsim.foundation.utils.stage.get_default_stage

.. autofunction:: isaacsim.foundation.utils.stage.get_active_stage

.. autofunction:: isaacsim.foundation.utils.stage.use_stage

.. _isaacsim-foundation-utils-python-prim:

Prim
====

All prim functions operate on the active stage returned by
``get_active_stage`` (thread-local override via ``use_stage``, falling back
to the process-wide default set by ``set_default_stage``).
Paths are absolute USD path strings (e.g. ``"/World/Cube"``).

.. autofunction:: isaacsim.foundation.utils.prim.find_matching_prim_paths

.. autofunction:: isaacsim.foundation.utils.prim.get_all_matching_child_prims

.. autofunction:: isaacsim.foundation.utils.prim.get_first_matching_child_prim

.. autofunction:: isaacsim.foundation.utils.prim.get_first_matching_parent_prim

.. _isaacsim-foundation-utils-python-semantics:

Semantics
=========

Semantic labels are stored under ``SemanticsLabelsAPI`` keyed by taxonomy
(default: ``"class"``).

.. autofunction:: isaacsim.foundation.utils.semantics.add_labels

.. autofunction:: isaacsim.foundation.utils.semantics.get_labels

.. autofunction:: isaacsim.foundation.utils.semantics.remove_labels

.. autofunction:: isaacsim.foundation.utils.semantics.remove_all_labels
