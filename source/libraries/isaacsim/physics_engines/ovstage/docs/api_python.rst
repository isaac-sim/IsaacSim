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

.. _isaacsim-physics-engines-ovstage-api-python:

==============================
Python guide and API reference
==============================

.. currentmodule:: isaacsim.physics_engines.ovstage

Use :func:`setup` before importing a backend that needs the packaged OVStage
runtime directly. :func:`get_native_handle` registers a Python stage for later
lookup, while :func:`as_native_handle` normalizes integer and nanobind capsule
representations.

.. autosummary::

    setup
    get_native_handle
    as_native_handle
    lookup_stage

.. autofunction:: setup
.. autofunction:: get_native_handle
.. autofunction:: as_native_handle
.. autofunction:: lookup_stage
