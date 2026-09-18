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

.. _isaacsim-foundation-usd-openusd:
.. _isaacsim-foundation-usd-openusd-overview:

===============================
isaacsim.foundation.usd.openusd
===============================

``isaacsim.foundation.usd.openusd`` is a thin C++ facade over OpenUSD.
It exposes stage lifecycle, prim authoring, schema queries, variant
selection, and attribute I/O through a stable ABI based on opaque integer
stage handles and standard C++ types. Callers never hold raw
``UsdStageRefPtr`` or ``SdfPath`` objects across the boundary, so they do not
need to link against OpenUSD directly.

.. toctree::
    :maxdepth: 2

    api_cpp
