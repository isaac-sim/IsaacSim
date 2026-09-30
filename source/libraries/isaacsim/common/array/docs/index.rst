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

.. _isaacsim-common-array:
.. _isaacsim-common-array-overview:

=====================
isaacsim.common.array
=====================

``isaacsim.common.array`` provides a lightweight multi-dimensional array type for
exchanging tensor data across C++ and Python boundaries in Isaac Sim. It is not a
general-purpose math library; it covers the construction, inspection, slicing,
device transfer, and dtype conversion operations needed to move data between
simulation components.

.. _isaacsim-common-array-overview-design:

Design
------

An ``Array`` combines a contiguous element buffer with a ``Shape``, a ``Dtype``, and
a ``Device``. The buffer is managed through shared ownership, so copying an ``Array``
shares the allocation rather than duplicating it. Use ``copy()`` when an independent,
compacted copy is required, or ``clone()`` to also preserve the offset of a slice.

The three supporting value types are:

- ``Shape`` --- an ordered sequence of signed 64-bit dimension sizes. Supports
  NumPy-style broadcasting rules and -1 dimension inference for ``reshape()``.
- ``Dtype`` --- an element type token covering ``bool``, the fixed-width integer types
  from ``<cstdint>``, ``float``, and ``double``.
- ``Device`` --- a compute target, either the CPU (ordinal -1) or a CUDA GPU
  (non-negative ordinal). ``DeviceGuard`` provides RAII device switching.

CUDA operations load the platform CUDA runtime dynamically. The runtime must be
discoverable through the host library search path as ``libcudart.so`` on Linux or
``cudart64_12.dll`` on Windows.

.. toctree::
    :maxdepth: 2

    api_cpp
