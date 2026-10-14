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

An ``Array`` combines an element buffer with a ``Shape``, element ``strides``, a
``Dtype``, and a ``Device``. The buffer is managed through shared ownership, so
copying an ``Array`` shares the allocation rather than duplicating it. Use
``contiguous()`` to compact a strided view when needed --- it decides on layout
alone --- ``copy()`` when an independent, writeable compacted copy is required, or
``clone()`` to preserve the layout and the offset of a slice.

``broadcastTo()`` materializes any changed shape; an unchanged shape shares storage unless
``copy`` is true. ``transpose()`` always returns independent contiguous storage. Use
``broadcastView()`` and ``transposeView()`` to explicitly request shared strided views.
Replicated broadcast views are read-only.

The contiguous ``fromBuffer(data, shape, dtype, device, offset)`` overload retains the caller's
owner and infers only the declared byte span. The caller remains responsible for allocation size.
Use the capacity-taking overload to validate a known allocation bound or supply strides.

The three supporting value types are:

- ``Shape`` --- an ordered sequence of signed 64-bit dimension sizes. Supports
  NumPy-style broadcasting rules and -1 dimension inference for ``reshape()``.
- ``Dtype`` --- an element type token covering ``bool``, the fixed-width integer types
  from ``<cstdint>``, ``float``, and ``double``.
- ``Device`` --- a compute target, either the CPU (ordinal -1) or a CUDA GPU
  (non-negative ordinal). ``DeviceGuard`` provides RAII device switching.

CUDA operations load the NVIDIA CUDA driver dynamically (``libcuda.so.1`` on Linux,
``nvcuda.dll`` on Windows). It is installed by the display driver in a location the
loader already searches, so no CUDA toolkit or redistributable is required at runtime
and nothing has to be added to the library search path. On a machine with no CUDA
driver the library still loads and its CPU paths remain fully usable.

.. _isaacsim-common-array-native-consumers:

DLPack and native consumers
---------------------------

``fromDLPack()`` borrows contiguous tensors, ignoring irrelevant strides on singleton
axes and empty tensors. ``toDLPack()`` returns borrowed data and metadata; the source
Array must outlive the tensor. ``fromDLPackBorrowed()`` additionally preserves
nonnegative strided layouts. Explicit managed/versioned exports retain read access
until their deleter runs. The Python caster retains allocation ownership after
conversion, allowing synchronized native ``Array::set()`` calls while a Python alias lives.
CUDA conversion orders the default consumer stream after recorded producer work;
callers remain responsible for synchronizing subsequent writes with Python consumers.

Rebuild native consumers against matching headers and libraries. Subclasses that
accessed the protected ``m_storage`` variant or ``details::CpuStorage`` /
``details::CudaStorage`` must migrate to ``data()`` for a borrowed pointer,
``buffer()`` for retained allocation ownership, and ``fromBuffer()`` for buffer
adoption. Use ``acquireRead()`` / ``acquireWrite()`` for managed stream access.
The storage representation and binary layout are not compatible with those old
subclass implementations; replacing storage through protected members is not a
supported extension contract.

.. toctree::
    :maxdepth: 2

    api_cpp
