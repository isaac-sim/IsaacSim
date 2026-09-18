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

.. _isaacsim-common-array-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

Include ``isaacsim/common/array/Array.hpp`` and link ``isaacsim::common-array``.
The header transitively includes ``Device.hpp``, ``DLPack.hpp``, ``Dtype.hpp``,
``Functions.hpp``, and ``Shape.hpp``.

All examples on this page assume the following namespace alias:

.. code-block:: cpp

    #include <isaacsim/common/array/Array.hpp>

    namespace array = isaacsim::common::array;

.. _isaacsim-common-array-api-cpp-array:

Array
=====

``array::Array`` is a multi-dimensional array backed by a reference-counted buffer
on either the CPU or a CUDA device.

Construction
------------

Construct an ``array::Array`` from a scalar, a ``std::vector``, or a
``std::vector<std::vector>``. The element type is inferred from the value unless
you supply an explicit ``array::Dtype``:

.. code-block:: cpp

    // Scalar --- produces a 0-D float32 array.
    array::Array scalar(3.14f);

    // 1-D --- produces a float32 array of shape (3,).
    array::Array vector(std::vector<float>{1.0f, 2.0f, 3.0f});

    // 2-D with explicit dtype --- produces an int32 array of shape (2, 3).
    array::Array matrix(std::vector<std::vector<int32_t>>{{1, 2, 3}, {4, 5, 6}},
                        array::Dtype::Int32());

    // On a CUDA device.
    array::Array cuda_array(std::vector<float>{1.0f, 2.0f, 3.0f}, std::nullopt, array::Device::Cuda());

Inspection
----------

.. code-block:: cpp

    array::Array a(std::vector<float>{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
    a = a.reshape(array::Shape({2, 3}));

    a.shape();   // array::Shape({2, 3})
    a.ndim();    // 2
    a.size();            // 6
    a.nbytes();  // 24
    a.dtype();   // array::Dtype::Float32()
    a.device();  // array::Device::Cpu()

Use ``data()`` to obtain a raw pointer to element 0, or ``buffer()`` to retrieve
the underlying shared allocation (which may start before element 0 for sliced views):

.. code-block:: cpp

    const float* ptr = static_cast<const float*>(a.data());

Views and reshaping
-------------------

``reshape()`` and ``at()`` return views over the same buffer with no data duplication,
as do copy-construction and copy-assignment; ``copy()`` produces an independent,
compacted deep copy:

.. code-block:: cpp

    array::Array flat(std::vector<float>{1.0f, 2.0f, 3.0f, 4.0f});

    // Infer the second dimension with -1.
    array::Array matrix = flat.reshape(array::Shape({2, -1}));  // shape (2, 2)

    // Slice along axis 0 --- returns a (2,) view.
    array::Array row = matrix.at(0);

    // Negative indices count from the end.
    array::Array last_row = matrix.at(-1);

    // Shallow copy --- shares the buffer, shape, dtype and offset of the source.
    array::Array alias = matrix;

    // Deep copy --- independent, compacted buffer (offset reset to 0).
    array::Array compact = matrix.copy();

    // Deep copy --- preserves full buffer layout including any pre-element offset.
    array::Array independent = matrix.clone();

Device and dtype conversion
---------------------------

``toDevice()`` and ``toDtype()`` return the same array if no conversion is needed
(optionally forcing a copy with the second argument), or a new array otherwise:

.. code-block:: cpp

    array::Array cpu_array(std::vector<float>{1.0f, 2.0f});

    // Transfer to GPU --- allocates a new CUDA buffer.
    array::Array gpu_array = cpu_array.toDevice(array::Device::Cuda());

    // Cast to double --- allocates a new buffer.
    array::Array f64 = cpu_array.toDtype(array::Dtype::Float64());

    // Always return an independent copy, even when no conversion is needed.
    array::Array copy = cpu_array.toDevice(array::Device::Cpu(), /*copy=*/true);

Broadcasting
------------

``broadcastTo()`` replicates elements along broadcast axes following NumPy rules.
It returns the same array if no broadcasting is needed
(optionally forcing a copy with the second argument), or a new array otherwise:

.. code-block:: cpp

    array::Array row(std::vector<float>{1.0f, 2.0f, 3.0f});   // shape (3,)
    array::Array tiled = row.broadcastTo(array::Shape({4, 3})); // shape (4, 3)

Reading and writing elements
----------------------------

Use ``get<T>()`` to extract the array contents as a C++ scalar or nested vector.
The template argument selects the output rank:

.. code-block:: cpp

    array::Array a(std::vector<std::vector<float>>{{1.0f, 2.0f}, {3.0f, 4.0f}});

    // 2-D output.
    auto matrix = a.get<std::vector<std::vector<float>>>();

    // Scalar --- array must contain exactly one element.
    array::Array scalar(42.0f);
    float value = scalar.get<float>();

    // item() is stricter: requires exactly one element regardless of shape.
    float value = scalar.item<float>();

Use ``set()`` to overwrite the array's contents. The argument is broadcast and cast
to match the array's shape and dtype:

.. code-block:: cpp

    array::Array a(std::vector<float>{1.0f, 2.0f, 3.0f});

    // From a scalar or vector.
    a.set(std::vector<float>{4.0f, 5.0f, 6.0f});

    // From another array --- broadcast and cast automatically.
    array::Array b(7.0f);
    a.set(b);

String representation
---------------------

``toString()`` returns a human-readable summary:

.. code-block:: cpp

    array::Array a(std::vector<float>{1.0f, 2.0f});
    a.toString();
    // Array([1, 2], shape=(2,), dtype='float32', device='cpu')

.. _isaacsim-common-array-api-cpp-dtype:

Dtype
=====

``array::Dtype`` identifies the scalar element type of an array. Supported element
types are ``bool``, the fixed-width integer types from ``<cstdint>``, ``float``, and ``double``.

Construction
------------

Construct an ``array::Dtype`` from a ``Kind`` enumerator, a string name, or a named
factory:

.. code-block:: cpp

    array::Dtype a = array::Dtype::Float32();
    array::Dtype b = array::Dtype(array::Dtype::Kind::eFloat32);
    array::Dtype c = array::Dtype("float32");       // equivalent to both above
    array::Dtype d = array::Dtype::fromString("float32");

    // Compile-time mapping from a C++ type.
    array::Dtype e = array::Dtype::fromType<float>();  // array::Dtype::Float32()

Named factories
---------------

.. list-table::
   :header-rows: 1
   :widths: 25 20 15

   * - Factory
     - Kind
     - Size (bytes)
   * - ``Dtype::Bool()``
     - ``eBool``
     - 1
   * - ``Dtype::Int8()``
     - ``eInt8``
     - 1
   * - ``Dtype::Int16()``
     - ``eInt16``
     - 2
   * - ``Dtype::Int32()``
     - ``eInt32``
     - 4
   * - ``Dtype::Int64()``
     - ``eInt64``
     - 8
   * - ``Dtype::UInt8()``
     - ``eUInt8``
     - 1
   * - ``Dtype::UInt16()``
     - ``eUInt16``
     - 2
   * - ``Dtype::UInt32()``
     - ``eUInt32``
     - 4
   * - ``Dtype::UInt64()``
     - ``eUInt64``
     - 8
   * - ``Dtype::Float32()``
     - ``eFloat32``
     - 4
   * - ``Dtype::Float64()``
     - ``eFloat64``
     - 8

Queries
-------

.. code-block:: cpp

    array::Dtype t = array::Dtype::Float32();

    t.kind();        // array::Dtype::Kind::eFloat32
    t.size();        // 4
    t.isFloating();  // true
    t.isIntegral();  // false
    t.isSigned();    // true
    t.isUnsigned();  // false
    t.toString();    // "float32"

    array::Dtype::Float32() == array::Dtype::Float32();  // true
    array::Dtype::Float32() != array::Dtype::Int32();    // true

.. _isaacsim-common-array-api-cpp-shape:

Shape
=====

``array::Shape`` holds an ordered sequence of dimension sizes as signed 64-bit
integers. An empty ``Shape`` is zero-dimensional (scalar).

Construction
------------

.. code-block:: cpp

    array::Shape s1({3, 4, 5});                           // 3-D shape from initializer list
    array::Shape s2(std::vector<int32_t>{3, 4, 5});       // from a SupportedShapeSpecification vector
    array::Shape s3(12);                                   // 1-D shape with dimension 12

Queries
-------

.. code-block:: cpp

    array::Shape s({3, 4, 5});

    s.ndim();        // 3
    s.size(); // 60
    s.shape();   // std::vector<int64_t>{3, 4, 5}
    s[0];            // 3
    s[-1];           // 5  (negative indices count from the end)
    s.toString();    // "(3, 4, 5)"

Broadcasting and reshape
------------------------

``canBroadcastTo()`` checks NumPy broadcasting compatibility. ``resolve()`` expands
a -1 placeholder in a target shape to the correct dimension size:

.. code-block:: cpp

    array::Shape src({1, 4});
    src.canBroadcastTo(array::Shape({3, 4}));  // true
    src.canBroadcastTo(array::Shape({3, 5}));  // false

    array::Shape base({2, 6});
    base.resolve(array::Shape({3, -1}));  // array::Shape({3, 4})

.. _isaacsim-common-array-api-cpp-device:

Device
======

``array::Device`` identifies a compute device: the CPU (ordinal -1) or a CUDA GPU
(non-negative ordinal).

Construction
------------

.. code-block:: cpp

    array::Device cpu     = array::Device::Cpu();
    array::Device gpu0    = array::Device::Cuda();      // ordinal 0
    array::Device gpu1    = array::Device::Cuda(1);     // ordinal 1
    array::Device from_str = array::Device::fromString("cuda:0");

    // From a SupportedDeviceSpecification.
    array::Device d1 = array::Device(int32_t(0));       // CUDA device 0
    array::Device d2 = array::Device(std::string("cpu"));

Queries
-------

.. code-block:: cpp

    array::Device d = array::Device::Cuda(1);

    d.ordinal();      // 1
    d.isCpu();        // false
    d.isCuda();       // true
    d.isAvailable();  // true if CUDA device 1 is present
    d.toString();     // "cuda:1"

    array::Device::Cpu() == array::Device::Cpu();   // true
    array::Device::Cpu() != array::Device::Cuda();  // true

.. _isaacsim-common-array-api-cpp-device-guard:

DeviceGuard
===========

``array::DeviceGuard`` is an RAII guard that sets the active CUDA device on
construction and restores the previous device on destruction. It has no effect when
targeting the CPU.

``DeviceGuard`` is non-copyable and non-movable.

.. code-block:: cpp

    {
        array::DeviceGuard guard(array::Device::Cuda(1));
        // CUDA calls here run on device 1.
    }
    // Previous device is restored.

    // Construct directly from an ordinal.
    {
        array::DeviceGuard guard(1);
        // ...
    }


.. _isaacsim-common-array-api-cpp-functions:

Functions
=========

Free functions in ``isaacsim/common/array/Functions.hpp`` that create ``array::Array``
instances, rearrange their elements, and apply element-wise arithmetic and logical
operations, all following NumPy broadcasting and type-promotion rules. Some functions
have an operator alias and overloads accepting a scalar, ``std::vector``, or
``std::vector<std::vector>`` operand in place of an ``Array``.

Creation
--------

``empty()``, ``zeros()``, and ``ones()`` allocate a new array of a given shape.
``empty()`` leaves the elements unspecified and is for buffers you
are about to overwrite entirely; the other two fill every element:

.. code-block:: cpp

    array::Array a = array::empty({2, 3});                          // uninitialized, float64, CPU
    array::Array b = array::zeros({2, 3}, array::Dtype::Int32());   // all 0
    array::Array c = array::ones(
        {4}, array::Dtype::Float32(), array::Device::Cuda());  // all 1.0f, on the GPU

    array::zeros({4}, array::Dtype::Bool());  // all false
    array::ones({4}, array::Dtype::Bool());   // all true

    array::empty(array::Shape());  // 0-D: one element, as in np.empty(())

.. note::

   The dtype defaults to ``float64``, as in NumPy. Isaac Sim generally works in
   single precision, so pass an explicit ``array::Dtype::Float32()`` where that is
   what you want --- the default doubles the footprint and costs a conversion at the
   boundary otherwise.

Element-wise operations and reductions
--------------------------------------

The reductions --- ``all()``, ``any()``, ``sum()``, ``prod()``, ``amin()``, and
``amax()`` --- reduce the whole array, regardless of its rank, to a 0-D
``array::Array`` on the operand's device, so a CUDA result stays on the device.
Read the result with ``item<T>()``. ``all()`` and ``any()`` always produce ``bool``,
counting an element as true when it is non-zero, so ``NaN`` is true.
``sum()`` and ``prod()`` accumulate in a wider dtype so narrow integers do not overflow:
``bool`` and the signed integers widen to ``int64``, the unsigned integers to ``uint64``,
while ``float32`` and ``float64`` keep their own dtype. ``amin()`` and ``amax()`` keep the
operand's dtype.

Empty arrays reduce to the operation's identity: ``true`` for
``all()``, ``false`` for ``any()``, ``0`` for ``sum()``, and ``1`` for ``prod()``.
``amin()`` and ``amax()`` have no identity and throw ``std::invalid_argument``.

.. note::

   ``amin()`` and ``amax()`` are named after NumPy's ``numpy.amin``/``numpy.amax``
   aliases rather than ``min``/``max`` because ``<windows.h>`` defines ``min`` and
   ``max`` as function-like macros. The preprocessor runs before name lookup, so
   the namespace does not protect the call: ``array::min(a)`` would expand the
   macro and fail to compile wherever ``<windows.h>`` is included without
   ``NOMINMAX``.

.. code-block:: cpp

    array::Array a(std::vector<float>{1.0f, 2.0f, 3.0f});
    array::Array b(std::vector<float>{10.0f, 20.0f, 30.0f});
    array::Array c(std::vector<float>{1.0f, 0.0f, 3.0f});

    array::add(a, b);        // same as a + b
    array::add(a, 1.0f);     // same as a + b (scalar/vector overload)
    array::subtract(a, b);   // same as a - b
    array::multiply(a, b);   // same as a * b
    array::divide(a, b);     // same as a / b, always float32/float64

    array::negative(a);      // same as -a
    array::absolute(a);      // element-wise |x|
    array::logicalNot(a);    // same as !a, always bool

    array::all(c).item<bool>();   // false --- element 1 is zero
    array::any(c).item<bool>();   // true
    array::sum(c).item<float>();  // 4.0f
    array::prod(c).item<float>(); // 0.0f
    array::amin(c).item<float>(); // 0.0f
    array::amax(c).item<float>(); // 3.0f

Rearranging elements
--------------------

``transpose()`` permutes an array's axes and ``take()`` gathers elements along a
single axis. ``transpose()`` reverses the axes when ``axes`` is
omitted; with an explicit permutation, axis ``i`` of the result is axis ``axes[i]``
of the operand. ``take()`` selects the listed positions along ``axis``, resizing that axis
to the number of indices, and gathers from the flattened operand when ``axis`` is
omitted. Negative axes count from the last dimension and negative indices from the
end of the axis:

.. code-block:: cpp

    array::Array a(std::vector<std::vector<float>>{{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}});

    array::transpose(a);  // shape (3, 2) --- {{1, 4}, {2, 5}, {3, 6}}

    // Same, with axes given explicitly. `axes` is an std::optional, so a braced
    // permutation needs its type spelled out.
    array::transpose(a, std::vector<int64_t>{-1, -2});

    array::take(a, {2, 1}, 1);  // shape (2, 2) --- {{3, 2}, {6, 5}}
    array::take(a, {0, -1});    // shape (2,)   --- {1, 6}, from the flattened operand

The two are distinct operations, and only ``take()`` reorders the components *within*
an axis. Converting an ``(N, 4)`` array of ``w, x, y, z`` quaternions to ``x, y, z, w``
is a ``take()`` along the last axis --- ``transpose()`` would instead give a ``(4, N)``
array with the component order untouched:

.. code-block:: cpp

    array::Array xyzw = array::take(wxyz, {1, 2, 3, 0}, 1);
    array::Array back = array::take(xyzw, {3, 0, 1, 2}, 1);

An out-of-range axis throws ``std::invalid_argument`` and an out-of-range index
throws ``std::out_of_range``, mirroring NumPy's split between ``AxisError`` (a
``ValueError``) and ``IndexError``.

.. note::

   Where NumPy returns a view, both functions return a newly allocated array holding
   a copy of the elements. An ``array::Array`` is always densely packed and stores no
   per-axis strides, so a permuted or gathered view is not representable.


.. _isaacsim-common-array-api-cpp-dlpack-conversion:

DLPack conversion
=================

Use ``fromDLPack()`` and ``toDLPack()`` to create zero-copy, C-contiguous views. The
conversion supports the Array scalar dtypes on CPU and CUDA devices:

.. code-block:: cpp

    array::Array values(std::vector<float>{1.0f, 2.0f, 3.0f});

    // The DLTensor borrows data and shape metadata from values.
    DLTensor tensor = array::toDLPack(values);

    // The Array borrows the DLTensor's data allocation and copies its metadata.
    array::Array view = array::fromDLPack(tensor);

The source allocation must outlive a view imported with ``fromDLPack()``. An Array
exported with ``toDLPack()`` must remain alive and must not be moved from or assigned
while the ``DLTensor`` is in use. The conversions do not synchronize CUDA streams.
