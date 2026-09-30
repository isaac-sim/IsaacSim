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

.. _isaacsim-common-exceptions-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

Include ``isaacsim/common/exceptions/Exceptions.hpp`` and link ``isaacsim::common-exceptions``:

.. code-block:: cpp

    #include <isaacsim/common/exceptions/Exceptions.hpp>

    using isaacsim::common::exceptions::PrimPathError;
    using isaacsim::common::exceptions::IsaacSimException;

    void validatePrimPath(const std::string& primPath)
    {
        if (!isValid(primPath))
        {
            throw PrimPathError(primPath);
        }
    }

    try
    {
        validatePrimPath("/World/does_not_exist");
    }
    catch (const IsaacSimException& error)
    {
        // Handles PrimPathError, PrimPathStringError, AttributeNameError, ValueTypeError, and any future Isaac Sim
        // exception.
    }

``IsaacSimException`` derives from ``std::exception``, so ``catch (const std::exception&)`` also works. Each subclass
exposes accessors for the structured data used to build its ``what()`` message:

.. code-block:: cpp

    using isaacsim::common::exceptions::ValueTypeError;

    try
    {
        throw ValueTypeError("scale", "float", "str");
    }
    catch (const ValueTypeError& error)
    {
        // error.getAttributeName() == "scale"
        // error.getExpectedType() == "float"
        // error.getActualType() == "str"
    }

The declarations and Doxygen comments in ``Exceptions.hpp`` are the authoritative C++ reference.
