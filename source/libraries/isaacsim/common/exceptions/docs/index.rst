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

.. _isaacsim-common-exceptions:
.. _isaacsim-common-exceptions-overview:

==========================
isaacsim.common.exceptions
==========================

``isaacsim.common.exceptions`` defines the C++ and Python exception hierarchy raised by Isaac Sim's own validation
code. Every exception derives from a common ``IsaacSimException`` base, so callers can catch any Isaac Sim exception
generically, or catch a specific subtype to inspect the structured data it carries.

Four concrete exceptions are provided:

- ``PrimPathError`` --- a USD prim path does not refer to a usable prim. Carries the offending path.
- ``PrimPathStringError`` --- a string does not parse as a valid USD prim path. Carries the offending string.
- ``AttributeNameError`` --- a caller referenced an attribute name that does not exist or is not accepted. Carries the
  offending attribute name.
- ``ValueTypeError`` --- a value's type does not match the type expected for an attribute. Carries the attribute name,
  the expected type, and the actual type.

Each exception formats a human-readable message from its fields, available through ``what()`` in C++ and
``str(exception)`` in Python. The C++ base derives from ``std::exception``, so callers may also catch
``std::exception`` when they only need a generic failure signal.

.. _isaacsim-common-exceptions-python-usage:

Python usage
------------

.. code-block:: python

    from isaacsim.common.exceptions import PrimPathError, IsaacSimException

    try:
        ...
    except PrimPathError as error:
        print(error.prim_path)
    except IsaacSimException:
        # Handles any other Isaac Sim exception.
        raise

See :doc:`api_cpp` and :doc:`api_python` for language-specific references.

.. toctree::
    :maxdepth: 2

    api_cpp
    api_python
