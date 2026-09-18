..
   Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

.. _isaacsim-foundation-usd-ovstage:
.. _isaacsim-foundation-usd-ovstage-overview:

===============================
isaacsim.foundation.usd.ovstage
===============================

``isaacsim.foundation.usd.ovstage`` is a thin C++ facade over ovstage.
It exposes stage lifecycle, prim authoring, schema queries,
and attribute I/O through a stable ABI based on opaque integer
stage handles and standard C++ types. Callers never hold raw
``ovstage_instance_t`` or ``ovx_primpath_t`` instances across the boundary,
so they do not need to link against ovstage directly.

.. toctree::
    :maxdepth: 2

    api_cpp
