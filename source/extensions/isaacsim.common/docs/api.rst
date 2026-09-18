..
   Copyright (c) 2026, NVIDIA CORPORATION. All rights reserved.
   NVIDIA CORPORATION and its licensors retain all intellectual property
   and proprietary rights in and to this software, related documentation
   and any modifications thereto. Any use, reproduction, disclosure or
   distribution of this software and related documentation without an express
   license agreement from NVIDIA CORPORATION is strictly prohibited.

===============================
Isaac Sim Common extension API
===============================

The extension provides :mod:`isaacsim.common.logging` and :mod:`isaacsim.common.profiling` to dependent extensions.
Use :class:`isaacsim.common.logging.Logger` for application logging and
:func:`isaacsim.common.profiling.zone` for profiler-visible regions of work.
