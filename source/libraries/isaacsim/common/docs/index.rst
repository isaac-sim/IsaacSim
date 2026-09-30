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

.. _isaacsim-common-library:

===============
isaacsim-common
===============

The ``isaacsim-common`` distribution (CMake group ``isaacsim_common``) contains shared
infrastructure used by the standalone Isaac Sim libraries. It provides common array
and device types, exception types, logging, profiling, and string utilities for native
and Python code. The ``isaacsim.common.ovstage`` module defines the shared native
OVStage transform-journal contract.

Use these modules when multiple libraries need the same low-level contracts. The
distribution does not depend on another Isaac Sim library distribution.
