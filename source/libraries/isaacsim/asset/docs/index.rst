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

.. _isaacsim-asset-library:

==============
isaacsim-asset
==============

The ``isaacsim-asset`` distribution (CMake group ``isaacsim_asset``) provides Python
tools for transforming Universal Scene Description (USD) assets and implementing asset
importers. Use its transformation rules to apply repeatable changes to USD assets and
its importer utilities to share common importer behavior.

The distribution keeps reusable asset processing separate from application-specific
workflows. Its modules can be installed and used independently of the legacy Isaac Sim
extension APIs.
