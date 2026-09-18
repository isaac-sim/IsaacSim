// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "gl/Scene.h"

#include <ovstage/ovstage.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* CPU expansion deep-copies prototype geometry for every generated draw.  The
     * environment override may lower or raise the operational default, but it may
     * never bypass this reviewed process-wide safety ceiling. */
    enum
    {
        OVGL_POINT_INSTANCER_DEFAULT_CPU_LIMIT = 20000,
        OVGL_POINT_INSTANCER_ABSOLUTE_CPU_LIMIT = 100000,
    };

    /* Expand portable ovstage PointInstancers into heap-owned SceneMesh draws.
     * The base scene must already contain every ordinary Gprim, including the
     * relationship targets used as prototypes.  External prototypes remain
     * ordinary draws; prototypes below their owning PointInstancer are hidden and
     * exist only through the generated placements. */
    int ovgl_expand_point_instancers(ovstage_instance_t* stage, ovstage_ordinal_t ordinal, Scene* scene);

    const char* ovgl_point_instancer_last_error(void);

#ifdef __cplusplus
}
#endif
