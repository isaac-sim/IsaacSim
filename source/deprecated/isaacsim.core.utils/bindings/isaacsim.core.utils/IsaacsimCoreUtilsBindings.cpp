// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <carb/BindingsPythonUtils.h>

#include <isaacsim/core/includes/Conversions.h>
#include <isaacsim/core/includes/Math.h>
#include <isaacsim/core/includes/Transforms.h>
#include <isaacsim/core/utils/PrimUtils.h>

CARB_BINDINGS("isaacsim.core.utils.python")


namespace
{
PYBIND11_MODULE(_isaac_utils, m)
{
    using namespace carb;
    using namespace isaacsim::core::includes::math;
    using namespace isaacsim::core::utils;
    // We use carb data types, must import bindings for them
    auto carbModule = py::module::import("carb");

    m.def("_find_matching_prim_paths", &findMatchingPrimPaths);

    auto math = m.def_submodule("math");

    math.doc() =
        R"pbdoc( 
            
        Math Utils
        -----------

        This submodule provides math bindings for vector operations, and other facilitators such as `lerp` functions.
            
        
        )pbdoc";

    // Basic operations between types (Add, Sub, Mul)
    math.def(
        "mul", [](const carb::Float3& a, float x) { return a * x; }, py::is_operator(),
        R"pbdoc( Scales a 3D vector by a given value
        
        Args:
            arg0 (:obj:`carb.Float3`): 3D vector

            arg1 (:obj:`float`): scale factor

        Returns:
            :obj:`carb.Float3`: scaled vector.
        )pbdoc");
    math.def(
        "mul", [](const carb::Float4& a, float x) { return a * x; }, py::is_operator(),
        R"pbdoc( Scales a 4D vector by a given value
        
        Args:
            arg0 (:obj:`carb.Float4`): 4D vector

            arg1 (:obj:`float`): scale factor

        Returns:
            :obj:`carb.Float4`: scaled vector.
        )pbdoc");
    math.def(
        "mul", [](const carb::Float4& a, const carb::Float4& x) { return a * x; }, py::is_operator(),
        R"pbdoc( Performs a Quaternion rotation between two 4D vectors
        
        Args:
            arg0 (:obj:`carb.Float4`): first 4D quaternion vector

            arg1 (:obj:`carb.Float4`): second 4D quaternion vector

        Returns:
            :obj:`carb.Float4`: rotated 4D quaternion vector.
        )pbdoc");

    math.def(
        "add", [](const carb::Float3& a, const carb::Float3& x) { return a + x; }, py::is_operator(),
        R"pbdoc( Adds two 3D vectors
        Args:
            arg0 (:obj:`carb.Float3`): First 3D vector

            arg1 (:obj:`carb.Float3`): Second 3D vector

        Returns:

            :obj:`carb.Float3`: ``arg0 + arg1``.
        )pbdoc");

    // Vector and transform operations
    math.def("cross", &isaacsim::core::includes::math::cross,
             R"pbdoc(
             Performs Cross product between 3D vectors
             Args:
                 arg0 (:obj:`carb.Float3`): 3D vector

                 arg1 (:obj:`carb.Float3`): 3D vector

             Returns:

                :obj:`carb.Float3`: cross poduct ``arg0 x arg1``.
             )pbdoc");

    math.def("dot", py::overload_cast<const carb::Float3&, const carb::Float3&>(&isaacsim::core::includes::math::dot),
             R"pbdoc(Performs Dot product between 3D vectors
             Args:
                 arg0 (:obj:`carb.Float3`): 3D vector

                 arg1 (:obj:`carb.Float3`): 3D vector

             Returns:

                 :obj:`carb.Float3`: dot poduct ``arg0 * arg1``.
             )pbdoc");

    math.def("dot", py::overload_cast<const carb::Float4&, const carb::Float4&>(&isaacsim::core::includes::math::dot),
             R"pbdoc(Performs Dot product between 4D vectors
             Args:
                 arg0 (:obj:`carb.Float4`): 4D vector

                 arg1 (:obj:`carb.Float4`): 4D vector

             Returns:

                 :obj:`carb.Float4`: dot poduct ``arg0 * arg1``.

             )pbdoc");
    math.def("inverse", py::overload_cast<const carb::Float4&>(&isaacsim::core::includes::math::inverse),
             R"pbdoc(Gets Inverse Quaternion
             Args:

                 arg0 (:obj:`carb.Float4`): quaternion

             Returns:
             
                 :obj:`carb.Float4`: The inverse quaternion

             )pbdoc");

    math.def("normalize", py::overload_cast<const carb::Float3&>(&isaacsim::core::includes::math::normalize),
             R"pbdoc(
                Gets normalized 3D vector

                Args:

                    arg0 (:obj:`carb.Float3`): 3D Vector

                Returns:

                    :obj:`carb.Float3`: Normalized 3D Vector
                    
                )pbdoc");
    math.def("normalize", py::overload_cast<const carb::Float4&>(&isaacsim::core::includes::math::normalize),
             R"pbdoc(
                Gets normalized 4D vector
                Args:

                    arg0 (:obj:`carb.Float4`): 4D Vector
                    
                Returns:

                    :obj:`carb.Float4`: Normalized 4D Vector
                )pbdoc");
    math.def("rotate", isaacsim::core::includes::math::rotate,
             R"pbdoc(
                rotates the 3D vector arg1 by the quaternion `arg0`

                Args:
                
                    arg0 (:obj:`carb.Float4`): quaternion

                    arg1 (:obj:`carb.Float3`): 3D Vector

                Returns:

                    :obj:`carb.Float3`: Rotated 3D Vector
                )pbdoc");

    math.def("transform_inv",
             py::overload_cast<const pxr::GfTransform&, const pxr::GfTransform&>(
                 &isaacsim::core::includes::math::transformInv),
             R"pbdoc(
                Computes local Transform of arg1 with respect to arg0: `inv(arg0)*arg1`

                Args:
                
                    arg0 (`pxr.Transform`): origin Transform

                    arg1 (`pxr.Transform`): Transform

                Returns:

                    :obj:`oxr.Transform`: resulting transform of ``inv(arg0)*arg1``
                )pbdoc");


    // Utility functions
    math.def("get_basis_vector_x", &isaacsim::core::includes::math::getBasisVectorX,
             R"pbdoc(
                Gets Basis vector X of quaternion

                Args:

                    arg0 (:obj:`carb.Float4`): Quaternion

                Returns:

                    :obj:`carb.Float3`: Basis Vector X
                    
                )pbdoc");
    math.def("get_basis_vector_y", &isaacsim::core::includes::math::getBasisVectorY,
             R"pbdoc(
                Gets Basis vector Y of quaternion

                Args:

                    arg0 (:obj:`carb.Float4`): Quaternion

                Returns:

                    :obj:`carb.Float3`: Basis Vector Y
                    
                )pbdoc");
    math.def("get_basis_vector_z", &isaacsim::core::includes::math::getBasisVectorZ,
             R"pbdoc(
                Gets Basis vector Z of quaternion

                Args:

                    arg0 (:obj:`carb.Float4`): Quaternion

                Returns:

                    :obj:`carb.Float3`: Basis Vector Z
                    
                )pbdoc");

    math.def(
        "lerp",
        py::overload_cast<const carb::Float3&, const carb::Float3&, const float>(&isaacsim::core::includes::math::lerp),
        R"pbdoc(
                Performs Linear interpolation between points arg0 and arg1

                Args:

                    arg0 (:obj:`carb.Float3`): Point

                    arg1 (:obj:`carb.Float3`): Point

                    arg2 (:obj:`float`): distance from 0 to 1, where 0 is closest to arg0, and 1 is closest to arg1

                Returns:

                    :obj:`carb.Float3`: Interpolated point
                    
                )pbdoc");
    math.def(
        "lerp",
        py::overload_cast<const carb::Float4&, const carb::Float4&, const float>(&isaacsim::core::includes::math::lerp),
        R"pbdoc(
                Performs Linear interpolation between quaternions arg0 and arg1

                Args:

                    arg0 (:obj:`carb.Float4`): Quaternion

                    arg1 (:obj:`carb.Float4`): Quaternion

                    arg2 (:obj:`float`): distance from 0 to 1, where 0 is closest to arg0, and 1 is closest to arg1

                Returns:

                    :obj:`carb.Float4`: Interpolated quaternion
                    
                )pbdoc");

    math.def(
        "slerp",
        py::overload_cast<const carb::Float4&, const carb::Float4&, const float>(&isaacsim::core::includes::math::slerp),
        R"pbdoc(
                Performs Spherical Linear interpolation between quaternions arg0 and arg1

                Args:

                    arg0 (:obj:`carb.Float4`): Quaternion

                    arg1 (:obj:`carb.Float4`): Quaternion

                    arg2 (:obj:`float`): distance from 0 to 1, where 0 is closest to arg0, and 1 is closest to arg1

                Returns:

                    :obj:`carb.Float4`: Interpolated quaternion
                    
                )pbdoc");

    auto transforms = m.def_submodule("transforms");

    transforms.def(
        "set_transform",
        [](const long int stageId, const std::string& primPath, const carb::Float3& translation,
           const carb::Float4& rotation)
        {
            pxr::UsdStageWeakPtr stage =
                pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

            pxr::UsdPrim prim = stage->GetPrimAtPath(pxr::SdfPath(primPath));

            if (prim)
            {

                isaacsim::core::includes::transforms::setTransform(
                    prim, isaacsim::core::includes::conversions::asGfVec3f(translation),
                    isaacsim::core::includes::conversions::asGfQuatf(rotation));
            }
            else
            {
                CARB_LOG_ERROR("Set Transform Prim %s Not Valid", primPath.c_str());
            }
        },
        R"pbdoc(
                Set transform for an object in the stage, handles physics objects if simulation is running using dynamic control

                Args:

                    stageId (:obj:`int`): Stage ID
                    
                    primPath (:obj:`str`): Path to the prim
                    
                    translation (:obj:`carb.Float3`): Translation vector
                    
                    rotation (:obj:`carb.Float4`): Rotation quaternion
                    
                )pbdoc");

    transforms.def(
        "set_scale",
        [](const long int stageId, const std::string& primPath, const carb::Float3& scale)
        {
            pxr::UsdStageWeakPtr stage =
                pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

            pxr::UsdPrim prim = stage->GetPrimAtPath(pxr::SdfPath(primPath));

            if (prim)
            {

                isaacsim::core::includes::transforms::setScale(
                    prim, isaacsim::core::includes::conversions::asGfVec3f(scale));
            }
            else
            {
                CARB_LOG_ERROR("Set Scale Prim %s Not Valid", primPath.c_str());
            }

            // return new
            // // MapGenerator(physXPtr, stage);
        },
        R"pbdoc(
                Set scale for an object in the stage

                Args:

                    stageId (:obj:`int`): Stage ID
                    
                    primPath (:obj:`str`): Path to the prim
                    
                    scale (:obj:`carb.Float3`): Scale vector
                    
                )pbdoc");
}
}
