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

#pragma once
#include <carb/Types.h>

#include <usdrt/gf/matrix.h>
#include <usdrt/gf/quat.h>
#include <usdrt/gf/vec.h>

#include <PxActor.h>

namespace isaacsim
{
namespace core
{
namespace includes
{

namespace conversions
{
/**
 * @brief Converts a carb::Float3 into a pxr::GfVec3f.
 * @details Performs a direct component-wise conversion from Carb's Float3 to USD's GfVec3f.
 *
 * @param[in] v Input vector in Carb format
 * @return pxr::GfVec3f Equivalent vector in USD format
 *
 * @note No precision loss as both types use single precision
 */
inline pxr::GfVec3f asGfVec3f(const carb::Float3& v)
{
    return pxr::GfVec3f(v.x, v.y, v.z);
}

/**
 * @brief Converts a carb::Float4 into a pxr::GfQuatf.
 * @details
 * Converts a quaternion from Carb to USD format, handling component reordering:
 * - Carb format: (x, y, z, w)
 * - USD format: (w, x, y, z)
 *
 * @param[in] q Input quaternion in Carb format
 * @return pxr::GfQuatf Equivalent quaternion in USD format
 *
 * @note Component order is adjusted during conversion
 */
inline pxr::GfQuatf asGfQuatf(const carb::Float4& q)
{
    return pxr::GfQuatf(q.w, q.x, q.y, q.z);
}

/**
 * @brief Converts a carb::Float3 into a pxr::GfVec3d.
 * @details
 * Performs a component-wise conversion with promotion to double precision:
 * - x, y, z components are converted from float to double
 *
 * @param[in] v Input vector in Carb format (single precision)
 * @return pxr::GfVec3d Equivalent vector in USD format (double precision)
 *
 * @note Involves precision promotion from float to double
 */
inline pxr::GfVec3d asGfVec3d(const carb::Float3& v)
{
    return pxr::GfVec3d(v.x, v.y, v.z);
}

/**
 * @brief Converts a carb::Float4 into a pxr::GfQuatd.
 * @details
 * Converts a quaternion from Carb to USD format with precision promotion:
 * - Handles component reordering (x,y,z,w) -> (w,x,y,z)
 * - Promotes single precision to double precision
 *
 * @param[in] q Input quaternion in Carb format (single precision)
 * @return pxr::GfQuatd Equivalent quaternion in USD format (double precision)
 *
 * @note Combines component reordering with precision promotion
 */
inline pxr::GfQuatd asGfQuatd(const carb::Float4& q)
{
    return pxr::GfQuatd(q.w, q.x, q.y, q.z);
}

/**
 * @brief Converts a carb::Float4 into a pxr::GfRotation.
 * @details
 * Creates a USD rotation representation from a Carb quaternion:
 * 1. Converts the quaternion to GfQuatd format
 * 2. Constructs a GfRotation from the quaternion
 *
 * @param[in] q Input quaternion in Carb format
 * @return pxr::GfRotation Equivalent rotation in USD format
 *
 * @see asGfQuatd
 */
inline pxr::GfRotation asGfRotation(const carb::Float4& q)
{
    return pxr::GfRotation(asGfQuatd(q));
}


/**
 * @brief Converts position and rotation into a pxr::GfTransform.
 * @details
 * Creates a complete transform from separate position and rotation:
 * 1. Sets rotation using quaternion conversion
 * 2. Sets translation using vector conversion
 *
 * @param[in] p Position vector in Carb format
 * @param[in] r Rotation quaternion in Carb format
 * @return pxr::GfTransform Equivalent transform in USD format
 *
 * @see asGfRotation
 * @see asGfVec3d
 */
inline pxr::GfTransform asGfTransform(const carb::Float3& p, const carb::Float4& r)
{
    pxr::GfTransform trans;
    trans.SetRotation(asGfRotation(r));
    trans.SetTranslation(asGfVec3d(p));
    return trans;
}


/**
 * @brief Converts pxr::GfVec3f to carb::Float3.
 * @details
 * Performs direct component-wise conversion from USD to Carb format.
 *
 * @param[in] v Input vector in USD format
 * @return carb::Float3 Equivalent vector in Carb format
 *
 * @note No precision loss as both types use single precision
 */
inline carb::Float3 asCarbFloat3(const pxr::GfVec3f& v)
{
    return carb::Float3{ v[0], v[1], v[2] };
}

/**
 * @brief Converts pxr::GfVec3d to carb::Float3.
 * @details
 * Performs component-wise conversion with precision demotion:
 * - x, y, z components are converted from double to float
 *
 * @param[in] v Input vector in USD format (double precision)
 * @return carb::Float3 Equivalent vector in Carb format (single precision)
 *
 * @warning Potential precision loss during double to float conversion
 */
inline carb::Float3 asCarbFloat3(const pxr::GfVec3d& v)
{
    return carb::Float3{ static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]) };
}


/**
 * @brief convert physx::PxQuat to carb::Float4
 *
 * @param[in] q Input quaternion in PhysX format
 * @return carb::Float4 Equivalent quaternion in Carb format
 *
 * @note Component order is adjusted during conversion
 */
inline carb::Float4 asCarbFloat4(const physx::PxQuat& q)
{
    return carb::Float4{ q.x, q.y, q.z, q.w };
}


/**
 * @brief Converts pxr::GfQuatf to carb::Float4.
 * @details
 * Converts quaternion with component reordering:
 * - USD format: (w, x, y, z)
 * - Carb format: (x, y, z, w)
 *
 * @param[in] v Input quaternion in USD format
 * @return carb::Float4 Equivalent quaternion in Carb format
 *
 * @note Component order is adjusted during conversion
 */
inline carb::Float4 asCarbFloat4(const pxr::GfQuatf& v)
{
    const pxr::GfVec3f& imag = v.GetImaginary();
    return carb::Float4{ imag[0], imag[1], imag[2], v.GetReal() };
}

/**
 * @brief Converts pxr::GfQuatd to carb::Float4.
 * @details
 * Converts quaternion with component reordering and precision demotion:
 * 1. Extracts imaginary and real parts
 * 2. Converts from double to float
 * 3. Reorders components from USD to Carb format
 *
 * @param[in] v Input quaternion in USD format (double precision)
 * @return carb::Float4 Equivalent quaternion in Carb format (single precision)
 *
 * @warning Potential precision loss during double to float conversion
 */
inline carb::Float4 asCarbFloat4(const pxr::GfQuatd& v)
{
    const pxr::GfVec3d& imag = v.GetImaginary();
    return carb::Float4{ static_cast<float>(imag[0]), static_cast<float>(imag[1]), static_cast<float>(imag[2]),
                         static_cast<float>(v.GetReal()) };
}

/**
 * @brief Converts carb::Float3 into PhysX vector.
 * @details
 * Performs direct component-wise conversion to PhysX format.
 *
 * @param[in] v Input vector in Carb format
 * @return PxVec3 Equivalent vector in PhysX format
 *
 * @note No precision loss as both types use single precision
 */
inline ::physx::PxVec3 asPxVec3(const carb::Float3& v)
{
    return ::physx::PxVec3{ v.x, v.y, v.z };
}

/**
 * @brief Converts pxr::GfVec3f into PhysX vector.
 * @details
 * Performs direct component-wise conversion from USD to PhysX format.
 *
 * @param[in] v Input vector in USD format
 * @return PxVec3 Equivalent vector in PhysX format
 *
 * @note No precision loss as both types use single precision
 */
inline ::physx::PxVec3 asPxVec3(const pxr::GfVec3f& v)
{
    return ::physx::PxVec3{ v[0], v[1], v[2] };
}

/**
 * @brief Converts usdrt::GfVec3f into PhysX vector.
 * @details
 * Performs direct component-wise conversion from USD runtime to PhysX format.
 *
 * @param[in] v Input vector in USD runtime format
 * @return PxVec3 Equivalent vector in PhysX format
 *
 * @note No precision loss as both types use single precision
 */
inline ::physx::PxVec3 asPxVec3(const usdrt::GfVec3f& v)
{
    return ::physx::PxVec3{ v[0], v[1], v[2] };
}

/**
 * @brief Converts pxr::GfVec3d into PhysX vector.
 * @details
 * Performs component-wise conversion with precision demotion:
 * - x, y, z components are converted from double to float
 *
 * @param[in] v Input vector in USD format (double precision)
 * @return PxVec3 Equivalent vector in PhysX format (single precision)
 *
 * @warning Potential precision loss during double to float conversion
 */
inline ::physx::PxVec3 asPxVec3(const pxr::GfVec3d& v)
{
    return ::physx::PxVec3{ static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]) };
}

/**
 * @brief Converts usdrt::GfVec3d into PhysX vector.
 * @details
 * Performs component-wise conversion with precision demotion:
 * - x, y, z components are converted from double to float
 *
 * @param[in] v Input vector in USD runtime format (double precision)
 * @return PxVec3 Equivalent vector in PhysX format (single precision)
 *
 * @warning Potential precision loss during double to float conversion
 */
inline ::physx::PxVec3 asPxVec3(const usdrt::GfVec3d& v)
{
    return ::physx::PxVec3{ static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2]) };
}

/**
 * @brief Converts carb::Float4 into PhysX quaternion.
 * @details
 * Converts quaternion with component reordering:
 * - Carb format: (x, y, z, w)
 * - PhysX format: (x, y, z, w)
 *
 * @param[in] q Input quaternion in Carb format
 * @return PxQuat Equivalent quaternion in PhysX format
 *
 * @note Component order matches between formats
 */
inline ::physx::PxQuat asPxQuat(const carb::Float4& q)
{
    return ::physx::PxQuat{ q.x, q.y, q.z, q.w };
}

/**
 * @brief Converts pxr::GfQuatf into PhysX quaternion.
 * @details
 * Converts quaternion with component reordering:
 * - USD format: (w, x, y, z)
 * - PhysX format: (x, y, z, w)
 *
 * @param[in] v Input quaternion in USD format
 * @return PxQuat Equivalent quaternion in PhysX format
 *
 * @note Component order is adjusted during conversion
 */
inline ::physx::PxQuat asPxQuat(const pxr::GfQuatf& v)
{
    const pxr::GfVec3f& imag = v.GetImaginary();
    return ::physx::PxQuat{ imag[0], imag[1], imag[2], v.GetReal() };
}

/**
 * @brief Converts usdrt::GfQuatf into PhysX quaternion.
 * @details
 * Converts quaternion with component reordering:
 * - USD runtime format: (w, x, y, z)
 * - PhysX format: (x, y, z, w)
 *
 * @param[in] v Input quaternion in USD runtime format
 * @return PxQuat Equivalent quaternion in PhysX format
 *
 * @note Component order is adjusted during conversion
 */
inline ::physx::PxQuat asPxQuat(const usdrt::GfQuatf& v)
{
    const usdrt::GfVec3f& imag = v.GetImaginary();
    return ::physx::PxQuat{ imag[0], imag[1], imag[2], v.GetReal() };
}
/**
 * @brief Converts pxr::GfQuatd into PhysX quaternion.
 * @details
 * Converts quaternion with component reordering and precision demotion:
 * 1. Extracts imaginary and real parts
 * 2. Converts from double to float
 * 3. Reorders components from USD runtime to PhysX format
 *
 * @param[in] v Input quaternion in USD runtime format (double precision)
 * @return PxQuat Equivalent quaternion in PhysX format (single precision)
 *
 * @warning Potential precision loss during double to float conversion
 */
inline ::physx::PxQuat asPxQuat(const pxr::GfQuatd& v)
{
    const pxr::GfVec3d& imag = v.GetImaginary();
    return ::physx::PxQuat{ static_cast<float>(imag[0]), static_cast<float>(imag[1]), static_cast<float>(imag[2]),
                            static_cast<float>(v.GetReal()) };
}
/**
 * @brief Converts usdrt::GfQuatd into PhysX quaternion.
 * @details
 * Converts quaternion with component reordering and precision demotion:
 * 1. Extracts imaginary and real parts
 * 2. Converts from double to float
 * 3. Reorders components from USD runtime to PhysX format
 *
 * @param[in] v Input quaternion in USD runtime format (double precision)
 * @return PxQuat Equivalent quaternion in PhysX format (single precision)
 *
 * @warning Potential precision loss during double to float conversion
 */
inline ::physx::PxQuat asPxQuat(const usdrt::GfQuatd& v)
{
    const usdrt::GfVec3d& imag = v.GetImaginary();
    return ::physx::PxQuat{ static_cast<float>(imag[0]), static_cast<float>(imag[1]), static_cast<float>(imag[2]),
                            static_cast<float>(v.GetReal()) };
}


/**
 * @brief Converts USD transform into PhysX transform.
 * @details
 * Creates a complete PhysX transform from USD transform:
 * 1. Extracts translation and rotation components
 * 2. Converts to PhysX format with potential precision demotion
 *
 * @param[in] trans Input transform in USD format
 * @return PxTransform Equivalent transform in PhysX format
 *
 * @warning Potential precision loss when converting from double precision USD types
 * @see asPxVec3
 * @see asPxQuat
 */
inline ::physx::PxTransform asPxTransform(const pxr::GfTransform& trans)
{
    ::physx::PxTransform p;
    const pxr::GfVec3d& pos = trans.GetTranslation();
    const pxr::GfQuatd& rot = trans.GetRotation().GetQuat();

    p.p.x = static_cast<float>(pos[0]);
    p.p.y = static_cast<float>(pos[1]);
    p.p.z = static_cast<float>(pos[2]);
    p.q.x = static_cast<float>(rot.GetImaginary()[0]);
    p.q.y = static_cast<float>(rot.GetImaginary()[1]);
    p.q.z = static_cast<float>(rot.GetImaginary()[2]);
    p.q.w = static_cast<float>(rot.GetReal());
    return p;
}

/**
 * @brief Converts USD runtime matrix into PhysX transform.
 * @details
 * Creates a PhysX transform from a USD runtime 4x4 matrix:
 * 1. Creates a GfTransform from the matrix
 * 2. Extracts translation and rotation components
 * 3. Converts to PhysX format with potential precision demotion
 *
 * @param[in] mat Input 4x4 matrix in USD runtime format
 * @return PxTransform Equivalent transform in PhysX format
 *
 * @warning Potential precision loss when converting from double precision USD types
 * @see asPxTransform(const pxr::GfTransform&)
 */
inline ::physx::PxTransform asPxTransform(const usdrt::GfMatrix4d& mat)
{
    ::physx::PxTransform p;
    const pxr::GfMatrix4d* gfMat = reinterpret_cast<const pxr::GfMatrix4d*>(&mat);
    pxr::GfTransform trans(*gfMat);
    const pxr::GfVec3d& pos = trans.GetTranslation();
    const pxr::GfQuatd& rot = trans.GetRotation().GetQuat();

    p.p.x = static_cast<float>(pos[0]);
    p.p.y = static_cast<float>(pos[1]);
    p.p.z = static_cast<float>(pos[2]);
    p.q.x = static_cast<float>(rot.GetImaginary()[0]);
    p.q.y = static_cast<float>(rot.GetImaginary()[1]);
    p.q.z = static_cast<float>(rot.GetImaginary()[2]);
    p.q.w = static_cast<float>(rot.GetReal());
    return p;
}

/**
 * @brief Converts USD runtime translation and orientation into PhysX transform.
 * @details
 * Creates a PhysX transform from separate position and orientation:
 * 1. Converts translation vector to PxVec3
 * 2. Converts orientation quaternion to PxQuat
 *
 * @param[in] translation Position vector in USD runtime format
 * @param[in] orientation Rotation quaternion in USD runtime format
 * @return PxTransform Equivalent transform in PhysX format
 *
 * @warning Potential precision loss when converting from double precision USD types
 * @see asPxVec3
 * @see asPxQuat
 */
inline ::physx::PxTransform asPxTransform(const usdrt::GfVec3d& translation, const usdrt::GfQuatd& orientation)
{
    ::physx::PxTransform p;
    p.p.x = static_cast<float>(translation[0]);
    p.p.y = static_cast<float>(translation[1]);
    p.p.z = static_cast<float>(translation[2]);
    p.q.x = static_cast<float>(orientation.GetImaginary()[0]);
    p.q.y = static_cast<float>(orientation.GetImaginary()[1]);
    p.q.z = static_cast<float>(orientation.GetImaginary()[2]);
    p.q.w = static_cast<float>(orientation.GetReal());
    return p;
}

}
}
}
}
