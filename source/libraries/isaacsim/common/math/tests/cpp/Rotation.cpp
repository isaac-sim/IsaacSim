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

#include <doctest/doctest.h>
#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/common/array/Dtype.hpp>
#include <isaacsim/common/array/Functions.hpp>
#include <isaacsim/common/math/Rotation.hpp>

#define _USE_MATH_DEFINES
#include <cmath>
#include <stdexcept>
#include <vector>

using isaacsim::common::array::Array;
using isaacsim::common::array::Device;
using isaacsim::common::array::Dtype;
using isaacsim::common::array::Shape;
namespace math = isaacsim::common::math;

#define SKIP_IF_CUDA_UNAVAILABLE(device)                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        if ((device) == Device::Cuda() && !(device).isAvailable())                                                     \
        {                                                                                                              \
            MESSAGE("Skipping: no CUDA device available (" << (device).toString() << ")");                             \
            return;                                                                                                    \
        }                                                                                                              \
    } while (false)

namespace
{

//! Half the angle of a quarter turn, which is what a quaternion's components carry.
const double quarterTurnAngle = std::atan(1.0);

//! A quarter turn about Z, ordered `xyzw`.
const std::vector<double> quarterTurnAboutZ{ 0.0, 0.0, std::sin(quarterTurnAngle), std::cos(quarterTurnAngle) };
//! A quarter turn about Y, ordered `xyzw`.
const std::vector<double> quarterTurnAboutY{ 0.0, std::sin(quarterTurnAngle), 0.0, std::cos(quarterTurnAngle) };
//! The identity rotation, ordered `xyzw`.
const std::vector<double> identityRotation{ 0.0, 0.0, 0.0, 1.0 };

//! Checks every element of a result against expected values.
void checkClose(const Array& result, const std::vector<double>& expected, double tolerance = 1.0e-12)
{
    // Flattened first: get<std::vector<double>>() takes a 1-D array, and batched results are 2-D.
    const std::vector<double> values = result.toDevice(Device::Cpu()).flatten().get<std::vector<double>>();
    REQUIRE_EQ(values.size(), expected.size());
    for (size_t index = 0; index < expected.size(); ++index)
    {
        CHECK(std::abs(values[index] - expected[index]) <= tolerance);
    }
}

} // namespace

TEST_SUITE("Rotation")
{

    TEST_CASE("normalizeQuaternion()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Scales to unit length and keeps the operand's device")
        {
            Array q(std::vector<double>{ 0.0, 0.0, 0.0, 2.0 }, Dtype::Float64(), device);
            Array result = math::normalizeQuaternion(q);
            CHECK_EQ(result.device(), device);
            CHECK_EQ(result.dtype(), Dtype::Float64());
            checkClose(result, identityRotation);
        }

        SUBCASE("A zero quaternion normalizes to the identity, not to NaNs")
        {
            Array q(std::vector<double>{ 0.0, 0.0, 0.0, 0.0 }, Dtype::Float64(), device);
            checkClose(math::normalizeQuaternion(q), identityRotation);
        }

        SUBCASE("Batches keep their leading axis")
        {
            Array q(std::vector<std::vector<double>>{ { 0.0, 0.0, 0.0, 2.0 }, { 0.0, 0.0, 0.0, -3.0 } },
                    Dtype::Float64(), device);
            Array result = math::normalizeQuaternion(q);
            CHECK_EQ(result.shape(), Shape({ 2, 4 }));
            checkClose(result, { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0 });
        }
    }

    TEST_CASE("multiplyQuaternions()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Composition does not commute")
        {
            Array z(quarterTurnAboutZ, Dtype::Float64(), device);
            Array y(quarterTurnAboutY, Dtype::Float64(), device);
            const std::vector<double> forward =
                math::multiplyQuaternions(z, y).toDevice(Device::Cpu()).flatten().get<std::vector<double>>();
            const std::vector<double> reverse =
                math::multiplyQuaternions(y, z).toDevice(Device::Cpu()).flatten().get<std::vector<double>>();
            CHECK_NE(forward, reverse);
        }

        SUBCASE("The right operand applies first")
        {
            // Rotating +X by (Z after Y) must match rotating it by Y and then by Z.
            Array z(quarterTurnAboutZ, Dtype::Float64(), device);
            Array y(quarterTurnAboutY, Dtype::Float64(), device);
            Array probe(std::vector<double>{ 1.0, 0.0, 0.0 }, Dtype::Float64(), device);
            Array composed = math::rotateByQuaternion(math::multiplyQuaternions(z, y), probe);
            Array stepwise = math::rotateByQuaternion(z, math::rotateByQuaternion(y, probe));
            checkClose(composed, stepwise.toDevice(Device::Cpu()).flatten().get<std::vector<double>>(), 1.0e-12);
        }

        SUBCASE("The identity leaves a rotation unchanged")
        {
            Array q(quarterTurnAboutZ, Dtype::Float64(), device);
            Array identity(identityRotation, Dtype::Float64(), device);
            checkClose(math::multiplyQuaternions(q, identity), quarterTurnAboutZ);
        }
    }

    TEST_CASE("rotateByQuaternion() and rotateByQuaternionInverse()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("A quarter turn about Z takes +X onto +Y")
        {
            Array q(quarterTurnAboutZ, Dtype::Float64(), device);
            Array v(std::vector<double>{ 1.0, 0.0, 0.0 }, Dtype::Float64(), device);
            checkClose(math::rotateByQuaternion(q, v), { 0.0, 1.0, 0.0 });
        }

        SUBCASE("The inverse undoes the rotation")
        {
            Array q(quarterTurnAboutZ, Dtype::Float64(), device);
            Array v(std::vector<double>{ 0.3, -0.7, 1.1 }, Dtype::Float64(), device);
            checkClose(math::rotateByQuaternionInverse(q, math::rotateByQuaternion(q, v)), { 0.3, -0.7, 1.1 });
        }

        SUBCASE("An unnormalized quaternion rotates as its unit form does")
        {
            Array unit(quarterTurnAboutZ, Dtype::Float64(), device);
            std::vector<double> scaled = quarterTurnAboutZ;
            for (double& component : scaled)
            {
                component *= 5.0;
            }
            Array v(std::vector<double>{ 1.0, 2.0, 3.0 }, Dtype::Float64(), device);
            checkClose(math::rotateByQuaternion(Array(scaled, Dtype::Float64(), device), v),
                       math::rotateByQuaternion(unit, v).toDevice(Device::Cpu()).flatten().get<std::vector<double>>());
        }

        SUBCASE("One quaternion broadcasts across a batch of vectors")
        {
            Array q(quarterTurnAboutZ, Dtype::Float64(), device);
            Array v(std::vector<std::vector<double>>{ { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 } }, Dtype::Float64(), device);
            Array result = math::rotateByQuaternion(q, v);
            CHECK_EQ(result.shape(), Shape({ 2, 3 }));
            checkClose(result, { 0.0, 1.0, 0.0, -1.0, 0.0, 0.0 });
        }
    }

    TEST_CASE("dotQuaternions()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("A quaternion against itself is its squared norm")
        {
            Array q(quarterTurnAboutZ, Dtype::Float64(), device);
            CHECK(std::abs(math::dotQuaternions(q, q).toDevice(Device::Cpu()).item<double>() - 1.0) <= 1.0e-12);
        }

        SUBCASE("A quaternion against its negation is negative")
        {
            Array q(quarterTurnAboutZ, Dtype::Float64(), device);
            Array negated = isaacsim::common::array::negative(q);
            CHECK(math::dotQuaternions(q, negated).toDevice(Device::Cpu()).item<double>() < 0.0);
        }
    }

    TEST_CASE("averageQuaternions()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("Averaging one rotation returns it")
        {
            Array q(std::vector<std::vector<double>>{ quarterTurnAboutZ }, Dtype::Float64(), device);
            checkClose(math::averageQuaternions(q), quarterTurnAboutZ);
        }

        SUBCASE("Signs are aligned, so a quaternion and its negation do not cancel")
        {
            std::vector<double> negated = quarterTurnAboutZ;
            for (double& component : negated)
            {
                component = -component;
            }
            Array window(std::vector<std::vector<double>>{ quarterTurnAboutZ, negated }, Dtype::Float64(), device);
            // Without alignment this sums to zero and normalizes to the identity.
            checkClose(math::averageQuaternions(window), quarterTurnAboutZ);
        }

        SUBCASE("The mean of a window lies between its ends")
        {
            Array window(
                std::vector<std::vector<double>>{ identityRotation, quarterTurnAboutZ }, Dtype::Float64(), device);
            Array mean = math::averageQuaternions(window);
            const double toIdentity = math::dotQuaternions(mean, Array(identityRotation, Dtype::Float64(), device))
                                          .toDevice(Device::Cpu())
                                          .item<double>();
            const double toQuarter = math::dotQuaternions(mean, Array(quarterTurnAboutZ, Dtype::Float64(), device))
                                         .toDevice(Device::Cpu())
                                         .item<double>();
            CHECK(std::abs(toIdentity - toQuarter) <= 1.0e-12);
        }
    }

    TEST_CASE("conjugateQuaternion() and inverseQuaternion()")
    {
        auto device = GENERATE(Device::Cpu(), Device::Cuda());
        SKIP_IF_CUDA_UNAVAILABLE(device);

        SUBCASE("The conjugate negates the vector part and keeps the scalar")
        {
            Array q(std::vector<double>{ 0.1, 0.2, 0.3, 0.4 }, Dtype::Float64(), device);
            checkClose(math::conjugateQuaternion(q), { -0.1, -0.2, -0.3, 0.4 });
        }

        SUBCASE("For a unit quaternion the inverse is the conjugate")
        {
            Array q(quarterTurnAboutZ, Dtype::Float64(), device);
            checkClose(math::inverseQuaternion(q),
                       math::conjugateQuaternion(q).toDevice(Device::Cpu()).flatten().get<std::vector<double>>());
        }

        SUBCASE("An unnormalized quaternion still inverts to something that undoes it")
        {
            Array q(std::vector<double>{ 0.0, 0.0, 2.0, 2.0 }, Dtype::Float64(), device);
            checkClose(math::multiplyQuaternions(q, math::inverseQuaternion(q)), identityRotation, 1.0e-12);
        }

        SUBCASE("A zero quaternion inverts to the identity rather than dividing by zero")
        {
            Array q(std::vector<double>{ 0.0, 0.0, 0.0, 0.0 }, Dtype::Float64(), device);
            checkClose(math::inverseQuaternion(q), identityRotation);
        }
    }

    TEST_CASE("Empty batches")
    {
        SUBCASE("An empty batch averages to the identity rather than reading off the end")
        {
            Array none = isaacsim::common::array::take(
                Array(std::vector<std::vector<double>>{ identityRotation }, Dtype::Float64(), Device::Cpu()),
                std::vector<int64_t>{}, 0);
            REQUIRE_EQ(none.shape(), Shape({ 0, 4 }));
            checkClose(math::averageQuaternions(none), identityRotation);
        }

        SUBCASE("The element-wise operations return a well-formed empty result")
        {
            Array none = isaacsim::common::array::take(
                Array(std::vector<std::vector<double>>{ identityRotation }, Dtype::Float64(), Device::Cpu()),
                std::vector<int64_t>{}, 0);
            CHECK_EQ(math::normalizeQuaternion(none).shape(), Shape({ 0, 4 }));
            CHECK_EQ(math::conjugateQuaternion(none).shape(), Shape({ 0, 4 }));
        }
    }

    TEST_CASE("averageQuaternions() weighs rotations, not magnitudes")
    {
        SUBCASE("Scaling an element does not pull the mean toward it")
        {
            std::vector<double> scaled = quarterTurnAboutZ;
            for (double& component : scaled)
            {
                component *= 5.0;
            }
            Array unitPair(std::vector<std::vector<double>>{ identityRotation, quarterTurnAboutZ }, Dtype::Float64(),
                           Device::Cpu());
            Array scaledPair(
                std::vector<std::vector<double>>{ identityRotation, scaled }, Dtype::Float64(), Device::Cpu());
            checkClose(math::averageQuaternions(scaledPair),
                       math::averageQuaternions(unitPair).flatten().get<std::vector<double>>(), 1.0e-12);
        }

        SUBCASE("A zero first element does not disable the sign alignment")
        {
            std::vector<double> negated = quarterTurnAboutZ;
            for (double& component : negated)
            {
                component = -component;
            }
            Array window(std::vector<std::vector<double>>{ { 0.0, 0.0, 0.0, 0.0 }, quarterTurnAboutZ, negated },
                         Dtype::Float64(), Device::Cpu());
            // The zero element normalizes to the identity; the other two must not cancel each other.
            Array mean = math::averageQuaternions(window);
            CHECK(std::abs(math::dotQuaternions(mean, Array(quarterTurnAboutZ, Dtype::Float64(), Device::Cpu()))
                               .item<double>()) > 0.5);
        }
    }

    TEST_CASE("Shape validation")
    {
        SUBCASE("A trailing axis that is not four is rejected")
        {
            Array wrong(std::vector<double>{ 1.0, 2.0, 3.0 }, Dtype::Float64(), Device::Cpu());
            CHECK_THROWS_AS(math::normalizeQuaternion(wrong), std::invalid_argument);
        }

        SUBCASE("Operands on different devices are rejected")
        {
            if (Device::Cuda().isAvailable())
            {
                Array host(identityRotation, Dtype::Float64(), Device::Cpu());
                Array deviceSide(identityRotation, Dtype::Float64(), Device::Cuda());
                CHECK_THROWS_AS(math::multiplyQuaternions(host, deviceSide), std::invalid_argument);
            }
        }

        SUBCASE("Batches that neither match nor broadcast are rejected")
        {
            Array three(std::vector<std::vector<double>>{ identityRotation, identityRotation, identityRotation },
                        Dtype::Float64(), Device::Cpu());
            Array two(std::vector<std::vector<double>>{ identityRotation, identityRotation }, Dtype::Float64(),
                      Device::Cpu());
            CHECK_THROWS_AS(math::multiplyQuaternions(three, two), std::invalid_argument);
        }
    }

    TEST_CASE("rotationFormat")
    {
        // Same rotation as quarterTurnAboutZ, reordered scalar-first.
        const std::vector<double> quarterTurnAboutZWxyz{ std::cos(quarterTurnAngle), 0.0, 0.0,
                                                         std::sin(quarterTurnAngle) };

        SUBCASE("wxyz round-trips through the quaternion-valued functions")
        {
            Array q(quarterTurnAboutZWxyz, Dtype::Float64(), Device::Cpu());
            checkClose(math::normalizeQuaternion(q, "wxyz"), quarterTurnAboutZWxyz);
            checkClose(math::conjugateQuaternion(q, "wxyz"),
                       { std::cos(quarterTurnAngle), 0.0, 0.0, -std::sin(quarterTurnAngle) });
            // Composing two quarter turns about Z gives a half turn about Z: wxyz {0, 0, 0, 1}.
            checkClose(math::multiplyQuaternions(q, q, "wxyz"), { 0.0, 0.0, 0.0, 1.0 });
        }

        SUBCASE("rotateByQuaternion() accepts a wxyz quaternion")
        {
            Array q(quarterTurnAboutZWxyz, Dtype::Float64(), Device::Cpu());
            Array v(std::vector<double>{ 1.0, 0.0, 0.0 }, Dtype::Float64(), Device::Cpu());
            checkClose(math::rotateByQuaternion(q, v, "wxyz"), { 0.0, 1.0, 0.0 });
        }

        SUBCASE("dotQuaternions() does not depend on which supported format both operands share")
        {
            Array xyzw(quarterTurnAboutZ, Dtype::Float64(), Device::Cpu());
            Array wxyz(quarterTurnAboutZWxyz, Dtype::Float64(), Device::Cpu());
            CHECK(std::abs(math::dotQuaternions(xyzw, xyzw).item<double>() -
                           math::dotQuaternions(wxyz, wxyz, "wxyz").item<double>()) <= 1.0e-12);
        }

        SUBCASE("An unsupported format is rejected")
        {
            Array q(quarterTurnAboutZ, Dtype::Float64(), Device::Cpu());
            CHECK_THROWS_AS(math::normalizeQuaternion(q, "zyxw"), std::invalid_argument);
        }
    }
}
