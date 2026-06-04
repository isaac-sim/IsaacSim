// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifdef _WIN32
#    pragma warning(push)
#    pragma warning(disable : 4996)
#endif

#define CARB_EXPORTS

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "isaacsim/core/includes/Pose.h"
#include "isaacsim/core/includes/UsdUtilities.h"

#include <carb/Framework.h>
#include <carb/logging/Log.h>

#include <isaacsim/core/includes/Conversions.h>
#include <isaacsim/sensors/physics/IPhysicsSensor.h>
#include <isaacsim/sensors/physics/ImuSensor.h>
#include <omni/physx/IPhysx.h>
#include <pxr/usd/usdPhysics/scene.h>

#include <PxActor.h>


#if defined(_WIN32)
#    include <PxArticulationLink.h>
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wpragmas"
#    include <PxArticulationLink.h>
#    pragma GCC diagnostic pop
#endif

#include <PxRigidDynamic.h>
#include <PxScene.h>
#include <map>
#include <string>
#include <vector>

namespace isaacsim
{
namespace sensors
{
namespace physics
{
ImuSensor::~ImuSensor()
{
    reset();
    m_rawBuffer.clear();
    m_sensorReadings.clear();
    m_sensorReadingsSensorFrame.clear();
    m_rigidBodyDataBuffer = nullptr;
}

void ImuSensor::initialize(float* rigidBodyDataBuffer, size_t& dataBufferIndex)
{
    m_dataBufferIndex = dataBufferIndex;
    m_rigidBodyDataBuffer = rigidBodyDataBuffer;
    onComponentChange();
}


IsReading ImuSensor::getSensorReading(const std::function<IsReading(std::vector<IsReading>, float)>& interpolateFunction,
                                      const bool& getLatestValue,
                                      const bool& readGravity)
{
    if (m_props.sensorPeriod > 0 && m_props.sensorPeriod < m_timeDelta && m_timeDelta - m_props.sensorPeriod > 0.001)
    {
        CARB_LOG_WARN_ONCE(
            "*** warning: IMU sensor frequency is higher than physics frequency, returning the latest physics value");
    }

    IsReading sensorReading = IsReading();

    if (m_enabled)
    {
        // if sensor period is shorter than physics downtime, or user choose latest value return current value
        // or internal time + sensor period time is behind the last step time, then something went wrong
        // i.e. sensor was disabled for a long time and then re-enabled
        // get the latest time and measurement
        if (m_props.sensorPeriod <= m_timeDelta || m_sensorTime + m_props.sensorPeriod < m_sensorReadings[1].time ||
            getLatestValue)
        {
            sensorReading = m_sensorReadings[0];
            sensorReading.isValid = true;
            if (m_props.sensorPeriod > 0 && m_sensorTime + m_props.sensorPeriod < m_sensorReadings[1].time)
            {
                CARB_LOG_WARN("*** warning IMU sensor time out of sync, using latest measurements");
            }
        }
        else
        {
            sensorReading.time = m_sensorTime;
            sensorReading.isValid = true;
            float timeRatio = 1;

            if (m_sensorReadingsSensorFrame[1].time != m_sensorReadingsSensorFrame[0].time)
            {
                timeRatio = (m_sensorTime - m_sensorReadingsSensorFrame[1].time) /
                            (m_sensorReadingsSensorFrame[0].time - m_sensorReadingsSensorFrame[1].time);
            }
            else
            {
                timeRatio = (m_sensorTime - m_sensorReadingsSensorFrame[1].time) / static_cast<float>(m_timeDelta);
            }
            // user didn't pass in a interpolation function
            if (!interpolateFunction)
            {
                sensorReading.linAccX =
                    lerp(m_sensorReadingsSensorFrame[1].linAccX, m_sensorReadingsSensorFrame[0].linAccX, timeRatio);
                sensorReading.linAccY =
                    lerp(m_sensorReadingsSensorFrame[1].linAccY, m_sensorReadingsSensorFrame[0].linAccY, timeRatio);
                sensorReading.linAccZ =
                    lerp(m_sensorReadingsSensorFrame[1].linAccZ, m_sensorReadingsSensorFrame[0].linAccZ, timeRatio);

                sensorReading.angVelX =
                    lerp(m_sensorReadingsSensorFrame[1].angVelX, m_sensorReadingsSensorFrame[0].angVelX, timeRatio);
                sensorReading.angVelY =
                    lerp(m_sensorReadingsSensorFrame[1].angVelY, m_sensorReadingsSensorFrame[0].angVelY, timeRatio);
                sensorReading.angVelZ =
                    lerp(m_sensorReadingsSensorFrame[1].angVelZ, m_sensorReadingsSensorFrame[0].angVelZ, timeRatio);

                sensorReading.orientation.w = lerp(m_sensorReadingsSensorFrame[1].orientation.w,
                                                   m_sensorReadingsSensorFrame[0].orientation.w, timeRatio);
                sensorReading.orientation.x = lerp(m_sensorReadingsSensorFrame[1].orientation.x,
                                                   m_sensorReadingsSensorFrame[0].orientation.x, timeRatio);
                sensorReading.orientation.y = lerp(m_sensorReadingsSensorFrame[1].orientation.y,
                                                   m_sensorReadingsSensorFrame[0].orientation.y, timeRatio);
                sensorReading.orientation.z = lerp(m_sensorReadingsSensorFrame[1].orientation.z,
                                                   m_sensorReadingsSensorFrame[0].orientation.z, timeRatio);
            }
            // use user's interpolation function
            else
            {
                sensorReading = interpolateFunction(m_sensorReadingsSensorFrame, m_sensorTime);
            }
        }

        if (readGravity)
        {
            sensorReading.linAccX += static_cast<float>(m_gravitySensorFrame[0]);
            sensorReading.linAccY += static_cast<float>(m_gravitySensorFrame[1]);
            sensorReading.linAccZ += static_cast<float>(m_gravitySensorFrame[2]);
        }
    }
    return sensorReading;
}

void ImuSensor::reset()
{
    m_rawBuffer.resize(m_rawBufferSize, IsRawData());
    m_sensorReadings.resize(m_rawBufferSize, IsReading());
    m_sensorReadingsSensorFrame.resize(m_rawBufferSize, IsReading());
    m_sensorTime = 0;
}

void ImuSensor::onPhysicsStep()
{
    if (m_rigidBodyDataBuffer == nullptr)
    {
        return;
    }
    // vW linear velocity in world frame
    omni::math::linalg::vec3d vW = omni::math::linalg::vec3d(m_rigidBodyDataBuffer[m_dataBufferIndex],
                                                             m_rigidBodyDataBuffer[m_dataBufferIndex + 1],
                                                             m_rigidBodyDataBuffer[m_dataBufferIndex + 2]);

    // wW angular velocity in world frame
    omni::math::linalg::vec3d wW = omni::math::linalg::vec3d(m_rigidBodyDataBuffer[m_dataBufferIndex + 3],
                                                             m_rigidBodyDataBuffer[m_dataBufferIndex + 4],
                                                             m_rigidBodyDataBuffer[m_dataBufferIndex + 5]);


    // Get transformation matrix from body to world
    usdrt::GfMatrix4d rBw =
        isaacsim::core::includes::pose::computeWorldXformNoCache(m_stage, m_usdrtStage, m_prim.GetPath())
            .GetOrthonormalized();

    // Inverse to get transformation matrix from world to body
    usdrt::GfMatrix4d rWb = rBw.GetInverse();

    // sensor orientation in world frame
    usdrt::GfMatrix3d rW = rBw.ExtractRotationMatrix();
    omni::math::linalg::quatd qWb = rW.ExtractRotation();

    // velocity of sensor frame in sensor frame
    omni::math::linalg::vec3d vB = rWb.TransformDir(vW);

    // angular velocity of sensor frame in sensor frame
    omni::math::linalg::vec3d wB = rWb.TransformDir(wW);

    // gravity that the IMU experience in sensor frame
    m_gravitySensorFrame = rWb.TransformDir(m_gravity);

    // we then finite diff vB to get a_b, to reduce noise, average multiple finite diffs
    // save raw data into a buffer list , buffer 0 always saves the latest velocities
    if (!m_rawBuffer.empty())
    {
        m_rawBuffer.pop_back();
    }

    const omni::math::linalg::vec3d imaginary = qWb.GetImaginary();

    // read in new data
    m_rawBuffer.insert(m_rawBuffer.begin(), IsRawData());
    m_rawBuffer[0].time = static_cast<float>(m_timeSeconds);
    m_rawBuffer[0].dt = static_cast<float>(m_timeDelta);
    m_rawBuffer[0].linVelX = static_cast<float>(vB[0]);
    m_rawBuffer[0].linVelY = static_cast<float>(vB[1]);
    m_rawBuffer[0].linVelZ = static_cast<float>(vB[2]);
    m_rawBuffer[0].angVelX = static_cast<float>(wB[0]);
    m_rawBuffer[0].angVelY = static_cast<float>(wB[1]);
    m_rawBuffer[0].angVelZ = static_cast<float>(wB[2]);
    m_rawBuffer[0].orientation.w = static_cast<float>(qWb.GetReal());
    m_rawBuffer[0].orientation.x = static_cast<float>(imaginary[0]);
    m_rawBuffer[0].orientation.y = static_cast<float>(imaginary[1]);
    m_rawBuffer[0].orientation.z = static_cast<float>(imaginary[2]);

    if (!m_sensorReadings.empty())
    {
        m_sensorReadings.pop_back();
    }
    m_sensorReadings.insert(m_sensorReadings.begin(), IsReading());

    // signal processing
    m_sensorReadings[0].time = static_cast<float>(m_timeSeconds);
    // ang_vel output strategy: average past m_angularVelocityFilterSize timesteps
    float tmpSumX = 0, tmpSumY = 0, tmpSumZ = 0;
    for (int i = 0; i < m_angularVelocityFilterSize; i++)
    {
        tmpSumX += m_rawBuffer[i].angVelX;
        tmpSumY += m_rawBuffer[i].angVelY;
        tmpSumZ += m_rawBuffer[i].angVelZ;
    }
    m_sensorReadings[0].angVelX = static_cast<float>(tmpSumX / m_angularVelocityFilterSize);
    m_sensorReadings[0].angVelY = static_cast<float>(tmpSumY / m_angularVelocityFilterSize);
    m_sensorReadings[0].angVelZ = static_cast<float>(tmpSumZ / m_angularVelocityFilterSize);

    // lin acc output strategy: average m_linearAccelerationFilterSize finite diffs
    // say if m_linearAccelerationFilterSize = 2, we do (([0] - [2]) / (2dt) + ([1] - [3]) / (2dt))/2
    tmpSumX = 0.0f;
    tmpSumY = 0.0f;
    tmpSumZ = 0.0f;
    for (int i = 0; i < m_linearAccelerationFilterSize; i++)
    {
        float dt = m_rawBuffer[i].time - m_rawBuffer[i + m_linearAccelerationFilterSize].time;

        if (dt > 1e-10)
        {
            tmpSumX += (m_rawBuffer[i].linVelX - m_rawBuffer[i + m_linearAccelerationFilterSize].linVelX) / dt;
            tmpSumY += (m_rawBuffer[i].linVelY - m_rawBuffer[i + m_linearAccelerationFilterSize].linVelY) / dt;
            tmpSumZ += (m_rawBuffer[i].linVelZ - m_rawBuffer[i + m_linearAccelerationFilterSize].linVelZ) / dt;
        }
    }


    // average acc
    m_sensorReadings[0].linAccX = static_cast<float>(tmpSumX / m_linearAccelerationFilterSize);
    m_sensorReadings[0].linAccY = static_cast<float>(tmpSumY / m_linearAccelerationFilterSize);
    m_sensorReadings[0].linAccZ = static_cast<float>(tmpSumZ / m_linearAccelerationFilterSize);

    // // add gravity
    // m_sensorReadings[0].linAccX += static_cast<float>(g_b[0]);
    // m_sensorReadings[0].linAccY += static_cast<float>(g_b[1]);
    // m_sensorReadings[0].linAccZ += static_cast<float>(g_b[2]);

    float tmpSumW = 0.0;
    tmpSumX = 0.0f;
    tmpSumY = 0.0f;
    tmpSumZ = 0.0f;

    for (int i = 0; i < m_orientationFilterSize; i++)
    {
        tmpSumW += m_rawBuffer[i].orientation.w;
        tmpSumX += m_rawBuffer[i].orientation.x;
        tmpSumY += m_rawBuffer[i].orientation.y;
        tmpSumZ += m_rawBuffer[i].orientation.z;
    }

    m_sensorReadings[0].orientation.w = static_cast<float>(tmpSumW / m_orientationFilterSize);
    m_sensorReadings[0].orientation.x = static_cast<float>(tmpSumX / m_orientationFilterSize);
    m_sensorReadings[0].orientation.y = static_cast<float>(tmpSumY / m_orientationFilterSize);
    m_sensorReadings[0].orientation.z = static_cast<float>(tmpSumZ / m_orientationFilterSize);

    if (m_props.sensorPeriod <= m_timeDelta)
    {
        m_sensorTime = m_sensorReadings[0].time;
    }
    else if (m_sensorTime + m_props.sensorPeriod <= m_sensorReadings[0].time)
    {
        m_sensorTime += m_props.sensorPeriod;
        m_sensorReadingsSensorFrame = m_sensorReadings;
    }
    // }
}

bool ImuSensor::findValidParent()
{
    pxr::UsdPrim tempPrim = this->m_stage->GetPrimAtPath(this->m_prim.GetPath()).GetParent();

    while (tempPrim.IsValid() && tempPrim.GetName().GetString() != "/")
    {
        // check if it's a rigid body
        bool rigidBodyEnabled = false;
        tempPrim.GetAttribute(pxr::TfToken("physics:rigidBodyEnabled")).Get(&rigidBodyEnabled);
        if (rigidBodyEnabled)
        {
            m_parentPrim = tempPrim;
            return true;
        }
        // go to parent
        tempPrim = tempPrim.GetParent();
    }
    CARB_LOG_WARN("No valid parent for %s with a rigid body api was found.", this->m_prim.GetPath().GetString().c_str());
    return false;
}

void ImuSensor::onComponentChange()
{
    IsaacSensorComponentBase::onComponentChange();


    // get orientation quad sensor period, and translate

    using namespace isaacsim::robot::schema::sensors;

    isaacsim::core::includes::safeGetAttribute(m_prim.GetAttribute(kSensorPeriodAttr), this->m_props.sensorPeriod);
    isaacsim::core::includes::safeGetAttribute(
        m_prim.GetAttribute(kLinearAccelerationFilterWidthAttr), this->m_linearAccelerationFilterSize);
    isaacsim::core::includes::safeGetAttribute(
        m_prim.GetAttribute(kAngularVelocityFilterWidthAttr), this->m_angularVelocityFilterSize);
    isaacsim::core::includes::safeGetAttribute(
        m_prim.GetAttribute(kOrientationFilterWidthAttr), this->m_orientationFilterSize);

    // reject 0 or negative rolling avg size
    m_linearAccelerationFilterSize = std::max(m_linearAccelerationFilterSize, 1);
    m_angularVelocityFilterSize = std::max(m_angularVelocityFilterSize, 1);
    m_orientationFilterSize = std::max(m_orientationFilterSize, 1);

    int maxRollingSize =
        std::max(std::max(m_linearAccelerationFilterSize, m_angularVelocityFilterSize), m_orientationFilterSize);

    // size of the raw data must be 2 times larger than the max rolling avg size
    // also the buffer should be sufficiently large (20)
    if (this->m_rawBufferSize != 2 * maxRollingSize)
    {
        this->m_rawBufferSize = std::max(2 * maxRollingSize, 20);
        m_rawBuffer.resize(m_rawBufferSize, IsRawData());
        m_sensorReadings.resize(m_rawBufferSize, IsReading());
    }
    pxr::GfQuatd sensorQuat(0.0);
    m_prim.GetAttribute(pxr::TfToken("xformOp:orient")).Get(&sensorQuat);
    sensorQuat.Normalize();

    pxr::GfVec3d position(0.0);
    m_prim.GetAttribute(pxr::TfToken("xformOp:translate")).Get(&position);


    omni::math::linalg::quatd rotate(
        sensorQuat.GetReal(), omni::math::linalg::vec3d(sensorQuat.GetImaginary()[0], sensorQuat.GetImaginary()[1],
                                                        sensorQuat.GetImaginary()[2]));

    m_props.orientation.SetRotate(rotate);

    m_unitScale = UsdGeomGetStageMetersPerUnit(m_stage);
    // gravity that the IMU experiences in world frame
    omni::math::linalg::vec3d dir = omni::math::linalg::vec3d(0, 0, -1.0f);
    float mag = 9.80665f;
    m_gravity = mag / m_unitScale * -dir;
    // If a scene exists we try reading gravity from it
    pxr::UsdPrimRange range = m_stage->Traverse();
    omni::physx::IPhysx* physxPtr = carb::getCachedInterface<omni::physx::IPhysx>();

    for (pxr::UsdPrimRange::iterator iter = range.begin(); iter != range.end(); ++iter)
    {
        pxr::UsdPrim prim = *iter;

        if (prim.IsA<pxr::UsdPhysicsScene>())
        {
            pxr::UsdPhysicsScene scene(prim);
            // Try to get the actual physics scene's gravity vector
            ::physx::PxScene* physxScenePtr = static_cast<::physx::PxScene*>(
                physxPtr->getPhysXPtr(prim.GetPrimPath(), omni::physx::PhysXType::ePTScene));

            if (physxScenePtr)
            {
                ::physx::PxVec3 gravity = physxScenePtr->getGravity();
                m_gravity = -omni::math::linalg::vec3d(gravity.x, gravity.y, gravity.z) / m_unitScale;
            }
            else
            {
                // Fallback onto USD values
                isaacsim::core::includes::safeGetAttribute(scene.GetGravityMagnitudeAttr(), mag);
                pxr::GfVec3f dirAttr;
                isaacsim::core::includes::safeGetAttribute(scene.GetGravityDirectionAttr(), dirAttr); // (0, 0, -1.0f)
                dir.Set(
                    static_cast<double>(dirAttr[0]), static_cast<double>(dirAttr[1]), static_cast<double>(dirAttr[2]));
                m_gravity = static_cast<double>(mag) / m_unitScale * -dir;
            }
        }
    }

    if (m_previousEnabled != this->m_enabled)
    {
        if (m_enabled)
        {
            this->onPhysicsStep(); // force on physics step to run to get up to date value
            m_sensorTime = static_cast<float>(m_timeSeconds);
            m_rawBuffer.resize(m_rawBufferSize, IsRawData());
            m_sensorReadings.resize(m_rawBufferSize, IsReading());
            m_sensorReadingsSensorFrame = m_sensorReadings;
        }
        else
        {
            this->onStop();
        }
        m_previousEnabled = this->m_enabled;
    }
}

void ImuSensor::onStop()
{
    reset();
}

void ImuSensor::printIsReading(const IsReading& reading)
{
    CARB_LOG_INFO("Is Reading");
    CARB_LOG_INFO("time: %f", reading.time);
    CARB_LOG_INFO("ang vel x: %f", reading.angVelX);
    CARB_LOG_INFO("ang vel y: %f", reading.angVelY);
    CARB_LOG_INFO("ang vel z: %f", reading.angVelZ);

    CARB_LOG_INFO("lin accel x: %f", reading.linAccX);
    CARB_LOG_INFO("lin accel y: %f", reading.linAccY);
    CARB_LOG_INFO("lin accel z: %f", reading.linAccZ);
    CARB_LOG_INFO("orientation xyzw: (%f, %f, %f, %f)", reading.orientation.x, reading.orientation.y,
                  reading.orientation.z, reading.orientation.w);
}

}
}
}
#ifdef _WIN32
#    pragma warning(pop)
#endif
