// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#define CARB_EXPORTS

#include <carb/PluginUtils.h>

#include <isaacsim/ros2/core/Ros2Distro.h>
#include <isaacsim/ros2/core/Ros2Node.h>
#include <isaacsim/ros2/tf_viewer/ITransformListener.h>
#include <isaacsim/ros2/tf_viewer/Tf2Factory.h>

/**
 * @brief Plugin descriptor for the ROS 2 transform listener plugin.
 * @details
 * Defines metadata for the transform listener plugin including name, description,
 * and hot reload capability.
 */
const struct carb::PluginImplDesc g_kPluginDesc = { "isaacsim.ros2.tf_viewer.plugin", "Transform Listener", "NVIDIA",
                                                    carb::PluginHotReload::eDisabled, "dev" };

namespace isaacsim
{
namespace ros2
{
namespace tf_viewer
{

/**
 * @class TransformListener
 * @brief Implementation of the ITransformListener interface.
 * @details
 * This class provides functionality to listen for ROS 2 transform messages
 * and maintains a buffer of the transformation data. It inherits from
 * isaacsim::ros2::core::Ros2Node to interact with the ROS 2 system.
 */
class TransformListener : public ITransformListener, isaacsim::ros2::core::Ros2Node
{
public:
    /**
     * @brief Initializes the transform listener for a specific ROS 2 distribution.
     * @details
     * Loads the distribution-specific library, creates a tf2 factory and buffer.
     *
     * @param[in] rosDistro The ROS 2 distribution to use (e.g., "humble", "foxy").
     * @return True if initialization was successful, false otherwise.
     */
    bool initialize(const std::string& rosDistro) override
    {
        if (!isaacsim::ros2::core::isRos2DistroSupported(rosDistro))
        {
            CARB_LOG_ERROR("Unsupported ROS_DISTRO: %s", rosDistro.c_str());
            return false;
        }
        if (!m_libraryLoader)
        {
            try
            {
                m_libraryLoader =
                    std::make_shared<isaacsim::core::includes::LibraryLoader>("isaacsim.ros2.tf_viewer." + rosDistro);
            }
            catch (const std::exception& e)
            {
                CARB_LOG_ERROR(
                    "Exception while loading isaacsim.ros2.tf_viewer.%s library: %s", rosDistro.c_str(), e.what());
                return false;
            }
        }
        if (!m_tf2Factory)
        {

            using CreateFactoryBinding = Tf2Factory* (*)();
            CreateFactoryBinding createFactory = (m_libraryLoader->getSymbol<CreateFactoryBinding>("createFactory"));
            if (!createFactory)
            {
                CARB_LOG_ERROR("Unable to load symbols from the isaacsim.ros2.tf_viewer.%s library", rosDistro.c_str());
                return false;
            }
            m_tf2Factory = createFactory();
        }
        if (!m_buffer)
        {
            m_buffer = m_tf2Factory->createBuffer();
            m_buffer->clear();
        }
        return true;
    }

    /**
     * @brief Finalizes the transform listener by releasing ROS 2 resources.
     * @details
     * Cleans up subscribers and resets the ROS 2 node.
     */
    void finalize() override
    {
        if (m_subscriberTf)
        {
            m_subscriberTf.reset();
            m_subscriberTf = nullptr;
        }
        if (m_messageTfStatic)
        {
            m_messageTfStatic.reset();
            m_messageTfStatic = nullptr;
        }
        Ros2Node::reset();
    }

    /**
     * @brief Processes incoming transform messages.
     * @details
     * Initializes the ROS 2 node if needed, creates subscribers for transform
     * topics (/tf and /tf_static), and processes any pending messages.
     *
     * @return True if processing was successful, false otherwise.
     */
    bool spin() override
    {
        if (!isInitialized())
        {
            if (!initializeNodeHandle("isaacsim_tf_viewer", "", 0))
            {
                CARB_LOG_ERROR("Unable to create isaacsim.ros2.tf_viewer ROS2 node");
                return false;
            }
        }

        if (!m_subscriberTf)
        {
            isaacsim::ros2::core::Ros2QoSProfile qos;
            qos.depth = 100;
            m_messageTf = m_factory->createTfTreeMessage();
            m_subscriberTf =
                m_factory->createSubscriber(m_nodeHandle.get(), "/tf", m_messageTf->getTypeSupportHandle(), qos);
            return true;
        }
        if (!m_subscriberTfStatic)
        {
            isaacsim::ros2::core::Ros2QoSProfile qos;
            qos.depth = 100;
            m_messageTfStatic = m_factory->createTfTreeMessage();
            m_subscriberTfStatic = m_factory->createSubscriber(
                m_nodeHandle.get(), "/tf_static", m_messageTfStatic->getTypeSupportHandle(), qos);
            return true;
        }

        bool status = true;
        status &= _subscriberCallback(false);
        status &= _subscriberCallback(true);
        return status;
    }

    /**
     * @brief Resets the transform buffer by clearing all stored transforms.
     * @details
     * Clears all transformation data stored in the buffer.
     */
    void reset() override
    {
        if (!m_buffer)
        {
            return;
        }
        m_buffer->clear();
    }

    /**
     * @brief Computes transforms relative to the specified root frame.
     * @details
     * Calculates the transformation data for all frames relative to the specified
     * root frame and populates the internal data structures with the results.
     *
     * @param[in] rootFrame The reference frame for all transformations.
     */
    void computeTransforms(const std::string& rootFrame) override
    {
        if (!m_buffer)
        {
            return;
        }
        // clear containers
        m_frames.clear();
        m_relations.clear();
        m_transforms.clear();
        // get all frames
        m_frames = m_buffer->getFrames();
        // get transformations
        std::string parentFrame;
        for (auto& frame : m_frames)
        {
            bool retval = m_buffer->getParentFrame(frame, parentFrame);
            if (retval)
            {
                m_relations.push_back({ frame, parentFrame });
                double translation[3], rotation[4];
                retval = m_buffer->getTransform(rootFrame, frame, translation, rotation);
                if (retval)
                {
                    m_transforms[frame] = { { translation[0], translation[1], translation[2] },
                                            { rotation[0], rotation[1], rotation[2], rotation[3] } };
                }
            }
        }
    }

    /**
     * @brief Gets the list of available frame IDs.
     * @details
     * Returns the list of frame IDs that was computed in the most recent call to computeTransforms.
     *
     * @return Vector of frame ID strings.
     */
    const std::vector<std::string>& getFrames() override
    {
        return m_frames;
    };

    /**
     * @brief Gets the parent-child frame relationships.
     * @details
     * Returns the list of parent-child relationships between frames that was
     * computed in the most recent call to computeTransforms.
     *
     * @return Vector of tuples containing child-parent frame ID pairs.
     */
    const std::vector<std::tuple<std::string, std::string>>& getRelations() override
    {
        return m_relations;
    };

    /**
     * @brief Gets the transformation data for all frames.
     * @details
     * Returns the map of transforms (translation and rotation) relative to the root
     * frame that was computed in the most recent call to computeTransforms.
     *
     * @return Map of frame IDs to transform data (translation and rotation).
     */
    const std::unordered_map<std::string,
                             std::tuple<std::tuple<double, double, double>, std::tuple<double, double, double, double>>>&
    getTransforms() override
    {
        return m_transforms;
    };

private:
    /** @brief Library loader for distribution-specific implementation. */
    std::shared_ptr<isaacsim::core::includes::LibraryLoader> m_libraryLoader = nullptr;

    /** @brief Factory for creating tf2 objects. */
    Tf2Factory* m_tf2Factory = nullptr;

    /** @brief Subscriber for the /tf topic. */
    std::shared_ptr<isaacsim::ros2::core::Ros2Subscriber> m_subscriberTf = nullptr;

    /** @brief Subscriber for the /tf_static topic. */
    std::shared_ptr<isaacsim::ros2::core::Ros2Subscriber> m_subscriberTfStatic = nullptr;

    /** @brief Message buffer for /tf messages. */
    std::shared_ptr<isaacsim::ros2::core::Ros2TfTreeMessage> m_messageTf = nullptr;

    /** @brief Message buffer for /tf_static messages. */
    std::shared_ptr<isaacsim::ros2::core::Ros2TfTreeMessage> m_messageTfStatic = nullptr;

    /** @brief Buffer for storing transform data. */
    std::shared_ptr<Ros2BufferCore> m_buffer = nullptr;

    /** @brief List of available frame IDs. */
    std::vector<std::string> m_frames;

    /** @brief List of parent-child frame relationships. */
    std::vector<std::tuple<std::string, std::string>> m_relations;

    /** @brief Map of frame IDs to transformation data. */
    std::unordered_map<std::string, std::tuple<std::tuple<double, double, double>, std::tuple<double, double, double, double>>>
        m_transforms;

    /**
     * @brief Processes messages from a transform subscriber.
     * @details
     * Retrieves transform messages from the specified subscriber and adds them
     * to the transform buffer.
     *
     * @param[in] isStatic Whether to process messages from the static transform subscriber.
     * @return True if processing was successful, false otherwise.
     */
    bool _subscriberCallback(bool isStatic)
    {
        if (!m_buffer)
        {
            return false;
        }
        auto subscriber = isStatic ? m_subscriberTfStatic : m_subscriberTf;
        auto message = isStatic ? m_messageTfStatic : m_messageTf;
        while (subscriber->spin(message->getPtr()))
        {
            m_buffer->setTransform(message->getPtr(), "", isStatic);
        }
        return true;
    }
};

} // namespace tf_viewer
} // namespace ros2
} // namespace isaacsim

CARB_PLUGIN_IMPL(g_kPluginDesc, isaacsim::ros2::tf_viewer::TransformListener)

/**
 * @brief Fills the interface with function pointers.
 * @details
 * This function is called by the Carbonite plugin system to initialize
 * the plugin interface.
 *
 * @param[in,out] iface The interface instance to be filled.
 */
void fillInterface(isaacsim::ros2::tf_viewer::TransformListener& iface)
{
}
