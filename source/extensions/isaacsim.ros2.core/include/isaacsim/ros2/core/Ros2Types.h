// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/core/includes/LibraryLoader.h>
#include <nlohmann/json.hpp>
#include <omni/fabric/Type.h>

#include <cstdint>
#include <string>
#include <vector>

namespace isaacsim
{
namespace ros2
{
namespace core
{

/**
 * @enum BackendMessageType
 * @brief Enumerations of ROS 2 message types.
 * @details
 * Defines the various types of messages that can be handled by the ROS 2 core,
 * including topics, services, and actions with their respective components.
 */
enum class BackendMessageType : uint8_t
{
    eMessage = 0, /**< Topic message. */
    eRequest, /**< Service request (`_Request`). */
    eResponse, /**< Service response (`_Response`). */
    eGoal, /**< Action goal (`_Goal`). */
    eResult, /**< Action result (`_Result`). */
    eFeedback, /**< Action feedback (`_Feedback`). */
    eSendGoalRequest, /**< Action goal request (`_SendGoal_Request`). */
    eSendGoalResponse, /**< Action goal response (`_SendGoal_Response`). */
    eFeedbackMessage, /**< Action feedback (`_FeedbackMessage`). */
    eGetResultRequest, /**< Action result request (`_GetResult_Request`). */
    eGetResultResponse, /**< Action result response (`_GetResult_Response`). */
};

/**
 * @struct DynamicMessageField
 * @brief Structure that encapsulates a dynamic message field.
 * @details
 * Provides a flexible container for ROS 2 message fields with support for
 * both ROS and OmniGraph data types, including hierarchical field names.
 */
struct DynamicMessageField
{
    std::string name; /**< Field name. Hierarchical names (e.g.: MESSAGE fields are unrolled and concatenated by `:`).
                       */
    uint8_t rosType; /**< ROS data type from rosidl_typesupport_introspection_c/field_types.h. */
    bool isArray; /**< Whether the field is an array. */
    std::string ognType; /**< OmniGraph data type name from omni.graph.docs. */
    omni::fabric::BaseDataType dataType; /**< Fabric data type. */

    /**
     * @brief Splits the field name into hierarchical components.
     * @details
     * Generates a list of field names by splitting the hierarchical name using
     * the specified delimiter.
     *
     * @param[in] delimiter Character used to split the name.
     * @return List of field names in hierarchical order.
     *
     * @note The default delimiter is ':' which matches how hierarchical fields are stored.
     */
    std::vector<std::string> names(char delimiter = ':')
    {
        std::vector<std::string> fieldNames;
        std::stringstream stringStream(name);
        std::string segment;
        while (std::getline(stringStream, segment, delimiter))
        {
            fieldNames.push_back(segment);
        }
        return fieldNames;
    }
};

/**
 * @struct TfTransformStamped
 * @brief Structure that encapsulates geometry_msgs/msg/TransformStamped data.
 * @details
 * Provides a container for transform data between coordinate frames,
 * including timestamp, frame IDs, translation, and rotation components.
 */
struct TfTransformStamped
{
    double timeStamp; /**< Time in seconds. */
    std::string parentFrame; /**< Transform frame with which this data is associated. */
    std::string childFrame; /**< Frame ID of the child frame to which this transform points. */

    // Translation components
    double translationX; /**< Translation of child frame from parent frame (x-axis) in meters. */
    double translationY; /**< Translation of child frame from parent frame (y-axis) in meters. */
    double translationZ; /**< Translation of child frame from parent frame (z-axis) in meters. */

    // Quaternion components
    double rotationX; /**< Rotation of child frame from parent frame (quaternion x-component). */
    double rotationY; /**< Rotation of child frame from parent frame (quaternion y-component). */
    double rotationZ; /**< Rotation of child frame from parent frame (quaternion z-component). */
    double rotationW; /**< Rotation of child frame from parent frame (quaternion w-component). */
};

/**
 * @class Ros2Message
 * @brief Base class for all ROS 2 message encapsulations.
 * @details
 * Provides the foundation for ROS 2 message handling with basic functionality
 * for message pointer management and type support.
 */
class Ros2Message
{
public:
    /**
     * @brief Retrieves the message pointer.
     * @details
     * Returns the pointer to the underlying ROS 2 message if it has been
     * properly created and initialized.
     *
     * @return Pointer to the message or nullptr if not initialized.
     *
     * @note This method does not perform type checking - the caller is responsible for
     *       proper casting to the appropriate message type.
     */
    void* getPtr()
    {
        return m_msg;
    }

    /**
     * @brief Gets the type support handle for the message.
     * @details
     * Returns a pointer to the ROS IDL message type support data structure.
     * The actual type depends on the message category:
     * - Topic: rosidl_message_type_support_t
     * - Service: rosidl_service_type_support_t
     * - Action: rosidl_action_type_support_t
     *
     * @return Pointer to the type support structure or nullptr.
     */
    virtual const void* getTypeSupportHandle() = 0;

protected:
    void* m_msg = nullptr; /**< Message pointer. */
};

/**
 * @class Ros2DynamicMessage
 * @brief Class implementing dynamic ROS 2 messages.
 * @details
 * Provides runtime definition and manipulation of ROS 2 message types.
 * Supports both JSON and vector-based message processing.
 *
 * @warning JSON processing has significant computational overhead compared to vector processing.
 */
class Ros2DynamicMessage : public Ros2Message
{
public:
    /**
     * @brief Generates a human-readable message structure summary.
     * @details
     * Creates a formatted table showing the message structure including:
     * \code{.unparsed}
     * Message: sensor_msgs/msg/PointCloud2 (topic | message)
     * Idx Array ROS type                  OGN type                  Name
     * === ===== ========================= ========================= ====
     * 0   no    INT32 (int32_t)           eInt (int32_t)            header:stamp:sec
     * 1   no    UINT32 (uint32_t)         eUInt (uint32_t)          header:stamp:nanosec
     * 2   no    STRING (std::string)      eToken (std::string)      header:frame_id
     * 3   no    UINT32 (uint32_t)         eUInt (uint32_t)          height
     * 4   no    UINT32 (uint32_t)         eUInt (uint32_t)          width
     * 5   yes   MESSAGE (nlohmann::json)  eUnknown (nlohmann::json) fields
     * 6   no    BOOLEAN (bool)            eBool (bool)              is_bigendian
     * 7   no    UINT32 (uint32_t)         eUInt (uint32_t)          point_step
     * 8   no    UINT32 (uint32_t)         eUInt (uint32_t)          row_step
     * 9   yes   UINT8 (uint8_t)           eUInt (uint32_t)          data
     * 10  no    BOOLEAN (bool)            eBool (bool)              is_dense
     * \endcode
     *
     * @param[in] print Whether to output the summary to console.
     * @return Formatted string containing the message structure.
     */
    virtual std::string generateSummary(bool print) = 0;

    /**
     * @brief Reads message data as JSON.
     * @details
     * Retrieves the current message data as a JSON object for easy manipulation.
     *
     * @return Reference to JSON object containing message data.
     *
     * @warning JSON processing is convenient but has higher computational overhead.
     */
    virtual const nlohmann::json& readData() = 0;

    /**
     * @brief Reads message data as vector of shared pointers.
     * @details
     * Retrieves the current message data as a vector of type-erased shared pointers,
     * which can hold either ROS or OmniGraph data types.
     *
     * @param[in] asOgnType Whether to return OmniGraph or ROS 2 data types.
     * @return Vector of message field data.
     *
     * @note Vector-based operations are more efficient than JSON for performance-critical code.
     */
    virtual const std::vector<std::shared_ptr<void>>& readData(bool asOgnType) = 0;

    /**
     * @brief Writes message data from JSON.
     * @details
     * Updates the message with data provided in a JSON object.
     *
     * @param[in] data JSON object containing message data.
     *
     * @warning JSON processing is convenient but has higher computational overhead.
     */
    virtual void writeData(const nlohmann::json& data) = 0;

    /**
     * @brief Writes message data from vector of shared pointers.
     * @details
     * Updates the message with data provided in a vector of type-erased shared pointers.
     *
     * @param[in] data Vector containing message field data.
     * @param[in] fromOgnType Whether input data uses OmniGraph types.
     *
     * @note Vector-based operations are more efficient than JSON for performance-critical code.
     */
    virtual void writeData(const std::vector<std::shared_ptr<void>>& data, bool fromOgnType) = 0;

    /**
     * @brief Gets the message field descriptions.
     * @details
     * Provides access to the field metadata that describes the message structure.
     *
     * @return Vector of field descriptions.
     */
    const std::vector<DynamicMessageField>& getMessageFields()
    {
        return m_messagesFields;
    };

    /**
     * @brief Gets the message data container as vector.
     * @details
     * Returns a constant vector of non-constant shared pointers, allowing
     * modification of field values but not container structure.
     * This means that the elements of the vector (the message fields) cannot be modified but their content
     * (the message fields' value) can. This is particularly useful when writing the message data using a vector
     * as a container since it is not necessary to create pointers to the required data types.
     * See \ref Ros2DynamicMessage::writeData for use example.
     *
     * @param[in] asOgnType Whether to return OmniGraph or ROS 2 data types.
     * @return Vector container with shared pointers to field data.
     *
     * @note The returned container is read-only, but the pointed-to data can be modified.
     */
    const std::vector<std::shared_ptr<void>>& getVectorContainer(bool asOgnType)
    {
        return asOgnType ? m_messageVectorOgnContainer : m_messageVectorRosContainer;
    };

    /**
     * @brief Checks message validity.
     * @details
     * Verifies if the underlying message pointer has been properly initialized.
     *
     * @return True if message is properly created and initialized, false otherwise.
     */
    bool isValid()
    {
        return m_msg != nullptr;
    }

protected:
    std::vector<DynamicMessageField> m_messagesFields; /**< Message fields description. */
    nlohmann::json m_messageJsonContainer; /**< JSON message container. */
    std::vector<std::shared_ptr<void>> m_messageVectorRosContainer; /**< ROS 2 data types vector container. */
    std::vector<std::shared_ptr<void>> m_messageVectorOgnContainer; /**< OmniGraph data types vector container. */
};

/**
 * @class Ros2ContextHandle
 * @brief Encapsulates a ROS 2 context for initialization and cleanup.
 * @details
 * Base class that encapsulates a non-global state of a ROS 2 init/shutdown cycle
 * (`rcl_context_t`) instance used in the creation of ROS 2 nodes and other entities.
 * This class manages the lifecycle of a ROS 2 context, including initialization
 * parameters like domain ID, and provides access to the underlying context pointer.
 */
class Ros2ContextHandle
{
public:
    /**
     * @brief Retrieves the pointer to the ROS 2 context.
     * @details
     * Returns a pointer to the underlying ROS 2 context structure (`rcl_context_t`),
     * which is needed when creating nodes and other ROS 2 entities.
     *
     * @return Pointer to the ROS 2 context structure.
     *
     * @warning This method may return nullptr if the context is not properly initialized.
     *          Always check isValid() before using the returned pointer.
     */
    virtual void* getContext() = 0;

    /**
     * @brief Initializes the ROS 2 context.
     * @details
     * Initializes the RCL (ROS Client Library) context with the provided command line
     * arguments and domain ID settings. This prepares the context for use with
     * nodes, publishers, subscribers, and other ROS 2 entities.
     *
     * @param[in] argc Number of strings in argv.
     * @param[in] argv Command line arguments.
     * @param[in] setDomainId Whether to set the ROS domain ID (if true, overrides ROS_DOMAIN_ID environment variable).
     * @param[in] domainId ROS domain ID value to use if setDomainId is true.
     *
     * @note If setDomainId is false, the domain ID will be determined by the ROS_DOMAIN_ID environment
     *       variable, or will default to 0 if not set.
     */
    virtual void init(int argc, char const* const* argv, bool setDomainId = false, size_t domainId = 0) = 0;

    /**
     * @brief Checks if the context is valid.
     * @details
     * Verifies whether the object holds a valid and initialized ROS 2 context instance.
     *
     * @return True if the context is valid and initialized, false otherwise.
     *
     * @note This method should be called before attempting to use the context for any operations.
     */
    virtual bool isValid() = 0;

    /**
     * @brief Shuts down the ROS 2 context.
     * @details
     * Performs an orderly shutdown of the ROS 2 context, cleaning up all
     * resources associated with it. This should be called before destroying
     * the context handle to ensure proper cleanup.
     *
     * @param[in] shutdownReason Optional human-readable string describing the shutdown reason.
     * @return True if the shutdown was completed successfully, false otherwise.
     *
     * @note If shutdownReason is nullptr, no reason will be provided in logs.
     * @warning Failing to shut down a context properly may lead to resource leaks.
     */
    virtual bool shutdown(const char* shutdownReason = nullptr) = 0;
};

/**
 * @class Ros2NodeHandle
 * @brief Base class that encapsulates a ROS 2 node (`rcl_node_t`) instance.
 * @details
 * Provides the interface for managing ROS 2 nodes, which are the primary
 * entry points for communication in the ROS 2 system.
 */
class Ros2NodeHandle
{
public:
    /**
     * @brief Gets the pointer to the context handler object.
     * @details
     * Retrieves the context handler that owns this node.
     *
     * @return Pointer to the context handler.
     *
     * @see Ros2ContextHandle for context management details.
     */
    virtual Ros2ContextHandle* getContextHandle() = 0;

    /**
     * @brief Gets the pointer to the ROS 2 node structure.
     * @details
     * Returns a pointer to the underlying ROS 2 node structure (`rcl_node_t`).
     *
     * @return Pointer to the node structure.
     *
     * @warning This is a low-level access method that should be used with caution.
     */
    virtual void* getNode() = 0;
};

/**
 * @class Ros2Publisher
 * @brief Base class for ROS 2 publishers.
 * @details
 * Provides the interface for publishing messages to ROS 2 topics.
 * Publishers are used to send data from the application to other ROS 2 nodes.
 */
class Ros2Publisher
{
public:
    /**
     * @brief Sends a message to the topic.
     * @details
     * Publishes the provided message to the associated ROS 2 topic.
     *
     * @param[in] msg Pointer to the message to publish.
     *
     * @pre The message must be of the correct type for this publisher.
     * @pre The publisher must be valid (check with isValid()).
     */
    virtual void publish(const void* msg) = 0;

    /**
     * @brief Gets the number of existing subscriptions to the publisher topic.
     * @details
     * Queries the ROS 2 middleware to determine how many subscribers
     * are currently connected to this publisher's topic.
     *
     * @return Number of active subscriptions.
     *
     * @note This count may change as nodes join or leave the ROS 2 network.
     */
    virtual size_t getSubscriptionCount() = 0;

    /**
     * @brief Checks whether the publisher is valid.
     * @details
     * Verifies if the object holds a properly initialized ROS 2 publisher instance.
     *
     * @return True if the publisher is valid, false otherwise.
     *
     * @note A publisher may become invalid if its associated node or context is destroyed.
     */
    virtual bool isValid() = 0;
};

/**
 * @class Ros2Subscriber
 * @brief Base class for ROS 2 subscribers.
 * @details
 * Provides the interface for subscribing to messages from ROS 2 topics.
 * Subscribers receive data published by other ROS 2 nodes.
 */
class Ros2Subscriber
{
public:
    /**
     * @brief Processes incoming messages from the topic.
     * @details
     * Checks for and retrieves any available messages from the subscription queue.
     * If a message is available, it is copied into the provided message structure.
     *
     * @param[out] msg Pointer to store the received message.
     * @return True if a message was received, false otherwise.
     *
     * @pre The msg pointer must point to a valid message structure of the correct type.
     * @pre The subscriber must be valid (check with isValid()).
     */
    virtual bool spin(void* msg) = 0;

    /**
     * @brief Checks whether the subscriber is valid.
     * @details
     * Verifies if the object holds a properly initialized ROS 2 subscriber instance.
     *
     * @return True if the subscriber is valid, false otherwise.
     *
     * @note A subscriber may become invalid if its associated node or context is destroyed.
     */
    virtual bool isValid() = 0;
};

/**
 * @class Ros2Service
 * @brief Base class for ROS 2 service servers.
 * @details
 * Provides the interface for implementing ROS 2 service servers.
 * Service servers receive requests and send responses in a request-response pattern.
 */
class Ros2Service
{
public:
    /**
     * @brief Processes incoming service requests.
     * @details
     * Checks for and retrieves any pending service requests.
     * If a request is available, it is copied into the provided message structure.
     *
     * @param[out] requestMsg Pointer to store the received request message.
     * @return True if a request was received, false otherwise.
     *
     * @pre The requestMsg pointer must point to a valid request message structure of the correct type.
     * @pre The service server must be valid (check with isValid()).
     */
    virtual bool takeRequest(void* requestMsg) = 0;

    /**
     * @brief Sends a response to a service request.
     * @details
     * Sends the provided response message back to the client that made the request.
     *
     * @param[in] responseMsg Pointer to the response message to send.
     * @return True if the response was sent successfully, false otherwise.
     *
     * @pre The responseMsg pointer must point to a valid response message structure of the correct type.
     * @pre The service server must have previously received a request via takeRequest().
     * @pre The service server must be valid (check with isValid()).
     */
    virtual bool sendResponse(void* responseMsg) = 0;

    /**
     * @brief Checks whether the service server is valid.
     * @details
     * Verifies if the object holds a properly initialized ROS 2 service server instance.
     *
     * @return True if the service server is valid, false otherwise.
     *
     * @note A service server may become invalid if its associated node or context is destroyed.
     */
    virtual bool isValid() = 0;
};

/**
 * @class Ros2Client
 * @brief Base class for ROS 2 service clients.
 * @details
 * Provides the interface for implementing ROS 2 service clients.
 * Service clients send requests and receive responses in a request-response pattern.
 */
class Ros2Client
{
public:
    /**
     * @brief Sends a request to a service.
     * @details
     * Sends the provided request message to the associated service server.
     *
     * @param[in] requestMsg Pointer to the request message to send.
     * @return True if the request was sent successfully, false otherwise.
     *
     * @pre The requestMsg pointer must point to a valid request message structure of the correct type.
     * @pre The service client must be valid (check with isValid()).
     *
     * @note This method does not wait for a response. Use takeResponse() to retrieve the response.
     */
    virtual bool sendRequest(void* requestMsg) = 0;

    /**
     * @brief Retrieves the response to a service request.
     * @details
     * Checks for and retrieves the response to a previously sent request.
     * If a response is available, it is copied into the provided message structure.
     *
     * @param[out] responseMsg Pointer to store the received response message.
     * @return True if a response was received, false otherwise.
     *
     * @pre The responseMsg pointer must point to a valid response message structure of the correct type.
     * @pre The service client must have previously sent a request via sendRequest().
     * @pre The service client must be valid (check with isValid()).
     */
    virtual bool takeResponse(void* responseMsg) = 0;

    /**
     * @brief Checks whether the service client is valid.
     * @details
     * Verifies if the object holds a properly initialized ROS 2 service client instance.
     *
     * @return True if the service client is valid, false otherwise.
     *
     * @note A service client may become invalid if its associated node or context is destroyed.
     */
    virtual bool isValid() = 0;
};

/**
 * @class Ros2MessageInterface
 * @brief Base class for ROS 2 message definition/generation via ROS Interface Definition Language (IDL).
 * @details
 * Provides facilities for dynamically loading and creating ROS 2 message types
 * using the ROS IDL mechanism. This class handles the loading of message type
 * libraries and creation/destruction of message instances.
 */
class Ros2MessageInterface
{
public:
    /**
     * @brief Constructor for Ros2MessageInterface.
     * @details
     * Initializes the message interface by loading the required ROS IDL libraries
     * for the specified message type.
     *
     * @param[in] pkgName Message package name (e.g.: `"std_msgs"` for `std_msgs/msg/Int32`).
     * @param[in] msgSubfolder Message subfolder name (e.g.: `"msg"` for `std_msgs/msg/Int32`).
     * @param[in] msgName Message name (e.g.: `"Int32"` for `std_msgs/msg/Int32`).
     * @param[in] messageType Message type specification.
     * @param[in] showLoadingError Whether to print ROS IDL libraries load errors to the console.
     *
     * @note The full message type is constructed as `pkgName`/`msgSubfolder`/`msgName`.
     */
    Ros2MessageInterface(std::string pkgName,
                         std::string msgSubfolder,
                         std::string msgName,
                         BackendMessageType messageType = BackendMessageType::eMessage,
                         bool showLoadingError = false)
        : m_pkgName(pkgName), m_msgSubfolder(msgSubfolder), m_msgName(msgName), m_msgType(messageType)
    {
        m_generatorLibrary = std::make_shared<isaacsim::core::includes::LibraryLoader>(
            std::string(m_pkgName) + "__rosidl_generator_c", "", showLoadingError);
        m_typesupportLibrary = std::make_shared<isaacsim::core::includes::LibraryLoader>(
            std::string(m_pkgName) + "__rosidl_typesupport_c", "", showLoadingError);
        m_typesupportIntrospectionLibrary = std::make_shared<isaacsim::core::includes::LibraryLoader>(
            std::string(m_pkgName) + "__rosidl_typesupport_introspection_c", "", showLoadingError);
    }

    /**
     * @brief Gets the pointer to the struct containing ROS IDL message type support data.
     * @details
     * Retrieves a pointer to the ROS IDL type support structure for the message.
     * The pointer points to a ROS IDL struct if the message is found.
     * It is resolved by calling the `rosidl_typesupport_c` type support handle symbol
     * for the given message name.
     *
     * Message type | ROS IDL struct
     * --- | ---
     * Topic (Message) | `rosidl_message_type_support_t`
     * Service | `rosidl_service_type_support_t`
     * Action | `rosidl_action_type_support_t`
     *
     * @return The pointer to the struct, or nullptr if not found.
     *
     * @warning This method may return nullptr if the necessary ROS IDL libraries
     *          are not available in the system.
     */
    void* getTypeSupportHandleDynamic()
    {
        return m_typesupportLibrary->callSymbol<void*>("rosidl_typesupport_c__get_" + getTypeSupportSpec(false) +
                                                       "_type_support_handle__" + std::string(m_pkgName) + "__" +
                                                       std::string(m_msgSubfolder) + "__" + std::string(m_msgName));
    }

    /**
     * @brief Gets the pointer to the struct containing the ROS IDL introspection data.
     * @details
     * Retrieves a pointer to the ROS IDL introspection structure for the message.
     * The pointer points to a ROS IDL struct if the message is found.
     * It is resolved by calling the `rosidl_typesupport_introspection_c` type support handle symbol
     * for the given message name and type.
     *
     * Message type | ROS IDL struct
     * --- | ---
     * Topic (Message) | `rosidl_message_type_support_t`
     * Service | `rosidl_service_type_support_t`
     * Action | `rosidl_message_type_support_t`
     *
     * @return The pointer to the struct, or nullptr if not found.
     *
     * @warning This method may return nullptr if the necessary ROS IDL introspection libraries
     *          are not available in the system.
     */
    void* getTypeSupportIntrospectionHandleDynamic()
    {
        return m_typesupportIntrospectionLibrary->callSymbol<void*>(
            "rosidl_typesupport_introspection_c__get_" + getTypeSupportSpec(true) + "_type_support_handle__" +
            std::string(m_pkgName) + "__" + std::string(m_msgSubfolder) + "__" + std::string(m_msgName) +
            getMessageSpec(true));
    }

    /**
     * @brief Creates a new ROS 2 message instance.
     * @details
     * Allocates memory for a new message of the specified type and initializes it.
     * The pointer is resolved by calling the `rosidl_generator_c` `__create` symbol
     * for the given message.
     *
     * @return Pointer to the newly created message, or nullptr if creation failed.
     *
     * @note The returned message must be destroyed using the destroy() method
     *       to prevent memory leaks.
     */
    void* create()
    {
        return m_generatorLibrary->callSymbol<void*>(std::string(m_pkgName) + "__" + std::string(m_msgSubfolder) +
                                                     "__" + std::string(m_msgName) + getMessageSpec(false) + "__create");
    }

    /**
     * @brief Destroys a ROS 2 message.
     * @details
     * Finalizes the message by freeing its allocated memory.
     * The function is resolved by calling the `rosidl_generator_c` `__destroy` symbol
     * for the given message.
     *
     * @param[in] msg Pointer to the message to destroy.
     *
     * @pre The msg pointer must be a valid message previously created with create().
     *
     * @note This method does nothing if msg is nullptr.
     */
    template <typename T>
    void destroy(T msg)
    {
        if (!msg)
        {
            return;
        }
        m_generatorLibrary->callSymbolWithArg<void>(std::string(m_pkgName) + "__" + std::string(m_msgSubfolder) + "__" +
                                                        std::string(m_msgName) + getMessageSpec(false) + "__destroy",
                                                    msg);
    }

protected:
    std::string m_pkgName; //!< Message package name.
    std::string m_msgSubfolder; //!< Message subfolder name.
    std::string m_msgName; //!< Message name.
    BackendMessageType m_msgType; //!< Message type.
    std::shared_ptr<isaacsim::core::includes::LibraryLoader> m_typesupportIntrospectionLibrary; //!< ROS IDL type
                                                                                                //!< support
                                                                                                //!< introspection
                                                                                                //!< library.
    std::shared_ptr<isaacsim::core::includes::LibraryLoader> m_typesupportLibrary; //!< ROS IDL type support library.
    std::shared_ptr<isaacsim::core::includes::LibraryLoader> m_generatorLibrary; //!< ROS IDL generator library.

private:
    std::string getTypeSupportSpec(const bool& introspection)
    {
        switch ((m_msgType))
        {
        case BackendMessageType::eMessage:
            return "message";
        case BackendMessageType::eRequest:
        case BackendMessageType::eResponse:
            return "service";
        case BackendMessageType::eGoal:
        case BackendMessageType::eResult:
        case BackendMessageType::eFeedback:
        case BackendMessageType::eSendGoalRequest:
        case BackendMessageType::eSendGoalResponse:
        case BackendMessageType::eFeedbackMessage:
        case BackendMessageType::eGetResultRequest:
        case BackendMessageType::eGetResultResponse:
            return introspection ? "message" : "action";
        default:
            break;
        }
        return "";
    }
    std::string getMessageSpec(const bool& introspection)
    {
        switch ((m_msgType))
        {
        case BackendMessageType::eMessage:
            return "";
        case BackendMessageType::eRequest:
            return introspection ? "" : "_Request";
        case BackendMessageType::eResponse:
            return introspection ? "" : "_Response";
        case BackendMessageType::eGoal:
            return "_Goal";
        case BackendMessageType::eResult:
            return "_Result";
        case BackendMessageType::eFeedback:
            return "_Feedback";
        case BackendMessageType::eSendGoalRequest:
            return "_SendGoal_Request";
        case BackendMessageType::eSendGoalResponse:
            return "_SendGoal_Response";
        case BackendMessageType::eFeedbackMessage:
            return "_FeedbackMessage";
        case BackendMessageType::eGetResultRequest:
            return "_GetResult_Request";
        case BackendMessageType::eGetResultResponse:
            return "_GetResult_Response";
        default:
            break;
        }
        return "";
    }
};

}
}
}
