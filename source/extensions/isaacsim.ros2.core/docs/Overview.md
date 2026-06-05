# Overview

The isaacsim.ros2.core extension provides the foundational C++ backend that enables ROS 2 integration within Isaac Sim. It implements a Carbonite plugin exposing the `Ros2Bridge` interface, manages distribution-specific factory libraries, and supplies Python utilities for environment setup, camera information, namespace collection, and test scaffolding. All other ROS 2 bridge extensions (e.g., isaacsim.ros2.bridge) depend on this core extension.

<div align="center">

```{mermaid}
graph TD
    Plugin["Carbonite Plugin<br/>(Ros2Bridge)"]
    Factory["Ros2Factory (abstract)"]
    Backend["Per-Distro Backend Library<br/>(humble / jazzy)"]
    Context["Ros2ContextHandle<br/>(rcl_context_t)"]
    Node["Ros2NodeHandle<br/>(rcl_node_t)"]
    Pub["Ros2Publisher"]
    Sub["Ros2Subscriber"]
    Srv["Ros2Service"]
    Cli["Ros2Client"]
    Msg["Typed Messages<br/>(Clock, Image, Imu, ...)"]
    Dyn["Ros2DynamicMessage<br/>(runtime type loading)"]

    Plugin -->|getFactory| Factory
    Plugin -->|getDefaultContextHandleAddr| Context
    Factory -.->|implemented by| Backend
    Factory -->|createContextHandle| Context
    Factory -->|createNodeHandle| Node
    Factory -->|createPublisher| Pub
    Factory -->|createSubscriber| Sub
    Factory -->|createService| Srv
    Factory -->|createClient| Cli
    Factory -->|create*Message| Msg
    Factory -->|createDynamicMessage| Dyn
```

</div>

## Key Components

### Ros2Bridge (Carbonite Plugin Interface)

**The `Ros2Bridge` struct is the primary Carbonite plugin interface that bootstraps and exposes all ROS 2 functionality.** It is loaded by the Python extension at startup from the `isaacsim.ros2.core.plugin` shared library.

Key capabilities:
- `getDefaultContextHandleAddr` — Returns the memory address of the default `Ros2ContextHandle`, encapsulating the `rcl_context_t` init/shutdown cycle state used when creating ROS 2 nodes
- `getFactory` — Returns the `Ros2Factory` instance for the detected ROS 2 distribution
- `getStartupStatus` — Verifies that both the factory and context handler are properly instantiated
- `addHandle` / `getHandle` / `removeHandle` — Handle registry for tracking ROS 2 entity lifetimes

### Ros2Factory (Abstract Factory)

**`Ros2Factory` is an abstract base class whose concrete implementation is provided by the per-distro backend library.** It exposes factory methods for creating every ROS 2 communication primitive and message type.

Factory methods include:
- **Communication primitives**: `createContextHandle`, `createNodeHandle`, `createPublisher`, `createSubscriber`, `createService`, `createClient`
- **Typed messages**: `createClockMessage`, `createImuMessage`, `createCameraInfoMessage`, `createImageMessage`, `createCompressedImageMessage`, `createNitrosBridgeImageMessage`, `createBoundingBox2DMessage`, `createBoundingBox3DMessage`, `createOdometryMessage`, `createRawTfTreeMessage`, `createTfTreeMessage`, `createSemanticLabelMessage`, `createJointStateMessage`, `createPointCloudMessage`, `createLaserScanMessage`, `createTwistMessage`, `createAckermannDriveStampedMessage`
- **Dynamic messages**: `createDynamicMessage` — Creates messages at runtime from package/subfolder/name triples (e.g., `"std_msgs"`, `"msg"`, `"Int32"`), supporting topics, service request/response, and action goal/result/feedback types
- **Validation**: `validateTopicName`, `validateNamespaceName`, `validateNodeName`

### Communication Abstractions

The C++ layer defines base classes for all ROS 2 communication entities:

- **`Ros2ContextHandle`** — Wraps `rcl_context_t` with `init`, `shutdown`, `isValid`, and `getContext` methods. Supports optional domain ID override.
- **`Ros2NodeHandle`** — Wraps `rcl_node_t`, providing access to the owning context and raw node pointer.
- **`Ros2Publisher`** — Publishes messages via `publish`, queries active subscription count with `getSubscriptionCount`.
- **`Ros2Subscriber`** — Receives messages via `spin`, which checks the subscription queue and copies data into the provided message structure.
- **`Ros2Service`** — Service server that receives requests via `takeRequest` and responds via `sendResponse`.
- **`Ros2Client`** — Service client that sends requests via `sendRequest` and retrieves responses via `takeResponse`.

### Message System

The extension provides two approaches to message handling:

**Typed messages** — Pre-defined C++ classes (e.g., `Ros2ClockMessage`, `Ros2ImageMessage`, `Ros2PointCloudMessage`) that directly wrap ROS 2 message structs for common sensor and navigation message types.

**Dynamic messages** — The `Ros2DynamicMessage` class loads arbitrary ROS 2 message types at runtime through the ROS IDL introspection libraries. It supports both JSON and vector-based read/write interfaces, field metadata introspection via `DynamicMessageField`, and OmniGraph-compatible type mapping. The `BackendMessageType` enum covers topics, service request/response, and full action lifecycle (goal, result, feedback, and their send/get wrappers).

### Dynamic Backend Loading

At startup, the extension detects the sourced ROS 2 distribution from the `ROS_DISTRO` environment variable. If no distribution is sourced, it falls back to a default based on the Ubuntu version (Humble for 22.04, Jazzy for 24.04) and loads internal ROS 2 libraries bundled with the extension. When a sourced distribution does not have a matching backend, the extension falls back to the Jazzy backend, leveraging ROS 2 C API compatibility across distributions.

The per-distro backend shared libraries (located in the `library/` directory) provide the concrete `Ros2Factory` implementation that links against the appropriate ROS client library (`rcl`) and message packages for each distribution.

### Python Bindings

The `isaacsim.ros2.core.bindings._ros2_core` module exposes the `Ros2Bridge` interface to Python, providing `acquire_ros2_core_interface` and `release_ros2_core_interface` functions for plugin lifecycle management.

## Python Utilities

### Environment Setup (`ros2_common`)

- `get_ubuntu_version` — Detects the Ubuntu version and maps it to the supported ROS 2 distribution (Humble or Jazzy)
- `restore_ros2_python_paths` — Re-adds ROS 2 Python paths to `sys.path` that Isaac Sim removed at startup, using `OLD_PYTHONPATH` cross-referenced against `AMENT_PREFIX_PATH`
- `setup_ros2_environment` — Configures platform-specific environment variables for ROS 2 library loading
- `print_environment_setup_instructions` — Outputs setup instructions when automatic configuration fails

### Camera Info (`camera_info_utils`)

- {func}`read_camera_info <isaacsim.ros2.core.read_camera_info>` — Reads camera prim attributes from a render product path and populates a `sensor_msgs/msg/CameraInfo` message with intrinsic parameters, distortion model (plumb_bob, rational_polynomial, or equidistant), and projection matrices. Supports OpenCV Pinhole, OpenCV Fisheye, and legacy physical distortion models.
- {func}`compute_relative_pose <isaacsim.ros2.core.compute_relative_pose>` — Computes the translation and rotation matrix between two camera prims for stereo camera configurations.

### Namespace Collection (`collect_namespace`)

- {func}`collect_namespace <isaacsim.ros2.core.collect_namespace>` — Traverses the USD prim hierarchy upwards from a camera prim, collecting `isaac:namespace` attributes to build a ROS 2 namespace string. Returns the input namespace as-is if one is explicitly provided.

### Test Utilities (`ros2_test_case`)

- {class}`ROS2TestCase <isaacsim.ros2.core.ROS2TestCase>` — Extends `TimedAsyncTestCase` with ROS 2 lifecycle management. Provides `create_node`, `create_publisher`, `create_subscription`, `start_async_spinning`, and `simulate_until_condition` helper methods with automatic cleanup of all ROS 2 resources during teardown.

## Configuration

The extension centralizes ROS 2 bridge settings under `exts."isaacsim.ros2.bridge"`:

| Setting | Description |
|---|---|
| `ros_distro` | ROS 2 distribution to fall back to if none is sourced (default: `"system_default"`) |
| `publish_without_verification` | Allow publishers to publish without active subscriptions (default: `false`) |
| `publish_multithreading_disabled` | Disable multithreading in the ROS2PublishImage OmniGraph node (default: `false`) |
| `publish_with_queue_thread` | Enable queue-based publish thread for image publishing (default: `true`) |
| `publish_queue_thread_sleep_us` | Sleep interval for the queue-based publish thread in microseconds (default: `1000`) |
| `enable_nitros_bridge` | Enable image publishing via NITROS for optimized GPU image transport (default: `false`) |

## Integration

The extension integrates with the Carbonite framework for plugin loading, **omni.kit** for extension lifecycle management, and the ROS 2 Client Library (rcl) at the C API level. It serves as the foundation for all ROS 2 OmniGraph nodes and bridge components in Isaac Sim. The Python layer uses USD APIs directly for render product and transformation queries, integrates with **isaacsim.sensors.experimental.rtx** for camera and sensor APIs, **isaacsim.core.experimental.utils** for stage utilities, and **isaacsim.core.simulation_manager** for test fixtures.
