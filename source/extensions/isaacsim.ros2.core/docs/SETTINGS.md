```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

### exts."isaacsim.ros2.bridge".ros_distro
- **Default Value**: "system_default"
- **Description**: ROS 2 Bridge settings are centralized in the core extension. ROS 2 distributions to fall back to if none were sourced.

### exts."isaacsim.ros2.bridge".publish_without_verification
- **Default Value**: false
- **Description**: Whether ROS 2 publishers are allowed to publish even if there is no active subscription for their topics.

### exts."isaacsim.ros2.bridge".publish_multithreading_disabled
- **Default Value**: false
- **Description**: Whether to disable multithreading in the *ROS2PublishImage* OmniGraph node.

### exts."isaacsim.ros2.bridge".publish_with_queue_thread
- **Default Value**: true
- **Description**: Whether to enable the queue-based publish thread in the *ROS2PublishImage* OmniGraph node.

### exts."isaacsim.ros2.bridge".publish_queue_thread_sleep_us
- **Default Value**: 1000
- **Description**: How long the above queue-based publish thread sleeps between publishes in microseconds.

### exts."isaacsim.ros2.bridge".enable_nitros_bridge
- **Default Value**: false
- **Description**: Whether to enable image publishing via NITROS.
