# Public API for module isaacsim.ros2.urdf:

## Classes

- class RobotDefinitionReader
  - def __init__(self)
  - def on_description_received(self, _: str)
  - def service_call(self, node: typing.Any)
  - def start_get_robot_description(self, node_name: str)

## Functions

- def package_path_to_system_path(package_name: str, relative_path: str = '') -> str
- def replace_package_urls_with_paths(input_string: str) -> tuple[str, bool]
