# Public API for module isaacsim.ros2.nodes:

## Classes

- class ROS2NodesExtension(omni.ext.IExt)
  - def on_startup(self, ext_id)
  - def on_shutdown(self)
  - def register_nodes(self)
  - def unregister_nodes(self)

- class CompressedImageManager
  - class def attach(cls, render_product_path: str)
  - class def detach(cls, render_product_path: str)
  - class def get_writer(cls, render_product_path: str, use_system_time: bool = False) -> rep.Writer
