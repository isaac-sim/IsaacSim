# Public API for module isaacsim.core.nodes:

## Classes

- class BaseResetNode
  - def __init__(self, initialize: bool = False)
  - def on_stop_play(self, event: carb.eventdispatcher.Event)
  - def custom_reset(self)
  - def reset(self)

- class BaseWriterNode(BaseResetNode)
  - def __init__(self, initialize: bool = False)
  - def custom_reset(self)
  - def append_writer(self, writer: rep.Writer)
  - def attach_writers(self, render_product_path: str | list[str])
  - def attach_writer(self, writer: rep.Writer, render_product_path: str | list[str])
  - def post_attach(self, writer: rep.Writer, render_product: str | list[str])

- class WriterRequest
  - def __init__(self, writer: rep.Writer, render_product_path: str | list[str], activate: bool = True)

## Functions

- def register_annotator_from_node_with_telemetry(*args: Any, **kwargs: Any)
- def register_node_writer_with_telemetry(*args: Any, **kwargs: Any)
