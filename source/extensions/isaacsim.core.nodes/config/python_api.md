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
  - def append_writer(self, writer)
  - def attach_writers(self, render_product_path)
  - def attach_writer(self, writer, render_product_path)
  - def post_attach(self, writer, render_product)

- class WriterRequest
  - def __init__(self, writer: rep.Writer, render_product_path: str | list[str], activate: bool = True)
