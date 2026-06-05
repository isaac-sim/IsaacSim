# Public API for module isaacsim.replicator.episode_recorder.ui:

## Classes

- class EpisodeRecorderPanel
  - def __init__(self)
  - def build(self)
  - def on_stage_closed(self)
  - def destroy(self)

- class EpisodeRecorderUIExtension(omni.ext.IExt, MenuHelperExtensionFull)
  - WINDOW_NAME: str
  - MENU_GROUP: str
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

- class EpisodeRecorderWindow(ui.Window)
  - def __init__(self, title: str)
  - def destroy(self)
