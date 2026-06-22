
# Public API for module isaacsim.examples.interactive.kaya_gamepad:

## Classes

- class KayaGamepad(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)

- class KayaGamepadExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.examples.interactive.omnigraph_keyboard:

## Classes

- class OmnigraphKeyboard(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)

- class OmnigraphKeyboardExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.examples.interactive.robo_party:

## Classes

- class RoboParty(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)

- class RoboPartyExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.examples.interactive.hello_world:

## Classes

- class HelloWorld(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - def world_cleanup(self)

- class HelloWorldExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.examples.interactive.surface_gripper:

No public API

# Public API for module isaacsim.examples.interactive.user_examples:

No public API

# Public API for module isaacsim.examples.interactive.getting_started:

## Classes

- class GettingStarted(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)

- class GettingStartedExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

- class GettingStartedRobot(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - def on_physics_step(self, step_size: float, context: object)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)

- class GettingStartedRobotExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)
