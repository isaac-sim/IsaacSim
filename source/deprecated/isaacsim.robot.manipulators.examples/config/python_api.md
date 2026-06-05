# Public API for module isaacsim.robot.manipulators.examples:

No public API

# Public API for module isaacsim.robot.manipulators.examples.interactive.pick_place:

## Classes

- class FrankaPickPlaceInteractive(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)
  - def get_controller_status(self) -> dict
  - def is_executing(self) -> bool
  - async def execute_pick_place_async(self)

- class FrankaPickPlaceExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.robot.manipulators.examples.interactive.follow_target:

## Classes

- class UR10FollowTargetInteractive(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)
  - def get_controller_status(self) -> dict
  - def is_following(self) -> bool
  - def set_ik_method(self, method: str)
  - async def start_following_async(self)
  - async def stop_following_async(self)

- class UR10FollowTargetExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)
