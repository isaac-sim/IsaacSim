# Public API for module isaacsim.robot.experimental.manipulators.examples:

No public API

# Public API for module isaacsim.robot.experimental.manipulators.examples.interactive.bin_filling:

## Classes

- class BinFilling(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)
  - async def on_fill_bin_event_async(self)

- class BinFillingExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.robot.experimental.manipulators.examples.interactive.follow_target_ik:

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
  - async def start_following_async(self) -> bool
  - async def stop_following_async(self) -> bool

- class UR10FollowTargetExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.robot.experimental.manipulators.examples.interactive.follow_target_motion_generation:

## Classes

- class FollowTarget(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)
  - def obstacles_exist(self) -> bool

# Public API for module isaacsim.robot.experimental.manipulators.examples.interactive.path_planning:

## Classes

- class PathPlanning(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)
  - def walls_exist(self) -> bool
  - def is_trajectory_active(self) -> bool

- class PathPlanningExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.robot.experimental.manipulators.examples.interactive.pick_place:

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
  - async def execute_pick_place_async(self) -> bool | None

- class FrankaPickPlaceExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.robot.experimental.manipulators.examples.interactive.replay_follow_target:

## Classes

- class ReplayFollowTarget(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)

# Public API for module isaacsim.robot.experimental.manipulators.examples.interactive.robo_factory:

## Classes

- class RoboFactory(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def physics_cleanup(self)
