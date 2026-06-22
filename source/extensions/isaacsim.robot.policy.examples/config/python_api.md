# Public API for module isaacsim.robot.policy.examples:

# Public API for module isaacsim.robot.policy.examples.robots:

## Classes

- class AnymalFlatTerrainPolicy(PolicyController)
  - def __init__(self, prim_path: str, root_path: str | None = None, usd_path: str | None = None, position: list[float] | None = None, orientation: list[float] | None = None, policy_path: str | None = None, env_config_path: str | None = None)
  - def forward(self, dt: float, command: object)
  - def initialize(self)

- class FrankaOpenDrawerPolicy(PolicyController)
  - def __init__(self, prim_path: str, cabinet: Articulation, root_path: str | None = None, usd_path: str | None = None, position: list[float] | None = None, orientation: list[float] | None = None)
  - def forward(self, dt: float)
  - def initialize(self)

- class Go2FlatTerrainPolicy(PolicyController)
  - def __init__(self, prim_path: str, root_path: str | None = None, usd_path: str | None = None, position: list[float] | None = None, orientation: list[float] | None = None, policy_path: str | None = None, env_config_path: str | None = None)
  - def forward(self, dt: float, command: object)

- class H1FlatTerrainPolicy(PolicyController)
  - def __init__(self, prim_path: str, root_path: str | None = None, usd_path: str | None = None, position: list[float] | None = None, orientation: list[float] | None = None, policy_path: str | None = None, env_config_path: str | None = None)
  - def forward(self, dt: float, command: object)
  - def initialize(self)

- class SpotFlatTerrainPolicy(PolicyController)
  - def __init__(self, prim_path: str, root_path: str | None = None, usd_path: str | None = None, position: list[float] | None = None, orientation: list[float] | None = None, policy_path: str | None = None, env_config_path: str | None = None)
  - def forward(self, dt: float, command: object)

# Public API for module isaacsim.robot.policy.examples.interactive.franka:

## Classes

- class FrankaExample(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def on_physics_step(self, dt: float, context: object)
  - def physics_cleanup(self)

- class FrankaExampleExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.robot.policy.examples.interactive.go2:

## Classes

- class Go2Example(BaseSample)
  - def __init__(self)
  - async def load_world_async(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def on_physics_step(self, dt: float, context: object)
  - def physics_cleanup(self)

- class Go2ExampleExtension(omni.ext.IExt)
  - def on_startup(self, ext_id: str)
  - def on_shutdown(self)

# Public API for module isaacsim.robot.policy.examples.interactive.humanoid:

## Classes

- class HumanoidExample(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def on_physics_step(self, dt: float, context: object)
  - def physics_cleanup(self)

# Public API for module isaacsim.robot.policy.examples.interactive.quadruped:

## Classes

- class QuadrupedExample(BaseSample)
  - def __init__(self)
  - def setup_scene(self)
  - async def setup_post_load(self)
  - async def setup_pre_reset(self)
  - async def setup_post_reset(self)
  - async def setup_post_clear(self)
  - def on_physics_step(self, dt: float, context: object)
  - def physics_cleanup(self)
