# Public API for module isaacsim.simulation_app:

## Classes

- class SimulationApp
  - DEFAULT_LAUNCHER_CONFIG: Dict
  - RENDERER_DEFAULTS: Dict
  - def __init__(self, launch_config: dict = None, experience: str = '')
  - def update(self)
  - def set_setting(self, setting: str, value: Any)
  - def reset_render_settings(self)
  - def run_coroutine(self, coroutine: asyncio.Coroutine, run_until_complete: bool = True) -> asyncio.Task | asyncio.Future | Any
  - def close(self, wait_for_replicator: bool = True, skip_cleanup: bool = False, exit_code: int = 0)
  - def is_running(self) -> bool
  - def is_exiting(self) -> bool
  - [property] def app(self) -> omni.kit.app.IApp
  - [property] def context(self) -> omni.usd.UsdContext

- class AppFramework
  - def __init__(self, name: str = 'kit', argv: list[str] | None = None)
  - def update(self)
  - def close(self)
  - [property] def app(self) -> omni.kit.app.IApp
  - [property] def framework(self) -> typing.Any
