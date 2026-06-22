# Public API for module isaacsim.core.utils:

# Public API for module isaacsim.core.utils.commands:

## Classes

- class IsaacSimSpawnPrim(omni.kit.commands.Command)
  - def __init__(self, usd_path: str, prim_path: str, translation: carb.Float3 = (0, 0, 0), rotation: carb.Float4 = (0, 0, 0, 1))
  - def do(self) -> bool
  - def undo(self)

- class IsaacSimTeleportPrim(omni.kit.commands.Command)
  - def __init__(self, prim_path: str, translation: carb.Float3 = (0, 0, 0), rotation: carb.Float4 = (0, 0, 0, 1))
  - def do(self) -> bool
  - def undo(self)

- class IsaacSimScalePrim(omni.kit.commands.Command)
  - def __init__(self, prim_path: str, scale: carb.Float3 = (0, 0, 0))
  - def do(self) -> bool
  - def undo(self)

- class IsaacSimDestroyPrim(omni.kit.commands.Command)
  - def __init__(self, prim_path: str)
  - def do(self) -> bool
  - def undo(self)

## Functions

- def get_current_stage(fabric: bool = False) -> Usd.Stage | usdrt.Usd._Usd.Stage
- def get_current_stage_id() -> int

## Variables

- transforms: Unknown
