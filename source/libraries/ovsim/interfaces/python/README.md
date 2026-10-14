# OV SIM Python interfaces

This directory defines the shared Python callable aliases for the OV SIM API. They are currently source-only: no
distribution installs them, and no Python provider is checked against them yet.

The sibling C++ interfaces are shared by `isaacsim-foundation`, `isaacsim-physics`, and `isaacsim-ovsim`. Each package
installs a copy of the C++ headers so its SDK remains self-contained. They do not install these Python aliases; their
Python APIs use package-specific bindings backed by the verified C++ contracts.

When a Python OV SIM provider is added, it must expose concrete free functions whose signatures match the aliases it
implements and add provider-side static verification.

## Layout

```
ovsim/
├── __init__.py
└── interfaces/
    ├── __init__.py
    ├── data/
    │   └── __init__.py        # ovsim.interfaces.data
    └── control/
        ├── __init__.py
        ├── authoring/
        │   └── __init__.py    # ovsim.interfaces.control.authoring
        └── simulation/
            └── __init__.py    # ovsim.interfaces.control.simulation
```

## Verification

To check a provider statically, assign each implementation function to a variable annotated with the corresponding
alias in `verify_interfaces.py`. Replace `provider` below with the implementation module:

```python
import provider

import ovsim.interfaces.control.authoring as authoring_interface
import ovsim.interfaces.control.simulation as simulation_interface
import ovsim.interfaces.data as data_interface

_create_stage: authoring_interface.CreateStageFunction = provider.control.authoring.create_stage
_open_stage: authoring_interface.OpenStageFunction = provider.control.authoring.open_stage
_import_stage: authoring_interface.ImportStageFromStringFunction = provider.control.authoring.import_stage_from_string
_define_prim: authoring_interface.DefinePrimFunction = provider.control.authoring.define_prim
_play: simulation_interface.PlayFunction = provider.control.simulation.play
_initialize: simulation_interface.InitializeFunction = provider.control.simulation.initialize
_read: data_interface.ReadFunction = provider.data.read
_write: data_interface.WriteFunction = provider.data.write
```

A static type checker such as mypy or pyright reports signatures that do not match. These aliases do not perform
runtime validation.

## Notes

- Default argument values are not captured by `Callable` aliases. Packages may differ in which parameters carry
  defaults without triggering a static type checker error here; document canonical defaults in the package module.
- Callers invoking through an alias must pass every parameter explicitly, even those that have defaults in the
  underlying implementation.
