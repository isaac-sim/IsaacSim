# OV SIM C++ interfaces

This directory contains the canonical C++ callable-signature aliases for the OV SIM API. It is shared interface source,
not a separately registered module or distribution.

Header ownership is package-level: every distribution whose public SDK exposes or includes these interfaces installs an
identical copy at `include/ovsim/interfaces`. This keeps each SDK self-contained without making one distribution the
owner of another distribution's public includes. The canonical source remains this directory; package CMake files must
install it unchanged rather than maintain private copies.

The current consumers are:

- `isaacsim-foundation`, whose OV SIM authoring, simulation, and data provider headers include the canonical aliases;
- `isaacsim-physics`, whose OV SIM authoring, simulation, and data provider headers include the same aliases; and
- `isaacsim-ovsim`, whose local and gRPC clients, public API, protocol support, and server use the aliases and whose
  package installs one shared copy for its public SDK.

Concrete providers must expose free functions whose signatures match every alias they implement.

## Layout

```
ovsim/interfaces/
├── control/
│   ├── authoring/
│   │   └── Authoring.hpp   # ovsim::interfaces::control::authoring
│   └── simulation/
│       └── Simulation.hpp  # ovsim::interfaces::control::simulation
├── data/
│   └── Data.hpp            # ovsim::interfaces::data
└── details/
    └── Exception.hpp       # Shared implementation exceptions
```

## Verification

Provider-side `VerifyInterfaces.cpp` translation units enforce the current contract at compile time for Foundation,
Physics, and the local OV SIM client. They include each applicable canonical interface header alongside the concrete
header, then assign concrete function addresses to variables of the alias types. For example:

```cpp
#include <isaacsim/foundation/ovsim/control/authoring/Authoring.hpp>
#include <isaacsim/foundation/ovsim/control/simulation/Simulation.hpp>
#include <isaacsim/foundation/ovsim/data/Data.hpp>
#include <ovsim/interfaces/control/authoring/Authoring.hpp>
#include <ovsim/interfaces/control/simulation/Simulation.hpp>
#include <ovsim/interfaces/data/Data.hpp>

namespace authoringInterface = ovsim::interfaces::control::authoring;
namespace simulationInterface = ovsim::interfaces::control::simulation;
namespace dataInterface = ovsim::interfaces::data;
namespace foundationControl = isaacsim::foundation::ovsim::control;
namespace foundationData = isaacsim::foundation::ovsim::data;

[[maybe_unused]] const dataInterface::ReadFunction g_kRead = &foundationData::read;
[[maybe_unused]] const authoringInterface::CreateStageFunction g_kCreateStage =
    &foundationControl::authoring::createStage;
[[maybe_unused]] const simulationInterface::PlayFunction g_kPlay = &foundationControl::simulation::play;
```

A mismatching return type, parameter type, or parameter count causes a compile error on that assignment.

## Notes

- Default argument values are not part of a callable signature. Packages may differ in which parameters carry defaults
  without triggering a compile error here; document canonical defaults in the package header.
- Callers invoking through an alias must pass every parameter explicitly, even those that have defaults in the
  concrete declaration.
