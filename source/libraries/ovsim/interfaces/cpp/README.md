# OV SIM C++ interfaces

This directory defines the shared C++ function-signature aliases for the OV SIM API. It is source used by several
packages, not a separate module or distribution.

Every distribution whose public SDK uses these interfaces installs the same headers at `include/ovsim/interfaces`.
This keeps each SDK self-contained without making packages depend on another package's include directory. This
directory remains the source of truth; package CMake files install it unchanged instead of maintaining copies.

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

Each provider has a `VerifyInterfaces.cpp` file that checks the contract at compile time. It includes the shared
interface and the provider's concrete header, then assigns each function to the corresponding alias type. For example:

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

The assignment fails to compile if the return type, parameter types, or parameter count do not match.

## Notes

- Default argument values are not part of a callable signature. Packages may differ in which parameters carry defaults
  without triggering a compile error here; document canonical defaults in the package header.
- Callers invoking through an alias must pass every parameter explicitly, even those that have defaults in the
  concrete declaration.
