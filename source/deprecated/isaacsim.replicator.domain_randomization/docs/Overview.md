# Overview

```{deprecated} 6.0.0
This extension is deprecated in favor of `isaacsim.replicator.experimental.domain_randomization`.
```

The isaacsim.replicator.domain_randomization extension provided domain randomization capabilities specifically designed for Reinforcement Learning (RL) applications. It enabled randomization of simulation parameters with PhysX directly, bypassing USD updates for improved performance during RL training.

## Key Components

### Trigger Module

The {mod}`trigger <isaacsim.replicator.domain_randomization.trigger>` module provides the entry point for domain randomization through {func}`on_rl_frame <isaacsim.replicator.domain_randomization.trigger.on_rl_frame>`, which creates action graph contexts for multi-environment RL scenarios.

### Gate Module

The {mod}`gate <isaacsim.replicator.domain_randomization.gate>` module controls when randomization occurs through timing gates: {func}`on_interval <isaacsim.replicator.domain_randomization.gate.on_interval>` for periodic randomization and {func}`on_env_reset <isaacsim.replicator.domain_randomization.gate.on_env_reset>` for reset-triggered randomization.

### Physics View Module

The {mod}`physics_view <isaacsim.replicator.domain_randomization.physics_view>` module handles view registration and randomization execution. Key functions include {func}`register_simulation_context <isaacsim.replicator.domain_randomization.physics_view.register_simulation_context>`, {func}`register_rigid_prim_view <isaacsim.replicator.domain_randomization.physics_view.register_rigid_prim_view>`, {func}`register_articulation_view <isaacsim.replicator.domain_randomization.physics_view.register_articulation_view>`, {func}`randomize_simulation_context <isaacsim.replicator.domain_randomization.physics_view.randomize_simulation_context>`, {func}`randomize_rigid_prim_view <isaacsim.replicator.domain_randomization.physics_view.randomize_rigid_prim_view>`, {func}`randomize_articulation_view <isaacsim.replicator.domain_randomization.physics_view.randomize_articulation_view>`, and {func}`step_randomization <isaacsim.replicator.domain_randomization.physics_view.step_randomization>`.

## Functionality

The following methods provide randomization functionalities of various parameters 
pertaining to `isaacsim.core.prims.RigidPrim`, `isaacsim.core.prims.Articulation`,
and `isaacsim.core.api.SimulationContext`. These methods are designed to perform randomizations in 
simulations where *update to USD* is disabled for faster simulation speed. These methods directly
set randomized values to PhysX as opposed to existing methods in omni.replicator.core, 
such as `omni.replicator.core.modify.pose` or `omni.replicator.core.physics.physics_material`, which
utilizes USD APIs to set values and hence cannot be used when *update to USD* is disabled. 
Therefore, the following methods provided in `isaacsim.replicator.domain_randomization` are particularly useful 
for domain randomization in Reinforcement Learning (RL) as RL use cases benefit immensely from 
disabling *update to USD* to achieve faster simulation speed.

The following is a simple demo that showcases the workflow and the various features of
`isaacsim.replicator` for domain randomization. This demo script can be launched as
a standalone example, which can be found by going to Isaac Sim's root directory and
go to `standalone_examples/api/isaacsim.replicator.domain_randomization/randomization_demo.py`.

The `isaacsim.replicator` extension for domain randomization functions by constructing
an OmniGraph action graph, which consists of nodes that generate random values, regulate
frequency intervals of various randomization properties, and write the random values to PhysX.
This action graph gets executed according to the way in which the triggers are set up. Note 
that it is necessary to register the views to be randomized before constructing this action graph.

The first step is to create an entry point of the action graph using `with dr.trigger.on_rl_frame(num_envs=num_envs):`.
It is worth noting that all views to be used with this extension must have the same number of 
encapsulated prims, equaling `num_envs` that is passed as an argument to the `on_rl_frame` trigger.

After creating this entry point, there are two types of gates that determine when the nodes can write
to PhysX: `on_interval(interval)` and `on_env_reset()`; these gates work in conjunction with 
`isaacsim.replicator.physics_view.step_randomization(reset_inds)`. There exists an internal
step counter for every environment in the views. Every time the `step_randomization` method is called, 
it resets the counter to zero for every environment listed in the `reset_inds` argument while
incrementing the counter for every other environment. The `on_interval(interval)` gate then ensures 
the nodes to write to PhysX whenever the counter for an environment is a multiple of the `interval`
argument. The `on_env_reset()` gate writes to PhysX whenever an environment's counter gets reset by
the `reset_inds` argument passed in the `step_randomization` method. 

Within each gate, the following methods can be used to randomize view properties: `randomize_simulation_context`, 
`randomize_rigid_prim_view` and `randomize_articulation_view`. For each of these methods, there 
exists an `operation` argument, which can be set to `direct`, `additive`, or `scaling`. `direct` 
sets the random values directly to PhysX, while the behavior of `additive` and `scaling` depends on 
the gate it is controlled by. Under `on_env_reset()`, `additive` adds the random values to the default
values of a given attribute of the view, and `scaling` multiplies the random values to the default values.
Under `on_interval(interval)`, `additive` and `scaling` adds or multiplies the random values to the
last values set during `on_env_reset()`. In essence, `on_env_reset()` randomization should be treated
as *correlated noise* that lasts until the next reset, while `on_interval(interval)` randomization should 
be treated as *uncorrelated noise*.

After setting up this action graph, it is necessary to run `omni.replicator.core.orchestrator.run()`.

```python
 from isaacsim import SimulationApp
 simulation_app = SimulationApp({"headless": False})

 import numpy as np
 from isaacsim.core.api import World
 from isaacsim.core.prims import Articulation, RigidPrim
 from isaacsim.core.utils.prims import get_prim_at_path, define_prim
 from isaacsim.core.utils.stage import get_current_stage, add_reference_to_stage
 from isaacsim.storage.native import get_assets_root_path
 from isaacsim.core.api.objects import DynamicSphere
 from isaacsim.core.cloner import GridCloner

 # create the world
 world = World(stage_units_in_meters=1.0, physics_prim_path="/physicsScene", backend="numpy")
 world.scene.add_default_ground_plane()

 # set up grid cloner
 cloner = GridCloner(spacing=1.5)
 cloner.define_base_env("/World/envs")
 define_prim("/World/envs/env_0")

 # set up the first environment
 DynamicSphere(prim_path="/World/envs/env_0/object", radius=0.1, position=np.array([0.75, 0.0, 0.2]))
 add_reference_to_stage(
     usd_path=get_assets_root_path()+ "/Isaac/Robots/Franka/franka.usd", 
     prim_path="/World/envs/env_0/franka",
 )

 # clone environments
 num_envs = 4
 prim_paths = cloner.generate_paths("/World/envs/env", num_envs)
 env_pos = cloner.clone(source_prim_path="/World/envs/env_0", prim_paths=prim_paths)

 # creates the views and set up world
 object_view = RigidPrim(prim_paths_expr="/World/envs/*/object", name="object_view")
 franka_view = Articulation(prim_paths_expr="/World/envs/*/franka", name="franka_view")
 world.scene.add(object_view)
 world.scene.add(franka_view)
 world.reset()

 num_dof = franka_view.num_dof

 # set up randomization with isaacsim.replicator, imported as dr
 import isaacsim.replicator.domain_randomization as dr
 import omni.replicator.core as rep

 dr.physics_view.register_simulation_context(world)
 dr.physics_view.register_rigid_prim_view(object_view)
 dr.physics_view.register_articulation_view(franka_view)

 with dr.trigger.on_rl_frame(num_envs=num_envs):
     with dr.gate.on_interval(interval=20):
         dr.physics_view.randomize_simulation_context(
             operation="scaling",
             gravity=rep.distribution.uniform((1, 1, 0.0), (1, 1, 2.0)),
         )
     with dr.gate.on_interval(interval=50):
         dr.physics_view.randomize_rigid_prim_view(
             view_name=object_view.name,
             operation="direct",
             force=rep.distribution.uniform((0, 0, 2.5), (0, 0, 5.0)),
         )
     with dr.gate.on_interval(interval=10):
         dr.physics_view.randomize_articulation_view(
             view_name=franka_view.name,
             operation="direct",
             joint_velocities=rep.distribution.uniform(tuple([-2]*num_dof), tuple([2]*num_dof)),
         )
     with dr.gate.on_env_reset():
         dr.physics_view.randomize_rigid_prim_view(
             view_name=object_view.name,
             operation="additive",
             position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
             velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
         )
         dr.physics_view.randomize_articulation_view(
             view_name=franka_view.name,
             operation="additive",
             joint_positions=rep.distribution.uniform(tuple([-0.5]*num_dof), tuple([0.5]*num_dof)),
             position=rep.distribution.normal((0.0, 0.0, 0.0), (0.2, 0.2, 0.0)),
         )
 rep.orchestrator.run()


 frame_idx = 0
 while simulation_app.is_running():
     if world.is_playing():
         reset_inds = list()
         if frame_idx % 200 == 0:
             # triggers reset every 200 steps
             reset_inds = np.arange(num_envs)
         dr.physics_view.step_randomization(reset_inds)
         world.step(render=True)
         frame_idx += 1
```
