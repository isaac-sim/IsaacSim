---
name: behavior-tree-generation
description: "LLM-driven Behavior Tree Generation for Isaac Sim: turn a natural-language scenario into behavior-tree files. Use when generating a tree, authoring context/schema, or scripting the planner."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
---

# Behavior Tree Generation

## Purpose

Turn a natural-language scenario into behavior-tree output using an LLM-driven planner. This is a focused sub-skill of Action and Event Data Generation, packaged as `omni.ai.behavior_tree_gen.core` (scripted pipeline + API) and `omni.ai.behavior_tree_gen.bridge` (Kit UI).

## Prerequisites

- Installed Isaac Sim with the Action and Event Data Generation app (`$ISAAC_SIM_DIR`).
- NVIDIA GPU with a current driver (`nvidia-smi`) for an actual run (offline helper scripts need neither).
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$WORKSPACE_DIR`.
- `$NVIDIA_API_KEY` for the chat/embedding models (`prepare_runtime` fails without it).

## Limitations

- Requires a valid NVIDIA API key; `prepare_runtime()` does not fall back to a local model.
- Strict call order — `setup_workspace()` → `prepare_runtime()` (must return `success=True`) → `generate_behavior_tree()`.
- Bundled example actions (e.g. `MoveTo`) are transitional: they demonstrate extensibility, not production quality, and can misbehave.
- This *generates* a behavior tree from text; it is distinct from the hand-authored actor `behavior_tree` JSON consumed by an Actor SDG (`isaacsim.replicator.agent`) config.

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/starter_context.py` | Emit a starter actor/object context JSON or metadata schema | CLI flags via argparse (see script --help) |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke with `run_script()`:

```
run_script("scripts/starter_context.py", args=["--help"])
```

Turn a **natural-language scenario** into **behavior-tree output** using an LLM-driven planner.
Part of Isaac Sim's Action and Event Data Generation feature (launch the app with
`isaac-sim.action_and_event_data_generation.sh`). This skill covers only the behavior-tree
generation workflow.

## When to use (vs siblings)

Use to **turn a natural-language scenario into a behavior tree** (LLM pipeline). Distinct from the
hand-authored actor `behavior_tree` config that an Actor SDG (`isaacsim.replicator.agent`) group
*consumes* — this *generates* the tree.

## Environment

Follows the library env-var contract (see `isaac-sim-orchestrator`): `$ISAAC_SIM_DIR`,
`$WORKSPACE_DIR`. Needs `$NVIDIA_API_KEY` for the chat/embedding models (`prepare_runtime` fails
without it). Write outputs to `$WORKSPACE_DIR/bt` instead of a hardcoded path.

> **Not the same as the actor `behavior_tree` config key.** An Actor SDG (`isaacsim.replicator.agent`)
> group *consumes* a hand-authored JSON behavior tree to drive a character/robot group. This skill *generates* a
> behavior tree from a text scenario via an LLM pipeline. The output of this workflow can seed
> the trees the actor skill runs, but the two are different systems.

## Extensions

| Extension | Role |
|---|---|
| `omni.ai.behavior_tree_gen.core` | Reusable pipeline + public scripted API (`...core.api`). |
| `omni.ai.behavior_tree_gen.bridge` | Kit UI windows, bundled example loaders; wraps the core API. The bridge loads the core as a dependency. |

## Run (UI)

1. Enable `omni.ai.behavior_tree_gen.bridge` (it pulls in `.core`).
2. Open **Tools > Behavior Tree Gen**.
3. Optional: **Window > Examples > Behavior Tree Gen Examples** → load the bundled **Basic
   Scene** or **Warehouse Scene**. This loads a demo stage and pre-fills the workflow panels.
4. In **Behavior Tree Gen**: confirm the **Context Cache Files** (context JSON, node catalogs,
   metadata schemas), the **Network Config** (NVIDIA API key + model JSON), and the **Output
   Settings** folder; enter the scenario text in the **Planner** panel; click **Run Pipeline**.

Output behavior-tree files are written under the selected output folder; planner/RAG cache goes
under the derived cache directory.

## Run (scripted API)

The UI is a thin wrapper over three public calls in `omni.ai.behavior_tree_gen.core.api`, used
**in this exact order** — each prepares state the next consumes:

```python
import os
from pathlib import Path
from omni.ai.behavior_tree_gen.core import api as core_api

OUTPUT_DIR = Path(os.environ["WORKSPACE_DIR"]) / "bt"   # not "Your/Output/Folder/Path"

session = core_api.setup_workspace(          # 1. sync — build the reusable PlannerSession
    cache_dir=str(OUTPUT_DIR / "planner_cache"),
    output_dir=str(OUTPUT_DIR),
    context_data_paths=actor_context_paths + object_context_paths,
    node_catalog_paths=node_catalog_paths,
    actor_schema_path=actor_schema_path,
    object_schema_path=object_schema_path,
)

runtime = await core_api.prepare_runtime(    # 2. async — configure LLM/embeddings/RAG/Action IR
    session,
    api_key=API_KEY,                          # NVIDIA API key (UI, carb setting, or NVIDIA_API_KEY)
    model_selection_config_path=model_selection_config_path,
)
if not runtime.success:
    raise RuntimeError(runtime.message)

result = await core_api.generate_behavior_tree(session, SCENARIO)   # 3. async — emit the tree
if not result.success:
    raise RuntimeError(result.error_message)
print(result.behavior_tree_folder_path)
```

`setup_workspace()` is synchronous; `prepare_runtime()` and `generate_behavior_tree()` are
coroutines. In Script Editor, wrap all three in one `async def` and
`asyncio.ensure_future(run())`. See `references/api-and-inputs.md` for the full parameter list,
return fields, and the required-inputs breakdown.

## Required inputs (minimum)

- **Scenario text** — the natural-language goal.
- **Output folder** — writable; holds generated trees + reusable cache.
- **NVIDIA API key** — needed by `prepare_runtime()` for NVIDIA-hosted chat/embedding models
  (from the UI, a carb setting, or the `NVIDIA_API_KEY` env var).
- **Context JSON** — actor + object instances (`ActorInfo` / `InteractableObjectInfo`).
- **Node-catalog JSON** — the behavior-tree nodes the planner may use.
- **Metadata schemas** — actor/object JSON Schemas that give `metadata` fields meaning.

**Authoring context/schema:** prefer the bundled example files under the bridge's
`data/example/context_info/` (and `.../schemas/`) as your reference — they match the current
build. As an optional offline quick-start you can also generate a starter context + schema pair
(then edit them):

```bash
python3 scripts/starter_context.py --entity object --id Table > table_context.json
python3 scripts/starter_context.py --emit-schema object > object_metadata_schema.json
```

## Verify it worked

```bash
# result.behavior_tree_folder_path is the authoritative location; it lives under the output_dir
# you passed to setup_workspace ($WORKSPACE_DIR/bt).
ls "$WORKSPACE_DIR/bt" 2>/dev/null && echo "tree written" || echo "no tree — check NVIDIA_API_KEY + that prepare_runtime returned success"
```

A successful run sets `result.success` and writes tree files under the output folder; failures are
almost always a missing `$NVIDIA_API_KEY` or `prepare_runtime` not returning success before
`generate_behavior_tree`.

## Integration points

- **Consumes:** actor/object **context JSON** + **node-catalog JSON** + **metadata schemas** + a
  scenario string; an `$NVIDIA_API_KEY`.
- **Produces:** behavior-tree output files that can seed the `behavior_tree` key of an Actor SDG
  (`isaacsim.replicator.agent`) group.

## Troubleshooting

- **Call order** — `setup_workspace` → `prepare_runtime` → `generate_behavior_tree`.
  `prepare_runtime()` must return `success=True` before `generate_behavior_tree()` works.
- **Missing API key** — `prepare_runtime()` fails without a valid NVIDIA API key; it does not
  fall back to a local model.
- **Context vs schema** — context supplies instance data; the schema defines the `metadata`
  structure. Base fields (`id`, `semantic_description`, `supported_interactions`,
  `entity_type`) stay top-level; schema-defined fields go under `metadata`. Required by the
  shipped schemas: actors need `metadata.prim_path` + `metadata.actor_type`; objects need
  `metadata.prim_path` + `metadata.interactable_type`.
- **Stale workspace** — after editing a tracked input file (context, catalog, schema, model
  config), reload the workspace so the typed models rebuild.
- **Example actions are transitional** — bundled custom actions (e.g. `MoveTo`) can misbehave
  (paths overlapping the target); they demonstrate extensibility, not production quality.
