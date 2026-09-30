---
name: generate-incident-config
description: "Author and validate the IRI event config YAML defining topple, fire, and spill incidents. Use when writing or debugging an incident config; to drive events use run-incident-events."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
  owner: Isaac Sim
  service: isaacsim.replicator.incident
  version: "1.0.0"
  reviewed: "2026-07-27"
---

# Generate an IRI Incident Config

## Purpose

Produce the YAML that defines which incidents happen in a scene and when — the
file the **Event Config File** panel loads and that
`IncidentManager.setup_incidents_from_config_file()` consumes. Covers the three
event types in `isaacsim.replicator.incident.core` (topple, fire, spill), their
item groups, and the three trigger types.

## Prerequisites

- An installed Isaac Sim carrying `isaacsim.replicator.incident.core`
  (`$ISAAC_SIM_DIR`) — needed only to read the extension version, not to run.
- Python 3 with PyYAML for the generator script. No GPU, no Kit session.
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$WORKSPACE_DIR`.

## Limitations

- **This repo does not build the extension.** `isaacsim.replicator.incident.core`
  is authored in the `metrosim` repo and consumed here from `extscache`. This
  skill describes and generates its config format; schema changes belong upstream.
- The config alone does nothing. Items must carry incident tags in the USD stage
  first, and the config must then be applied to a live stage — both belong to
  `run-incident-events`.
- Validation here is **structural and offline**. It cannot tell you whether
  `/World/Box01` exists or is tagged; only a live stage can.
- Schema-backed validation needs a build whose `isaacsim.replicator.incident.core`
  ships `data/config_schema/iri_config.schema.json`. Against builds without it the
  script falls back to its own equivalent checks, which must be kept in step with
  the extension by hand.
- `FireEvent` accepts no radius. `PyroEventProperty` defines `flammable_item.item`
  and nothing else, and `generate_pyro_event_from_property` hardcodes the nearby
  radius to `1.0` with the config read commented out. The extension's own shipped
  `default_config.yaml` still carries a `flammable_nearby_radius` key that has no
  effect.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| `does not contain header:isaacsim.replicator.incident` | Top-level header missing or misspelled — note it is **not** `.core` | Nest everything under `isaacsim.replicator.incident:` |
| `version info is missing` | No `version` key inside the header block | Add `version:`; generate with `--from-ext` to get it right |
| `Invalid config file version` | **Major** component differs from the installed extension | Only the major is compared — any `0.x` config version passes against a `0.x` build |
| A key you set has no effect | `EventPropertyBase.set_value` copies only keys it already knows; unknown keys are dropped with no error | Run `--validate`; it reports every dropped key |
| `--validate` reports `mode=builtin` unexpectedly | The build ships no schema file, no `--from-ext` given, or `jsonschema>=4.0` is not importable | The `note:` line names the cause; pass `--from-ext "$ISAAC_SIM_DIR"`, or `--require-schema` to make it fatal |
| `[seed] value must be a non-negative number` | `global.seed` is negative | Use a non-negative integer |
| Events load but never fire | Event has no `trigger` block | Every event needs a trigger; `--validate` flags this as an error |
| Only the first event happens | Several events target the same random sentinel and tagged items ran out | Tag more items, or pin `item:` to specific prim paths |

## When to use this skill

| You want | Skill |
|---|---|
| Write / fix / validate an incident config YAML | **this skill** |
| Tag prims, apply a config to a live stage, record a report | `run-incident-events` |
| Caption a scene or build scene graphs for VLM training | `vlm-scene-captioning` |
| Static-scene Replicator SDG with the standard writers | `data-collection-sim` |

## Config anatomy

```yaml
isaacsim.replicator.incident:      # header — the extension family, not ".core"
  version: EXT_VERSION             # from the installed extension; only the MAJOR component is compared
  global:
    report_dir: /path/to/output    # where incidents_report.json lands
    seed: 123456                   # non-negative int; seeds random item selection
  event:
    event_list:
    - ToppleEvent:                 # one single-key mapping per event
        name: shelf collapse       # also the incident report's top-level key
        topple_item:
          item: $random_loose_item$
          topple_nearby_radius: 1.5
        trigger:
          type: time
          time: 3
```

| Event key | Item group | Item keys |
|---|---|---|
| `ToppleEvent` | `topple_item` | `item`, `topple_nearby_radius` |
| `FireEvent` | `flammable_item` | `item` |
| `SpillEvent` | `leakable_item` | `item`, `target_size`, `leak_duration` |

`item` takes a tagged prim path or one of `$random_loose_item$`,
`$random_flammable_item$`, `$random_leakable_item$`. Full key semantics,
trigger schemas, and the loader's parsing rules are in
[`references/config-schema.md`](references/config-schema.md); the failure modes
that produce a silently wrong config are in
[`references/authoring-notes.md`](references/authoring-notes.md). An index of both is in [`references/README.md`](references/README.md).

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/starter_iri_config.py` | Generate an IRI config from event specs, or validate an existing one against the extension schema when available | CLI flags via argparse (see script `--help`) |

## Running scripts

The script is offline. From agent runtimes that expose skill execution helpers:

```
run_script("scripts/starter_iri_config.py", args=["--help"])
```

Directly:

```bash
python3 skills/generate-incident-config/scripts/starter_iri_config.py \
    --from-ext "$ISAAC_SIM_DIR" \
    --event "topple:time=3,radius=1.5" \
    --event "fire:time=6,item=/World/Pallet" \
    --event "spill:time=9,target_size=1.5,leak_duration=5" \
    --report-dir "$WORKSPACE_DIR/EventsResult" \
    --output "$WORKSPACE_DIR/iri_config.yaml"
```

## Workflow

1. **Generate.** Pass `--from-ext "$ISAAC_SIM_DIR"` so the `version` is read from
   the installed build rather than guessed. With no `--event` flags you get one
   of each type on a staggered time trigger.

2. **Aim the events.** Leave `item` at its random sentinel to let the extension
   pick among tagged prims, or set `item=/World/Box01` to pin a specific one. A
   pinned path must be tagged on the stage — this skill cannot check that.

3. **Validate.** Run before every handoff. It reports version-major mismatches,
   missing triggers, bad seeds, and — most usefully — keys the loader would drop
   in silence.

   ```bash
   python3 scripts/starter_iri_config.py --validate "$WORKSPACE_DIR/iri_config.yaml" \
       --from-ext "$ISAAC_SIM_DIR"
   ```

   Expect `IRI_CONFIG_OK: <path> mode=... errors=0 warnings=0`. Exit code is `1`
   when errors are present, so it drops straight into CI.

   `mode=` reports which rule set was used — see [Validation modes](#validation-modes).
   Add `--strict` to treat unknown keys as errors rather than warnings.

4. **Hand off.** Give the path to `run-incident-events` `action=setup`, or load
   it from the **Event Config File** panel. The config is *not* stored in the USD
   stage — it saves and loads separately from the scene.

## Validation modes

Newer `isaacsim.replicator.incident.core` builds ship a JSON Schema at
`data/config_schema/iri_config.schema.json`. When it is present the script
validates against it; otherwise it uses its own equivalent checks. Selection is
automatic, and the result line reports which ran:

| `mode=` | Rules used | When |
|---|---|---|
| `schema` | The extension's shipped `iri_config.schema.json` — the same document the extension validates with at load time | A build that ships the schema, reachable via `--from-ext`, and `jsonschema>=4.0` importable |
| `builtin` | Equivalent checks written into this script | Build without the schema, no `--from-ext`, or no Draft 2020-12 `jsonschema` available |

Prefer `mode=schema` when you can: it cannot drift from the extension. `builtin`
exists so the script works everywhere, including against builds that predate the
schema — the two agree on every fixture the extension ships.

| Flag | Effect |
|---|---|
| `--from-ext <path>` | Finds the schema automatically (also derives the version) |
| `--schema <path>` | Points at a schema explicitly |
| `--require-schema` | Fails instead of falling back to `builtin`. Use in CI once the pinned build ships the schema |
| `--strict` | Unknown keys become errors instead of warnings, matching the extension's `strict_config_validation` default |

## Editing an existing config by hand

Editing directly is fine; the risk is that mistakes are silent. `set_value()`
walks the keys it already knows and ignores the rest, so a typo leaves the
default in place and the run proceeds with the wrong value. Re-run `--validate`
after any hand edit.

Two starting points ship with the extension:

- `<ext>/config/default_config.yaml` — one of each event type. Note it declares
  a stale `version` (which still passes, since only the major is compared) and
  carries the inert `flammable_nearby_radius` key.
- The **Event Config File** panel's save button, which always writes the running
  build's version.

## Related skills

- `run-incident-events` — tags prims, applies this config to a live stage, records the report.
- `isaac-sim-remote` — the transport `run-incident-events` rides on.
- `data-collection-sim` — Replicator writers that capture the incident semantic labels.
- `isaac-sim-orchestrator` — env-var contract and skill routing.
