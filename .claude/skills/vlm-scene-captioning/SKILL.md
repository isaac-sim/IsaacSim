---
name: vlm-scene-captioning
description: "Generate image + caption pairs and structured scene graphs (IRC, isaacsim.replicator.caption.core) for VLM training. Use when captioning scenes or wiring IRC into an actor/object SDG run."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
---

# VLM Scene Captioning (IRC)

## Purpose

Generate image + caption pairs and scene graphs from a loaded scene for training vision-language models — a focused sub-skill of Action and Event Data Generation (`isaacsim.replicator.caption.core`). It builds a scene graph from 3D ground truth, then calls an NVIDIA NIM LLM to produce brief / global / QA captions.

## Prerequisites

- Installed Isaac Sim with the Action and Event Data Generation app (`$ISAAC_SIM_DIR`).
- NVIDIA GPU with a current driver (`nvidia-smi`) for an actual run (offline helper scripts need neither).
- Shell env contract from `isaac-sim-orchestrator`: `$ISAAC_SIM_DIR`, `$WORKSPACE_DIR`.
- `$NVIDIA_API_KEY` for caption generation (scene-graph-only needs none).

## Limitations

- Captions require `$NVIDIA_API_KEY`; NIM credits are limited and expire (scene-graph-only export needs no key).
- A standalone config's `version` must exactly match `settings.VERSION` of the installed `isaacsim.replicator.caption.core` — **not** its `[package] version`. The two have diverged; `scripts/starter_irc_config.py --from-ext` reads the former.
- `load_config_file()` does not apply `camera_prim_path` / `output_path`; they must also be set via `ReplicatorCaptionSettings` (see the `CaptionAPI` example).
- Prims without a semantic label are not captured by annotators and never enter the scene graph.
- `camera_prim_path` must resolve to a real camera in the scene, or IRC will not run.

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/starter_irc_config.py` | Generate a standalone IRC caption config (offline quick-start) | CLI flags via argparse (see script --help) |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke with `run_script()`:

```
run_script("scripts/starter_irc_config.py", args=["--help"])
```

Generate **image + caption pairs** and **scene graphs** from a loaded scene, for training
vision-language models. Part of Isaac Sim's Action and Event Data Generation feature; IRC is the
`isaacsim.replicator.caption.core` extension (launch the app with
`isaac-sim.action_and_event_data_generation.sh`).

## When to use (vs siblings)

Use to **caption scenes / build scene graphs** for VLM training — standalone, via `CaptionAPI`, or
as a per-frame writer inside Actor or Object SDG. It does not simulate anything itself.

## Environment

Follows the library env-var contract (see `isaac-sim-orchestrator`): `$ISAAC_SIM_DIR` (build/install
root), `$WORKSPACE_DIR` (scratch/output root). Needs `$NVIDIA_API_KEY` for captions (scene-graph-only
needs no key). Write outputs under `$WORKSPACE_DIR` rather than a hardcoded path.

## How it works

IRC first builds a **scene graph** — nodes are objects, edges are spatial relationships derived
from 3D ground truth (a Support Tree rooted at the floor). It then calls an **NVIDIA NIM LLM** to
turn that graph into captions (each is a `caption_configs` toggle):

- **brief** (`brief_caption`) — short description;
- **global** (`global_caption`, labeled **Full Caption** in the UI) — overall scene description;
- **QA** (`qa_caption`) — question/answer pairs testing scene understanding.

Only prims captured by Replicator annotators enter the scene graph, so prims need semantic labels
(`attach_label_to_usd` can auto-generate them from the prim-path basename).

## Three ways to run

**1. UI panel.** Enable `isaacsim.replicator.caption.core`; open **Tools > Action and Event Data
Generation > VLM Scene Captioning**. In **Caption Settings** load a stage USD (a demo
`Samples/Replicator/Captioning/test_caption.usda` ships in Isaac Sim Assets) → **Load Scene**;
enter the **API key** in **Model Settings** → **Accept**; pick caption level (**Brief** / **Full**),
set the **Input Camera Prim Path** + **Output Path** → **Generate Scene Graph**. Position a camera
so a region of interest dominates the view to caption that ROI.

**2. Python `CaptionAPI`.**

```python
import asyncio
from isaacsim.replicator.caption.core.api import CaptionAPI
from isaacsim.replicator.caption.core.settings import ReplicatorCaptionSettings

CaptionAPI.set_model_params(
    url="https://integrate.api.nvidia.com/v1",
    name="deepseek-ai/deepseek-v4-flash-0731",
    key=API_KEY,          # your NVIDIA NIM key (e.g. from the NVIDIA_API_KEY env var)
    max_tokens=16384,     # optional; defaults to the model_max_tokens setting (1024)
    timeout=60.0,         # optional; defaults to the model_timeout setting (30.0)
    enable_thinking=False,  # optional; defaults to the model_enable_thinking setting (off)
)

# A model can resolve to a real deployment and still refuse every request. Check before a long run,
# otherwise the failure only shows up as null captions in scene_graph_caption.json.
ok, message = CaptionAPI.check_model()
if not ok:
    raise RuntimeError(message)

CaptionAPI.load_config_file("/path/to/irc_config.yaml")   # optional: load a standalone config

# REQUIRED even after load_config_file(). It stores the parsed config but does not apply
# camera_prim_path / output_path; get_captions() reads both from ReplicatorCaptionSettings.
# Skip this and the run logs "None is not a valid camera path", writes nothing, and returns None.
ReplicatorCaptionSettings.set_target_camera_prim_path("/World/Cameras/Camera")
ReplicatorCaptionSettings.set_output_folder_path("/path/to/output")

task = asyncio.ensure_future(CaptionAPI.get_captions())    # async; returns None, writes to disk
task.add_done_callback(lambda f: print(f.result()))
```

`get_captions()` is annotated `-> dict` but returns `None`; results land on disk under
`<output_path>/<camera_id>/` (`Data/object_data.json`, `Image/<camera_id>.png`, and the
caption/scene-graph files). Verify the files, not the return value.

**3. As a per-frame writer in another SDG run** (captions every frame at runtime):

- **Inside Actor SDG (IRA)** — add IRC's `SceneGraphWriter` under `replicator.writers` in the
  actor config (`scene_graph_interval` / `caption_interval` control cadence).
- **Inside Object SDG (IRO)** — add a `caption_configs` block with
  `caption_writer: CombinedIROSceneGraphWriter` (or `IROSceneGraphWriter` to suppress IRO's other
  outputs) and set `output_switches.caption: True`.

Neither writer takes model url/name/key/`max_tokens`/`timeout`/`enable_thinking` as a config
param — both call the model with no override arguments, so they always read them from the
persisted extension settings:

```
/persistent/exts/isaacsim.replicator.caption.core/model_max_tokens
/persistent/exts/isaacsim.replicator.caption.core/model_timeout
/persistent/exts/isaacsim.replicator.caption.core/model_enable_thinking
```

These live at the carb-settings layer specifically so every IRC entry point — the UI panel, the
`CaptionAPI`, and any extension embedding `SceneGraphWriter` / `CombinedIROSceneGraphWriter` /
`IROSceneGraphWriter` — shares one place to tune them. Set them once before the run via the Model
Settings UI panel (its fields are bound directly to these settings) or by editing the carb
settings directly, e.g. to work around a model timeout at the source:
`carb.settings.get_settings().set("/persistent/exts/isaacsim.replicator.caption.core/model_timeout", 120.0)`.
`CaptionAPI.set_model_params()` does **not** reach either writer — it only affects the standalone
`CaptionAPI.get_captions()` call path. See `references/config-and-writers.md` for the full setting
list.

See `references/config-and-writers.md` for the standalone config schema, every `caption_configs`
key, the writer parameters, and exact output paths.

**Starting a config:** prefer the extension's shipped `config/default_config.yaml` or the UI panel
— they carry the correct `version` (IRC requires an **exact** match against `settings.VERSION`). As
an offline quick-start you can also generate one; `--from-ext` derives the version from the
installed extension's `settings.py` rather than hardcoding it:

```bash
python3 scripts/starter_irc_config.py --from-ext <build-dir> \
    --camera-prim-path /World/Cameras/Camera --output-path $WORKSPACE_DIR/irc > irc_config.yaml
# --from-ext accepts a build/extscache root, an extension dir, or an extension.toml;
# use --version only if you already know the exact installed version.
```

## Verify it worked

```bash
# captions land under <output>/<Camera>/Captions/
ls "$WORKSPACE_DIR/irc"/*/Captions/scene_graph_caption.json 2>/dev/null && echo "captions written" || echo "no captions — check NVIDIA_API_KEY + that prims have semantic labels"
```

Empty/absent captions usually mean a missing `$NVIDIA_API_KEY` or prims without semantic labels
(unlabeled prims never enter the scene graph).

## Integration points
- **Consumes:** a loaded scene (standalone) OR a live Actor SDG / Object SDG run it attaches to as
  a writer (`SceneGraphWriter` / `CombinedIROSceneGraphWriter`).
- **Produces:** scene graphs (`full`/`pruned`) + `brief`/`global`/`qa` captions under the output
  dir.

## Troubleshooting

- **API key only for captions** — `NVIDIA_API_KEY` is required to generate captions, **not** to
  export scene graphs alone. Set it in the environment (`export NVIDIA_API_KEY=<key>`) or the UI.
  NIM credits are limited and expire.
- **`version` must match `settings.VERSION`, not the extension version** — the config's `version`
  is compared against `VERSION` in the extension's `settings.py`, which is **not** the same as its
  `[package] version` in `extension.toml`. The two diverged when config-file support landed and
  have not been re-synced since, so a config carrying the `[package] version` is always rejected
  with `Config file version <package> does not match the current version <settings>`. Use
  `scripts/starter_irc_config.py --from-ext <build>`, which reads `settings.VERSION`, or copy the
  extension's own `config/default_config.yaml`. The loader also rejects any unknown top-level or
  `caption_configs` key.
- **`load_config_file()` does not set the camera or output path** — it stores the parsed config
  only. `get_captions()` reads both from `ReplicatorCaptionSettings`, so a config-only setup logs
  `Error as warning: None is not a valid camera path`, writes nothing, and returns `None`. Call
  `ReplicatorCaptionSettings.set_target_camera_prim_path(...)` and `.set_output_folder_path(...)`
  as well.
- **`Association matrix shape mismatch: expected 3 dimensions, got 1`** — raised from
  `scene_graph/utils.py` during scene-graph construction after the image and `object_data.json`
  are already written. Seen on a stage whose prims were auto-labelled via `attach_label_to_usd`.
  Prefer authored semantic labels; the capture stage succeeding does not mean the graph stage will.
- **A camera must exist** — `camera_prim_path` must resolve to a real camera in the scene, or IRC
  won't run; without `scene_path` it captions whatever stage is already loaded.
- **Unlabeled prims are invisible** — prims without a semantic label aren't captured by annotators
  and never enter the scene graph. Use `attach_label_to_usd` / `use_ai_label` to label them.
- **UI hang on load** — if the panel hangs fetching sample assets, launch with
  `--/persistent/isaac/asset_root/timeout=1.0`.
