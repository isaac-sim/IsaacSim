---
name: physx-to-newton-conversion
description: "Convert a legacy PhysX-only USD asset (for example an Isaac Sim 5.x robot) to a Newton-capable package with the PhysX to MJC Conversion preset. Use when a PhysX asset must also run under Newton."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim <isaac-sim@nvidia.com>
---

# Legacy PhysX -> Newton Conversion

## Purpose

Add Newton-readable physics schemas to an existing PhysX-authored USD robot, then restructure it into a variant package that loads under either engine. The conversion authors the `Mjc*` schemas that Newton's MuJoCo solver consumes (`MjcJointAPI`, `MjcActuator`) and Newton's own schemas (`NewtonMimicAPI`, `NewtonArticulationRootAPI`). The package keeps them in separate layers: the `mujoco` variant holds the `Mjc*` schemas, `physics` holds solver-neutral USD Physics plus `Newton*`, and `physx` holds the `Physx*` schemas. Use this for assets that already simulate in PhysX; for a new robot description use `urdf-mjcf-to-usd-conversion`.

This skill orchestrates the existing conversion in `isaacsim.asset.importer.utils` (`physx_asset_to_mjc.convert_physx_asset_to_mjc`, the same utilities the URDF/MJCF importers use) through `PhysxToMjcConversionRule`; it adds sequencing, audits and checks, not new conversion logic. Fix conversion behavior in those extensions, not here.

## Prerequisites

- Built Isaac Sim (`$ISAAC_SIM_DIR`) with `isaacsim.asset.transformer.rules` and `isaacsim.physics.newton`; the runtime check needs a GPU.
- A legacy USD with authored `PhysicsDriveAPI` gains and a `PhysxArticulationAPI` articulation root (that is what gains `NewtonArticulationRootAPI`).

## Limitations

- Targets Isaac Sim 6 / Kit 110 unless a section states otherwise.
- **No PhysX parity guarantee.** PhysX-to-Newton is a broad area; Newton does not cover every PhysX feature, and PhysX support is not a Newton priority. Treat the output as a Newton starting point to validate and tune, and expect to adjust physics parameters (contacts, solver iterations, timestep). The Newton backend is experimental and has been tested on a limited set of robots (G1, H1, T1, UR5e, Wonik Allegro, Shadow Hand).
- Newton rejects or mishandles several PhysX-authored traits that this conversion cannot fix: reversed joint direction (`body0` must be the parent), zero-size collision shapes, joints outside an articulation root, USD composition errors, negative scale on colliders, `metersPerUnit` other than 1, and mass plus density without `diagonalInertia`. See the Limitations section of `docs/isaacsim/physics/newton_physics.rst`. The script's `PREFLIGHT:` warnings cover all of these except reversed joints and composition errors.
- **`mjc:ref` is not authored from drive targets.** `mjc:ref` is the joint zero point (radians or meters), not a drive target, so the conversion does not copy `targetPosition` into it. Builds without this change copied the value unchanged, which put degrees in a radians field and shifted the zero point: the Isaac Sim 6.0/6.1 Franka Panda went non-finite on its `mujoco` variant, and with the value converted to radians it still settled outside its joint limits. The PhysX target pose is therefore not carried into the `mujoco` variant; on the Franka the joints settle at their own equilibrium inside the limits. Author an initial pose separately if you need one. The conversion leaves an existing `mjc:ref` as it is, because a stale copy of the drive target cannot be told apart from a genuine reference (MJCF-origin assets carry `mjc:ref` equal to their target). Re-convert from the original PhysX asset, not from an older build's output. The script's preflight warns when the input already has a non-zero revolute `mjc:ref`, and it warns again after conversion.
- `PhysxTendon*` schemas (for example the Shadow Hand finger tendons) are **not** converted and produce no conversion warning; they remain in the `physx` variant only, so tendon-coupled motion is absent under Newton.
- Only revolute and prismatic joints convert. Generic D6 and spherical joints are warned about and left untouched, so they stay PhysX-only.
- The conversion reports success even when a joint gets no usable drive gains; see [Missing gains](#missing-gains).
- Pass 1 exports only the input's root layer (`flatten_source` is off) and copies its dependencies into `source_assets/`. For a heavily layered asset (sublayers/payloads with their own relative references), flatten it first (`stage.Flatten().Export(...)`) or verify the package opens without unresolved paths.
- Does not replace official NVIDIA documentation for unsupported edge cases.

## Troubleshooting

| Error / symptom | Cause | Solution |
|---|---|---|
| Looked for a reverse or URDF rule and found `MjcToPhysxConversionRule` / `UrdfToMjcPhysxConversionRule` | Wrong direction: those go MJC->PhysX or URDF->both | Use `PhysxToMjcConversionRule` (preset **PhysX to MJC Conversion**) |
| Pass 1 passes but a joint's actuator has no `mjc:gainPrm` | The PhysX drive has damping = 0 (including stiffness-only position drives) or the joint has no drive | Author damping (and stiffness for position control) on the joint drive, then re-run from the original asset |
| `Stiffness and damping not available joint ...` warning | Same as above, one per joint | Same fix |
| Warning about D6 / spherical joints | Multi-DOF joints are not converted | Replace with single-axis joints, or accept PhysX-only behavior for those joints |
| Pass 2 interface is named `mujoco.usda` | Pass 2 names its interface after its input file (pass 1's `payloads/mujoco.usda`) | Set `profile.interface_asset_name = "<robot>.usda"` before pass 2 (the script does this), or rename the interface file |
| No `MjcActuator` found / audit sees nothing | The interface's `Physics` variant has no default selection | Select `Physics = mujoco` before traversing or simulating |
| Active engine is `physx`, not `newton` | Manual run: `default_engine` not set. Script run: `isaacsim.physics.newton` failed to load | Launch with `--/exts/isaacsim.core.simulation_manager/default_engine=newton`; for the script, check the log for the Newton extension error |
| `non-zero mjc:ref` warning, or `--check-runtime` reports non-finite DOFs on the `mujoco` variant (the `physx` variant is fine) | An older converter build copied the PhysX drive target (degrees) into `mjc:ref` (radians zero point) | Re-convert the original PhysX asset with a current build (re-converting an older build's output keeps the value), or delete `mjc:ref` on the revolute joints in `out/package/payloads/Physics/mujoco.usda` and re-run the check |
| `PREFLIGHT:` warning | The source asset has a trait Newton does not support (see Limitations) | Fix it in the source asset first; the conversion does not repair it |
| Newton reports `Reversed joints are not supported` | PhysX asset has `body0`/`body1` swapped on a joint | Swap them on the affected joints in the source asset, then re-convert |
| Script refuses with `... is not empty` | Re-running into the same `--output`; the transformer would reuse stale layers | Delete `pass1/` and `package/` under the output, or pick a new `--output` |

## Available Scripts

| Script | Purpose | Arguments |
|---|---|---|
| `scripts/convert_physx_to_newton.py` | Run both passes, audit gains, optional Newton runtime check | `--input`, `--output`, `--check-runtime`, `--steps`, `--device`, `--require-gains` |

## Running scripts

From agent runtimes that expose skill execution helpers, invoke helpers with `run_script()`:

```python
run_script("scripts/convert_physx_to_newton.py", args=["--help"])
```

From a built Isaac Sim tree, run the same file with `./python.sh` (Linux) or `python.bat` (Windows) from `_build/*/release`.

## Workflow

Two Asset Transformer passes, in this order. Both profiles ship in `isaacsim.asset.transformer.rules/data/`.

| Pass | Profile (`data/`) | Rule | Result |
|---|---|---|---|
| 1 | `physx_to_mjc_conversion.json` (**PhysX to MJC Conversion**) | `PhysxToMjcConversionRule` | `payloads/mujoco.usda`: the source root layer plus `Mjc*`/`Newton*` schemas and `<defaultPrim>/Physics/<joint>_actuator` prims |
| 2 | `isaacsim_structure.json` (**Isaac Sim Structure**) | structure/routing rules | Variant package: interface `<name>.usda` with `Physics` variants `mujoco`, `physx`, `physics`, `none` (no default selection), plus `payloads/` |

Pass 2 takes pass 1's `payloads/mujoco.usda` as its input. Write each pass into a fresh directory.

### One command

```bash
# Linux (from _build/linux-x86_64/release)
./python.sh <skill>/scripts/convert_physx_to_newton.py --input legacy_robot.usd --output out/ --check-runtime
# Windows (from _build/windows-x86_64/release); forward slashes work in python.bat paths
python.bat <skill>/scripts/convert_physx_to_newton.py --input legacy_robot.usd --output out/ --check-runtime
```

Writes `out/pass1/` and `out/package/` (interface `out/package/legacy_robot.usda`), then prints `PREFLIGHT:` warnings, pass results, the gain audit, and (with `--check-runtime`) the Newton check. Exit code is nonzero on any failed rule, on zero converted actuators, or on a failed runtime check; add `--require-gains` to also fail on missing gains. With no arguments it converts a built-in synthetic asset as a self-test that fails unless exactly the gainless non-mimic joint is flagged and Newton steps the result.

### Step by step (standalone runner)

`source/standalone_examples/api/isaacsim.asset.transformer/run_asset_transformer.py` runs one profile at a time:

```bash
RULES=<isaacsim.asset.transformer.rules>/data
./python.sh .../run_asset_transformer.py --input legacy_robot.usd --profile $RULES/physx_to_mjc_conversion.json --output out/pass1
./python.sh .../run_asset_transformer.py --input out/pass1/payloads/mujoco.usda --profile $RULES/isaacsim_structure.json --output out/package
```

This produces `out/package/mujoco.usda` as the interface (named after pass 2's input). In the Asset Transformer UI, select the **PhysX to MJC Conversion** preset and click **Execute Actions**, then repeat with **Isaac Sim Structure** on the result. Programmatic use: `AssetTransformerManager().run(input_stage=..., profile=RuleProfile.from_json(...), package_root=...)`; set `profile.interface_asset_name` on the pass 2 profile to control the interface name.

### Missing gains

The rule writes `mjc:gainPrm`/`mjc:biasPrm` only when the PhysX drive has damping > 0 (stiffness > 0 gives position control, stiffness = 0 velocity control). A stiffness-only drive, a drive with both gains at 0, or a joint without a drive produces an `MjcActuator` with no gains, and the rule still passes. Audit the `mujoco` variant before calling the asset converted:

```python
from pxr import Usd

stage = Usd.Stage.Open("out/package/legacy_robot.usda")
stage.SetEditTarget(stage.GetSessionLayer())  # do not dirty the interface file
stage.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics").SetVariantSelection("mujoco")
for prim in stage.Traverse():
    if prim.GetTypeName() == "MjcActuator":
        gains = prim.GetAttribute("mjc:gainPrm").Get()
        if not gains or not any(gains):
            print("no gains:", prim.GetPath())
```

This snippet also reports mimic followers (joints with `NewtonMimicAPI`); those are driven by their leader joint, not an actuator, so ignore them. `convert_physx_to_newton.py` runs the same audit and skips mimic followers automatically. Fix by authoring damping (and stiffness) on the joint's `PhysicsDriveAPI` in the source asset and re-running pass 1. For tuning, continue with the Gain Tuner workflow in the docs, then validate with `usd-articulation`.

### Worked example: Isaac Sim 5.1 ShadowHand

Run on the real 5.1 asset (`Isaac/Robots/ShadowRobot/ShadowHand/shadow_hand.usd` under the 5.1 asset root; the 6.1 root also ships a hand-authored `shadow_hand_instanceable_newton.usd` for comparison):

- Pass 1 and pass 2 pass. The source has 24 revolute joints and **no** `PhysicsDriveAPI`, so all 24 actuators come out gainless (`WARNING: 24 of 24 MjcActuator(s) have no gains`), and Newton steps the 24 DOFs as a passive, finite system.
- `PREFLIGHT` flags the 8 tendon prims; they survive only in the `physx` variant.
- Authoring `PhysicsDriveAPI:angular` stiffness 5 / damping 1 on all 24 joints in a copy of the source (the values the shipped 6.1 Newton asset uses) and re-running gives `Gain audit: all 24 MjcActuators have gains`, and `--require-gains --check-runtime` exits 0.

### Worked example: Isaac Sim 6.1 Franka Panda

`Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd` (6.0 and 6.1 are identical in this respect) is a layered, PhysX-only asset: 27 layers, `Mesh`/`Gripper` variant sets, 8 drives, a mimic finger joint. Converting it directly (no flattening):

- Both passes pass, all 9 actuators get gains (kp 400, kd 80, `forceRange` from `maxForce`), and the package's `Gripper`/`Mesh` variants are preserved.
- With the `mjc:ref` change, no `mjc:ref` is authored, `--check-runtime` passes, and every joint settles inside its limits. On a build without it, `--check-runtime` **fails**: the `mujoco` variant is non-finite by frame 5 (5 revolute joints get `mjc:ref` equal to their degree targets, e.g. 174.0) while the `physx` variant under Newton and the raw asset stay finite. Converting those values to radians is finite but leaves joint4 and joint6 outside their limits.
- Pass 1's `payloads/mujoco.usda` alone is not loadable (`USD stage has composition errors`): its relative references point at the original location. Use the pass 2 package, not the pass 1 file.

### Validate

1. **Dependencies:** the package keeps relative references (`payloads/...`, `Textures/`); copy the whole output directory, not only the interface file.
2. **Variants:** open the interface and confirm `Physics` offers `mujoco` and `physx` (`prim.GetVariantSets().GetVariantSet("Physics").GetVariantNames()`).
3. **Bounded Newton run:** select `Physics = mujoco`, run with the Newton engine for a fixed number of frames, and check the articulation resolves with DOFs and DOF positions are finite (`--check-runtime`, default 120 frames). Failing this check means the asset is not training-ready; passing it is a smoke test only and does not show that actuated joints track their targets.
4. **PhysX fallback:** select `Physics = physx` under the PhysX engine to confirm the original behavior survived.

## Related Skills

- `urdf-mjcf-to-usd-conversion` -- importing a new URDF/MJCF (not an existing PhysX asset).
- `physics-simulation` -- Newton vs PhysX backend selection and solver settings.
- `usd-articulation` -- validating the resulting articulation.
- `usd-composition-architecture` -- the layered layout the package follows.
- `docs/isaacsim/physics/newton_physics.rst` (Limitations) and `docs/isaacsim/robot_setup/asset_transformer_rules.rst` (rule reference).
