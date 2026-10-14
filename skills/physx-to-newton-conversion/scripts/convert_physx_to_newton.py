# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Convert a legacy PhysX-only USD asset to a Newton-capable package (Kit 110).

Runs the two Asset Transformer passes, audits the MuJoCo actuators for missing
gains, and optionally runs a bounded Newton runtime check:

1. ``PhysX to MJC Conversion`` preset (``PhysxToMjcConversionRule``) adds the
   ``Mjc*`` / ``Newton*`` schemas on top of the existing PhysX drives.
2. ``Isaac Sim Structure`` profile splits the result into a variant-based package
   whose interface file is named after the input asset.

With no ``--input`` the script builds a tiny synthetic legacy asset (one gained
joint, one gainless joint, one gainless mimic follower) and converts that, so it
doubles as a smoke test: it fails unless exactly the gainless non-mimic joint is
flagged and Newton steps the result.

Run with $ISAAC_SIM_DIR/python.sh (Linux) or python.bat (Windows):

    ./python.sh convert_physx_to_newton.py --input legacy.usd --output out/ --check-runtime
    ./python.sh convert_physx_to_newton.py            # self-test on a synthetic asset
"""

import argparse
import difflib
import os
import sys
import tempfile

parser = argparse.ArgumentParser(description="Convert a legacy PhysX USD asset to a Newton package.")
parser.add_argument(
    "--input", "-i", default=None, help="Legacy PhysX USD to convert (default: synthetic self-test asset)"
)
parser.add_argument(
    "--output",
    "-o",
    default=None,
    help="Output directory; its pass1/ and package/ must not exist yet (default: temp dir)",
)
parser.add_argument("--check-runtime", action="store_true", help="Step the converted asset under Newton and check it")


def _positive_int(value: str) -> int:
    """argparse type: an integer >= 1."""
    number = int(value)
    if number < 1:
        raise argparse.ArgumentTypeError(f"must be >= 1, got {number}")
    return number


parser.add_argument("--steps", type=_positive_int, default=120, help="Frames to step in the runtime check")
parser.add_argument("--device", choices=["cpu", "cuda"], default="cuda", help="Simulation device for the runtime check")
parser.add_argument("--require-gains", action="store_true", help="Exit nonzero when any MjcActuator has no gains")
# Unknown arguments are forwarded to Kit by SimulationApp (e.g. --/some/setting=value), but a
# misspelling of one of ours (e.g. --requre-gains) or a bare USD path without --input would be
# silently ignored -- the latter would even run the self-test instead -- so reject those.
args, unknown = parser.parse_known_args()
_OWN_FLAGS = ["--input", "--output", "--check-runtime", "--steps", "--device", "--require-gains"]
for _arg in unknown:
    if _arg.lower().endswith((".usd", ".usda", ".usdc", ".usdz")) and not _arg.startswith("-"):
        parser.error(f"unexpected positional argument {_arg!r}; pass the asset with --input")
    if _arg.startswith("--") and not _arg.startswith("--/"):
        _close = difflib.get_close_matches(_arg.split("=", 1)[0], _OWN_FLAGS, n=1, cutoff=0.8)
        if _close:
            parser.error(f"unrecognized argument {_arg!r}; did you mean {_close[0]!r}?")

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {
        "headless": True,
        "extra_args": [
            "--/exts/isaacsim.core.simulation_manager/default_engine=newton",
            "--enable",
            "isaacsim.physics.newton",
            "--enable",
            "isaacsim.physics.newton.tensors",
        ],
    }
)

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.asset.transformer import AssetTransformerManager, RuleProfile
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

PHYSICS_VARIANT_SET = "Physics"
NEWTON_VARIANT = "mujoco"
SELF_TEST_GAINLESS = {"j2_nogain_actuator"}  # j3_mimic is also gainless but must be skipped
SELF_TEST_ACTUATORS = 3


def build_legacy_asset(path: str) -> None:
    """Author a fixed-base three-joint PhysX-only arm.

    ``j1`` has drive gains, ``j2_nogain`` has none, and ``j3_mimic`` is a gainless
    mimic follower of ``j1`` that the gain audit must not report.

    Args:
        path: Destination ``.usda`` path.
    """
    from pxr import PhysxSchema

    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/arm").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    PhysxSchema.PhysxArticulationAPI.Apply(root)
    for name, z in (("base", 0.0), ("l1", 0.2), ("l2", 0.4), ("l3", 0.6)):
        link = UsdGeom.Xform.Define(stage, f"/arm/{name}")
        link.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z))
        UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        UsdPhysics.MassAPI.Apply(link.GetPrim()).CreateMassAttr(1.0)
        cube = UsdGeom.Cube.Define(stage, f"/arm/{name}/geom")
        cube.CreateSizeAttr(0.1)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    # Fixed base: anchor the base link to the world (body0 unset) instead of making it kinematic.
    root_joint = UsdPhysics.FixedJoint.Define(stage, "/arm/root_joint")
    root_joint.CreateBody1Rel().SetTargets([Sdf.Path("/arm/base")])
    for name, body0, body1, kp, kd in (
        ("j1", "base", "l1", 100.0, 10.0),
        ("j2_nogain", "l1", "l2", 0.0, 0.0),
        ("j3_mimic", "l2", "l3", None, None),
    ):
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/arm/{name}")
        joint.CreateAxisAttr("Y")
        joint.CreateBody0Rel().SetTargets([Sdf.Path(f"/arm/{body0}")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(f"/arm/{body1}")])
        # Links are 0.2 apart: put the joint midway so the bodies keep their authored poses.
        joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.1))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, -0.1))
        joint.CreateLowerLimitAttr(-90.0)
        joint.CreateUpperLimitAttr(90.0)
        PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
        if kp is None:
            # Mimic follower: no drive, motion follows j1.
            mimic = PhysxSchema.PhysxMimicJointAPI.Apply(joint.GetPrim(), UsdPhysics.Tokens.rotY)
            mimic.CreateReferenceJointRel().SetTargets([Sdf.Path("/arm/j1")])
            mimic.CreateGearingAttr(1.0)
            continue
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateStiffnessAttr(kp)
        drive.CreateDampingAttr(kd)
        drive.CreateTargetPositionAttr(0.0)
    stage.Save()


def run_profile(input_path: str, profile_name: str, output_dir: str, interface_name: str | None = None) -> bool:
    """Run one Asset Transformer profile shipped with ``isaacsim.asset.transformer.rules``.

    Args:
        input_path: Source USD path.
        profile_name: Profile JSON file name under the rules extension ``data/`` folder.
        output_dir: Output package root.
        interface_name: Optional interface file name override (``RuleProfile.interface_asset_name``).

    Returns:
        True if every rule passed.
    """
    rules_path = app_utils.get_extension_path("isaacsim.asset.transformer.rules")
    if not rules_path:
        raise RuntimeError("extension isaacsim.asset.transformer.rules is not available in this build")
    profile_path = os.path.join(rules_path, "data", profile_name)
    with open(profile_path, encoding="utf-8") as f:
        profile = RuleProfile.from_json(f.read())
    if interface_name:
        profile.interface_asset_name = interface_name
    os.makedirs(output_dir, exist_ok=True)
    report = AssetTransformerManager().run(input_stage=input_path, profile=profile, package_root=output_dir)
    ok = bool(report.results)
    for result in report.results:
        if not result.success:
            ok = False
            print(f"  [FAIL] {result.rule.name}: {result.error}")
    print(f"  {profile.profile_name}: {'ok' if ok else 'FAILED'} ({len(report.results)} rules)")
    return ok


def _is_mimic_joint(prim: Usd.Prim) -> bool:
    """Return True if *prim* is a mimic follower (its motion comes from another joint, not an actuator)."""
    return any(name == "NewtonMimicAPI" or name.startswith("PhysxMimicJointAPI") for name in prim.GetAppliedSchemas())


def preflight(input_path: str) -> list[str]:
    """Warn about PhysX-asset traits Newton does not handle (see ``newton_physics.rst`` Limitations).

    Checks only what is cheap to detect; reversed joint direction (``body0`` must be the
    parent) and USD composition errors are not checked here. The conversion cannot fix any
    of these, so the warnings do not stop the run.

    Args:
        input_path: Legacy PhysX USD to inspect.

    Returns:
        Human-readable warnings, empty when none of the checks fire.
    """
    stage = Usd.Stage.Open(input_path)
    warnings = []
    if UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
        warnings.append(f"metersPerUnit is {UsdGeom.GetStageMetersPerUnit(stage)}; Newton does not convert units")
    xform_cache = UsdGeom.XformCache()
    zero_size, negative_scale, no_inertia, orphan_joints, tendons, existing_refs = [], [], [], [], [], []
    for prim in stage.Traverse():
        schemas = prim.GetAppliedSchemas()
        if prim.IsA(UsdPhysics.Joint) and not any(
            stage.GetPrimAtPath(path).HasAPI(UsdPhysics.ArticulationRootAPI) for path in prim.GetPath().GetPrefixes()
        ):
            orphan_joints.append(str(prim.GetPath()))
        if any(name.startswith("PhysxTendon") for name in schemas):
            tendons.append(str(prim.GetPath()))
        ref = prim.GetAttribute("mjc:ref") if prim.IsA(UsdPhysics.RevoluteJoint) else None
        if ref and ref.HasAuthoredValue() and ref.Get():
            existing_refs.append(str(prim.GetPath()))
        if "PhysicsMassAPI" in schemas:
            mass_api = UsdPhysics.MassAPI(prim)
            mass, density, inertia = (
                mass_api.GetMassAttr(),
                mass_api.GetDensityAttr(),
                mass_api.GetDiagonalInertiaAttr(),
            )
            if mass.HasAuthoredValue() and density.HasAuthoredValue() and not inertia.HasAuthoredValue():
                no_inertia.append(str(prim.GetPath()))
        if "PhysicsCollisionAPI" not in schemas:
            continue
        if xform_cache.GetLocalToWorldTransform(prim).GetDeterminant() < 0:
            negative_scale.append(str(prim.GetPath()))
        dims = {
            "Cube": ["size"],
            "Sphere": ["radius"],
            "Capsule": ["radius", "height"],
            "Cylinder": ["radius", "height"],
            "Cone": ["radius", "height"],
        }.get(prim.GetTypeName(), [])
        if any(prim.GetAttribute(d).Get() == 0 for d in dims):
            zero_size.append(str(prim.GetPath()))
    for label, paths, note in (
        ("joint(s) outside any articulation root", orphan_joints, "Newton fails to initialize"),
        ("zero-size collision shape(s)", zero_size, "Newton rejects non-plane shapes of size 0"),
        ("collider(s) under a negative scale", negative_scale, "Newton mishandles negative scale; mirror the mesh"),
        ("body(ies) with mass and density but no diagonalInertia", no_inertia, "Newton ignores the density"),
        ("prim(s) with PhysxTendon schemas", tendons, "not converted; they survive only in the physx variant"),
        (
            "revolute joint(s) that already have a non-zero mjc:ref",
            existing_refs,
            "the conversion leaves it as is; if an older converter copied the PhysX drive target here, re-convert "
            "from the original PhysX asset, or delete mjc:ref",
        ),
    ):
        if paths:
            warnings.append(f"{len(paths)} {label} (e.g. {paths[0]}): {note}")
    return warnings


def selftest_existing_ref_preflight(workdir: str) -> bool:
    """Check that preflight flags an input that already carries a non-zero revolute ``mjc:ref``.

    Args:
        workdir: Directory for the throwaway asset.

    Returns:
        True when the warning fires for a joint with ``mjc:ref`` set and not for one without.
    """
    path = os.path.join(workdir, "existing_ref.usda")
    stage = Usd.Stage.CreateNew(path)
    root = UsdGeom.Xform.Define(stage, "/arm").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    for name, ref in (("with_ref", 174.0), ("no_ref", None)):
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/arm/{name}").GetPrim()
        if ref is not None:
            joint.CreateAttribute("mjc:ref", Sdf.ValueTypeNames.Float).Set(ref)
    stage.Save()
    flagged = [w for w in preflight(path) if "already have a non-zero mjc:ref" in w]
    return len(flagged) == 1 and "/arm/with_ref" in flagged[0] and "1 revolute" in flagged[0]


def audit_gains(interface_path: str) -> tuple[list[str], int]:
    """List MjcActuator prims that carry no usable gains, on the ``mujoco`` variant.

    The conversion reports success when a joint's PhysX drive has no damping
    (stiffness-only drives included) or no drive at all; the actuator is then
    authored without ``mjc:gainPrm``. Mimic followers are not reported.

    Args:
        interface_path: Interface USD produced by the Isaac Sim Structure pass.

    Returns:
        ``(paths, count)``: actuators with missing or all-zero ``mjc:gainPrm``, and
        the total number of actuators inspected.

    Raises:
        RuntimeError: If the interface has no default prim or no ``Physics=mujoco`` variant.
    """
    stage = Usd.Stage.Open(interface_path)
    # Keep the variant selection in the session layer so the interface file is never dirtied.
    stage.SetEditTarget(stage.GetSessionLayer())
    root = stage.GetDefaultPrim()
    if not root:
        raise RuntimeError(f"{interface_path} has no default prim")
    variant_set = root.GetVariantSets().GetVariantSet(PHYSICS_VARIANT_SET)
    if NEWTON_VARIANT not in variant_set.GetVariantNames():
        raise RuntimeError(f"{interface_path} has no {PHYSICS_VARIANT_SET}={NEWTON_VARIANT} variant")
    variant_set.SetVariantSelection(NEWTON_VARIANT)
    missing = []
    count = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "MjcActuator":
            continue
        count += 1
        attr = prim.GetAttribute("mjc:gainPrm")
        gains = attr.Get() if attr and attr.HasAuthoredValue() else None
        if gains and any(gains):
            continue
        target_rel = prim.GetRelationship("mjc:target")
        targets = target_rel.GetTargets() if target_rel else []
        if targets and all(_is_mimic_joint(stage.GetPrimAtPath(t)) for t in targets):
            continue
        missing.append(str(prim.GetPath()))
    return missing, count


def audit_reference_positions(interface_path: str) -> list[tuple[str, float]]:
    """List revolute joints whose ``mjc:ref`` is non-zero on the ``mujoco`` variant.

    Older converter builds copied the PhysX drive ``targetPosition`` (degrees) into
    ``mjc:ref``, which is a zero point in radians, not a target. That made the Isaac Sim
    6.0/6.1 Franka Panda non-finite under Newton. Current builds do not author it.

    Args:
        interface_path: Interface USD produced by the Isaac Sim Structure pass.

    Returns:
        ``(joint_path, mjc:ref value)`` pairs for revolute joints with a non-zero reference.
    """
    stage = Usd.Stage.Open(interface_path)
    stage.SetEditTarget(stage.GetSessionLayer())
    stage.GetDefaultPrim().GetVariantSets().GetVariantSet(PHYSICS_VARIANT_SET).SetVariantSelection(NEWTON_VARIANT)
    flagged = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        ref = prim.GetAttribute("mjc:ref")
        value = ref.Get() if ref and ref.HasAuthoredValue() else None
        if value:
            flagged.append((str(prim.GetPath()), float(value)))
    return flagged


def check_newton_runtime(interface_path: str, steps: int, device: str) -> bool:
    """Step the converted asset under Newton and verify joint state stays finite.

    This is a smoke check only: it does not verify that actuated joints track targets.

    Args:
        interface_path: Interface USD produced by the Isaac Sim Structure pass.
        steps: Frames to simulate.
        device: ``cpu`` or ``cuda``.

    Returns:
        True when the active engine is Newton, the articulation resolves with at
        least one DOF, and every sampled DOF position is finite.
    """
    engine = SimulationManager.get_active_physics_engine()
    if engine != "newton":
        print(f"  runtime: active engine is '{engine}', expected 'newton' (did isaacsim.physics.newton load?)")
        return False
    stage_utils.create_new_stage()
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device=device)
    stage_utils.add_reference_to_stage(
        usd_path=interface_path, path="/World/robot", variants=[(PHYSICS_VARIANT_SET, NEWTON_VARIANT)]
    )
    simulation_app.update()
    app_utils.play()
    try:
        robot = Articulation("/World/robot")
        if not robot.is_physics_tensor_entity_valid() or robot.num_dofs == 0:
            print("  runtime: articulation did not resolve under Newton (no DOFs)")
            return False
        for _ in range(steps):
            simulation_app.update()
        positions = robot.get_dof_positions().numpy()
    finally:
        app_utils.stop()
    finite = bool(positions.size) and bool(np.isfinite(positions).all())
    print(f"  runtime: {positions.shape[-1]} DOFs after {steps} frames, finite={finite}")
    return finite


def main() -> int:
    """Run the conversion and checks.

    Returns:
        Process exit code: 0 on success, 1 on any failure.
    """
    app_utils.enable_extension("isaacsim.asset.transformer")
    app_utils.enable_extension("isaacsim.asset.transformer.rules")
    simulation_app.update()

    workdir = os.path.abspath(args.output or tempfile.mkdtemp(prefix="physx_to_newton_"))
    pass1_dir = os.path.join(workdir, "pass1")
    pass2_dir = os.path.join(workdir, "package")
    for path in (pass1_dir, pass2_dir):
        # The transformer reopens and appends to existing layers, so stale output would leak into the package.
        if os.path.isdir(path) and os.listdir(path):
            print(f"Output {path} is not empty; remove it or choose another --output")
            return 1

    self_test = args.input is None
    check_runtime = args.check_runtime or self_test
    if self_test:
        input_path = os.path.join(workdir, "legacy_arm.usda")
        os.makedirs(workdir, exist_ok=True)
        build_legacy_asset(input_path)
        print(f"Self-test: built synthetic legacy asset {input_path}")
    else:
        input_path = os.path.abspath(args.input)
        if not os.path.isfile(input_path):
            print(f"Input not found: {input_path}")
            return 1
    interface_name = os.path.splitext(os.path.basename(input_path))[0] + ".usda"

    for warning in preflight(input_path):
        print(f"PREFLIGHT: {warning}")

    print("Pass 1: PhysX to MJC Conversion")
    if not run_profile(input_path, "physx_to_mjc_conversion.json", pass1_dir):
        return 1
    pass1_output = os.path.join(pass1_dir, "payloads", "mujoco.usda")

    print("Pass 2: Isaac Sim Structure")
    # Without an explicit name, pass 2 would call the interface after its input (mujoco.usda).
    if not run_profile(pass1_output, "isaacsim_structure.json", pass2_dir, interface_name=interface_name):
        return 1
    interface_path = os.path.join(pass2_dir, interface_name)
    if not os.path.isfile(interface_path):
        print(f"Package interface missing: {interface_path}")
        return 1
    print(f"Package interface: {interface_path}")

    status = 0
    missing, actuator_count = audit_gains(interface_path)
    if actuator_count == 0:
        print("ERROR: no MjcActuator found on the mujoco variant (no revolute/prismatic joints converted)")
        status = 1
    elif missing:
        print(
            f"WARNING: {len(missing)} of {actuator_count} MjcActuator(s) have no gains (author damping, then re-convert):"
        )
        for path in missing:
            print(f"  {path}")
        if args.require_gains:
            status = 1
    else:
        print(f"Gain audit: all {actuator_count} MjcActuators have gains")

    bad_refs = audit_reference_positions(interface_path)
    if bad_refs:
        print(
            f"WARNING: {len(bad_refs)} revolute joint(s) have a non-zero mjc:ref, copied from the PhysX drive target by "
            "an older converter build. mjc:ref is a zero-point offset in radians, not a target, so Newton can be unstable "
            "or settle outside the joint limits. Re-convert the original PhysX asset with a current build (converting "
            "an already converted asset keeps the value), or delete mjc:ref in payloads/Physics/mujoco.usda:"
        )
        for path, value in bad_refs:
            print(f"  {path}: mjc:ref = {value}")

    if self_test and not selftest_existing_ref_preflight(workdir):
        print("Self-test FAILED: preflight did not flag a revolute joint that already has a non-zero mjc:ref")
        status = 1

    if self_test:
        flagged = {path.rsplit("/", 1)[-1] for path in missing}
        if flagged != SELF_TEST_GAINLESS or actuator_count != SELF_TEST_ACTUATORS:
            print(
                f"Self-test FAILED: gain audit flagged {sorted(flagged)} of {actuator_count} actuators, "
                f"expected {sorted(SELF_TEST_GAINLESS)} of {SELF_TEST_ACTUATORS}"
            )
            status = 1

    if check_runtime and not check_newton_runtime(interface_path, args.steps, args.device):
        status = 1
    return status


try:
    exit_code = main()
except Exception as exc:
    import traceback

    traceback.print_exc()
    print(f"Conversion failed: {exc}")
    exit_code = 1
# close() must carry the status: under fast shutdown it terminates the process itself, so a
# bare sys.exit(exit_code) would never run and every failure would exit 0. sys.exit covers
# the non-fast-shutdown path, where close() returns.
simulation_app.close(exit_code=exit_code)
sys.exit(exit_code)
