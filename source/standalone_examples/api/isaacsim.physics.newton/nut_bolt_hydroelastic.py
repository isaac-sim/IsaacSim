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

"""Thread an M16 nut onto a fixed bolt with Newton hydroelastic contact.

Per-shape contact settings are authored on USD through ``NewtonSDFCollisionAPI`` and
``NewtonMaterialAPI``. Settings without a USD equivalent are applied with ``NewtonConfig``.

    ./python.sh standalone_examples/api/isaacsim.physics.newton/nut_bolt_hydroelastic.py
    ./python.sh standalone_examples/api/isaacsim.physics.newton/nut_bolt_hydroelastic.py --test
"""

from __future__ import annotations

import argparse
import math
import sys

from isaacsim import SimulationApp

parser = argparse.ArgumentParser(description="Newton nut/bolt hydroelastic contact example.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cuda", help="Simulation device")
args, _ = parser.parse_known_args()

required_extensions = ["isaacsim.physics.newton", "isaacsim.physics.newton.tensors"]
if args.test:
    # The default Base experience includes interactive examples that are unrelated to this
    # headless test and may perform remote asset checks during startup.
    required_extensions.extend(
        [
            "isaacsim.core.experimental.objects",
            "isaacsim.storage.native",
            "omni.hydra.usdrt_delegate",
            "omni.kit.loop",
            "omni.kit.telemetry",
            "omni.kit.usd.mdl",
            "omni.usd.metrics.assembler.ui",
            "usdrt.scenegraph",
        ]
    )

extra_args = ["--/exts/isaacsim.core.simulation_manager/default_engine=newton"]
for extension in required_extensions:
    extra_args.extend(["--enable", extension])

simulation_app = SimulationApp(
    {"headless": args.test, "extra_args": extra_args},
    experience=None if args.test else "",
)

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.experimental.objects import DistantLight, GroundPlane
from isaacsim.core.experimental.prims import RigidPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.physics.newton import (
    CollisionConfig,
    HydroelasticConfig,
    MuJoCoSolverConfig,
    NewtonConfig,
    configure_newton,
)
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

BOLT_PATH = "/World/bolt"
NUT_PATH = "/World/nut"
BOLT_BODY_PATH = f"{BOLT_PATH}/factory_bolt_loose"
BOLT_ROOT_JOINT_PATH = f"{BOLT_PATH}/root_joint"
NUT_BODY_PATH = f"{NUT_PATH}/factory_nut_loose"

# M16 asset geometry [m]: the bolt shank ends at 0.035 and the nut collision mesh starts
# 0.010 above its prim origin. Start the nut a fraction of a millimeter onto the shank.
BOLT_TIP_HEIGHT = 0.035
NUT_MESH_BASE_OFFSET = 0.010
NUT_ENGAGEMENT = 0.0005
NUT_YAW = math.pi / 8.0


def fail(message: str) -> None:
    """Shut down the app and report a nonzero process status."""
    carb.log_error(message)
    app_utils.stop()
    simulation_app.close(exit_code=1)
    # ``close`` returns when fast shutdown is disabled.
    sys.exit(1)


def setup_collider(prim: Usd.Prim, material: UsdShade.Material) -> None:
    """Author SDF, hydroelastic, and contact settings on a collider and bind its physics material."""
    if not prim.HasAPI("NewtonSDFCollisionAPI"):
        prim.ApplyAPI("NewtonSDFCollisionAPI")
    # Opt into hydroelastic contact and cook an SDF fine enough for the M16 thread.
    prim.GetAttribute("newton:hydroelasticEnabled").Set(True)
    # The default 1e10 contact is too compliant for reliable engagement at this scale.
    prim.GetAttribute("newton:hydroelasticStiffness").Set(1.0e11)
    prim.GetAttribute("newton:sdfMaxResolution").Set(128)
    prim.GetAttribute("newton:sdfNarrowBandInner").Set(-0.005)
    prim.GetAttribute("newton:sdfNarrowBandOuter").Set(0.005)
    # Keep hydroelastic contacts on the mesh surface. A 5 mm gap per collider admits
    # separated contact patches across several 2 mm thread pitches and destabilizes engagement.
    prim.GetAttribute("newton:contactMargin").Set(0.0)
    prim.GetAttribute("newton:contactGap").Set(0.0)
    # The Factory assets ship with the PhysX SDF collider setup, which NewtonSDFCollisionAPI
    # supersedes. Clearing the approximation token avoids a parse warning.
    prim.GetAttribute("physics:approximation").Set("none")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics"
    )


engine = SimulationManager.get_active_physics_engine()
if engine != "newton":
    fail(f"This example requires the Newton physics engine. Active engine is '{engine}'.")
print(f"Using physics engine: {engine} on device: {args.device}")

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    fail("Could not find Isaac Sim assets folder")

stage_utils.create_new_stage()
stage_utils.set_stage_units(meters_per_unit=1.0)
stage_utils.set_stage_up_axis("Z")
# The thread pitch is 2 mm, so contacts have to be resolved at a fraction of that.
SimulationManager.setup_simulation(dt=1.0 / 240.0, device=args.device)

GroundPlane("/World/ground", positions=[0.0, 0.0, -0.01])
DistantLight("/World/DistantLight").set_intensities(300.0)

factory_dir = f"{assets_root_path}/Isaac/IsaacLab/Factory"
stage_utils.add_reference_to_stage(usd_path=f"{factory_dir}/factory_bolt_m16.usd", path=BOLT_PATH)
XformPrim(
    paths=BOLT_PATH,
    positions=[[0.0, 0.0, 0.0]],
    orientations=[[1.0, 0.0, 0.0, 0.0]],
    reset_xform_op_properties=True,
)

# Seat the nut on the bolt tip, rotated so its threads do not start perfectly aligned.
stage_utils.add_reference_to_stage(usd_path=f"{factory_dir}/factory_nut_m16.usd", path=NUT_PATH)
XformPrim(
    paths=NUT_PATH,
    positions=[[0.0, 0.0, BOLT_TIP_HEIGHT - NUT_MESH_BASE_OFFSET - NUT_ENGAGEMENT]],
    orientations=[[math.cos(NUT_YAW * 0.5), 0.0, 0.0, math.sin(NUT_YAW * 0.5)]],
    reset_xform_op_properties=True,
)

stage = stage_utils.get_current_stage(backend="usd")

# The bolt ships as a fixed-base articulation. Turning it into a static collider keeps it
# anchored without adding a single-joint articulation to the solver.
bolt_body = stage.GetPrimAtPath(BOLT_BODY_PATH)
bolt_body.RemoveAPI(UsdPhysics.RigidBodyAPI)
bolt_body.RemoveAPI(UsdPhysics.ArticulationRootAPI)
stage.GetPrimAtPath(BOLT_ROOT_JOINT_PATH).SetActive(False)

# Low friction lets the nut turn under gravity; stiff, well damped contacts keep the threads
# from interpenetrating.
fastener_material = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/fastener")
material_api = UsdPhysics.MaterialAPI.Apply(fastener_material.GetPrim())
material_api.CreateStaticFrictionAttr().Set(0.01)
material_api.CreateDynamicFrictionAttr().Set(0.01)
material_api.CreateRestitutionAttr().Set(0.0)
fastener_material.GetPrim().ApplyAPI("NewtonMaterialAPI")
fastener_material.GetPrim().GetAttribute("newton:torsionalFriction").Set(0.0)
fastener_material.GetPrim().GetAttribute("newton:rollingFriction").Set(0.0)
fastener_material.GetPrim().GetAttribute("newton:contactStiffness").Set(1.0e7)
fastener_material.GetPrim().GetAttribute("newton:contactDamping").Set(1.0e4)

for root_path in (BOLT_PATH, NUT_PATH):
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if prim.IsA(UsdGeom.Gprim) and prim.HasAPI(UsdPhysics.CollisionAPI):
            setup_collider(prim, fastener_material)

# Settings without a USD attribute equivalent. Hydroelastic patches on a helical thread need a
# large contact budget; mc_edge_clamp_min=0 avoids biasing thread contact vertices; buffer_mult_iso
# holds the full thread patch.
max_contacts = 40_000
configure_newton(
    NewtonConfig(
        num_substeps=2,
        # Elliptic cones avoid the directional bias of pyramidal friction on the helical thread.
        solver_cfg=MuJoCoSolverConfig(njmax=max_contacts, nconmax=max_contacts, cone="elliptic"),
        collision_cfg=CollisionConfig(
            rigid_contact_max=max_contacts,
            hydroelastic=HydroelasticConfig(mc_edge_clamp_min=0.0, buffer_mult_iso=2),
        ),
    )
)

if args.test:
    app_utils.play()
else:
    print("Scene ready. Press Play in the viewport to watch the nut thread onto the bolt.")

nut = RigidPrim(NUT_BODY_PATH)
start_height = None
start_orientation = None
max_rotation = 0.0
frame_count = 0
while simulation_app.is_running():
    simulation_app.update()
    frame_count += 1
    if SimulationManager.is_simulating():
        position, orientation = nut.get_world_poses()
        position = position.numpy()[0]
        orientation = orientation.numpy()[0]
        if not all(math.isfinite(float(value)) for value in (*position, *orientation)):
            pose = f"position={position}, orientation={orientation}"
            fail(f"Nut pose became nonfinite at frame {frame_count}: {pose}")
        if args.test:
            # Descent and quaternion rotation alone also pass for a nut falling off the bolt.
            radial_offset = math.hypot(float(position[0]), float(position[1]))
            # The local Z axis must stay aligned with the bolt, regardless of rotation about Z.
            axis_alignment = 1.0 - 2.0 * (float(orientation[1]) ** 2 + float(orientation[2]) ** 2)
            tilt = math.acos(max(-1.0, min(axis_alignment, 1.0)))
            if radial_offset > 0.002:
                fail(f"Test mode: nut left the bolt axis at frame {frame_count}: offset={radial_offset:.4f} m")
            if tilt > math.radians(15.0):
                fail(f"Test mode: nut tipped at frame {frame_count}: tilt={math.degrees(tilt):.1f} deg")
        if start_height is None:
            start_height = float(position[2])
            start_orientation = orientation.copy()
        else:
            # |q1 · q2| = cos(theta/2) for the relative rotation angle theta.
            dot = abs(float(orientation @ start_orientation))
            max_rotation = max(max_rotation, 2.0 * math.acos(min(dot, 1.0)))
    if args.test and frame_count >= 400:
        break

if start_height is not None:
    descent = start_height - float(nut.get_world_poses()[0].numpy()[0, 2])
    print(f"Nut descended {1000.0 * descent:.2f} mm along the bolt.")
    print(f"Nut rotated {math.degrees(max_rotation):.1f} degrees.")
    if args.test:
        min_descent = 0.010
        min_rotation = math.radians(45.0)
        if descent < min_descent:
            fail(f"Test mode: nut descent {descent:.4f} m is below the {min_descent:.4f} m threshold")
        if max_rotation < min_rotation:
            fail(
                f"Test mode: nut rotation {math.degrees(max_rotation):.1f} deg is below the "
                f"{math.degrees(min_rotation):.1f} deg threshold"
            )
elif args.test:
    fail("Test mode: simulation did not produce a nut pose")

app_utils.stop()
simulation_app.close()
