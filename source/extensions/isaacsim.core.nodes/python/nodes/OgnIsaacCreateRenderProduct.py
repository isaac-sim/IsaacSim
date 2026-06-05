# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

import contextlib

import carb
import carb.eventdispatcher
import omni
import omni.graph.core
import omni.replicator.core as rep
import omni.timeline
from isaacsim.core.nodes import BaseResetNode
from isaacsim.core.nodes.ogn.OgnIsaacCreateRenderProductDatabase import OgnIsaacCreateRenderProductDatabase
from pxr import Gf, Usd, UsdRender

SRTX_ENABLED = "/exts/omni.replicator.srtx/enabled"


def get_existing_render_product(camera_path: str, resolution: tuple[int, int]) -> str | None:
    """Find an existing render product matching the given camera and resolution."""
    render_products = rep.functional.get.renderproduct()
    for render_product in render_products:
        targets = render_product.GetRelationship("camera").GetTargets()
        if not targets:
            continue
        if str(targets[0]) == str(camera_path) and tuple(render_product.GetAttribute("resolution").Get()) == resolution:
            return str(render_product.GetPath())
    return None


class OgnIsaacCreateRenderProductInternalState(BaseResetNode):
    """Internal state for the IsaacCreateRenderProduct OmniGraph node."""

    def __init__(self) -> None:
        self.render_product_path = None
        self.resolution = (0, 0)
        self.camera_path = ""
        self.rp_sub_stop = None
        self.rp_sub_play = None
        super().__init__(initialize=False)

    def on_timeline_stop(self, event: carb.eventdispatcher.Event) -> None:
        self.initialized = False


class OgnIsaacCreateRenderProduct:
    """Isaac Sim Create Render Product."""

    @staticmethod
    def internal_state() -> OgnIsaacCreateRenderProductInternalState:
        return OgnIsaacCreateRenderProductInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        if db.inputs.enabled is False:
            return False

        if len(db.inputs.cameraPrim) == 0:
            db.log_error("Camera prim must be specified")
            return False

        stage = omni.usd.get_context().get_stage()
        use_srtx = carb.settings.get_settings().get_as_bool(SRTX_ENABLED)
        ctx = contextlib.nullcontext() if use_srtx else Usd.EditContext(stage, stage.GetSessionLayer())
        with ctx:
            if state.render_product_path is None:
                render_product_path = None
                render_product_prims = db.inputs.renderProductPrim
                if len(render_product_prims) > 1:
                    carb.log_warn(
                        f"Multiple render product prims provided (`{render_product_prims}`), only the first one will be used"
                    )
                elif len(render_product_prims) == 1:
                    render_product_path = str(render_product_prims[0])

                # If the render product path is not provided or is invalid, try to find a matching render product
                if render_product_path is None or not stage.GetPrimAtPath(render_product_path):
                    render_product_path = get_existing_render_product(
                        db.inputs.cameraPrim[0].GetString(), (db.inputs.width, db.inputs.height)
                    )

                # If an existing render product is valid, use it
                if render_product_path is not None:
                    # Ensure there's a hydratexture backing it if not running with SRTX
                    if not use_srtx:
                        try:
                            rep.vp_manager.attach_hydra_texture(render_product_path)
                        except Exception as e:
                            db.log_error(f"Error attaching hydra texture to render product {render_product_path}: {e}")
                            db.outputs.execOut = omni.graph.core.ExecutionAttributeState.DISABLED
                            return False
                    state.render_product_path = render_product_path
                    db.node.get_attribute("inputs:renderProductPrim").set([render_product_path])
                    db.outputs.renderProductPath = render_product_path
                    db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
                    return True

                # Create a new render product
                render_prod = rep.create.render_product(
                    db.inputs.cameraPrim[0].GetString(),
                    (db.inputs.width, db.inputs.height),
                    force_new=True,
                )

                state.render_product_path = render_prod.path
                state.resolution = (db.inputs.width, db.inputs.height)
                state.camera_path = db.inputs.cameraPrim[0].GetString()
                db.node.get_attribute("inputs:renderProductPrim").set([state.render_product_path])
                db.outputs.renderProductPath = state.render_product_path

                state.rp_sub_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
                    event_name=omni.timeline.GLOBAL_EVENT_STOP,
                    on_event=state.on_timeline_stop,
                    observer_name="isaacsim.core.nodes.OgnIsaacCreateRenderProduct.on_timeline_stop",
                )

            render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(state.render_product_path))
            if not render_prod_prim:
                raise RuntimeError(f'Invalid renderProduct "{state.render_product_path}"')
            if state.resolution != (db.inputs.width, db.inputs.height):
                render_prod_prim.GetResolutionAttr().Set(Gf.Vec2i(db.inputs.width, db.inputs.height))
                state.resolution = (db.inputs.width, db.inputs.height)
            if state.camera_path != db.inputs.cameraPrim[0].GetString():
                render_prod_prim.GetCameraRel().SetTargets([db.inputs.cameraPrim[0].GetString()])
                state.camera_path = db.inputs.cameraPrim[0].GetString()

        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node, graph_instance_id) -> None:
        try:
            state = OgnIsaacCreateRenderProductDatabase.per_instance_internal_state(node)
        except Exception:
            state = None

        if state is not None:
            # The render product is not explicitly destroyed here.
            # Manually calling destroy() leads to a crash in some cases.
            state.render_product_path = None
            state.rp_sub_stop = None
            state.rp_sub_play = None
