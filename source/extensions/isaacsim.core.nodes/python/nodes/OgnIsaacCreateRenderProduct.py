# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import carb
import omni
import omni.replicator.core as rep
from isaacsim.core.nodes import BaseResetNode
from isaacsim.core.nodes.ogn.OgnIsaacCreateRenderProductDatabase import OgnIsaacCreateRenderProductDatabase
from pxr import Gf, Usd, UsdRender


class OgnIsaacCreateRenderProductInternalState(BaseResetNode):
    def __init__(self):
        self.handle = None
        self.render_product_path = None
        self.factory = None
        self.resolution = [0, 0]
        self.camera_path = ""
        super().__init__(initialize=False)

    def on_stage_event(self, event: carb.events.IEvent):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            if self.handle:
                self.handle.hydra_texture.set_updates_enabled(False)
            self.initialized = False
        elif event.type == int(omni.timeline.TimelineEventType.PLAY):
            if self.handle:
                self.handle.hydra_texture.set_updates_enabled(True)


class OgnIsaacCreateRenderProduct:
    """
    Isaac Sim Create Hydra Texture
    """

    @staticmethod
    def internal_state():
        return OgnIsaacCreateRenderProductInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state
        if db.inputs.enabled is False:
            if state.handle is not None:
                state.handle.hydra_texture.set_updates_enabled(False)
            return False
        else:
            if state.handle is not None:
                state.handle.hydra_texture.set_updates_enabled(True)

        if len(db.inputs.cameraPrim) == 0:
            db.log_error(f"Camera prim must be specified")
            return False
        stage = omni.usd.get_context().get_stage()
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            if state.handle is None:
                state.handle = rep.create.render_product(
                    db.inputs.cameraPrim[0].GetString(), (db.inputs.width, db.inputs.height), force_new=True
                )
                state.resolution = (db.inputs.width, db.inputs.height)
                state.camera_path = db.inputs.cameraPrim[0].GetString()
                db.outputs.renderProductPath = state.handle.path

                state.rp_sub = (
                    omni.timeline.get_timeline_interface()
                    .get_timeline_event_stream()
                    .create_subscription_to_pop(state.on_stage_event, name="IsaacSimOGNCoreNodesRPEventHandler")
                )
            render_prod_prim = UsdRender.Product(stage.GetPrimAtPath(state.handle.path))
            if not render_prod_prim:
                raise RuntimeError(f'Invalid renderProduct "{state.handle.path}"')
            if state.resolution[0] != db.inputs.width or state.resolution[1] != db.inputs.height:
                render_prod_prim.GetResolutionAttr().Set(Gf.Vec2i(db.inputs.width, db.inputs.height))
                state.resolution = (db.inputs.width, db.inputs.height)
            if state.camera_path != db.inputs.cameraPrim[0].GetString():
                render_prod_prim.GetCameraRel().SetTargets([db.inputs.cameraPrim[0].GetString()])
                state.camera_path = db.inputs.cameraPrim[0].GetString()

        db.outputs.execOut = omni.graph.core.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnIsaacCreateRenderProductDatabase.per_instance_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            # TODO:
            # The render product is not explicitly destroyed here.
            # Manually calling destroy() here leads to a crash in some cases.
            # Instead it will be destroyed when the node is deleted.
            state.handle = None
            state.rp_sub = None
