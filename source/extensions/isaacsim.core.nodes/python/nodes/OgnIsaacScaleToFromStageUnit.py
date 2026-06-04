# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og


class OgnIsaacScaleToFromStageUnit:
    """Isaac Sim Scale To and From Stage Units."""

    @staticmethod
    def compute(db) -> bool:
        conversion = db.inputs.conversion
        value = db.inputs.value.value

        if conversion:
            if conversion not in (db.tokens.toStage, db.tokens.toMeters):
                db.log_error("Unable to use custom conversion option")
                return False
        else:
            db.log_error("No conversion option detected")
            return False

        if conversion in db.tokens.toStage:
            db.outputs.result.value = value / stage_utils.get_stage_units()[0]

        elif conversion in db.tokens.toMeters:
            db.outputs.result.value = value * stage_utils.get_stage_units()[0]

        return True

    @staticmethod
    def on_connection_type_resolve(node) -> None:
        int_types = (
            og.BaseDataType.UCHAR,
            og.BaseDataType.INT,
            og.BaseDataType.UINT,
            og.BaseDataType.INT64,
            og.BaseDataType.UINT64,
        )

        valtype = node.get_attribute("inputs:value").get_resolved_type()
        resultattr = node.get_attribute("outputs:result")
        resulttype = resultattr.get_resolved_type()

        # If Value is resolved, resolve result to match Value
        if valtype.base_type != og.BaseDataType.UNKNOWN and resulttype.base_type == og.BaseDataType.UNKNOWN:
            # If Value is an integral, then set it to a double
            new_type = valtype
            if valtype.base_type in int_types:
                new_type.base_type = og.BaseDataType.DOUBLE
            resultattr.set_resolved_type(new_type)
