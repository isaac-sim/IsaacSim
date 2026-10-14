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

"""Adapt public OVNewton 0.1.1 to OVStage 0.2 stage metadata locally."""

from __future__ import annotations

import math
from types import FunctionType, SimpleNamespace
from typing import Any


def build_model(stage: Any, *, ordinal: int = 1) -> Any:
    """Build a model using OVStage's root metadata without modifying OVNewton globals."""
    import ovstage
    from ovnewton._src import _build, _parse, _stage

    if getattr(_parse, "_STAGE_INFO_PATH", None) != "/__ovstage_population_stage_info__":
        raise RuntimeError("The OVNewton metadata adapter requires the public 0.1.1 parser contract.")

    def read_stage_units(stage: Any, path_dictionary: Any, ordinal: int) -> tuple[float, str]:
        with _stage._path_list_query(stage, path_dictionary, ["/"]) as query:
            columns = _stage.read_columns(
                stage, path_dictionary, query, ["usd-metadata:metersPerUnit", "usd-metadata:upAxis"], ordinal
            )
        meters = columns["usd-metadata:metersPerUnit"].get(0)
        up_axis = _parse._token_value(path_dictionary, columns["usd-metadata:upAxis"].get(0))
        if meters is None or not len(meters) or not math.isfinite(float(meters[0])) or float(meters[0]) <= 0:
            raise _parse.OvstageContractError("Population stage metadata has no valid metersPerUnit")
        if up_axis not in ("Y", "Z"):
            raise _parse.OvstageContractError("Population stage metadata has no valid upAxis")
        return float(meters[0]), up_axis

    # Only the model builder's direct metadata read needs adapting. Its other
    # helpers retain the original upstream module and function identities.
    parser = SimpleNamespace(**vars(_parse))
    parser.read_stage_units = read_stage_units
    build_globals = dict(_build.build_model.__globals__, _parse=parser)
    adapted_build = FunctionType(
        _build.build_model.__code__, build_globals, _build.build_model.__name__, _build.build_model.__defaults__
    )
    with ovstage.PathDictionary(stage) as path_dictionary:
        return adapted_build(stage, path_dictionary, ordinal=ordinal)
