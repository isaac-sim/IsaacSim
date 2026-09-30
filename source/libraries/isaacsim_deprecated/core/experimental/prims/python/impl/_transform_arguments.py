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

"""Constructor arguments that place a prim, which the foundation classes no longer accept.

The deprecated ``isaacsim.core.experimental`` wrappers took ``positions``, ``translations``, ``orientations`` and
``scales`` on construction and applied them to the newly wrapped prims. The
``isaacsim.foundation`` classes that replace them construct only, leaving placement to the pose setters, so
this module puts the arguments back.

They are applied through ``self``, so a wrapper that also carries the scalar-first pose overrides reads
``orientations`` as ``wxyz``, as the deprecated wrappers documented.
"""

from __future__ import annotations

import inspect
from typing import Any, TypeVar

__all__ = ["preserve_transform_arguments"]

#: Constructor arguments added to the wrapped class, in the order the deprecated wrappers declared them.
TRANSFORM_ARGUMENTS = ("positions", "translations", "orientations", "scales")

_T = TypeVar("_T", bound=type)


def _merge_signature(wrapper: Any, original: Any) -> None:
    """Advertise the wrapped constructor's own arguments alongside the added ones.

    Without this the wrapper reports only ``**kwargs``, hiding arguments such as ``sizes`` or
    ``apply_collision_apis``. Constructors bound by nanobind carry no introspectable signature, so for those the
    wrapper is left as it is.

    Args:
        wrapper: Replacement constructor to annotate.
        original: Constructor being wrapped.
    """
    try:
        parameters = list(inspect.signature(original).parameters.values())
    except (TypeError, ValueError):
        return
    added = [inspect.Parameter(name, inspect.Parameter.KEYWORD_ONLY, default=None) for name in TRANSFORM_ARGUMENTS]
    # A signature orders its parameters by kind, and the added ones are keyword-only.
    leading = [
        parameter
        for parameter in parameters
        if parameter.kind not in (inspect.Parameter.KEYWORD_ONLY, inspect.Parameter.VAR_KEYWORD)
    ]
    keyword_only = [parameter for parameter in parameters if parameter.kind is inspect.Parameter.KEYWORD_ONLY]
    variadic = [parameter for parameter in parameters if parameter.kind is inspect.Parameter.VAR_KEYWORD]
    wrapper.__signature__ = inspect.Signature(leading + added + keyword_only + variadic)


def preserve_transform_arguments(cls: _T) -> _T:
    """Add the deprecated placement arguments to a wrapper's constructor.

    The wrapped constructor keeps its own arguments and their defaults; they are forwarded unchanged. Apply this
    to a class that derives, directly or indirectly, from :class:`isaacsim.foundation.objects.Xform`.

    Args:
        cls: Deprecated wrapper class to modify in place.

    Returns:
        The same class, with its constructor replaced.
    """
    original = cls.__init__

    def __init__(  # noqa: D417
        self: Any,
        paths: Any,
        *,
        positions: Any = None,
        translations: Any = None,
        orientations: Any = None,
        scales: Any = None,
        **kwargs: Any,
    ) -> None:
        """Wrap the prims and place them.

        Args:
            paths: Prim path or paths to wrap.
            positions: Positions in the world frame. Mutually exclusive with ``translations``.
            translations: Translations in the local frame. Mutually exclusive with ``positions``.
            orientations: Orientations (shape ``(N, 4)``, quaternion ``wxyz``).
            scales: Scales to apply to the prims.
            **kwargs: Arguments accepted by the wrapped class.

        Raises:
            ValueError: If both ``positions`` and ``translations`` are specified.
        """  # noqa: DOC101, DOC103
        if positions is not None and translations is not None:
            raise ValueError("Both 'positions' and 'translations' are specified. Specify only one of them")
        original(self, paths, **kwargs)
        if positions is not None or orientations is not None:
            self.set_world_poses(positions, orientations)
        if translations is not None or orientations is not None:
            self.set_local_poses(translations, orientations)
        if scales is not None:
            self.set_local_scales(scales)

    _merge_signature(__init__, original)
    cls.__init__ = __init__
    return cls
