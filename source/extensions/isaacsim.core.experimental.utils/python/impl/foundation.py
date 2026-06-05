# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Functions for working with USD/USDRT foundations, e.g.: Scene Description Foundations (Sdf), Graphics Foundations (Gf)."""

from __future__ import annotations

from typing import Literal

import usdrt
from pxr import Sdf

from . import backend as backend_utils


def get_value_type_names(
    *, format: Literal[str, Sdf.ValueTypeNames, usdrt.Sdf.ValueTypeNames] = str
) -> list[str | Sdf.ValueTypeName | usdrt.Sdf.ValueTypeName]:
    """Get all supported (from Isaac Sim's Core API perspective) value type names.

    Args:
        format: Format to get the value type names in.

    Returns:
        List of value type names.

    Raises:
        ValueError: If the format is invalid.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.foundation as foundation_utils
        >>> import usdrt
        >>> from pxr import Sdf
        >>>
        >>> foundation_utils.get_value_type_names(format=str)
        ['asset', 'asset[]', 'bool', 'bool[]', 'color3d', ...]
        >>> foundation_utils.get_value_type_names(format=Sdf.ValueTypeNames)
        [<pxr.Sdf.ValueTypeName object at 0x...>, <pxr.Sdf.ValueTypeName object at 0x...>, ...]
        >>> foundation_utils.get_value_type_names(format=usdrt.Sdf.ValueTypeNames)
        [Sdf.ValueTypeName('asset'), Sdf.ValueTypeName('asset[]'), Sdf.ValueTypeName('bool'), ...]
    """
    if format is str:
        return [item[1] for item in _VALUE_TYPE_NAMES]
    elif format is Sdf.ValueTypeNames:
        return [item[2] for item in _VALUE_TYPE_NAMES]
    elif format is usdrt.Sdf.ValueTypeNames:
        return [item[3] for item in _VALUE_TYPE_NAMES]
    else:
        raise ValueError(f"Invalid format: '{format}'")


def resolve_value_type_name(
    type_name: str | Sdf.ValueTypeName | usdrt.Sdf.ValueTypeName, *, backend: str | None = None
) -> Sdf.ValueTypeName | usdrt.Sdf.ValueTypeName:
    """Resolve the value type name for a given (value) type name according to the USD/USDRT specifications.

    Backends: :guilabel:`usd`, :guilabel:`usdrt`, :guilabel:`fabric`.

    Args:
        type_name: Type name.
        backend: Backend to use to get the value type name. If not ``None``, it has precedence over the current backend
            set via the :py:func:`~isaacsim.core.experimental.utils.impl.backend.use_backend` context manager.

    Returns:
        Value type name instance.

    Raises:
        ValueError: If the backend is not supported.
        ValueError: If the type name is invalid.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.foundation as foundation_utils
        >>>
        >>> foundation_utils.resolve_value_type_name("color3f", backend="usd")
        <pxr.Sdf.ValueTypeName object at 0x...>
        >>> foundation_utils.resolve_value_type_name("color3f", backend="usdrt")
        Sdf.ValueTypeName('float3 (color)')
    """
    # get backend
    if backend is None:
        backend = backend_utils.get_current_backend(["usd", "usdrt", "fabric"])
    elif backend not in ["usd", "usdrt", "fabric"]:
        raise ValueError(f"Invalid backend: {backend}")
    # get value type name
    value_type_name = None
    if isinstance(type_name, str):
        value_type_name = (
            _VALUE_TYPE_NAMES_STR_TO_USD.get(type_name)
            if backend == "usd"
            else _VALUE_TYPE_NAMES_STR_TO_USDRT.get(type_name)
        )
    elif isinstance(type_name, Sdf.ValueTypeName):
        value_type_name = (
            type_name
            if backend == "usd"
            else _VALUE_TYPE_NAMES_STR_TO_USDRT.get(_VALUE_TYPE_NAMES_USD_TO_STR.get(type_name))
        )
    elif isinstance(type_name, usdrt.Sdf.ValueTypeName):
        value_type_name = (
            _VALUE_TYPE_NAMES_STR_TO_USD.get(_VALUE_TYPE_NAMES_USDRT_TO_STR.get(type_name))
            if backend == "usd"
            else type_name
        )
    # check if valid
    if value_type_name is None:
        raise ValueError(f"Invalid type name: '{type_name}' ({type(type_name)})")
    return value_type_name


def value_type_name_to_str(type_name: str | Sdf.ValueTypeName | usdrt.Sdf.ValueTypeName) -> str:
    """Get the string representation of a given value type name.

    Args:
        type_name: Value type name.

    Returns:
        String representation of the value type name.

    Raises:
        ValueError: If the type name is invalid.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.foundation as foundation_utils
        >>> import usdrt
        >>> from pxr import Sdf
        >>>
        >>> foundation_utils.value_type_name_to_str("color3f[]")
        'color3f[]'
        >>> foundation_utils.value_type_name_to_str(Sdf.ValueTypeNames.Color3fArray)
        'color3f[]'
        >>> foundation_utils.value_type_name_to_str(usdrt.Sdf.ValueTypeNames.Color3fArray)
        'color3f[]'
    """
    string = None
    if type_name in _VALUE_TYPE_NAMES_STR_TO_USD:
        string = type_name
    elif isinstance(type_name, Sdf.ValueTypeName):
        string = _VALUE_TYPE_NAMES_USD_TO_STR.get(type_name)
    elif isinstance(type_name, usdrt.Sdf.ValueTypeName):
        string = _VALUE_TYPE_NAMES_USDRT_TO_STR.get(type_name)
    # check if valid
    if string is None:
        raise ValueError(f"Invalid type name: '{type_name}' ({type(type_name)})")
    return string


"""
Internal variables and functions.
"""

_VALUE_TYPE_NAMES = [
    ("Asset", "asset", Sdf.ValueTypeNames.Asset, usdrt.Sdf.ValueTypeNames.Asset),
    ("AssetArray", "asset[]", Sdf.ValueTypeNames.AssetArray, usdrt.Sdf.ValueTypeNames.AssetArray),
    ("Bool", "bool", Sdf.ValueTypeNames.Bool, usdrt.Sdf.ValueTypeNames.Bool),
    ("BoolArray", "bool[]", Sdf.ValueTypeNames.BoolArray, usdrt.Sdf.ValueTypeNames.BoolArray),
    ("Color3d", "color3d", Sdf.ValueTypeNames.Color3d, usdrt.Sdf.ValueTypeNames.Color3d),
    ("Color3dArray", "color3d[]", Sdf.ValueTypeNames.Color3dArray, usdrt.Sdf.ValueTypeNames.Color3dArray),
    ("Color3f", "color3f", Sdf.ValueTypeNames.Color3f, usdrt.Sdf.ValueTypeNames.Color3f),
    ("Color3fArray", "color3f[]", Sdf.ValueTypeNames.Color3fArray, usdrt.Sdf.ValueTypeNames.Color3fArray),
    ("Color3h", "color3h", Sdf.ValueTypeNames.Color3h, usdrt.Sdf.ValueTypeNames.Color3h),
    ("Color3hArray", "color3h[]", Sdf.ValueTypeNames.Color3hArray, usdrt.Sdf.ValueTypeNames.Color3hArray),
    ("Color4d", "color4d", Sdf.ValueTypeNames.Color4d, usdrt.Sdf.ValueTypeNames.Color4d),
    ("Color4dArray", "color4d[]", Sdf.ValueTypeNames.Color4dArray, usdrt.Sdf.ValueTypeNames.Color4dArray),
    ("Color4f", "color4f", Sdf.ValueTypeNames.Color4f, usdrt.Sdf.ValueTypeNames.Color4f),
    ("Color4fArray", "color4f[]", Sdf.ValueTypeNames.Color4fArray, usdrt.Sdf.ValueTypeNames.Color4fArray),
    ("Color4h", "color4h", Sdf.ValueTypeNames.Color4h, usdrt.Sdf.ValueTypeNames.Color4h),
    ("Color4hArray", "color4h[]", Sdf.ValueTypeNames.Color4hArray, usdrt.Sdf.ValueTypeNames.Color4hArray),
    ("Double", "double", Sdf.ValueTypeNames.Double, usdrt.Sdf.ValueTypeNames.Double),
    ("Double2", "double2", Sdf.ValueTypeNames.Double2, usdrt.Sdf.ValueTypeNames.Double2),
    ("Double2Array", "double2[]", Sdf.ValueTypeNames.Double2Array, usdrt.Sdf.ValueTypeNames.Double2Array),
    ("Double3", "double3", Sdf.ValueTypeNames.Double3, usdrt.Sdf.ValueTypeNames.Double3),
    ("Double3Array", "double3[]", Sdf.ValueTypeNames.Double3Array, usdrt.Sdf.ValueTypeNames.Double3Array),
    ("Double4", "double4", Sdf.ValueTypeNames.Double4, usdrt.Sdf.ValueTypeNames.Double4),
    ("Double4Array", "double4[]", Sdf.ValueTypeNames.Double4Array, usdrt.Sdf.ValueTypeNames.Double4Array),
    ("DoubleArray", "double[]", Sdf.ValueTypeNames.DoubleArray, usdrt.Sdf.ValueTypeNames.DoubleArray),
    ("Float", "float", Sdf.ValueTypeNames.Float, usdrt.Sdf.ValueTypeNames.Float),
    ("Float2", "float2", Sdf.ValueTypeNames.Float2, usdrt.Sdf.ValueTypeNames.Float2),
    ("Float2Array", "float2[]", Sdf.ValueTypeNames.Float2Array, usdrt.Sdf.ValueTypeNames.Float2Array),
    ("Float3", "float3", Sdf.ValueTypeNames.Float3, usdrt.Sdf.ValueTypeNames.Float3),
    ("Float3Array", "float3[]", Sdf.ValueTypeNames.Float3Array, usdrt.Sdf.ValueTypeNames.Float3Array),
    ("Float4", "float4", Sdf.ValueTypeNames.Float4, usdrt.Sdf.ValueTypeNames.Float4),
    ("Float4Array", "float4[]", Sdf.ValueTypeNames.Float4Array, usdrt.Sdf.ValueTypeNames.Float4Array),
    ("FloatArray", "float[]", Sdf.ValueTypeNames.FloatArray, usdrt.Sdf.ValueTypeNames.FloatArray),
    ("Frame4d", "frame4d", Sdf.ValueTypeNames.Frame4d, usdrt.Sdf.ValueTypeNames.Frame4d),
    ("Frame4dArray", "frame4d[]", Sdf.ValueTypeNames.Frame4dArray, usdrt.Sdf.ValueTypeNames.Frame4dArray),
    ("Half", "half", Sdf.ValueTypeNames.Half, usdrt.Sdf.ValueTypeNames.Half),
    ("Half2", "half2", Sdf.ValueTypeNames.Half2, usdrt.Sdf.ValueTypeNames.Half2),
    ("Half2Array", "half2[]", Sdf.ValueTypeNames.Half2Array, usdrt.Sdf.ValueTypeNames.Half2Array),
    ("Half3", "half3", Sdf.ValueTypeNames.Half3, usdrt.Sdf.ValueTypeNames.Half3),
    ("Half3Array", "half3[]", Sdf.ValueTypeNames.Half3Array, usdrt.Sdf.ValueTypeNames.Half3Array),
    ("Half4", "half4", Sdf.ValueTypeNames.Half4, usdrt.Sdf.ValueTypeNames.Half4),
    ("Half4Array", "half4[]", Sdf.ValueTypeNames.Half4Array, usdrt.Sdf.ValueTypeNames.Half4Array),
    ("HalfArray", "half[]", Sdf.ValueTypeNames.HalfArray, usdrt.Sdf.ValueTypeNames.HalfArray),
    ("Int", "int", Sdf.ValueTypeNames.Int, usdrt.Sdf.ValueTypeNames.Int),
    ("Int2", "int2", Sdf.ValueTypeNames.Int2, usdrt.Sdf.ValueTypeNames.Int2),
    ("Int2Array", "int2[]", Sdf.ValueTypeNames.Int2Array, usdrt.Sdf.ValueTypeNames.Int2Array),
    ("Int3", "int3", Sdf.ValueTypeNames.Int3, usdrt.Sdf.ValueTypeNames.Int3),
    ("Int3Array", "int3[]", Sdf.ValueTypeNames.Int3Array, usdrt.Sdf.ValueTypeNames.Int3Array),
    ("Int4", "int4", Sdf.ValueTypeNames.Int4, usdrt.Sdf.ValueTypeNames.Int4),
    ("Int4Array", "int4[]", Sdf.ValueTypeNames.Int4Array, usdrt.Sdf.ValueTypeNames.Int4Array),
    ("Int64", "int64", Sdf.ValueTypeNames.Int64, usdrt.Sdf.ValueTypeNames.Int64),
    ("Int64Array", "int64[]", Sdf.ValueTypeNames.Int64Array, usdrt.Sdf.ValueTypeNames.Int64Array),
    ("IntArray", "int[]", Sdf.ValueTypeNames.IntArray, usdrt.Sdf.ValueTypeNames.IntArray),
    ("Matrix2d", "matrix2d", Sdf.ValueTypeNames.Matrix2d, usdrt.Sdf.ValueTypeNames.Matrix2d),
    ("Matrix2dArray", "matrix2d[]", Sdf.ValueTypeNames.Matrix2dArray, usdrt.Sdf.ValueTypeNames.Matrix2dArray),
    ("Matrix3d", "matrix3d", Sdf.ValueTypeNames.Matrix3d, usdrt.Sdf.ValueTypeNames.Matrix3d),
    ("Matrix3dArray", "matrix3d[]", Sdf.ValueTypeNames.Matrix3dArray, usdrt.Sdf.ValueTypeNames.Matrix3dArray),
    ("Matrix4d", "matrix4d", Sdf.ValueTypeNames.Matrix4d, usdrt.Sdf.ValueTypeNames.Matrix4d),
    ("Matrix4dArray", "matrix4d[]", Sdf.ValueTypeNames.Matrix4dArray, usdrt.Sdf.ValueTypeNames.Matrix4dArray),
    ("Normal3d", "normal3d", Sdf.ValueTypeNames.Normal3d, usdrt.Sdf.ValueTypeNames.Normal3d),
    ("Normal3dArray", "normal3d[]", Sdf.ValueTypeNames.Normal3dArray, usdrt.Sdf.ValueTypeNames.Normal3dArray),
    ("Normal3f", "normal3f", Sdf.ValueTypeNames.Normal3f, usdrt.Sdf.ValueTypeNames.Normal3f),
    ("Normal3fArray", "normal3f[]", Sdf.ValueTypeNames.Normal3fArray, usdrt.Sdf.ValueTypeNames.Normal3fArray),
    ("Normal3h", "normal3h", Sdf.ValueTypeNames.Normal3h, usdrt.Sdf.ValueTypeNames.Normal3h),
    ("Normal3hArray", "normal3h[]", Sdf.ValueTypeNames.Normal3hArray, usdrt.Sdf.ValueTypeNames.Normal3hArray),
    ("Point3d", "point3d", Sdf.ValueTypeNames.Point3d, usdrt.Sdf.ValueTypeNames.Point3d),
    ("Point3dArray", "point3d[]", Sdf.ValueTypeNames.Point3dArray, usdrt.Sdf.ValueTypeNames.Point3dArray),
    ("Point3f", "point3f", Sdf.ValueTypeNames.Point3f, usdrt.Sdf.ValueTypeNames.Point3f),
    ("Point3fArray", "point3f[]", Sdf.ValueTypeNames.Point3fArray, usdrt.Sdf.ValueTypeNames.Point3fArray),
    ("Point3h", "point3h", Sdf.ValueTypeNames.Point3h, usdrt.Sdf.ValueTypeNames.Point3h),
    ("Point3hArray", "point3h[]", Sdf.ValueTypeNames.Point3hArray, usdrt.Sdf.ValueTypeNames.Point3hArray),
    ("Quatd", "quatd", Sdf.ValueTypeNames.Quatd, usdrt.Sdf.ValueTypeNames.Quatd),
    ("QuatdArray", "quatd[]", Sdf.ValueTypeNames.QuatdArray, usdrt.Sdf.ValueTypeNames.QuatdArray),
    ("Quatf", "quatf", Sdf.ValueTypeNames.Quatf, usdrt.Sdf.ValueTypeNames.Quatf),
    ("QuatfArray", "quatf[]", Sdf.ValueTypeNames.QuatfArray, usdrt.Sdf.ValueTypeNames.QuatfArray),
    ("Quath", "quath", Sdf.ValueTypeNames.Quath, usdrt.Sdf.ValueTypeNames.Quath),
    ("QuathArray", "quath[]", Sdf.ValueTypeNames.QuathArray, usdrt.Sdf.ValueTypeNames.QuathArray),
    ("String", "string", Sdf.ValueTypeNames.String, usdrt.Sdf.ValueTypeNames.String),
    ("StringArray", "string[]", Sdf.ValueTypeNames.StringArray, usdrt.Sdf.ValueTypeNames.StringArray),
    ("TexCoord2d", "texCoord2d", Sdf.ValueTypeNames.TexCoord2d, usdrt.Sdf.ValueTypeNames.TexCoord2d),
    ("TexCoord2dArray", "texCoord2d[]", Sdf.ValueTypeNames.TexCoord2dArray, usdrt.Sdf.ValueTypeNames.TexCoord2dArray),
    ("TexCoord2f", "texCoord2f", Sdf.ValueTypeNames.TexCoord2f, usdrt.Sdf.ValueTypeNames.TexCoord2f),
    ("TexCoord2fArray", "texCoord2f[]", Sdf.ValueTypeNames.TexCoord2fArray, usdrt.Sdf.ValueTypeNames.TexCoord2fArray),
    ("TexCoord2h", "texCoord2h", Sdf.ValueTypeNames.TexCoord2h, usdrt.Sdf.ValueTypeNames.TexCoord2h),
    ("TexCoord2hArray", "texCoord2h[]", Sdf.ValueTypeNames.TexCoord2hArray, usdrt.Sdf.ValueTypeNames.TexCoord2hArray),
    ("TexCoord3d", "texCoord3d", Sdf.ValueTypeNames.TexCoord3d, usdrt.Sdf.ValueTypeNames.TexCoord3d),
    ("TexCoord3dArray", "texCoord3d[]", Sdf.ValueTypeNames.TexCoord3dArray, usdrt.Sdf.ValueTypeNames.TexCoord3dArray),
    ("TexCoord3f", "texCoord3f", Sdf.ValueTypeNames.TexCoord3f, usdrt.Sdf.ValueTypeNames.TexCoord3f),
    ("TexCoord3fArray", "texCoord3f[]", Sdf.ValueTypeNames.TexCoord3fArray, usdrt.Sdf.ValueTypeNames.TexCoord3fArray),
    ("TexCoord3h", "texCoord3h", Sdf.ValueTypeNames.TexCoord3h, usdrt.Sdf.ValueTypeNames.TexCoord3h),
    ("TexCoord3hArray", "texCoord3h[]", Sdf.ValueTypeNames.TexCoord3hArray, usdrt.Sdf.ValueTypeNames.TexCoord3hArray),
    ("TimeCode", "timecode", Sdf.ValueTypeNames.TimeCode, usdrt.Sdf.ValueTypeNames.TimeCode),
    ("TimeCodeArray", "timecode[]", Sdf.ValueTypeNames.TimeCodeArray, usdrt.Sdf.ValueTypeNames.TimeCodeArray),
    ("Token", "token", Sdf.ValueTypeNames.Token, usdrt.Sdf.ValueTypeNames.Token),
    ("TokenArray", "token[]", Sdf.ValueTypeNames.TokenArray, usdrt.Sdf.ValueTypeNames.TokenArray),
    ("UChar", "uchar", Sdf.ValueTypeNames.UChar, usdrt.Sdf.ValueTypeNames.UChar),
    ("UCharArray", "uchar[]", Sdf.ValueTypeNames.UCharArray, usdrt.Sdf.ValueTypeNames.UCharArray),
    ("UInt", "uint", Sdf.ValueTypeNames.UInt, usdrt.Sdf.ValueTypeNames.UInt),
    ("UInt64", "uint64", Sdf.ValueTypeNames.UInt64, usdrt.Sdf.ValueTypeNames.UInt64),
    ("UInt64Array", "uint64[]", Sdf.ValueTypeNames.UInt64Array, usdrt.Sdf.ValueTypeNames.UInt64Array),
    ("UIntArray", "uint[]", Sdf.ValueTypeNames.UIntArray, usdrt.Sdf.ValueTypeNames.UIntArray),
    ("Vector3d", "vector3d", Sdf.ValueTypeNames.Vector3d, usdrt.Sdf.ValueTypeNames.Vector3d),
    ("Vector3dArray", "vector3d[]", Sdf.ValueTypeNames.Vector3dArray, usdrt.Sdf.ValueTypeNames.Vector3dArray),
    ("Vector3f", "vector3f", Sdf.ValueTypeNames.Vector3f, usdrt.Sdf.ValueTypeNames.Vector3f),
    ("Vector3fArray", "vector3f[]", Sdf.ValueTypeNames.Vector3fArray, usdrt.Sdf.ValueTypeNames.Vector3fArray),
    ("Vector3h", "vector3h", Sdf.ValueTypeNames.Vector3h, usdrt.Sdf.ValueTypeNames.Vector3h),
    ("Vector3hArray", "vector3h[]", Sdf.ValueTypeNames.Vector3hArray, usdrt.Sdf.ValueTypeNames.Vector3hArray),
]

# more efficient storage for lookups
_VALUE_TYPE_NAMES_STR_TO_USD = {}
_VALUE_TYPE_NAMES_USD_TO_STR = {}
_VALUE_TYPE_NAMES_STR_TO_USDRT = {}
_VALUE_TYPE_NAMES_USDRT_TO_STR = {}
for item in _VALUE_TYPE_NAMES:
    _VALUE_TYPE_NAMES_STR_TO_USD[item[1]] = item[2]
    _VALUE_TYPE_NAMES_USD_TO_STR[item[2]] = item[1]
    _VALUE_TYPE_NAMES_STR_TO_USDRT[item[1]] = item[3]
    _VALUE_TYPE_NAMES_USDRT_TO_STR[item[3]] = item[1]
