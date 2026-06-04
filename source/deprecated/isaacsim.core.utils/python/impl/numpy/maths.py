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

"""Mathematical utility functions for matrix operations and trigonometric computations."""

import numpy as np


def matmul(matrix_a: np.ndarray, matrix_b: np.ndarray) -> np.ndarray:
    """Perform matrix multiplication between two matrices.

    Args:
        matrix_a: First input matrix.
        matrix_b: Second input matrix.

    Returns:
        The result of matrix multiplication.
    """
    return np.matmul(matrix_a, matrix_b)


def sin(data: np.ndarray) -> np.ndarray:
    """Compute the sine of the input data.

    Args:
        data: Input array or scalar value.

    Returns:
        The sine values of the input data.
    """
    return np.sin(data)


def cos(data: np.ndarray) -> np.ndarray:
    """Compute the cosine of the input data.

    Args:
        data: Input array or scalar value.

    Returns:
        The cosine values of the input data.
    """
    return np.cos(data)


def transpose_2d(data: np.ndarray) -> np.ndarray:
    """Transpose the input 2D array.

    Args:
        data: Input 2D array to transpose.

    Raises:
        ValueError: If input array has more than 2 dimensions.

    Returns:
        The transposed array.
    """
    if data.ndim > 2:
        raise ValueError(f"transpose_2d expects a 1D or 2D array, got {data.ndim}D array.")
    return np.transpose(data)


def inverse(data: np.ndarray) -> np.ndarray:
    """Compute the matrix inverse of the input data.

    Args:
        data: Input matrix to invert.

    Returns:
        The inverse of the input matrix.
    """
    return np.linalg.inv(data)
