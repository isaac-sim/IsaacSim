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

"""Utility functions for mathematical operations and tensor manipulations using PyTorch."""

import os
import random

import numpy as np
import warp as wp
from isaacsim.core.deprecation_manager import import_module

torch = import_module("torch")


@torch.jit.script
def normalize(x, eps: float = 1e-9):  # noqa: ANN001, ANN201
    """Normalize a tensor along the last dimension."""  # noqa: DOC101, DOC103, DOC106, DOC107, DOC201
    return x / x.norm(p=2, dim=-1).clamp(min=eps, max=None).unsqueeze(-1)


@torch.jit.script
def scale_transform(x: torch.Tensor, lower: torch.Tensor, upper: torch.Tensor) -> torch.Tensor:
    """Normalizes a given input tensor to a range of [-1, 1].

    @note It uses pytorch broadcasting functionality to deal with batched input.

    Args:
        x: Input tensor of shape (N, dims).
        lower: The minimum value of the tensor. Shape (dims,)
        upper: The maximum value of the tensor. Shape (dims,)

    Returns:
        Normalized transform of the tensor. Shape (N, dims)
    """
    # default value of center
    offset = (lower + upper) * 0.5
    # return normalized tensor
    return 2 * (x - offset) / (upper - lower)


@torch.jit.script
def unscale_transform(x: torch.Tensor, lower: torch.Tensor, upper: torch.Tensor) -> torch.Tensor:
    """Denormalizes a given input tensor from range of [-1, 1] to (lower, upper).

    @note It uses pytorch broadcasting functionality to deal with batched input.

    Args:
        x: Input tensor of shape (N, dims).
        lower: The minimum value of the tensor. Shape (dims,)
        upper: The maximum value of the tensor. Shape (dims,)

    Returns:
        Denormalized transform of the tensor. Shape (N, dims)
    """
    # default value of center
    offset = (lower + upper) * 0.5
    # return normalized tensor
    return x * (upper - lower) * 0.5 + offset


@torch.jit.script
def copysign(a, b):  # noqa: ANN001, ANN201
    """Return a tensor with magnitude of a and sign of b."""  # noqa: DOC101, DOC103, DOC106, DOC107, DOC201
    # type: (float, Tensor) -> Tensor
    a = torch.tensor(a, device=b.device, dtype=torch.float).repeat(b.shape[0])
    return torch.abs(a) * torch.sign(b)


@torch.jit.script
def torch_rand_float(lower, upper, shape, device):  # noqa: ANN001, ANN201
    """Generate random floats uniformly distributed in [lower, upper]."""  # noqa: DOC101, DOC103, DOC106, DOC107, DOC201
    # type: (float, float, Tuple[int, int], str) -> Tensor
    return (upper - lower) * torch.rand(*shape, device=device) + lower


@torch.jit.script
def torch_random_dir_2(shape, device):  # noqa: ANN001, ANN201
    """Generate random 2D unit direction vectors."""  # noqa: DOC101, DOC103, DOC106, DOC107, DOC201
    # type: (Tuple[int, int], str) -> Tensor
    angle = torch_rand_float(-np.pi, np.pi, shape, device).squeeze(-1)
    return torch.stack([torch.cos(angle), torch.sin(angle)], dim=-1)


@torch.jit.script
def tensor_clamp(t: torch.Tensor, min_t: torch.Tensor, max_t: torch.Tensor) -> torch.Tensor:
    """Clamp tensor values element-wise between min and max tensors.

    Args:
        t: Input tensor to clamp.
        min_t: Tensor of minimum values.
        max_t: Tensor of maximum values.

    Returns:
        Tensor with values clamped between min_t and max_t.
    """
    return torch.max(torch.min(t, max_t), min_t)


@torch.jit.script
def scale(x: torch.Tensor, lower: torch.Tensor, upper: torch.Tensor) -> torch.Tensor:
    """Scale a tensor from [-1, 1] to [lower, upper] range.

    Args:
        x: Input tensor in [-1, 1] range.
        lower: Lower bound of the target range.
        upper: Upper bound of the target range.

    Returns:
        Tensor scaled to [lower, upper] range.
    """
    return 0.5 * (x + 1.0) * (upper - lower) + lower


@torch.jit.script
def unscale(x: torch.Tensor, lower: torch.Tensor, upper: torch.Tensor) -> torch.Tensor:
    """Scale a tensor from [lower, upper] range to [-1, 1].

    Args:
        x: Input tensor in [lower, upper] range.
        lower: Lower bound of the source range.
        upper: Upper bound of the source range.

    Returns:
        Tensor scaled to [-1, 1] range.
    """
    return (2.0 * x - upper - lower) / (upper - lower)


def unscale_np(x: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Denormalizes values from [-1, 1] range back to original [lower, upper] range using NumPy operations.

    Args:
        x: Input values to denormalize.
        lower: Lower bound of the target range.
        upper: Upper bound of the target range.

    Returns:
        Denormalized values in the original range.
    """
    return (2.0 * x - upper - lower) / (upper - lower)


def set_seed(seed: int, torch_deterministic: bool = False) -> int:
    """Set seed across modules.

    Sets random seeds for Python, NumPy, PyTorch, Warp, and environment variables to ensure
    reproducibility across different libraries. Optionally enables deterministic algorithms
    for PyTorch operations.

    Args:
        seed: Random seed value. If -1, generates a random seed or uses 42 if torch_deterministic is True.
        torch_deterministic: Whether to enable deterministic algorithms in PyTorch.

    Returns:
        The actual seed value that was set.
    """
    if seed == -1 and torch_deterministic:
        seed = 42
    elif seed == -1:
        seed = np.random.randint(0, 10000)
    print(f"Setting seed: {seed}")

    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    os.environ["PYTHONHASHSEED"] = str(seed)
    torch.cuda.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    wp.rand_init(seed)

    if torch_deterministic:
        # refer to https://docs.nvidia.com/cuda/cublas/index.html#cublasApi_reproducibility
        os.environ["CUBLAS_WORKSPACE_CONFIG"] = ":4096:8"
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True
        torch.use_deterministic_algorithms(True)
    else:
        torch.backends.cudnn.benchmark = True
        torch.backends.cudnn.deterministic = False

    return seed


def matmul(matrix_a: object, matrix_b: object) -> object:
    """Performs matrix multiplication between two tensors.

    Args:
        matrix_a: First input matrix.
        matrix_b: Second input matrix.

    Returns:
        Result of matrix multiplication.
    """
    return torch.matmul(matrix_a, matrix_b)


def sin(data: object) -> object:
    """Computes the sine of the input tensor.

    Args:
        data: Input tensor.

    Returns:
        Tensor with sine values computed element-wise.
    """
    return torch.sin(data)


def cos(data: object) -> object:
    """Computes the cosine of the input tensor.

    Args:
        data: Input tensor.

    Returns:
        Tensor with cosine values computed element-wise.
    """
    return torch.cos(data)


def transpose_2d(data: object) -> object:
    """Transposes a 2D tensor by swapping dimensions 0 and 1.

    Args:
        data: Input 2D tensor to transpose.

    Raises:
        ValueError: If input tensor has more than 2 dimensions.

    Returns:
        Transposed tensor with dimensions swapped.
    """
    if data.dim() > 2:
        raise ValueError(f"transpose_2d expects a 1D or 2D tensor, got {data.dim()}D tensor.")
    return torch.transpose(data, 1, 0)


def inverse(data: object) -> object:
    """Computes the matrix inverse of the input tensor.

    Args:
        data: Input tensor representing matrices to invert.

    Returns:
        Tensor containing the matrix inverses.
    """
    return torch.linalg.inv(data)
