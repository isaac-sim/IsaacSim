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

"""Pure math for joint drive natural frequency / damping (matches JointItem UI conventions).

Revolute joint stiffness in USD is stored in the same numeric convention used by
:class:`JointItem` in ``joint_table_widget.py``: an effective "degree-scaled"
stiffness where ``K_rad = K_stored / DEG_TO_RAD`` for frequency calculations.
"""

from __future__ import annotations

import math

DEG_TO_RAD = math.pi / 180.0


def meq_for_drive_frequency(*, use_force_drive: bool, m_eq: float) -> float:
    """Equivalent inertia (or mass) scalar used in natural-frequency formulas.

    Matches :class:`JointItem` behavior: acceleration drive uses ``1.0``;
    force drive uses ``m_eq`` with a fallback when zero.
    """
    if not use_force_drive:
        return 1.0
    return 1.0 if m_eq == 0 else m_eq


def natural_frequency_hz_from_stiffness_revolute_position(
    stiffness_stored: float, *, use_force_drive: bool, m_eq: float
) -> float:
    """Natural frequency (Hz) from stiffness for revolute position drive (non-mimic).

    Args:
        stiffness_stored: Stiffness as stored on the joint / in the UI model.
        use_force_drive: True if drive type is force (uses ``m_eq``).
        m_eq: Equivalent inertia from the gain tuner pipeline (kg*m^2).
    """
    m = meq_for_drive_frequency(use_force_drive=use_force_drive, m_eq=m_eq)
    stiffness_rad = stiffness_stored / DEG_TO_RAD
    return math.sqrt(stiffness_rad / m) / (2.0 * math.pi)


def damping_ratio_from_stiffness_damping_revolute_position(
    damping: float, stiffness_stored: float, *, use_force_drive: bool, m_eq: float
) -> float:
    """Damping ratio from stiffness and damping (JointItem formula, non-mimic).

    Uses the same radian-equivalent stiffness as :func:`natural_frequency_hz_from_stiffness_revolute_position`
    and :func:`stiffness_stored_and_damping_from_natural_frequency_revolute_position`
    (``K_rad = stiffness_stored / DEG_TO_RAD``), not ``sqrt(m * stiffness_stored)``.
    """
    if stiffness_stored <= 0:
        return 0.0
    m = meq_for_drive_frequency(use_force_drive=use_force_drive, m_eq=m_eq)
    stiffness_rad = stiffness_stored / DEG_TO_RAD
    return damping / (2.0 * math.sqrt(m * stiffness_rad))


def stiffness_stored_and_damping_from_natural_frequency_revolute_position(
    natural_freq_hz: float, damping_ratio: float, *, use_force_drive: bool, m_eq: float
) -> tuple[float, float]:
    """Compute stored stiffness and damping from ``f_n`` and ``zeta`` (natural-frequency mode).

    Returns:
        ``(stiffness_stored, damping)`` matching :meth:`JointItem.compute_drive_stiffness`
        and the damping update in :meth:`JointItem.on_update_damping_ratio`.
    """
    m = meq_for_drive_frequency(use_force_drive=use_force_drive, m_eq=m_eq)
    stiffness_rad = m * ((2.0 * math.pi * natural_freq_hz) ** 2)
    stiffness_stored = stiffness_rad * DEG_TO_RAD
    damping = damping_ratio * (2.0 * math.sqrt(m * stiffness_rad))
    return stiffness_stored, damping


def damping_from_damping_ratio_revolute_position(
    damping_ratio: float, stiffness_stored: float, *, use_force_drive: bool, m_eq: float
) -> float:
    """Damping from damping ratio given current stored stiffness (uses ``sqrt(m * K_rad)``)."""
    m = meq_for_drive_frequency(use_force_drive=use_force_drive, m_eq=m_eq)
    stiffness_rad = stiffness_stored / DEG_TO_RAD
    return damping_ratio * (2.0 * math.sqrt(m * stiffness_rad))
