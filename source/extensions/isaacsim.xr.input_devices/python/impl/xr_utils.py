# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import carb
import numpy as np
from pxr import Gf
from omni.kit.xr.core import XRCore, XRPoseValidityFlags
from typing import Dict, List, Tuple


def compute_delta_errors(a: Gf.Matrix4d, b: Gf.Matrix4d) -> Tuple[float, float]:
    """Return (translation_error_m, rotation_error_deg) between transforms a and b."""
    try:
        delta = a * b.GetInverse()
        t = delta.ExtractTranslation()
        trans_err = float(np.linalg.norm([t[0], t[1], t[2]]))
        q = delta.ExtractRotation().GetQuat()
        w = float(max(min(q.GetReal(), 1.0), -1.0))
        ang = 2.0 * float(np.arccos(w))
        ang_deg = float(np.degrees(ang))
        if ang_deg > 180.0:
            ang_deg = 360.0 - ang_deg
        return trans_err, ang_deg
    except Exception:
        return float('inf'), float('inf')


def average_transforms(mats: List[Gf.Matrix4d]) -> Gf.Matrix4d:
    """Average rigid transforms: mean translation and normalized mean quaternion (with sign alignment)."""
    if not mats:
        return None
    ref_quat = mats[0].ExtractRotation().GetQuat()
    ref = np.array([ref_quat.GetReal(), *ref_quat.GetImaginary()])
    acc_q = np.zeros(4, dtype=np.float64)
    acc_t = np.zeros(3, dtype=np.float64)
    for m in mats:
        t = m.ExtractTranslation()
        acc_t += np.array([t[0], t[1], t[2]], dtype=np.float64)
        q = m.ExtractRotation().GetQuat()
        qi = np.array([q.GetReal(), *q.GetImaginary()], dtype=np.float64)
        if np.dot(qi, ref) < 0.0:
            qi = -qi
        acc_q += qi
    mean_t = acc_t / float(len(mats))
    norm = np.linalg.norm(acc_q)
    if norm <= 1e-12:
        quat_avg = Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))
    else:
        qn = acc_q / norm
        quat_avg = Gf.Quatd(float(qn[0]), Gf.Vec3d(float(qn[1]), float(qn[2]), float(qn[3])))
    rot3 = Gf.Matrix3d().SetRotate(quat_avg)
    trans = Gf.Vec3d(float(mean_t[0]), float(mean_t[1]), float(mean_t[2]))
    return Gf.Matrix4d(rot3, trans)


def select_mode_cluster(mats: List[Gf.Matrix4d], trans_thresh_m: float = 0.02, rot_thresh_deg: float = 5.0) -> List[Gf.Matrix4d]:
    """Return the largest cluster (mode) under proximity thresholds."""
    if not mats:
        return []
    best_cluster: List[Gf.Matrix4d] = []
    for center in mats:
        cluster: List[Gf.Matrix4d] = []
        for m in mats:
            trans_err, rot_err = compute_delta_errors(m, center)
            if trans_err <= trans_thresh_m and rot_err <= rot_thresh_deg:
                cluster.append(m)
        if len(cluster) > len(best_cluster):
            best_cluster = cluster
    return best_cluster


def get_openxr_wrist_matrix(hand: str) -> Gf.Matrix4d:
    """Get OpenXR wrist world matrix if valid, else None."""
    hand = hand.lower()
    try:
        hand_device = XRCore.get_singleton().get_input_device(f"/user/hand/{hand}")
        if hand_device is None:
            return None
        joints = hand_device.get_all_virtual_world_poses()
        if 'wrist' not in joints:
            return None
        joint = joints['wrist']
        required = XRPoseValidityFlags.POSITION_VALID | XRPoseValidityFlags.ORIENTATION_VALID
        if (joint.validity_flags & required) != required:
            return None
        return joint.pose_matrix
    except Exception as e:
        carb.log_warn(f"OpenXR {hand} wrist fetch failed: {e}")
        return None


def get_vive_wrist_ids(vive_data: Dict):
    """Return tuple (wm0_id, wm1_id) if available, else Nones. Sort by key for stability."""
    wm_ids = [k for k in vive_data.keys() if len(k) >= 2 and k[:2] == 'WM']
    wm_ids.sort()
    if len(wm_ids) >= 2:
        return wm_ids[0], wm_ids[1]
    if len(wm_ids) == 1:
        return wm_ids[0], None
    return None, None 


def pose_to_matrix(pose: Dict) -> Gf.Matrix4d:
    """Convert position and orientation to a 4x4 matrix."""
    pos, ori = pose['position'], pose['orientation']
    quat = Gf.Quatd(ori[0], Gf.Vec3d(ori[1], ori[2], ori[3]))
    rot = Gf.Matrix3d().SetRotate(quat)
    trans = Gf.Vec3d(pos[0], pos[1], pos[2])
    return Gf.Matrix4d(rot, trans)

def matrix_to_pose(matrix: Gf.Matrix4d) -> Dict:
    """Convert a 4x4 matrix to position and orientation."""
    pos = matrix.ExtractTranslation()
    rot = matrix.ExtractRotation()
    quat = rot.GetQuat()
    return {
        'position': [pos[0], pos[1], pos[2]],
        'orientation': [quat.GetReal(), quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2]]
    }

def get_pairing_error(trans_errs: list, rot_errs: list) -> float:
    def _median(values: list) -> float:
        try:
            return float(np.median(np.asarray(values, dtype=np.float64)))
        except Exception:
            return float('inf')
    return _median(trans_errs) + 0.01 * _median(rot_errs)
