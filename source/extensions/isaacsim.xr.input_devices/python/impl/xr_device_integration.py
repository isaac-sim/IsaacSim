# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import carb
import time
import numpy as np
from typing import Dict
from pxr import Gf
from .manus_tracker import IsaacSimManusGloveTracker
from .vive_tracker import IsaacSimViveTracker
from omni.kit.xr.core import XR_INPUT_DEVICE_HAND_TRACKING_POSE_NAMES, XRCore, XRPoseValidityFlags


def get_xr_device_integration():
    """Get the existing XRDeviceIntegration instance from the extension."""
    try:
        import omni.ext
        from .extension import Extension
        ext_instance = Extension.get_instance()
        if ext_instance and hasattr(ext_instance, 'xr_integration'):
            return ext_instance.xr_integration
        else:
            carb.log_warn("Extension not loaded, creating new XRDeviceIntegration instance")
            return XRDeviceIntegration()
    except Exception as e:
        carb.log_warn(f"Failed to get extension instance: {e}, creating new XRDeviceIntegration")
        return XRDeviceIntegration()

class XRDeviceIntegration:
    def __init__(self):
        self.manus_tracker = IsaacSimManusGloveTracker()
        self.vive_tracker = IsaacSimViveTracker()
        self.last_update_time = 0
        self.update_rate = 60.0
        self.device_status = {
            'manus_gloves': {'connected': False, 'last_data_time': 0},
            'vive_trackers': {'connected': False, 'last_data_time': 0}
        }
        self.scene_T_lighthouse_static = None  
        # Calibration buffering for robust initialization
        self._calib_candidates = []  # List[Gf.Matrix4d]
        self._calib_start_time = None
        self._calib_required_samples = 15
        self._calib_max_seconds = 1.0
        self._calib_max_trans_error_m = 0.03
        self._calib_max_rot_error_deg = 7.5
        # Mode (cluster) selection thresholds for robust calibration
        self._mode_trans_threshold_m = 0.02
        self._mode_rot_threshold_deg = 5.0
        
    def register_devices(self):
        try:
            if self.manus_tracker.is_connected:
                carb.log_info("Manus gloves registered successfully")
                self.device_status['manus_gloves']['connected'] = True
            else:
                carb.log_warn("Failed to initialize Manus gloves")
            
            if self.vive_tracker.is_connected:
                carb.log_info("Vive trackers registered successfully")
                self.device_status['vive_trackers']['connected'] = True
            else:
                carb.log_warn("Failed to initialize Vive trackers")
                
        except Exception as e:
            carb.log_error(f"Failed to register XR devices: {e}")
    
    def update_manus(self):
        """Update raw Manus glove data. """
        if self.scene_T_lighthouse_static is None:
            # carb.log_warn("Manus glove data update skipped: scene_T_lighthouse_static is not initialized")
            return
        current_time = time.time()
        self.manus_tracker.update()
        self.device_status['manus_gloves']['last_data_time'] = current_time
    
    def update_vive(self):
        """Update raw Vive tracker data, and initialize coordinate transformation if it is the first data update. """
        current_time = time.time()
        self.vive_tracker.update()
        self.device_status['vive_trackers']['last_data_time'] = current_time
        try:
            # Initialize coordinate transformation from first Vive wrist position
            if self.scene_T_lighthouse_static is None:
                self._initialize_coordinate_transformation()
        except Exception as e:
            carb.log_error(f"Vive tracker update failed: {e}")

    def update_all_devices(self):
        current_time = time.time()
        
        if current_time - self.last_update_time >= 1.0 / self.update_rate:
            try:
                self.update_manus()
                self.update_vive()
                self.last_update_time = current_time
                
            except Exception as e:
                carb.log_error(f"Failed to update XR devices: {e}")

    def pose_to_matrix(self, pose: Dict) -> Gf.Matrix4d:
        """Convert position and orientation to a 4x4 matrix."""
        pos, ori = pose['position'], pose['orientation']
        quat = Gf.Quatd(ori[0], Gf.Vec3d(ori[1], ori[2], ori[3]))
        rot = Gf.Matrix3d().SetRotate(quat)
        trans = Gf.Vec3d(pos[0], pos[1], pos[2])
        return Gf.Matrix4d(rot, trans)
    
    def matrix_to_pose(self, matrix: Gf.Matrix4d) -> Dict:
        """Convert a 4x4 matrix to position and orientation."""
        pos = matrix.ExtractTranslation()
        rot = matrix.ExtractRotation()
        quat = rot.GetQuat()
        return {
            'position': [pos[0], pos[1], pos[2]],
            'orientation': [quat.GetReal(), quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2]]
        }
    
    def _get_openxr_left_wrist_matrix(self) -> Gf.Matrix4d:
        """Get OpenXR left wrist world matrix if valid, else None."""
        try:
            left_hand_device = XRCore.get_singleton().get_input_device("/user/hand/left")
            if left_hand_device is None:
                return None
            left_joints = left_hand_device.get_all_virtual_world_poses()
            if 'wrist' not in left_joints:
                return None
            joint = left_joints['wrist']
            required = XRPoseValidityFlags.POSITION_VALID | XRPoseValidityFlags.ORIENTATION_VALID
            if (joint.validity_flags & required) != required:
                return None
            return joint.pose_matrix
        except Exception as e:
            carb.log_warn(f"OpenXR wrist fetch failed: {e}")
            return None

    def _compute_delta_errors(self, a: Gf.Matrix4d, b: Gf.Matrix4d) -> (float, float):
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

    def _average_transforms(self, mats: list) -> Gf.Matrix4d:
        """Average a list of rigid transforms (simple mean of translation and normalized mean of quaternions)."""
        if not mats:
            return None
        # Use the first as reference for quaternion sign
        ref_quat = mats[0].ExtractRotation().GetQuat()
        ref = np.array([ref_quat.GetReal(), *ref_quat.GetImaginary()])
        acc_q = np.zeros(4, dtype=np.float64)
        acc_t = np.zeros(3, dtype=np.float64)
        for m in mats:
            t = m.ExtractTranslation()
            acc_t += np.array([t[0], t[1], t[2]], dtype=np.float64)
            q = m.ExtractRotation().GetQuat()
            qi = np.array([q.GetReal(), *q.GetImaginary()], dtype=np.float64)
            # Align sign to reference to avoid cancellation
            if np.dot(qi, ref) < 0.0:
                qi = -qi
            acc_q += qi
        mean_t = acc_t / float(len(mats))
        # Normalize quaternion
        norm = np.linalg.norm(acc_q)
        if norm <= 1e-12:
            # Fallback to identity rotation
            quat_avg = Gf.Quatd(1.0, Gf.Vec3d(0.0, 0.0, 0.0))
        else:
            qn = acc_q / norm
            quat_avg = Gf.Quatd(float(qn[0]), Gf.Vec3d(float(qn[1]), float(qn[2]), float(qn[3])))
        rot3 = Gf.Matrix3d().SetRotate(quat_avg)
        trans = Gf.Vec3d(float(mean_t[0]), float(mean_t[1]), float(mean_t[2]))
        return Gf.Matrix4d(rot3, trans)

    def _select_mode_cluster(self, mats: list) -> list:
        """Select the largest cluster (mode) of transforms under proximity thresholds."""
        if not mats:
            return []
        best_cluster = []
        for i, center in enumerate(mats):
            cluster = []
            for m in mats:
                trans_err, rot_err = self._compute_delta_errors(m, center)
                if trans_err <= self._mode_trans_threshold_m and rot_err <= self._mode_rot_threshold_deg:
                    cluster.append(m)
            if len(cluster) > len(best_cluster):
                best_cluster = cluster
        return best_cluster

    def _initialize_coordinate_transformation(self):
        """
        Initialize the scene to lighthouse coordinate transformation.
        The coordinate transformation is used to transform the wrist pose from lighthouse coordinate system to isaac sim scene coordinate.
        It is computed from multiple frames of AVP/OpenXR wrist pose and Vive wrist pose samples at the beginning of the session. 
        """
        vive_data = self.vive_tracker.get_all_tracker_data()
        left_wrist = self._get_init_ref_wrist_id(vive_data)
        if left_wrist is None:
            return

        # Start/continue calibration window
        if self._calib_start_time is None:
            self._calib_start_time = time.time()
            self._calib_candidates = []

        try:
            lighthouse_T_wrist = self.pose_to_matrix(vive_data[left_wrist])
            openxr_scene_T_wrist = self._get_openxr_left_wrist_matrix()
            if openxr_scene_T_wrist is None:
                # No valid AVP wrist this frame
                pass
            else:
                # Candidate scene_T_lighthouse from this frame
                wrist_T_lighthouse = lighthouse_T_wrist.GetInverse()
                candidate = wrist_T_lighthouse * openxr_scene_T_wrist
                # Validate by checking that lighthouse * candidate ~ openxr
                pred_scene_T_wrist = lighthouse_T_wrist * candidate
                trans_err, rot_err = self._compute_delta_errors(pred_scene_T_wrist, openxr_scene_T_wrist)
                if trans_err <= self._calib_max_trans_error_m and rot_err <= self._calib_max_rot_error_deg:
                    self._calib_candidates.append(candidate)
                else:
                    carb.log_warn(f"Calibration sample rejected: trans_err={trans_err:.3f} m, rot_err={rot_err:.1f} deg")

            # Decide to finalize
            enough_samples = len(self._calib_candidates) >= self._calib_required_samples
            timed_out = (time.time() - self._calib_start_time) >= self._calib_max_seconds
            if enough_samples or (timed_out and len(self._calib_candidates) >= max(3, self._calib_required_samples // 2)):
                cluster = self._select_mode_cluster(self._calib_candidates)
                if cluster and len(cluster) < len(self._calib_candidates):
                    carb.log_warn(f"Calibration mode selected: {len(cluster)}/{len(self._calib_candidates)} candidates")
                averaged = self._average_transforms(cluster if cluster else self._calib_candidates)
                if averaged is not None:
                    self.scene_T_lighthouse_static = averaged
                    carb.log_info(f"Calibrated scene_T_lighthouse with {len(cluster) if cluster else len(self._calib_candidates)} samples")
                else:
                    carb.log_warn("Calibration averaging failed; keeping transform uninitialized")
                # Reset calibration state
                self._calib_candidates = []
                self._calib_start_time = None
            elif timed_out and not self._calib_candidates:
                # No good samples; restart window
                self._calib_start_time = time.time()
        except Exception as e:
            carb.log_error(f"Failed to initialize coordinate transformation: {e}")
    
    def _get_init_ref_wrist_id(self, vive_data: Dict) -> str:
        """Get the wrist id for the initial reference wrist."""
        left_wrist = 'WM0'  # FIXME: hardcoded - assuming that WM0 is the left wrist tracker
        if left_wrist in vive_data:
            return left_wrist
        
        devices = []
        for device_key in vive_data.keys():
            devices.append(device_key)
            if len(device_key) >= 2 and device_key[:2] == 'WM':
                left_wrist = device_key
                return left_wrist
    
    def _transform_to_scene_coordinates(self, wrist_data: Dict) -> Dict:
        """Transform vive wrist coordinates to scene coordinates using scene_T_lighthouse_static."""
        if self.scene_T_lighthouse_static is None:
            return wrist_data
            
        try:
            transformed_matrix = self.pose_to_matrix(wrist_data) * self.scene_T_lighthouse_static
            # 90-degree ccw rotation on Y-axis and 90-degree ccw rotation on Z-axis
            # 10 cm offset on Y-axis for change in vive tracker position after flipping the hand
            Rcorr = Gf.Matrix4d(Gf.Matrix3d().SetRotate(Gf.Quatd(0.5, Gf.Vec3d(-0.5, 0.5, 0.5))), Gf.Vec3d(0, -0.1, 0))
            return self.matrix_to_pose(Rcorr * transformed_matrix)

        except Exception as e:
            carb.log_error(f"Failed to transform coordinates: {e}")
            return wrist_data
    
    def get_all_device_data(self) -> Dict:
        # Get raw data from trackers
        manus_data = self.manus_tracker.get_all_glove_data()
        vive_data = self.vive_tracker.get_all_tracker_data()
        vive_transformed = self._transform_vive_data(vive_data)
        scene_T_wrist = self._get_scene_T_wrist(vive_transformed)
        
        return {
            'manus_gloves': self._transform_manus_data(manus_data, scene_T_wrist),
            'vive_trackers': vive_transformed,
            'device_status': self.device_status
        }
    
    def _transform_vive_data(self, device_data: Dict) -> Dict:
        """Transform all joints in vive data to scene coordinates. """
        if self.scene_T_lighthouse_static is None:
            return device_data
        
        transformed_data = {}
        for joint_name, joint_data in device_data.items():
            transformed_data[joint_name] = self._transform_to_scene_coordinates(joint_data)
        
        return transformed_data

    def _get_scene_T_wrist(self, vive_data: Dict) -> Gf.Matrix4d:
        """Get the wrist position and orientation from vive data."""
        if 'WM0' in vive_data:
            return self.pose_to_matrix(vive_data['WM0']) # FIXME: only works for left hand
    
    def _transform_manus_data(self, manus_data: Dict, scene_T_wrist: Gf.Matrix4d) -> Dict:
        """Transform Manus glove data: wrist * wrist_T_joint."""
        if scene_T_wrist is None:
            return manus_data
        transformed_data = {}
        for joint_name, joint_data in manus_data.items():
            if joint_name.startswith('left_'):
                joint_transformed = self.pose_to_matrix(joint_data) * scene_T_wrist
                transformed_data[joint_name] = self.matrix_to_pose(joint_transformed)
        return transformed_data
    
    def cleanup(self):
        if hasattr(self, 'manus_tracker'):
            self.manus_tracker.cleanup()
        if hasattr(self, 'vive_tracker'):
            self.vive_tracker.cleanup() 