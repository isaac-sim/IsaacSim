# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import carb
import time
from typing import Dict
from pxr import Gf
from .manus_tracker import IsaacSimManusGloveTracker
from .vive_tracker import IsaacSimViveTracker
from .xr_utils import *


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
        self.device_status = {
            'manus_gloves': {'connected': False, 'last_data_time': 0},
            'vive_trackers': {'connected': False, 'last_data_time': 0},
            'left_hand_connected': False,
            'right_hand_connected': False
        }
        self.default_pose = {'position': [0, 0, 0],'orientation': [1, 0, 0, 0]}
        self.last_update_time = 0
        self.update_rate = 100.0
        # 90-degree ccw rotation on Y-axis and 90-degree ccw rotation on Z-axis
        self.rot_adjust = Gf.Matrix3d().SetRotate(Gf.Quatd(0.5, Gf.Vec3d(-0.5, 0.5, 0.5)))
        self.manus_update_frame = True
        self.scene_T_lighthouse_static = None  
        self._vive_left_id = None
        self._vive_right_id = None
        self._pairA_candidates = [] # Pair A: WM0->Left, WM1->Right
        self._pairB_candidates = [] # Pair B: WM1->Left, WM0->Right
        self._pairA_trans_errs = []
        self._pairA_rot_errs = []
        self._pairB_trans_errs = []
        self._pairB_rot_errs = []
        
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
    
    def get_all_device_data(self) -> Dict:
        """
        Get all device data. 
        Format: {
            'manus_gloves': {
                '{left/right}_{joint_index}': {
                    'position': [x, y, z],
                    'orientation': [w, x, y, z]
                },
                ...
            },
            'vive_trackers': {
                '{vive_tracker_id}': {
                    'position': [x, y, z],
                    'orientation': [w, x, y, z]
                },
                ...
            },
            'device_status': {
                'manus_gloves': {'connected': bool, 'last_data_time': float},
                'vive_trackers': {'connected': bool, 'last_data_time': float},
                'left_hand_connected': bool,
                'right_hand_connected': bool
            }
        }
        """
        if self.manus_update_frame:
            self.update_manus()
        else:
            self.update_vive()

        # Get raw data from trackers
        manus_data = self.manus_tracker.get_all_glove_data()
        vive_data = self.vive_tracker.get_all_tracker_data()
        vive_transformed = self._transform_vive_data(vive_data)
        scene_T_wrist = self._get_scene_T_wrist_matrix(vive_transformed)
        
        return {
            'manus_gloves': self._transform_manus_data(manus_data, scene_T_wrist),
            'vive_trackers': vive_transformed,
            'device_status': self.device_status
        }
    
    def cleanup(self):
        if hasattr(self, 'manus_tracker'):
            self.manus_tracker.cleanup()
        if hasattr(self, 'vive_tracker'):
            self.vive_tracker.cleanup()

    def update_manus(self):
        """Update raw Manus glove data. """
        current_time = time.time()
        if current_time - self.device_status['manus_gloves']['last_data_time'] < 1.0 / self.update_rate:
            return
        
        self.manus_tracker.update()
        self.device_status['manus_gloves']['last_data_time'] = current_time
        manus_data = self.manus_tracker.get_all_glove_data()
        self.device_status['left_hand_connected'] = 'left_0' in manus_data
        self.device_status['right_hand_connected'] = 'right_0' in manus_data
        self.manus_update_frame = False
    
    def update_vive(self):
        """Update raw Vive tracker data, and initialize coordinate transformation if it is the first data update. """
        current_time = time.time()
        if current_time - self.device_status['vive_trackers']['last_data_time'] < 1.0 / self.update_rate:
            return
        
        self.vive_tracker.update()
        self.device_status['vive_trackers']['last_data_time'] = current_time
        self.manus_update_frame = True
        try:
            # Initialize coordinate transformation from first Vive wrist position
            if self.scene_T_lighthouse_static is None:
                self._initialize_coordinate_transformation()
        except Exception as e:
            carb.log_error(f"Vive tracker update failed: {e}")

    def _initialize_coordinate_transformation(self):
        """
        Initialize the scene to lighthouse coordinate transformation.
        The coordinate transformation is used to transform the wrist pose from lighthouse coordinate system to isaac sim scene coordinate.
        It is computed from multiple frames of AVP/OpenXR wrist pose and Vive wrist pose samples at the beginning of the session. 
        """
        min_frames = 6
        tolerance = 3.0
        vive_data = self.vive_tracker.get_all_tracker_data()
        wm0_id, wm1_id = get_vive_wrist_ids(vive_data)
        if wm0_id is None and wm1_id is None:
            return

        try:
            # Fetch OpenXR wrists
            L, R, gloves = None, None, []
            if self.device_status['left_hand_connected']:
                gloves.append('left')
                L = get_openxr_wrist_matrix('left')
            if self.device_status['right_hand_connected']:
                gloves.append('right')
                R = get_openxr_wrist_matrix('right')
            
            M0, M1, vives = None, None, []
            if wm0_id is not None:
                vives.append(wm0_id)
                M0 = pose_to_matrix(vive_data[wm0_id])
            if wm1_id is not None:
                vives.append(wm1_id)
                M1 = pose_to_matrix(vive_data[wm1_id])

            TL0, TL1, TR0, TR1 = None, None, None, None
            # Compute transforms for available pairs
            if wm0_id is not None and L is not None:
                TL0 = M0.GetInverse() * L
                self._pairA_candidates.append(TL0)
            if wm1_id is not None and L is not None:
                TL1 = M1.GetInverse() * L
                self._pairB_candidates.append(TL1)
            if wm1_id is not None and R is not None:
                TR1 = M1.GetInverse() * R
                self._pairA_candidates.append(TR1)
            if wm0_id is not None and R is not None:
                TR0 = M0.GetInverse() * R
                self._pairB_candidates.append(TR0)

            # Per-frame pairing error if both candidates present
            if TL0 is not None and TR1 is not None and TL1 is not None and TR0 is not None:
                eT, eR = compute_delta_errors(TL0, TR1)
                self._pairA_trans_errs.append(eT)
                self._pairA_rot_errs.append(eR)
                eT, eR = compute_delta_errors(TL1, TR0)
                self._pairB_trans_errs.append(eT)
                self._pairB_rot_errs.append(eR)
            # Choose a mapping
            choose_A = None
            if len(self._pairA_candidates) == 0 and len(self._pairB_candidates) >= min_frames:
                choose_A = False
            elif len(self._pairB_candidates) == 0 and len(self._pairA_candidates) >= min_frames:
                choose_A = True
            elif len(self._pairA_trans_errs) >= min_frames and len(self._pairB_trans_errs) >= min_frames:
                errA = get_pairing_error(self._pairA_trans_errs, self._pairA_rot_errs)
                errB = get_pairing_error(self._pairB_trans_errs, self._pairB_rot_errs)
                carb.log_error(f"Wrist calibration: errA {errA}, errB {errB}")
                if errA <= errB and errA < tolerance:
                    choose_A = True
                elif errB <= errA and errB < tolerance:
                    choose_A = False
            if choose_A is None:
                return

            if choose_A:
                chosen_list = self._pairA_candidates
                self._vive_left_id, self._vive_right_id = wm0_id, wm1_id
            else:
                chosen_list = self._pairB_candidates
                self._vive_left_id, self._vive_right_id = wm1_id, wm0_id
            
            if len(chosen_list) >= min_frames:
                cluster = select_mode_cluster(chosen_list)
                carb.log_error(f"Wrist calibration: formed size {len(cluster)} cluster from {len(chosen_list)} samples")
                if len(cluster) >= min_frames // 2:
                    averaged = average_transforms(cluster)
                    self.scene_T_lighthouse_static = averaged
                    carb.log_error(f"Resolved mapping: {self._vive_left_id}->Left, {self._vive_right_id}->Right")

        except Exception as e:
            carb.log_error(f"Failed to initialize coordinate transformation: {e}")
    
    def _transform_vive_data(self, device_data: Dict) -> Dict:
        """Transform all joints in vive data to scene coordinates. """
        # 10 cm offset on Y-axis for change in vive tracker position after flipping the palm
        Rcorr = Gf.Matrix4d(self.rot_adjust, Gf.Vec3d(0, -0.1, 0))
        transformed_data = {}
        for joint_name, joint_data in device_data.items():
            transformed_pose = self.default_pose
            if self.scene_T_lighthouse_static is not None:
                transformed_matrix = pose_to_matrix(joint_data) * self.scene_T_lighthouse_static
                transformed_pose = matrix_to_pose(Rcorr * transformed_matrix)
            transformed_data[joint_name] = transformed_pose
        return transformed_data

    def _get_scene_T_wrist_matrix(self, vive_data: Dict) -> Dict:
        """Get the wrist position and orientation from vive data."""
        scene_T_wrist = {'left': Gf.Matrix4d().SetIdentity(), 'right': Gf.Matrix4d().SetIdentity()}
        if self._vive_left_id is not None:
            scene_T_wrist['left'] = pose_to_matrix(vive_data[self._vive_left_id])
        if self._vive_right_id is not None:
            scene_T_wrist['right'] = pose_to_matrix(vive_data[self._vive_right_id])
        return scene_T_wrist
    
    def _transform_manus_data(self, manus_data: Dict, scene_T_wrist: Dict) -> Dict:
        """Transform Manus glove data: relative joint poses to wrist -> joint in scene coordinates"""
        Rcorr = Gf.Matrix4d(self.rot_adjust, Gf.Vec3d(0, 0, 0)).GetInverse()
        transformed_data = {}
        for joint_name, joint_data in manus_data.items():
            hand, _ = joint_name.split('_')
            joint_mat = Rcorr * pose_to_matrix(joint_data) * scene_T_wrist[hand]
            transformed_data[joint_name] = matrix_to_pose(joint_mat)
        # Calculate palm with middle metacarpal and proximal
        transformed_data['left_25'] = get_palm(transformed_data, 'left')
        transformed_data['right_25'] = get_palm(transformed_data, 'right')
        return transformed_data
