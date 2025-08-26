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
        self.last_update_time = 0
        self.update_rate = 100.0
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
    
    def update_manus(self):
        """Update raw Manus glove data. """
        current_time = time.time()
        if current_time - self.device_status['manus_gloves']['last_data_time'] < 1.0 / self.update_rate:
            return
        
        self.manus_tracker.update()
        self.device_status['manus_gloves']['last_data_time'] = current_time
        manus_data = self.manus_tracker.get_all_glove_data()
        self.device_status['left_hand_connected'] = 'left_joint_0' in manus_data
        self.device_status['right_hand_connected'] = 'right_joint_0' in manus_data
    
    def update_vive(self):
        """Update raw Vive tracker data, and initialize coordinate transformation if it is the first data update. """
        current_time = time.time()
        if current_time - self.device_status['vive_trackers']['last_data_time'] < 1.0 / self.update_rate:
            return
        
        self.vive_tracker.update()
        self.device_status['vive_trackers']['last_data_time'] = current_time
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
        params = {
            'mapping_margin_trans_m': 0.015,
            'mapping_margin_rot_deg': 3.0,
            'mapping_min_pair_frames': 6
        }
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
            if len(gloves) == 2 and len(vives) == 2:
                eT, eR = compute_delta_errors(TL0, TR1)
                self._pairA_trans_errs.append(eT)
                self._pairA_rot_errs.append(eR)
                eT, eR = compute_delta_errors(TL1, TR0)
                self._pairB_trans_errs.append(eT)
                self._pairB_rot_errs.append(eR)

            # Choose a mapping
            enough_pair_frames = (len(self._pairA_trans_errs) >= params['mapping_min_pair_frames'] and
                                  len(self._pairB_trans_errs) >= params['mapping_min_pair_frames'])

            choose_A = len(self._pairA_candidates) > len(self._pairB_candidates)
            if enough_pair_frames:
                errA = get_pairing_error(self._pairA_trans_errs, self._pairA_rot_errs)
                errB = get_pairing_error(self._pairB_trans_errs, self._pairB_rot_errs)
                choose_A = errA <= errB

            if choose_A:
                chosen_list = self._pairA_candidates
                self._vive_left_id, self._vive_right_id = wm0_id, wm1_id
            else:
                chosen_list = self._pairB_candidates
                self._vive_left_id, self._vive_right_id = wm1_id, wm0_id
            
            if len(chosen_list) >= params['mapping_min_pair_frames']:
                cluster = select_mode_cluster(chosen_list)
                carb.log_error(f"Wrist calibration: formed size {len(cluster)} cluster from {len(chosen_list)} samples")
                if len(cluster) >= params['mapping_min_pair_frames'] // 2:
                    averaged = average_transforms(cluster)
                    self.scene_T_lighthouse_static = averaged
                    carb.log_error(f"Resolved mapping: {self._vive_left_id}->Left, {self._vive_right_id}->Right")
                else:
                    carb.log_error(f"Waiting for more samples to average wrist transforms")

        except Exception as e:
            carb.log_error(f"Failed to initialize coordinate transformation: {e}")
    

    def _transform_to_scene_coordinates(self, wrist_data: Dict) -> Dict:
        """Transform vive wrist coordinates to scene coordinates using scene_T_lighthouse_static."""
        if self.scene_T_lighthouse_static is None:
            return wrist_data
            
        try:
            transformed_matrix = pose_to_matrix(wrist_data) * self.scene_T_lighthouse_static
            # 90-degree ccw rotation on Y-axis and 90-degree ccw rotation on Z-axis
            # 10 cm offset on Y-axis for change in vive tracker position after flipping the palm
            Rcorr = Gf.Matrix4d(Gf.Matrix3d().SetRotate(Gf.Quatd(0.5, Gf.Vec3d(-0.5, 0.5, 0.5))), Gf.Vec3d(0, -0.1, 0))
            return matrix_to_pose(Rcorr * transformed_matrix)

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

    def _get_scene_T_wrist(self, vive_data: Dict) -> Dict:
        """Get the wrist position and orientation from vive data."""
        scene_T_wrist = {'left': Gf.Matrix4d().SetIdentity(), 'right': Gf.Matrix4d().SetIdentity()}
        if self._vive_left_id is not None:
            scene_T_wrist['left'] = pose_to_matrix(vive_data[self._vive_left_id])
        if self._vive_right_id is not None:
            scene_T_wrist['right'] = pose_to_matrix(vive_data[self._vive_right_id])
        return scene_T_wrist
    
    def _transform_manus_data(self, manus_data: Dict, scene_T_wrist: Dict) -> Dict:
        """Transform Manus glove data: wrist * wrist_T_joint."""
        transformed_data = {}
        for joint_name, joint_data in manus_data.items():
            if joint_name.startswith('left_'):
                joint_transformed = pose_to_matrix(joint_data) * scene_T_wrist['left']
                transformed_data[joint_name] = matrix_to_pose(joint_transformed)
            elif joint_name.startswith('right_'):
                joint_transformed = pose_to_matrix(joint_data) * scene_T_wrist['right']
                transformed_data[joint_name] = matrix_to_pose(joint_transformed)
        return transformed_data
    
    def cleanup(self):
        if hasattr(self, 'manus_tracker'):
            self.manus_tracker.cleanup()
        if hasattr(self, 'vive_tracker'):
            self.vive_tracker.cleanup() 