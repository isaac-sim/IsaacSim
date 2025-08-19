# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import carb
import numpy as np
from typing import Dict, List, Optional
from pxr import Gf
from omni.kit.xr.core import XRCore

try:
    from isaacsim.xr.input_devices._isaac_xr_input_devices import IsaacSimManusTracker
    _manus_tracker_available = True
except ImportError:
    carb.log_warn("IsaacSimManusTracker not available - using mock data for manus")
    _manus_tracker_available = False

hmd_pose = ((0.996499240398407, -0.06362161040306091, 0.054237786680459976, 0),
            (-0.06642970442771912, -0.20866860449314117, 0.9757277965545654, 0), 
            (-0.05075964331626892, -0.9759148359298706, -0.21216443181037903, 0), 
            (0.000521781446877867, 0.19946864247322083, 1.4816499948501587, 1))

class IsaacSimManusGloveTracker:
    def __init__(self):
        self.glove_data = {}
        self.is_connected = False
        
        if _manus_tracker_available:
            try:
                self._manus_tracker = IsaacSimManusTracker()
                success = self._manus_tracker.initialize()
                if success:
                    self.is_connected = True
                    carb.log_info("Manus glove tracker initialized with SDK")
                else:
                    carb.log_warn("Failed to initialize Manus SDK - using mock data")
                    self._manus_tracker = None
            except Exception as e:
                carb.log_warn(f"Failed to initialize Manus tracker: {e} - using mock data")
                self._manus_tracker = None
        else:
            self._manus_tracker = None
            carb.log_info("Manus glove tracker initialized (mock)")
        
    def update(self):
        if self._manus_tracker and self.is_connected:
            try:
                raw_data = self._manus_tracker.get_glove_data()
                for hand in ['left', 'right']:
                    self._populate_glove_data(raw_data, hand)
                
            except Exception as e:
                carb.log_error(f"Failed to update Manus glove data: {e}")
        else:
            # Provide mock data
            self.glove_data = {
                'left_joint_0': {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0])
                },
                'right_joint_0': {
                    'position': np.array([0.1, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0])
                }
            }
    
    def _populate_glove_data(self, raw_data: Dict, hand: str) -> None:
        """
        Convert raw Manus data to Isaac Sim format with individual joint entries.
        Adds to self.glove_data: {joint_name: {position: pos, orientation: ori}}
        """
        if f'{hand}_position' not in raw_data or f'{hand}_orientation' not in raw_data:
            # carb.log_error(f"No {hand} position or orientation data found")
            return
        
        position = raw_data[f'{hand}_position']
        orientation = raw_data[f'{hand}_orientation']
        joint_count = len(position) // 3  # 3 values per position
            
        for joint_idx in range(joint_count):
            # Extract joint position (x, y, z)
            pos_start = joint_idx * 3
            joint_pos = np.array([float(position[pos_start]), float(position[pos_start + 1]), float(position[pos_start + 2])])
            
            # Extract joint orientation (w, x, y, z)
            ori_start = joint_idx * 4
            joint_ori = np.array([float(orientation[ori_start]), float(orientation[ori_start + 1]), 
                        float(orientation[ori_start + 2]), float(orientation[ori_start + 3])])
            
            # Transform coordinates
            transformed_pos, transformed_ori = self._transform_manus_coordinates(joint_pos, joint_ori)
            joint_name = f"{hand}_joint_{joint_idx}"
             
            self.glove_data[joint_name] = {
                'position': transformed_pos,
                'orientation': transformed_ori
            }
    
    def _transform_manus_coordinates(self, manus_pos: np.ndarray, manus_ori: np.ndarray) -> tuple:
        """
        Transform Manus glove coordinates to Isaac Sim coordinate system.
        Uses HMD pose as reference to align with stage coordinate system.
        """
        # Create transformation matrix from Manus pose
        manus_quat = Gf.Quatd(manus_ori[0], Gf.Vec3d(manus_ori[1], manus_ori[2], manus_ori[3]))
        manus_rot = Gf.Matrix3d().SetRotate(manus_quat)
        manus_trans = Gf.Vec3d(manus_pos[0], manus_pos[1], manus_pos[2])
        manus_matrix = Gf.Matrix4d(manus_rot, manus_trans)
        
        # Apply coordinate system transformation: Z-up (Manus) to Y-up (Isaac Sim)
        coord_transform = Gf.Matrix4d().SetRotate(Gf.Quatd(0.7071068, Gf.Vec3d(0.7071068, 0.0, 0.0)))  # 90° around X
        
        # Apply coordinate system transformation: Z-up (Manus) to Y-up (Isaac Sim)
        transformed_matrix = manus_matrix * coord_transform
        
        # Use HMD pose to align with the same coordinate system as OpenXR hand tracking
        # The HMD pose represents where the stage coordinate system is relative to our devices
        hmd_transform = Gf.Matrix4d(hmd_pose)
        
        # Apply the same transformation that OpenXR.cpp applies internally
        # This aligns our external devices with the stage coordinate system
        transformed_matrix = transformed_matrix * hmd_transform
        
        # Extract transformed position and orientation
        transformed_pos = transformed_matrix.ExtractTranslation()
        transformed_rot = transformed_matrix.ExtractRotation()
        transformed_quat = transformed_rot.GetQuat()
        
        return (
            np.array([transformed_pos[0], transformed_pos[1], transformed_pos[2]]),
            np.array([transformed_quat.GetReal(), transformed_quat.GetImaginary()[0], 
                     transformed_quat.GetImaginary()[1], transformed_quat.GetImaginary()[2]])
        )
    
    def get_all_glove_data(self) -> Dict:
        return self.glove_data
    
    def cleanup(self):
        try:
            if self._manus_tracker:
                self._manus_tracker.cleanup()
            self.is_connected = False
        except Exception as e:
            carb.log_error(f"Error during Manus tracker cleanup: {e}") 