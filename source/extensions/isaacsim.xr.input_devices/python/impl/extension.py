# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import omni.ext
import carb
from .xr_device_integration import XRDeviceIntegration

class Extension(omni.ext.IExt):
    _instance = None
    
    def on_startup(self, ext_id):
        carb.log_info("IsaacSim XR Input Devices extension startup")
        self.xr_integration = XRDeviceIntegration()
        self._register_xr_devices()
        Extension._instance = self
        
    def on_shutdown(self):
        carb.log_info("IsaacSim XR Input Devices extension shutdown")
        if hasattr(self, 'xr_integration'):
            self.xr_integration.cleanup()
        Extension._instance = None
            
    def _register_xr_devices(self):
        self.xr_integration.register_devices() 
    
    @classmethod
    def get_instance(cls):
        return cls._instance
