# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate pre-ISP camera image processing pipeline."""

import argparse
import os

parser = argparse.ArgumentParser(description="Expose Omniverse pre-ISP camera image processing pipeline.")
parser.add_argument("--draw-output", action="store_true", help="Convert binary pipeline outputs to images and save.")
parser.add_argument(
    "--output-dir",
    type=str,
    default=os.path.join(os.getcwd(), "_example_output_isaacsim.sensors.camera", "camera_pre_isp_pipeline"),
    help="Output directory for pipeline outputs.",
)
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode.")
args, unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.graph.core as og
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Cube, DomeLight
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.sensors.camera import Camera
from omni.syntheticdata import SyntheticData, SyntheticDataStage

# Enable omni.sensors.nv.camera extension
enable_extension("omni.sensors.nv.camera")

# Create the output directory
output_dir = args.output_dir
os.makedirs(output_dir, exist_ok=True)

# Set up environment (light and cubes)
dome_light = DomeLight("/World/DomeLight")
dome_light.set_intensities(1000)

cube_left = Cube("/World/Cube_left", sizes=1.0, positions=np.array([-1.5, 0.0, 0.5]))
cube_left_material = OmniPbrMaterial("/World/Materials/cube_left")
cube_left_material.set_input_values("diffuse_color_constant", [1.0, 0.0, 0.0])
cube_left.apply_visual_materials(cube_left_material)

cube_right = Cube("/World/Cube_right", sizes=1.0, positions=np.array([1.5, 0.0, 0.5]))
cube_right_material = OmniPbrMaterial("/World/Materials/cube_right")
cube_right_material.set_input_values("diffuse_color_constant", [0.0, 1.0, 0.0])
cube_right.apply_visual_materials(cube_right_material)

# Create camera and attach render product
width, height = 640, 480
camera_position = np.array([0.0, 0.0, 4.5])
camera_rotation_as_euler = np.array([0, 90, 90])
camera = Camera(
    prim_path="/World/Camera",
    position=camera_position,
    resolution=(width, height),
    orientation=euler_angles_to_quaternion(camera_rotation_as_euler, degrees=True, extrinsic=False).numpy(),
)
camera.set_focal_length(1.5)
camera.initialize()
render_product_path = camera.get_render_product_path()

# Enable HDR color rendervar for render product attached to camera prim
SyntheticData.Get().enable_rendervar(render_product_path=render_product_path, render_var="HdrColor")

# Directly set post-render graph for render product attached to camera prim. Post-render graph is comprised of nodes
# from omni.sensors.nv.camera extension, enabling output at each stage of the pre-ISP pipeline.
wrapper_node = SyntheticData.Get().get_graph(
    stage=SyntheticDataStage.POST_RENDER, renderProductPath=render_product_path
)
wrapped_graph = wrapper_node.get_wrapped_graph()

# Create graph for pre-ISP pipeline
keys = og.Controller.Keys
_, nodes, _, _ = og.Controller.edit(
    wrapped_graph,
    {
        keys.CREATE_NODES: [
            ("CudaEntry", "omni.graph.nodes.GpuInteropRenderProductEntry"),
            ("Tex", "omni.sensors.nv.camera.CamTextureReadTaskNode"),
            ("CC", "omni.sensors.nv.camera.ColorCorrectionTaskNode"),
            ("CFA", "omni.sensors.nv.camera.CamCfa2x2EncoderTaskNode"),
            ("noise", "omni.sensors.nv.camera.CamGeneralPurposeNoiseTask"),
            ("com", "omni.sensors.nv.camera.CamCompandingTaskNode"),
            ("dec", "omni.sensors.nv.camera.CamIspDecompandingTaskNode"),
            ("dmz", "omni.sensors.nv.camera.CamIspRGGBDemosaicingTaskNode"),
            ("rgba", "omni.sensors.nv.camera.CamRGBADatatypeConverterTaskNode"),
            ("Enc", "omni.sensors.nv.camera.CamRGBEncoderTaskNode"),
            ("writeHdr", "omni.sensors.nv.camera.CamFileWriterTaskNode"),
            ("writeRaw", "omni.sensors.nv.camera.CamFileWriterTaskNode"),
            ("writeIsp", "omni.sensors.nv.camera.CamFileWriterTaskNode"),
        ],
        keys.CONNECT: [
            ("CudaEntry.outputs:gpu", "Tex.inputs:gpu"),
            ("CudaEntry.outputs:rp", "Tex.inputs:rp"),
            ("CudaEntry.outputs:simTime", "Tex.inputs:simTime"),
            ("CudaEntry.outputs:hydraTime", "Tex.inputs:hydraTime"),
            ("Tex.outputs:gpu", "CC.inputs:gpu"),
            ("Tex.outputs:rp", "CC.inputs:rp"),
            ("Tex.outputs:simTimeOut", "CC.inputs:simTimeIn"),
            ("Tex.outputs:hydraTimeOut", "CC.inputs:hydraTimeIn"),
            ("Tex.outputs:dest", "CC.inputs:src"),
            ("CC.outputs:gpu", "CFA.inputs:gpu"),
            ("CC.outputs:rp", "CFA.inputs:rp"),
            ("CC.outputs:simTimeOut", "CFA.inputs:simTimeIn"),
            ("CC.outputs:hydraTimeOut", "CFA.inputs:hydraTimeIn"),
            ("CC.outputs:dest", "CFA.inputs:src"),
            ("CFA.outputs:gpu", "noise.inputs:gpu"),
            ("CFA.outputs:rp", "noise.inputs:rp"),
            ("CFA.outputs:simTimeOut", "noise.inputs:simTimeIn"),
            ("CFA.outputs:hydraTimeOut", "noise.inputs:hydraTimeIn"),
            ("CFA.outputs:dest", "noise.inputs:src"),
            ("noise.outputs:gpu", "com.inputs:gpu"),
            ("noise.outputs:rp", "com.inputs:rp"),
            ("noise.outputs:simTimeOut", "com.inputs:simTimeIn"),
            ("noise.outputs:hydraTimeOut", "com.inputs:hydraTimeIn"),
            ("noise.outputs:dest", "com.inputs:src"),
            ("com.outputs:gpu", "dec.inputs:gpu"),
            ("com.outputs:rp", "dec.inputs:rp"),
            ("com.outputs:simTimeOut", "dec.inputs:simTimeIn"),
            ("com.outputs:hydraTimeOut", "dec.inputs:hydraTimeIn"),
            ("com.outputs:dest", "dec.inputs:src"),
            ("dec.outputs:gpu", "dmz.inputs:gpu"),
            ("dec.outputs:rp", "dmz.inputs:rp"),
            ("dec.outputs:simTimeOut", "dmz.inputs:simTimeIn"),
            ("dec.outputs:hydraTimeOut", "dmz.inputs:hydraTimeIn"),
            ("dec.outputs:dest", "dmz.inputs:src"),
            ("dmz.outputs:gpu", "rgba.inputs:gpu"),
            ("dmz.outputs:rp", "rgba.inputs:rp"),
            ("dmz.outputs:simTimeOut", "rgba.inputs:simTimeIn"),
            ("dmz.outputs:hydraTimeOut", "rgba.inputs:hydraTimeIn"),
            ("dmz.outputs:dest", "rgba.inputs:src"),
            ("rgba.outputs:gpu", "Enc.inputs:gpu"),
            ("rgba.outputs:rp", "Enc.inputs:rp"),
            ("rgba.outputs:simTimeOut", "Enc.inputs:simTimeIn"),
            ("rgba.outputs:hydraTimeOut", "Enc.inputs:hydraTimeIn"),
            ("rgba.outputs:dest", "Enc.inputs:src"),
            ("Tex.outputs:gpu", "writeHdr.inputs:gpu"),
            ("Tex.outputs:rp", "writeHdr.inputs:rp"),
            ("Tex.outputs:simTimeOut", "writeHdr.inputs:simTimeIn"),
            ("Tex.outputs:hydraTimeOut", "writeHdr.inputs:hydraTimeIn"),
            ("Tex.outputs:dest", "writeHdr.inputs:src"),
            ("com.outputs:gpu", "writeRaw.inputs:gpu"),
            ("com.outputs:rp", "writeRaw.inputs:rp"),
            ("com.outputs:simTimeOut", "writeRaw.inputs:simTimeIn"),
            ("com.outputs:hydraTimeOut", "writeRaw.inputs:hydraTimeIn"),
            ("com.outputs:dest", "writeRaw.inputs:src"),
            ("Enc.outputs:gpu", "writeIsp.inputs:gpu"),
            ("Enc.outputs:rp", "writeIsp.inputs:rp"),
            ("Enc.outputs:simTimeOut", "writeIsp.inputs:simTimeIn"),
            ("Enc.outputs:hydraTimeOut", "writeIsp.inputs:hydraTimeIn"),
            ("Enc.outputs:dest", "writeIsp.inputs:src"),
        ],
        keys.SET_VALUES: [
            ("Tex.inputs:aov", "HDR"),
            ("CC.inputs:output_float16", True),
            ("CC.inputs:Rr", 1.0),
            ("CC.inputs:Rg", 0.0),
            ("CC.inputs:Rb", 0.0),
            ("CC.inputs:Gr", 0.0),
            ("CC.inputs:Gg", 1.0),
            ("CC.inputs:Gb", 0.0),
            ("CC.inputs:Br", 0.0),
            ("CC.inputs:Bg", 0.0),
            ("CC.inputs:Bb", 1.0),
            ("CC.inputs:whiteBalance", [0.05, 0.05, 0.05]),
            ("CFA.inputs:CFA_CF00", [1, 0, 0]),
            ("CFA.inputs:CFA_CF01", [0, 1, 0]),
            ("CFA.inputs:CFA_CF10", [0, 1, 0]),
            ("CFA.inputs:CFA_CF11", [0, 0, 1]),
            ("CFA.inputs:cfaSemantic", "RGGB"),
            ("CFA.inputs:maximalValue", 16777215),
            ("CFA.inputs:flipHorizontal", 0),
            ("CFA.inputs:flipVertical", 0),
            ("noise.inputs:darkShotNoiseGain", 10.0),
            ("noise.inputs:darkShotNoiseSigma", 0.5),
            ("noise.inputs:hdrCombinationData", [(5.8, 4000), (58, 8000), (70, 16000)]),
            (
                "com.inputs:LinearCompandCoeff",
                [
                    [0, 0],
                    [244, 240],
                    [512, 430],
                    [768, 584],
                    [1024, 724],
                    [2048, 883],
                    [4096, 1150],
                    [8192, 1600],
                    [16384, 1768],
                    [32768, 2050],
                    [65536, 2354],
                    [131072, 2865],
                    [262144, 3195],
                    [524288, 3750],
                    [1048576, 3768],
                    [4194304, 3850],
                    [8388608, 3942],
                    [16777215, 4095],
                ],
            ),
            (
                "dec.inputs:LinearCompandCoeff",
                [
                    [0, 0],
                    [244, 240],
                    [512, 430],
                    [768, 584],
                    [1024, 724],
                    [2048, 883],
                    [4096, 1150],
                    [8192, 1600],
                    [16384, 1768],
                    [32768, 2050],
                    [65536, 2354],
                    [131072, 2865],
                    [262144, 3195],
                    [524288, 3750],
                    [1048576, 3768],
                    [4194304, 3850],
                    [8388608, 3942],
                    [16777215, 4095],
                ],
            ),
            ("dmz.inputs:bayerGrid", "RGGB"),
            ("dmz.inputs:outputFormat", "UINT16"),
            ("writeHdr.inputs:filename", os.path.join(output_dir, "1-hdr_input.rgba_f16..bin")),
            ("writeHdr.inputs:eachFrameOneFile", True),
            ("writeHdr.inputs:onlyLastFrame", True),
            ("writeRaw.inputs:filename", os.path.join(output_dir, "2-sensor_raw.r_u16..bin")),
            ("writeRaw.inputs:eachFrameOneFile", True),
            ("writeRaw.inputs:onlyLastFrame", True),
            ("writeIsp.inputs:filename", os.path.join(output_dir, "3-isp_output.rgb_u8..bin")),
            ("writeIsp.inputs:eachFrameOneFile", True),
            ("writeIsp.inputs:onlyLastFrame", True),
        ],
    },
)

if args.test:
    import omni.usd

    stage = omni.usd.get_context().get_stage()
    stage.Export(os.path.join(output_dir, "stage.usda"))

for _ in range(2):
    simulation_app.update()

if args.draw_output:
    import matplotlib.pyplot as plt
    import numpy as np

    # Read HDR float16 RGBA
    hdr_path = os.path.join(output_dir, "1-hdr_input.rgba_f16.0.bin")
    with open(hdr_path, "rb") as f:
        hdr_data = np.frombuffer(f.read(), dtype=np.float16)
    hdr_img = hdr_data.reshape((height, width, 4))

    # Read Raw Sensor output uint16 single channel
    raw_path = os.path.join(output_dir, "2-sensor_raw.r_u16.0.bin")
    with open(raw_path, "rb") as f:
        raw_data = np.frombuffer(f.read(), dtype=np.uint16)
    raw_img = raw_data.reshape((height, width))

    # Read ISP output uint8 RGB 3 channel
    isp_path = os.path.join(output_dir, "3-isp_output.rgb_u8.0.bin")
    with open(isp_path, "rb") as f:
        isp_data = np.frombuffer(f.read(), dtype=np.uint8)
    isp_img = isp_data.reshape((height, width, 3))

    hdr_vis = (hdr_img[:, :, :3] / np.max(hdr_img[:, :, :3]) * 255).astype(np.uint8)
    plt.imshow(hdr_vis)
    plt.title("HDR Input (normalized RGB)")
    plt.savefig(os.path.join(output_dir, "hdr_input.png"))

    plt.imshow(raw_img, cmap="gray")
    plt.title("Raw Sensor Output")
    plt.savefig(os.path.join(output_dir, "raw_sensor_output.png"))

    plt.imshow(isp_img)
    plt.title("ISP Output RGB")
    plt.savefig(os.path.join(output_dir, "isp_output.png"))


simulation_app.close()
