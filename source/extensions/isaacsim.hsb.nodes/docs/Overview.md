# Holoscan Sensor Bridge Nodes

The `isaacsim.hsb.nodes` extension provides OmniGraph nodes for the Holoscan Sensor Bridge (HSB):

- **HSBSend**: Sends data buffers via HSB using DLTensor format (supports GPU data directly).
- **RGBToVB1940**: GPU-accelerated conversion from RGB(A) to VB1940 CSI RAW10 frame format (both Linux RoCEv2 4p5b and COE AGX Thor 3p4b variants).
- **HSBCameraHelper**: Python helper node that configures Replicator writers for HSB camera publishing.

This extension depends on `isaacsim.hsb.core` for the backend library, and is loaded by the `isaacsim.hsb.bridge` umbrella extension.
