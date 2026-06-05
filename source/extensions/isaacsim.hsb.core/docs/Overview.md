# Holoscan Sensor Bridge Core

The `isaacsim.hsb.core` extension provides the backend library for the Holoscan Sensor Bridge (HSB), including:

- **HSBSender**: Manages HSB emulator lifecycle and sends DLTensor data via Linux (RoCEv2) or COE (IEEE 1722B) data planes.
- **RGBToVB1940 CUDA kernels**: GPU-accelerated conversion from RGB to VB1940 CSI RAW10 format.
- **Carbonite plugin**: Exposes the `IHsbCore` interface for Python bindings.

This extension is a dependency of `isaacsim.hsb.nodes` and `isaacsim.hsb.bridge`.
