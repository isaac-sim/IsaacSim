# Overview

This extension provides UI integration for creating camera and depth sensors in Isaac Sim. It adds menu items to the Create menu and context menus that enable users to create various camera and depth sensor prims from multiple vendors including Orbbec, Leopard Imaging, RealSense, Sensing, SICK, and Stereolabs.

## Functionality

The extension automatically registers sensor creation actions and organizes them into a hierarchical menu structure by vendor. Users can access sensor creation through two main pathways: the main Create menu under "Sensors > Camera and Depth Sensors" and context menus accessible via right-click in the viewport under "Isaac > Sensors".

**Supported camera sensors include:**
- Leopard Imaging: Hawk, Owl
- Sensing: Multiple SG series models with various configurations 
- SICK: Inspector83x

**Supported depth sensors include:**
- Orbbec: Gemini 2, FemtoMega, Gemini 335, Gemini 335L
- RealSense: D455, D457, D555
- Stereolabs: ZED_X

## Key Components

### Sensor Creation Actions

The extension creates specialized actions for each supported sensor type using the action registry. For depth sensors, it generates SingleViewDepthSensorAsset instances with proper initialization, while regular camera sensors create standard Xform prims with appropriate USD references.

### Menu Integration

Menu items are dynamically created and organized by vendor, providing a structured approach to sensor selection. The hierarchical organization helps users locate specific sensor models efficiently within the Isaac Sim interface.

## Integration

The extension uses **omni.kit.actions.core** to register sensor creation actions and **omni.kit.context_menu** to provide right-click access to sensor creation tools. It currently integrates with the deprecated `isaacsim.sensors.camera` for the underlying sensor implementation (pending migration to `isaacsim.sensors.experimental.rtx`) and `isaacsim.gui.components` for UI component support.
