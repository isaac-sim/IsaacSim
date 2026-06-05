# Overview

The omni.pip.compute extension bundles general-purpose compute packages including OpenCV (cv2) and other libraries used by Isaac Sim extensions for image processing and numerical computation. It is loaded early in the extension startup order to ensure these dependencies are available before other extensions initialize.

This extension is managed automatically and does not require manual configuration.
