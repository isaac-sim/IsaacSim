# Overview

This extension provides specialized writers for synthetic data generation workflows in Isaac Sim. The extension extends the core Replicator framework with Isaac Sim-specific writers that handle various machine learning training data formats including pose estimation, DOPE (Detection of Pose Estimation), YCB Video dataset, and PyTorch tensors.

## Key Components

### Dataset Format Writers

**{class}`DOPEWriter <isaacsim.replicator.writers.DOPEWriter>`** generates training data compatible with the DOPE (Detection of Pose Estimation) methodology. This writer processes pose annotations and creates structured datasets for training neural networks to detect and estimate 3D object poses from RGB images.

**{class}`YCBVideoWriter <isaacsim.replicator.writers.YCBVideoWriter>`** formats synthetic data according to the YCB Video Dataset specification, a standard benchmark for 6D object pose estimation. The writer handles RGB images, semantic segmentation, depth data, and pose annotations while maintaining compatibility with the original dataset structure.

**{class}`PoseWriter <isaacsim.replicator.writers.PoseWriter>`** creates pose estimation datasets in multiple formats with configurable output options. This writer supports visibility thresholds, frame filtering, and debug visualizations to generate clean training datasets for pose estimation tasks.

### Visualization and Analysis

**{class}`DataVisualizationWriter <isaacsim.replicator.writers.DataVisualizationWriter>`** overlays annotation data onto rendered images for visual verification of synthetic data generation. The writer supports 2D tight bounding boxes, 2D loose bounding boxes, and 3D bounding box visualizations on RGB or normal backgrounds, enabling quick quality assessment of generated training data.

### PyTorch Integration

**{class}`PytorchWriter <isaacsim.replicator.writers.PytorchWriter>`** integrates directly with PyTorch workflows by converting rendered data (e.g. RGB) from multiple cameras into batched PyTorch tensors. This writer works in conjunction with **{class}`PytorchListener <isaacsim.replicator.writers.PytorchListener>`** to provide real-time tensor data streaming, allowing machine learning pipelines to consume synthetic data without intermediate file storage.

**{class}`PytorchListener <isaacsim.replicator.writers.PytorchListener>`** acts as an observer that receives batched tensor data from {class}`PytorchWriter <isaacsim.replicator.writers.PytorchWriter>`. The listener provides methods to directly retrieve data sent to the writer as PyTorch tensors without needing to access data stored by ``omni.replicator``'s ``BackendDispatch``, enabling seamless integration with training loops.

## Integration

The extension integrates with **omni.replicator.core** to extend the base Writer class functionality. Each writer registers specific annotators required for their data format and handles the processing pipeline from raw render products to formatted output files or tensor streams.

Writers support both local filesystem and S3 storage backends, with configurable output formats and frame processing options. The extension automatically registers all writers with the Replicator system at startup, making them available through the WriterRegistry interface.

## Pytorch Online Writer and Listener

The `PytorchWriter` and `PytorchListener` are APIs for using `omni.replicator`'s writer API to retrieve 
various data such as RGB from the specified cameras (supports multiple cameras) and provides them to 
the user in both default format (e.g.: PNG for RGB data) and batched pytorch tensors. The `PytorchListener` 
provides an API to directly retrieve data sent to the `PytorchWriter` without the need to access the stored 
by `omni.replicator`'s `BackendDispatch`.
