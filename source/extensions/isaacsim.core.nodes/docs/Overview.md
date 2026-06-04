# Overview

The isaacsim.core.nodes extension provides OmniGraph nodes for Isaac Sim workflows including articulation control, simulation timing, camera info reading, viewport management, and physics step event handling. This extension serves as the computational backbone for synthetic data generation and robotics simulation workflows in Isaac Sim.

## Key Components

### {class}`BaseResetNode <isaacsim.core.nodes.BaseResetNode>`

The {class}`BaseResetNode <isaacsim.core.nodes.BaseResetNode>` provides automatic reset functionality when timeline playback stops. This base class ensures nodes return to their initial state when simulation stops, maintaining consistency across simulation runs.

```python
class CustomNode(BaseResetNode):
    def custom_reset(self):
        # Custom reset logic when timeline stops
        self.clear_data()
```

The node automatically subscribes to timeline events and calls the `custom_reset()` method when the stop event is triggered.

### {class}`BaseWriterNode <isaacsim.core.nodes.BaseWriterNode>`

The {class}`BaseWriterNode <isaacsim.core.nodes.BaseWriterNode>` extends {class}`BaseResetNode <isaacsim.core.nodes.BaseResetNode>` to manage replicator writers for synthetic data output. It provides functionality to attach and detach writers to render products, enabling automated data collection workflows.

Key capabilities include:
- Managing multiple writers simultaneously
- Automatic writer activation and deactivation
- Integration with render product pipelines
- Reset functionality that clears all active writers

```python
writer_node = BaseWriterNode()
writer_node.append_writer(my_writer)
writer_node.attach_writers(render_product_path)
```

### {class}`WriterRequest <isaacsim.core.nodes.WriterRequest>`

The {class}`WriterRequest <isaacsim.core.nodes.WriterRequest>` class encapsulates operations for managing writer attachment and detachment from render products. It provides a structured way to queue writer operations asynchronously.

Each request contains:
- The replicator writer to manage
- Target render product path(s)
- Activation state (attach or detach)

## Functionality

### Annotator Registration

The extension automatically registers various annotators for synthetic data processing:
- Time-based annotators for simulation and system time tracking
- Camera information readers for extracting camera parameters
- World pose readers for spatial data extraction
- Image conversion utilities (RGBA to RGB format)
- Depth-to-point-cloud converters
- Simulation gates for controlling data flow

### Data Processing Pipeline

The nodes enable comprehensive data extraction from rendered scenes, supporting robotics simulations with utilities for image format conversion, point cloud generation, and controlled data flow through simulation gates.

## C++ Plugin

The OmniGraph nodes in this extension are implemented in C++ as a Carbonite plugin (`isaacsim::core::nodes::CoreNodes`). The native plugin handles node registration via the OGN framework and provides performance-critical compute for image conversion, depth-to-point-cloud processing, simulation timing, and physics step event handling. Python bindings expose the `CoreNodes` interface through the `_isaacsim_core_nodes` module for plugin lifecycle management.

## Integration

The extension integrates with **omni.replicator.core** for synthetic data generation workflows and uses **omni.graph** as the computational framework. It connects to Isaac Sim's rendering and simulation systems through dedicated manager components for coordinated operation.
