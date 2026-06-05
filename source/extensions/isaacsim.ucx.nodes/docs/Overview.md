# Overview

The isaacsim.ucx.nodes extension provides OmniGraph nodes for high-performance communication using UCX (Unified Communication X) technology. This extension enables distributed simulation scenarios with low-latency data transfer through specialized OmniGraph nodes that publish sensor data, robot state, and camera images over UCX, as well as subscribe to incoming joint commands.

## Key Components

### Extension Lifecycle

The `UCXBridgeExtension` class manages the native plugin lifecycle and registers writer nodes with the Replicator writer registry. On startup it acquires the native UCX interface and registers annotator-based writer pipelines (e.g., image publish with RGB conversion and time stamping). On shutdown it releases the interface and unregisters writers.

## Relationships

The extension builds upon isaacsim.core.nodes for base node and writer functionality and integrates with isaacsim.ucx.core for the underlying UCX communication infrastructure. It connects with **omni.replicator.core** and **omni.syntheticdata** for annotator pipelines and camera data access.
