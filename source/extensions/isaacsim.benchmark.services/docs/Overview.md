# Overview

isaacsim.benchmark.services provides a comprehensive framework for performance benchmarking in Isaac Sim applications. The extension offers structured approaches to measure and analyze CPU, GPU, memory, and frame time performance through both synchronous and asynchronous benchmark implementations.

## Key Components

### {class}`BaseIsaacBenchmark <isaacsim.benchmark.services.BaseIsaacBenchmark>`

The synchronous benchmark class designed for standalone scripts that need deterministic performance measurements. It provides phase-based benchmarking where different stages of execution can be measured independently.

```python
benchmark = BaseIsaacBenchmark(benchmark_name="MyBenchmark", workflow_metadata={"metadata": []})
benchmark.set_phase("loading")
# load stage, configure sim, etc.
benchmark.store_measurements()
benchmark.set_phase("benchmark")
# run benchmark
benchmark.store_measurements()
benchmark.stop()
```

### {class}`BaseIsaacBenchmarkAsync <isaacsim.benchmark.services.BaseIsaacBenchmarkAsync>`

An asynchronous benchmark class that inherits from `**omni.kit.test.AsyncTestCase**`, enabling integration with Isaac Sim's test framework. This approach is ideal for test cases that require async operations and proper test lifecycle management.

```python
class MyBenchmark(BaseIsaacBenchmarkAsync):
    async def setUp(self):
        await super().setUp()

    async def test_my_benchmark(self):
        self.set_phase("loading")
        await self.fully_load_stage("path/to/stage.usd")
        await self.store_measurements()

        self.set_phase("benchmark")
        # ... run benchmark ...
        await self.store_measurements()

    async def tearDown(self):
        await super().tearDown()
```

## Functionality

### Phase-Based Measurement

Both benchmark classes organize measurements into distinct phases, allowing developers to isolate performance characteristics of different execution stages. Each phase can selectively enable frame time and runtime recording based on measurement requirements.

### Data Collection

The framework includes multiple data recorders that capture various performance metrics:
- **app_frametime**: Application frame timing measurements
- **cpu_continuous**: Continuous CPU utilization monitoring  
- **gpu_frametime**: GPU frame rendering measurements
- **hardware**: Hardware specifications and capabilities
- **memory**: Memory usage tracking
- **physics_frametime**: Physics simulation timing
- **render_frametime**: Rendering pipeline measurements
- **runtime**: Overall execution time tracking

### Stage Loading Integration

Both classes provide specialized methods for USD stage loading that ensure complete asset and material loading before proceeding with measurements. This eliminates timing inconsistencies caused by background loading operations.

### Custom Measurements

The framework supports storing custom measurement data alongside the standard metrics, enabling domain-specific performance analysis through the `store_custom_measurement` method.

## Configuration

The extension provides settings for customizing benchmark behavior:

- `metrics.nvdataflow_default_test_suite_name`: Sets the default test suite identifier for organized metric collection
- `metrics.metrics_output_folder`: Specifies the directory for metric output files  
- `metrics.randomize_filename_prefix`: Controls whether output filenames include random prefixes to distinguish multiple benchmark runs

## Dependencies

The extension integrates with Isaac Sim's core systems through several dependencies. It uses `**omni.kit.test**` to provide the async test case foundation for {class}`BaseIsaacBenchmarkAsync <isaacsim.benchmark.services.BaseIsaacBenchmarkAsync>`, and leverages `**omni.physics**` and `**omni.physics.physx**` for physics-related performance measurements.
