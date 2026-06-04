```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

## Settings Provided by the Extension

## exts."isaacsim.benchmark.services".metrics.nvdataflow_default_test_suite_name
- **Default Value**: "Isaac Sim Benchmarks"
- **Description**: Sets the default test suite name for Isaac Sim benchmarks.

## exts."isaacsim.benchmark.services".metrics.metrics_output_folder
- **Default Value**: "" (empty string, currently commented out)
- **Description**: Specifies the output folder path where metrics should be saved.

## exts."isaacsim.benchmark.services".metrics.randomize_filename_prefix
- **Default Value**: false
- **Description**: Controls whether to add a randomly generated string as a prefix to the output filename to distinguish runs.

## exts."isaacsim.benchmark.services".rtf_stability.window_wall_ms
- **Default Value**: 100.0
- **Description**: Wall-clock window size in milliseconds for each windowed RTF sample collected by the ``rtf_stability`` recorder.

## exts."isaacsim.benchmark.services".rtf_stability.export_window_samples
- **Default Value**: false
- **Description**: If true, the ``rtf_stability`` recorder also emits a list measurement of every windowed RTF sample (larger output files). It always emits mean, stdev, sample count, and mean-anchored stability metrics: fixed ±0.01/±0.10 **absolute** bands vs the phase mean windowed RTF, max absolute deviation from that mean, and the longest streak of consecutive windows outside the ±0.01 band. Band widths are fixed in code.
