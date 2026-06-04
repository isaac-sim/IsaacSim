# Overview

isaacsim.storage.native provides utilities for accessing Isaac Sim assets from Nucleus servers or S3 buckets. This extension handles asset path resolution, server connectivity checks, file system operations, and USD reference validation across both local and remote storage systems. It enables Isaac Sim to seamlessly work with assets stored on various backends while providing robust error handling and retry mechanisms for network operations.

## Concepts

### Asset Root Discovery

The extension automatically discovers and validates Isaac Sim asset root paths on connected Nucleus servers. It uses configurable retry logic with exponential backoff to handle transient network failures when checking server connectivity.

### Path Resolution

The extension provides intelligent path resolution that handles both local filesystem paths and Omniverse URLs. When an asset cannot be found at its original location, the system automatically attempts to resolve it relative to the configured Isaac Sim assets root path.

### {class}`Version <isaacsim.storage.native.Version>` Management

Asset versions are tracked using semantic versioning with the {class}`Version <isaacsim.storage.native.Version>` class. The extension can verify asset compatibility by reading version.txt files from asset root paths and comparing them against the current Isaac Sim application version.

## Functionality

### Server Operations

**Server discovery and validation** through {func}`build_server_list <isaacsim.storage.native.build_server_list>`, {func}`check_server <isaacsim.storage.native.check_server>`, and {func}`get_server_path <isaacsim.storage.native.get_server_path>` functions. These operations support both synchronous and asynchronous execution patterns, with the async versions providing retry logic for improved reliability.

**Asset downloading** from S3 buckets to Nucleus servers using {func}`download_assets_async <isaacsim.storage.native.download_assets_async>` with configurable concurrency limits and progress tracking.

**Folder management** with {func}`create_folder <isaacsim.storage.native.create_folder>` and {func}`delete_folder <isaacsim.storage.native.delete_folder>` operations for organizing assets on Nucleus servers.

### File System Operations

**Path utilities** including {func}`path_join <isaacsim.storage.native.path_join>`, {func}`path_relative <isaacsim.storage.native.path_relative>`, and {func}`path_dirname <isaacsim.storage.native.path_dirname>` that work consistently across local paths and Omniverse URLs. The {func}`is_local_path <isaacsim.storage.native.is_local_path>` function distinguishes between offline and online resources.

**File discovery** with {func}`find_files_recursive <isaacsim.storage.native.find_files_recursive>` and {func}`find_filtered_files <isaacsim.storage.native.find_filtered_files>` supporting regex pattern matching, depth limiting, and exclusion filters. These functions can traverse both local directories and remote Nucleus paths.

**File validation** through {func}`is_file <isaacsim.storage.native.is_file>`, {func}`is_dir <isaacsim.storage.native.is_dir>`, {func}`is_valid_usd_file <isaacsim.storage.native.is_valid_usd_file>`, and {func}`is_mdl_file <isaacsim.storage.native.is_mdl_file>` functions that work across different storage backends.

### USD Reference Management

**Reference analysis** with {func}`get_stage_references <isaacsim.storage.native.get_stage_references>` to extract all references from USD stages, and {func}`find_external_references <isaacsim.storage.native.find_external_references>` to identify references pointing outside a base path.

**Missing reference detection** through {func}`find_missing_references <isaacsim.storage.native.find_missing_references>`, {func}`layer_has_missing_references <isaacsim.storage.native.layer_has_missing_references>`, and {func}`prim_has_missing_references <isaacsim.storage.native.prim_has_missing_references>` functions that recursively validate USD file dependencies.

**Reference counting** via {func}`count_asset_references <isaacsim.storage.native.count_asset_references>` to analyze asset usage patterns across USD files.

## Integration

The extension integrates with **omni.client** for Omniverse operations and isaacsim.core.version for version compatibility checks. It provides both synchronous and asynchronous APIs to accommodate different usage patterns within Isaac Sim applications.

Authentication for Nucleus servers is handled automatically when the ETM_ACTIVE environment variable is set, enabling seamless access to protected asset repositories.
