```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

## Settings Provided by the Extension

## persistent.isaac.asset_root.default
   - **Default Value**: "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
   - **Description**: Default asset root path for Isaac Sim.

   **Resolution order** (highest to lowest priority):

   | Priority | Source | Example |
   |----------|--------|---------|
   | 1 | `ISAACSIM_ASSET_ROOT` environment variable | `export ISAACSIM_ASSET_ROOT=https://my-server` |
   | 2 | Command-line argument | `--/persistent/isaac/asset_root/default=https://my-server` |
   | 3 | Experience (`.kit`) file | `persistent.isaac.asset_root.default = "https://my-server"` |
   | 4 | Extension default (`extension.toml`) | `persistent.isaac.asset_root.default = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"` |

   At startup the extension reads the `ISAACSIM_ASSET_ROOT` environment variable and, if set, overwrites the setting regardless of any value provided by a `.kit` file or command-line argument. When the variable is unset, the normal Kit settings precedence applies (CLI > `.kit` > `extension.toml`).

## persistent.isaac.asset_root.timeout
   - **Default Value**: 5.0
   - **Description**: Timeout in seconds for asset root path to be resolved.

## persistent.isaac.asset_root.retry_attempts
   - **Default Value**: 3
   - **Description**: Number of retries for transient asset root connectivity checks.

## persistent.isaac.asset_root.retry_base_delay
   - **Default Value**: 0.5
   - **Description**: Initial retry delay in seconds using exponential backoff.
