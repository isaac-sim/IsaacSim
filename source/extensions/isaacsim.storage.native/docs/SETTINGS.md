```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

## Settings Provided by the Extension

## persistent.isaac.asset_root.default
   - **Default Value**: "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2"
   - **Description**: Default asset root path for Isaac Sim.

   **Resolution order** (highest to lowest priority):

   | Priority | Source | Example |
   |----------|--------|---------|
   | 1 | `ISAACSIM_ASSET_ROOT` environment variable | `export ISAACSIM_ASSET_ROOT=https://my-server` |
   | 2 | Asset Region Profile | `ISAACSIM_ASSET_REGION_PROFILE=<profile-name>` |
   | 3 | Command-line argument | `--/persistent/isaac/asset_root/default=https://my-server` |
   | 4 | Experience (`.kit`) file | `persistent.isaac.asset_root.default = "https://my-server"` |
   | 5 | Extension default (`extension.toml`) | `persistent.isaac.asset_root.default = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2"` |

   At startup the extension applies a selected Asset Region Profile, then reads the
   `ISAACSIM_ASSET_ROOT` environment variable. An explicit asset root therefore overrides
   every other source. When no profile or asset-root environment variable is set, the
   normal Kit settings precedence applies (CLI > `.kit` > `extension.toml`).

## exts."isaacsim.storage.native".asset_region_profiles
   - **Default Value**: Configured asset region profiles
   - **Description**: Named asset region profiles. The
     `ISAACSIM_ASSET_REGION_PROFILE` environment variable selects a profile at startup.
     On a fresh configuration, the extension's default asset root is used.

   A profile may define any of the following keys:

   | Key | Description |
   |-----|-------------|
   | `endpoint` | S3-compatible host to configure |
   | `bucket` | Bucket name; required when `region` is set |
   | `region` | Bucket region; required when `bucket` is set |
   | `cdn_url` | Optional CDN that serves reads while list and stat go direct to the bucket |
   | `cdn_for_list` | Whether list operations also go through the CDN. Defaults to `false` |
   | `http_headers` | Headers sent with every request, for provider compatibility quirks |
   | `asset_root` | Sets `persistent.isaac.asset_root.default` |
   | `usd_search_endpoint` | Sets the SimReady content browser's USD Search endpoint |

   A profile can define only `asset_root`. Storage-specific fields are optional. The
   S3 configuration is applied in memory and never writes to
   `~/.nvidia-omniverse/config/omniverse.toml`. An explicit `ISAACSIM_ASSET_ROOT` takes
   precedence over a profile's `asset_root`.

   Alibaba Cloud OSS returns a whole object for a Range extending past EOF where S3
   clamps to EOF, so multipart reads above 4 MB fail against it. The shipped regional
   profile avoids this by routing reads through a CDN that supplies
   `x-oss-range-behavior: standard`. A profile that points at an OSS bucket **without**
   a `cdn_url` must set the header itself:

   ```toml
   http_headers = { "x-oss-range-behavior" = "standard" }
   ```

   A profile's `asset_root` is written to `persistent.isaac.asset_root.default` and
   persists into later sessions. Clearing `ISAACSIM_ASSET_REGION_PROFILE` does not restore
   the previous root. Follow the reset instructions in the Accessing assets guide to restore
   the default profile's root.
   The endpoint, headers, and USD Search endpoint are session-scoped and are not persisted.
   See the Isaac Sim [Accessing assets](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/accessing_assets.html)
   installation guide for profile selection, local asset packs, precedence, and verification.

## persistent.isaac.asset_root.timeout
   - **Default Value**: 10.0
   - **Description**: Timeout in seconds for asset root path to be resolved.

## persistent.isaac.asset_root.retry_attempts
   - **Default Value**: 3
   - **Description**: Number of retries for transient asset root connectivity checks.

## persistent.isaac.asset_root.retry_base_delay
   - **Default Value**: 0.5
   - **Description**: Initial retry delay in seconds using exponential backoff.
