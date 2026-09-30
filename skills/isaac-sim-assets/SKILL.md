---
name: isaac-sim-assets
description: "Configure and troubleshoot Isaac Sim asset access through Asset Region Profiles, custom or local asset roots, downloadable asset packs, availability manifests, and the public storage APIs. Use for missing or unreachable Isaac Sim assets, regional routing, offline assets, or asset-root precedence; do not use for installing Isaac Sim."
license: Apache-2.0
metadata:
  author: NVIDIA Isaac Sim Team <info@nvidia.com>
---

# Isaac Sim Assets

## Purpose

Configure Isaac Sim to load assets from the default hosted service, a named Asset Region
Profile, or a local asset pack. Diagnose precedence and availability issues without
hardcoding storage endpoints.

## Source of truth

Use the Isaac Sim
[Accessing assets](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/accessing_assets.html)
guide for the shipped profile identifiers, availability manifests, downloadable asset packs,
and current launch instructions.

## Workflow

### 1. Identify the access mode

Choose one mode before changing configuration:

- Default hosted service: use the default Asset Region Profile.
- Regional hosted service: select the documented profile for the user's region.
- Local or custom root: set `ISAACSIM_ASSET_ROOT` to a versioned asset root.
- Missing asset: first determine whether the resolved root is wrong or the selected profile
  does not provide that asset.

### 2. Apply asset-root precedence

Resolve competing configuration in this order:

1. `ISAACSIM_ASSET_ROOT`
2. `ISAACSIM_ASSET_REGION_PROFILE`
3. Command-line setting
4. Experience file
5. Extension default

`ISAACSIM_ASSET_ROOT` overrides the root selected by a profile. If a profile appears to be
ignored, clear that variable and restart Isaac Sim:

```bash
# Linux
unset ISAACSIM_ASSET_ROOT
```

```bat
rem Windows Command Prompt
set ISAACSIM_ASSET_ROOT=
```

```powershell
# Windows PowerShell
Remove-Item Env:ISAACSIM_ASSET_ROOT -ErrorAction SilentlyContinue
```

Both environment-variable workflows update the persistent asset-root setting. Clearing an
environment variable alone might therefore leave the previously selected root in place.
Follow the reset procedure in the Accessing assets guide when returning to the default root.

### 3. Select a hosted profile

Use the identifier documented for the user's region:

```bash
export ISAACSIM_ASSET_REGION_PROFILE=PROFILE_NAME
./isaac-sim.sh
```

```bat
set "ISAACSIM_ASSET_REGION_PROFILE=PROFILE_NAME"
isaac-sim.bat
```

Do not copy a profile's endpoint into `ISAACSIM_ASSET_ROOT`. Profiles can configure routing,
asset search, and the asset root together.

### 4. Use a local asset pack

Download and verify every part of the complete asset pack before extraction. Confirm that
the extracted versioned root contains both `Isaac` and `NVIDIA` directories, then set only
the explicit root:

```bash
export ISAACSIM_ASSET_ROOT="$HOME/isaacsim_assets/Assets/Isaac/VERSION_DIRECTORY"
./isaac-sim.sh
```

```bat
set "ISAACSIM_ASSET_ROOT=C:\isaacsim_assets\Assets\Isaac\VERSION_DIRECTORY"
isaac-sim.bat
```

The downloadable packs are built from the default Asset Region Profile's asset set.

### 5. Resolve paths through the public API

Never hardcode a hosted profile's storage endpoint or derive object URLs from an availability
manifest. Resolve the configured root and join relative paths with the storage API:

```python
from isaacsim.storage.native import get_assets_root_path, path_join

asset_root = get_assets_root_path()
asset_path = path_join(asset_root, "Isaac/Robots/<asset-path>.usd")
```

Use `get_assets_root_path_async` in asynchronous code.

### 6. Verify and diagnose

- In the app, select **Utilities > Check Default Assets Root Path**.
- Confirm that the reported root matches the intended mode.
- For a missing asset on a non-default profile, check that profile's availability manifest.
- Treat an `available` manifest status as mirror availability, not proof that every dependency
  outside the manifest's covered path is present.
- If the asset is not provided, use the default profile or a verified local asset pack.
- Distinguish access denied, transport errors, and missing objects; do not treat them as the
  same failure.

## Boundaries

- Do not install Isaac Sim; route installation requests to `isaac-sim-installation`.
- Do not edit the user's `omniverse.toml` to select a profile.
- Do not expose or recommend direct bucket or CDN URLs.
- Do not assume that clearing `ISAACSIM_ASSET_REGION_PROFILE` restores the default root.
- Do not claim that a regional profile contains the complete default asset catalog.
