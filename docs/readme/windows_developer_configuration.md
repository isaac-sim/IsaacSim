---
orphan: true
---

# Windows C++ Developer Configuration

## Introduction

This document guides you through setting up this repository for C++ development on Windows using Microsoft Visual Studio and the Windows SDK.

**For New Users:** If you are new to Windows C++ development, this guide provides a step-by-step installation of Visual Studio 2026 Community and the Windows SDK, ensuring you have all the components required for standard development tasks.

**For Advanced Configurations:** If you already have Visual Studio and the Windows SDK installed but wish to specify exact versions, this guide will help you configure your environment using the `[repo_build.msbuild]` configuration within `repo.toml` at the project root.

## Configuration

To enable the Windows C++ build process:

- Set the `"platform:windows-x86_64".enabled` flag to `true` in your `repo.toml` file:

  ```toml
  [repo_build.build]
  "platform:windows-x86_64".enabled = true
  ```

- Set the `link_host_toolchain` flag to `true` in your `repo.toml` file:

  ```toml
  [repo_build.msbuild]
  link_host_toolchain = true
  ```

**Note:** If you already have Visual Studio and the Windows SDK installed, this might be the only change needed. The tooling will auto-detect installed components.

## Windows Long Paths Support

Isaac Sim builds may generate file paths exceeding the traditional Windows 260-character limit. To avoid build errors related to missing files or path length restrictions, Windows Long Paths support must be enabled.

### Automatic Check

The build script (`build.bat`) automatically checks the `LongPathsEnabled` registry setting and displays warnings if long paths support is not properly configured.

### Checking Long Paths Status

To manually verify if long paths are enabled on your system, run the following command in PowerShell:

```powershell
Get-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name "LongPathsEnabled"
```

The value should be `1` for enabled, or `0` for disabled.

### Enabling Long Paths Support

If long paths are not enabled, follow these steps:

1. **Open PowerShell as Administrator**
   - Right-click on PowerShell and select "Run as Administrator"

2. **Create or Set the Registry Value**

   If the registry value doesn't exist, create it:
   ```powershell
   New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force
   ```

   If it already exists but is set to `0`, update it:
   ```powershell
   Set-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" -Name "LongPathsEnabled" -Value 1
   ```

3. **Restart Your System**
   - A system restart is required for the changes to take effect.

**Note:** Long paths support requires Windows 10 version 1607 or later, or Windows 11.

## Compiler Version Checking

The Windows build process will check a handful of versions before starting.  It expects to find the following versions (defined in `repo.toml`):
 * VS 2026
 * MSVC v145
 * MSBuild 18.*

If you do not have these versions you can still start a build, run `build.bat --skip-compiler-version-check`

## Microsoft Visual Studio and Windows SDK Setup

### Basic Installation

#### Installing Visual Studio 2026 Community (Recommended)

Install using Winget by running the following command in PowerShell:

  ```powershell
   winget install --id=Microsoft.VisualStudio.Community -e --override "--add Microsoft.VisualStudio.Workload.NativeDesktop --includeRecommended"
   ```

   Ensure that the following versions are installed:

     - MSVC v145
     - WinSDK 10.0.26100.*

#### Installing Visual Studio 2022 Community (Also Supported)

Install using Winget by running the following command in PowerShell:

  ```powershell
   winget install --id=Microsoft.VisualStudio.2022.Community -e --override "--add Microsoft.VisualStudio.Workload.NativeDesktop --includeRecommended"
   ```

   Ensure that the following versions are installed:

     - MSVC v145
     - WinSDK 10.0.26100.*

#### Installing Windows SDK (as needed)

Usually, the Windows SDK is included with the "Desktop development with C++" workload. To verify or install it separately:

1. **Launch Visual Studio Installer**
   - Open the installer if it's not already running.

2. **Modify Installation**

   ![VS Modify](./vs_modify.png)
   - Click "Modify" on your Visual Studio installation.

3. **Verify Windows SDK**

   ![VS WinSDK Verify](./vs_winsdk_verify.png)
   - Ensure "Windows 11 SDK" is selected under "Optional" sections or "Individual components".

4. **Apply Changes**
   - Click "Modify" to install or update the SDK.

### Configuring an Existing Installation

#### Default Installation Paths

If Visual Studio and the Windows SDK are installed in default locations, the build tooling will auto-detect them without additional configuration.

- Default Windows SDK: `C:\Program Files (x86)\Windows Kits`
- Default Visual Studio 2019: `C:\Program Files (x86)\Microsoft Visual Studio`
- Default Visual Studio 2026: `C:\Program Files\Microsoft Visual Studio`

#### Non-Default Installation Paths

For installations at non-standard paths, specify them in `repo.toml`:

```toml
[repo_build.msbuild]
vs_path = "D:\\CustomPath\\Visual Studio\\2022\\Community"
winsdk_path = "D:\\CustomPath\\Windows Kits\\10\\bin\\10.0.19041.0"
```

Adjust and save the paths as needed.

**Note:** If the path entered is incorrect or invalid, the build system will fall back to auto-detection.

#### Multiple Installations

For multiple Visual Studio or Windows SDK installations, the latest version is used by default. If unspecified, default edition preference is "Enterprise", "Professional", "Community". To specify preferred versions, editions, or paths:

##### Visual Studio

```toml
[repo_build.msbuild]
vs_version = "18"
vs_edition = "Community"
vs_path = "D:\\AnotherPath\\Visual Studio\\2026\\Enterprise\\"
```


##### Windows SDK

```toml
[repo_build.msbuild]
winsdk_version = "10.0.19041.0"
winsdk_path = "D:\\CustomSDKPath\\Windows Kits\\10\\bin\\10.0.19041.0"
```

With these configurations, you control which versions the build system uses, ensuring consistency in environments with multiple installations.

## Additional Resources
- [Repo Build Documentation](https://docs.omniverse.nvidia.com/kit/docs/repo_build/1.0.0/)