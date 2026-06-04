```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

## Settings Provided by the Extension

## exts."isaacsim.gui.content_browser".timeout
   - **Default Value**: 5
   - **Description**: Time out for resolving the content browser settings

## exts."isaacsim.gui.content_browser".folders
   - **Default Value**: [
     "/Isaac/Robots",
     "/Isaac/Robots_Multiphysics",
     "/Isaac/Environments",
     "/Isaac/IsaacLab",
     "/Isaac/Materials",
     "/Isaac/People",
     "/Isaac/Props",
     "/Isaac/Samples",
     "/Isaac/Sensors",
     "/Isaac/SimReady"
   ]
   - **Description**: Folder paths to display in the content browser, relative to `persistent.isaac.asset_root.default`. Full URLs (http://, https://, omniverse://) are used as-is; relative paths are joined with the asset root at runtime.
