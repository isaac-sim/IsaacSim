```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

## Settings Provided by the Extension

### exts."isaacsim.gui.content_browser".timeout
   - **Default Value**: 5
   - **Description**: Time out for resolving the content browser settings

### exts."omni.simready.content.browser".content_root_urls
   - **Default Value**: [
     "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/SimReady"
   ]
   - **Description**: SimReady assets search locations list. Currently only the first entry is used.

### exts."omni.simready.content.browser".default_content_root_url_index
   - **Default Value**: 0
   - **Description**: Default content root url index into the content_root_urls list above

### exts."omni.simready.content.browser".usd_search_endpoint
   - **Default Value**: "https://search.stg.simready.omniverse.nvidia.com/"
   - **Description**: USD search server endpoint

### exts."isaacsim.gui.content_browser".folders
   - **Default Value**: [
     "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Robots",
     "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Environments",
     "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/IsaacLab",
     "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Materials",
     "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People",
     "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Props",
     "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Samples",
     "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Sensors",
     "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/SimReady"
   ]
   - **Description**: define the folders to be shown in the content browser
