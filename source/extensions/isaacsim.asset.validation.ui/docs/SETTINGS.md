```{csv-table}
**Extension**: {{ extension_version }},**Documentation Generated**: {sub-ref}`today`
```

# Settings

## exts."isaacsim.asset.validation.ui".profileFilter
- **Default Value**: [
  "Package",
  "Package-Candidate",
  "Package-NoBOM",
  "Prop-Robotics-Neutral",
  "Prop-Robotics-Physx",
  "Robot-Body-Neutral",
  "Robot-Body-Runnable"
]
- **Description**: Profile IDs excluded from the profile selector. Set to [] to show every bundled profile.

## exts."isaacsim.asset.validation.ui".profileNames."Robot-Body-Isaac"
- **Default Value**: "Robot Assets"
- **Description**: Optional display-name overrides keyed by profile ID. Unlisted IDs use the generated pretty name.

## exts."isaacsim.asset.validation.ui".profileNames."Prop-Robotics-Isaac"
- **Default Value**: "Props"
- **Description**: Display-name override for the Isaac robotics prop profile.
