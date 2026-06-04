# Overview

```{image} ../../../../source/extensions/isaacsim.asset.importer.mjcf/data/preview.png
```

To enable this extension, go to the Extension Manager menu and enable isaacsim.asset.importer.mjcf extension.


## High Level Code Overview

### Python
The `MJCF Importer` extension uses `MJCFImporterConfig`, a dataclass that stores configuration
settings for MJCF import operations. The UI extension allows users to modify these settings
through a graphical interface. Configuration fields can be set directly on the dataclass instance.

The main entry point is the `MJCFImporter` class in `python/impl/converter.py`, which takes
an optional `MJCFImporterConfig` instance. The importer uses the `mujoco-usd-converter` library
to convert MJCF files to USD format.

**Example usage:**

.. code-block:: python

    from isaacsim.asset.importer.mjcf import MJCFImporter, MJCFImporterConfig

    # Create configuration
    config = MJCFImporterConfig(
        mjcf_path="/path/to/robot.xml",
        usd_path="/path/to/output",
        merge_mesh=True,
        collision_from_visuals=True
    )

    # Create importer and import
    importer = MJCFImporter(config)
    output_path = importer.import_mjcf()

**Note:** The commands `MJCFCreateAsset` and `MJCFCreateImportConfig` in `python/impl/command.py`
are deprecated and should not be used in new code. Use `MJCFImporter` and `MJCFImporterConfig` directly instead.
