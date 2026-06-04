# Commands
Public command API for module **isaacsim.asset.importer.mjcf**:

- [MJCFCreateAsset](#mjcfcreateasset-deprecated)
- [MJCFCreateImportConfig](#mjcfcreateimportconfig-deprecated)


## MJCFCreateAsset (deprecated)

.. deprecated::
   Use ``MJCFImporter()`` directly instead.

This command parses and imports a given mjcf file. It is deprecated and will be removed in a future version.

### Arguments
- mjcf_path
- import_config
- prim_path
- dest_path

## MJCFCreateImportConfig (deprecated)

.. deprecated::
   Use ``MJCFImporterConfig()`` directly instead.

Returns an ImportConfig object that can be used while parsing and importing. It is deprecated and will be removed in a future version.
