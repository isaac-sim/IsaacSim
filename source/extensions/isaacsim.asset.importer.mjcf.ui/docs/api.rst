MJCF Importer UI Extension [isaacsim.asset.importer.mjcf.ui]
############################################################

MJCF Import UI Workflow
========================
Enable the MJCF UI extension to import MJCF assets through the asset importer UI.
The UI uses the same configuration object as the core importer.


Deprecated Commands
===================
The following commands are deprecated and provided for backward compatibility only.
New code should use :class:`MJCFImporter` and :class:`MJCFImporterConfig` directly.

.. py:class:: isaacsim.asset.importer.mjcf.ui.impl.command.MJCFCreateImportConfig

   Deprecated command to create an ImportConfig object.

   Should be used with the ``MJCFCreateAsset`` command.

   .. deprecated::
      Use ``MJCFImporterConfig()`` directly instead.

   .. py:method:: do() -> MJCFImporterConfig

      Execute the command to create an import configuration.

      :returns: New MJCFImporterConfig instance.

   .. py:method:: undo() -> None

      Undo the command (no-op).

.. py:class:: isaacsim.asset.importer.mjcf.ui.impl.command.MJCFCreateAsset(mjcf_path='', import_config=MJCFImporterConfig(), prim_path='', dest_path='')

   Deprecated command to parse and import an MJCF file.

   :param str mjcf_path: The absolute path to the MJCF file.
   :param MJCFImporterConfig import_config: Import configuration.
   :param str prim_path: Path to the robot on the USD stage.
   :param str dest_path: Destination path for robot USD. Default is "" which will load the robot in-memory on the open stage.

   .. deprecated::
      Use ``MJCFImporter()`` directly instead.

   .. py:method:: do() -> str

      Execute the command to import the MJCF file.

      :returns: Path to the imported USD file.

   .. py:method:: undo() -> None

      Undo the command (no-op).
