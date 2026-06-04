URDF Importer UI Extension [isaacsim.asset.importer.urdf.ui]
############################################################

URDF Import UI Workflow
========================
Enable the URDF UI extension to import URDF assets through the asset importer UI.
The UI uses the same configuration object as the core importer.


Deprecated Commands
===================
The following commands are deprecated and provided for backward compatibility only.
New code should use :class:`URDFImporter` and :class:`URDFImporterConfig` directly.

.. py:class:: isaacsim.asset.importer.urdf.ui.impl.command.URDFCreateImportConfig

   Deprecated command to create an ImportConfig object.

   Should be used with the ``URDFParseFile`` and ``URDFImportRobot`` commands.

   .. deprecated::
      Use ``URDFImporterConfig()`` directly instead.

   .. py:method:: do() -> URDFImporterConfig

      Execute the command to create an import configuration.

      :returns: New URDFImporterConfig instance.

   .. py:method:: undo() -> None

      Undo the command (no-op).

.. py:class:: isaacsim.asset.importer.urdf.ui.impl.command.URDFParseText(urdf_text='', import_config=URDFImporterConfig())

   Deprecated command to parse a URDF string.

   :param str urdf_text: The URDF string to parse.
   :param URDFImporterConfig import_config: Import configuration.

   .. deprecated::
      Parsing URDF strings is not supported. Use ``URDFImporter()`` with a file path instead.

   .. py:method:: do() -> NoReturn

      Execute the command to parse the URDF string.

      :raises RuntimeError: Parsing URDF strings is no longer supported.

   .. py:method:: undo() -> None

      Undo the command (no-op).

.. py:class:: isaacsim.asset.importer.urdf.ui.impl.command.URDFParseFile(urdf_path='', import_config=URDFImporterConfig())

   Deprecated command to parse a URDF file.

   :param str urdf_path: The absolute path to the URDF file.
   :param URDFImporterConfig import_config: Import configuration.

   .. deprecated::
      Use ``URDFImporter()`` directly instead.

   .. py:method:: do() -> NoReturn

      Execute the command to parse the URDF file.

      :raises RuntimeError: Parsing URDF files is no longer supported.

   .. py:method:: undo() -> None

      Undo the command (no-op).

.. py:class:: isaacsim.asset.importer.urdf.ui.impl.command.URDFImportRobot(urdf_path='', urdf_robot=None, import_config=URDFImporterConfig(), dest_path='', return_articulation_root_prim=False)

   Deprecated command to import a URDF robot.

   :param str urdf_path: The absolute path to the URDF file.
   :param object urdf_robot: The robot model from URDFParseFile (optional, for backward compatibility).
   :param URDFImporterConfig import_config: Import configuration.
   :param str dest_path: Destination path for robot USD. Default is "" which will load the robot in-memory on the open stage.
   :param bool return_articulation_root_prim: Whether to return the articulation root prim instead of the robot USD path.

   .. deprecated::
      Use ``URDFImporter()`` directly instead.

   .. py:method:: do() -> tuple[Result, str]

      Execute the command to import the URDF file.

      :returns: Tuple of (Result, prim_path).

   .. py:method:: undo() -> None

      Undo the command (no-op).
