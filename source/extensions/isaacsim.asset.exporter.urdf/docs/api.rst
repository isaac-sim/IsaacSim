URDF Export Extension [isaacsim.asset.exporter.urdf]
####################################################

URDF Exporter API
=================
The URDF exporter provides a Python API for converting USD articulated robots to URDF format.
Below is a sample demonstrating how to export a robot from an open USD stage.

.. code-block:: python
    :linenos:

    import omni.usd
    from isaacsim.asset.exporter.urdf import UsdToUrdfConverter

    # Get the current stage.
    stage = omni.usd.get_context().get_stage()

    # Configure and export.
    converter = UsdToUrdfConverter(
        stage=stage,
        root_prim_path=None,       # Uses the default prim
        mesh_dir_name="meshes",
        mesh_path_prefix="./",
    )

    output_path = converter.convert("/tmp/my_robot.urdf")
    print(f"Exported URDF to: {output_path}")

.. autoclass:: isaacsim.asset.exporter.urdf.UsdToUrdfConverter
    :members:
    :undoc-members:
    :no-show-inheritance:
