import isaacsim.core.utils.stage as stage_utils
import isaacsim.sensors.physx._range_sensor as _range_sensor
import isaacsim.test.docstring
import omni


class TestExtensionDocstrings(isaacsim.test.docstring.AsyncDocTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture"""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called"""
        super().tearDown()

    async def test_lidar_docstrings(self):

        # Add lidar
        result, lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/World/Lidar",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=0.0,
            high_lod=False,
            yaw_offset=0.0,
        )
        lidarPath = str(lidar.GetPath())
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await self.assertDocTests(_range_sensor)
