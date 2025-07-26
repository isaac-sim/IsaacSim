import isaacsim.asset.importer.mjcf._mjcf as mjcf_bindings
import isaacsim.core.experimental.utils.impl.stage as stage_utils
import isaacsim.test.docstring


class TestExtensionDocstrings(isaacsim.test.docstring.AsyncDocTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture"""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called"""
        super().tearDown()

    async def test_mjcf_docstrings(self):
        await self.assertDocTests(mjcf_bindings)
