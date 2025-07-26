import isaacsim.asset.gen.omap.bindings._omap as omap_bindings
import isaacsim.test.docstring
from isaacsim.core.utils.stage import create_new_stage_async


class TestExtensionDocstrings(isaacsim.test.docstring.AsyncDocTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture"""
        super().setUp()
        # create new stage
        await create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called"""
        super().tearDown()

    async def test_omap_docstrings(self):
        await self.assertDocTests(omap_bindings)

    async def test_omap_generator_docstrings(self):
        await self.assertDocTests(omap_bindings.Generator)
