# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Tests for BaseWriterNode render-product attachment handling."""

from unittest.mock import MagicMock, patch

import omni.kit.test
from isaacsim.core.nodes import BaseWriterNode, WriterRequest


class TestBaseWriterNode(omni.kit.test.AsyncTestCase):
    """Validate writer activation requests."""

    @patch("isaacsim.core.nodes.impl.base_writer_node.Usd.EditContext")
    @patch("isaacsim.core.nodes.impl.base_writer_node.omni.usd.get_context")
    @patch("isaacsim.core.nodes.impl.base_writer_node.rep.AnnotatorRegistry.get_annotator")
    async def test_noop_annotator_preserves_render_product_list(
        self, mock_get_annotator: MagicMock, mock_get_context: MagicMock, mock_edit_context: MagicMock
    ) -> None:
        """A list of render products must not be wrapped in another list."""
        stage = MagicMock()
        mock_get_context.return_value.get_stage.return_value = stage
        mock_edit_context.return_value.__enter__.return_value = None
        mock_edit_context.return_value.__exit__.return_value = False

        writer = MagicMock()
        writer.node_type_id = "TestWriter"
        writer._kwargs = {}
        writer._annotators = []
        noop = mock_get_annotator.return_value

        render_products = ["/Render/ProductA", "/Render/ProductB"]
        node = BaseWriterNode()
        node.post_attach = MagicMock()
        node._requests = [WriterRequest(writer, render_products, True)]

        node._process_activation_requests(MagicMock())

        writer.attach.assert_called_once_with(render_products)
        noop.attach.assert_called_once_with(render_products)
