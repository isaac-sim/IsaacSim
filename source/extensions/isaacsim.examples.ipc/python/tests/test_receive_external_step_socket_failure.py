# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression coverage for TCP receive socket-construction failures."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import omni.kit.test


_MODULE_PATH = Path(__file__).resolve().parents[1] / "nodes" / "OgnSimpleReceiveExternalStepPy.py"
_SPEC = importlib.util.spec_from_file_location("_ogn_receive_external_step_test", _MODULE_PATH)
_MODULE = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
_SPEC.loader.exec_module(_MODULE)


class TestReceiveExternalStepSocketFailure(omni.kit.test.AsyncTestCase):
    """Verify socket creation failures return False without secondary exceptions."""

    async def test_socket_constructor_failure_does_not_reference_unassigned_socket(self) -> None:
        state = _MODULE.OgnSimpleReceiveExternalStepPyInternalState.__new__(
            _MODULE.OgnSimpleReceiveExternalStepPyInternalState
        )
        state.listen_sock = None
        state.client_sock = None
        state.buf = bytearray()
        state.uri = ""
        db = SimpleNamespace(
            per_instance_state=state,
            inputs=SimpleNamespace(uri="127.0.0.1:8555"),
            outputs=SimpleNamespace(),
        )

        with patch.object(_MODULE.socket, "socket", side_effect=OSError("socket unavailable")):
            result = _MODULE.OgnSimpleReceiveExternalStepPy.compute(db)

        self.assertFalse(result)
        self.assertIsNone(state.listen_sock)
        self.assertEqual(state.uri, "")
