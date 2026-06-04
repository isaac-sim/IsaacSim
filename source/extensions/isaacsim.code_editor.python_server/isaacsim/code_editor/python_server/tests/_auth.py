# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Authentication helpers for python_server socket tests."""

from __future__ import annotations

import carb

_SETTINGS_PREFIX = "/exts/isaacsim.code_editor.python_server"
_AUTH_HEADER_PREFIX = "# isaacsim-python-server-token:"


def get_auth_token() -> str:
    """Return the configured test auth token."""
    return carb.settings.get_settings().get_as_string(f"{_SETTINGS_PREFIX}/auth_token")


def add_auth_header(source: str) -> str:
    """Prefix raw Python source with the authentication header."""
    return f"{_AUTH_HEADER_PREFIX} {get_auth_token()}\n{source}"


def add_auth_to_envelope(envelope: dict) -> dict:
    """Return a copy of *envelope* with the configured authentication token."""
    authenticated = dict(envelope)
    authenticated.setdefault("auth_token", get_auth_token())
    return authenticated
