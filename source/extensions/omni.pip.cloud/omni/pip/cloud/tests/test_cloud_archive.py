# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import sys

import omni.kit.test


class TestPipArchive(omni.kit.test.AsyncTestCase):
    # import all packages to make sure dependencies were not missed
    async def test_import_all(self):
        # isort: off
        # Must be at the top so that dependencies later on work
        import typing_extensions

        if sys.platform == "win32":
            import pywintypes  # noqa: F401

        # isort: on
        import azure.core
        import azure.identity
        import azure.storage.blob
        import boto3
        import cryptography
        import isodate
        import msal
        import msal_extensions
        import portalocker
        import requests
        import requests_oauthlib
        import s3transfer

        self.assertIsNotNone(azure.identity)
        self.assertIsNotNone(azure.storage.blob)
        self.assertIsNotNone(boto3)
        self.assertIsNotNone(cryptography)
        self.assertIsNotNone(isodate)
        self.assertIsNotNone(requests)
        self.assertIsNotNone(requests_oauthlib)
        self.assertIsNotNone(s3transfer)
        self.assertIsNotNone(typing_extensions)
        self.assertIsNotNone(azure.core)
        self.assertIsNotNone(msal)
        self.assertIsNotNone(msal_extensions)
        self.assertIsNotNone(portalocker)
