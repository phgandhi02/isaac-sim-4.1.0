# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import sys

import omni.kit.test


class TestPipArchive(omni.kit.test.AsyncTestCase):
    # import all packages to make sure dependencies were not missed
    async def test_import_all(self):
        # isort: off
        # Must be at the top so that dependencies later on work
        import typing_extensions

        if sys.platform == "win32":
            import pywintypes

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
