# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test
from omni.isaac.version import get_version, parse_version


class TestIsaacVersion(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        pass

    async def tearDown(self):
        pass

    async def test_version(self):
        parsed_version = parse_version("2000.0.0-beta.0+branch.0.hash.local")
        self.assertTrue(parsed_version.core == "2000.0.0")
        self.assertTrue(parsed_version.pretag == "beta")
        self.assertTrue(parsed_version.prebuild == "0")
        self.assertTrue(parsed_version.buildtag == "branch.0.hash.local")

        version = get_version()
        self.assertTrue(len(version) == 8)
