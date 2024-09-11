# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.isaac.doctest


class TestStandaloneDocTest(omni.isaac.doctest.AsyncDocTestCase):
    # Before running each test
    async def setUp(self):
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_async_doctest_case(self):
        from omni.isaac.doctest import StandaloneDocTestCase

        await self.assertDocTests(StandaloneDocTestCase)
