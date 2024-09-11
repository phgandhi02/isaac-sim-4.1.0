# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.kit.test


class TestOcs2Import(omni.kit.test.AsyncTestCase):
    """Test importing of OCS2."""

    # Before running each test
    async def setUp(self):
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_ocs2_import(self):
        # isaac-ocs2
        try:
            import ocs2  # noqa: F401
            import ocs2.mobile_manipulator  # noqa: F401
        except ImportError:
            self.assertFalse(True, "Failed to import ocs2")
