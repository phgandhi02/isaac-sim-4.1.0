# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.kit.test
from omni.isaac.core.utils.rotations import (
    euler_angles_to_quat,
    euler_to_rot_matrix,
    matrix_to_euler_angles,
    quat_to_euler_angles,
    rot_matrix_to_quat,
)
from scipy.spatial.transform import Rotation


class TestRotations(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        pass

    # After running each test
    async def tearDown(self):
        pass

    async def test_euler_angles_to_quat(self):
        roll, pitch, yaw = np.pi * np.random.rand(3)
        rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)

        x, y, z, w = rot.as_quat()
        self.assertTrue(
            np.all(
                np.isclose(
                    euler_angles_to_quat(np.array([roll, pitch, yaw])),
                    np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
            or np.all(
                np.isclose(
                    euler_angles_to_quat(np.array([roll, pitch, yaw])),
                    np.array([w, x, y, z]) * -1,
                    atol=1e-05,
                )
            ),
            f"{euler_angles_to_quat(np.array([roll, pitch, yaw]))} != {np.array([w, x, y, z])}",
        )
        pass

    async def test_quat_to_euler_angles(self):
        roll, pitch, yaw = np.pi * np.random.rand(3)
        rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)
        x, y, z, w = rot.as_quat()
        self.assertTrue(
            np.all(
                np.isclose(
                    euler_angles_to_quat(quat_to_euler_angles(np.array([w, x, y, z]))),
                    np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
            or np.all(
                np.isclose(
                    euler_angles_to_quat(quat_to_euler_angles(np.array([w, x, y, z]))),
                    -1 * np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
        )
        roll, pitch, yaw = [0.94965366, 2.3579875, 1.43057573]
        rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)
        x, y, z, w = rot.as_quat()
        self.assertTrue(
            np.all(
                np.isclose(
                    euler_angles_to_quat(quat_to_euler_angles(np.array([w, x, y, z]))),
                    np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
            or np.all(
                np.isclose(
                    euler_angles_to_quat(quat_to_euler_angles(np.array([w, x, y, z]))),
                    -1 * np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
        )
        pass

    async def test_euler_angles_to_matrix_to_quat(self):
        gt_quat = np.array([-0.5, -0.5, -0.5, 0.5])
        quat = rot_matrix_to_quat(
            euler_to_rot_matrix(matrix_to_euler_angles(np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])))
        )
        self.assertTrue(
            np.all(
                np.isclose(
                    gt_quat,
                    quat,
                    atol=1e-05,
                )
            )
            or np.all(
                np.isclose(
                    -1 * gt_quat,
                    quat,
                    atol=1e-05,
                )
            )
        )
