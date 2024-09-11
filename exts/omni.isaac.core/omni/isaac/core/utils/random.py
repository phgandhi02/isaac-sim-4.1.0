# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import math
import random
from typing import Tuple

# python
import numpy as np
from omni.isaac.core.utils.rotations import euler_angles_to_quat

# isaacsim
from omni.isaac.core.utils.transformations import get_translation_from_target, get_world_pose_from_relative

# omniverse
from pxr import Usd


def get_random_values_in_range(min_range: np.ndarray, max_range: np.ndarray) -> np.ndarray:
    """Get an array of random values where each element is between the corresponding min_range and max_range element.

    Args:
        min_range (np.ndarray): minimum values for each corresponding element of the array of random values. Shape is
                                (num_values, ).
        max_range (np.ndarray): maximum values for each corresponding element of the array of random values. Shape is
                                (num_values, ).

    Returns:
        np.ndarray: array of random values. Shape is (num_values, ).
    """

    return np.array([random.uniform(min_val, max_val) for min_val, max_val in zip(min_range, max_range)])


def get_random_translation_from_camera(
    min_distance: float, max_distance: float, fov_x: float, fov_y: float, fraction_to_screen_edge: float
) -> np.ndarray:
    """Get a random translation from the camera, in the camera's frame, that's in view of the camera.

    Args:
        min_distance (float): minimum distance away from the camera (along the optical axis) of the random
                                translation.
        max_distance (float): maximum distance away from the camera (along the optical axis) of the random
                                translation.
        fov_x (float): field of view of the camera in the x-direction in radians.
        fov_y (float): field of view of the camera in the y-direction in radians.
        fraction_to_screen_edge (float): maximum allowed fraction to the edge of the screen the translated point may
                                            appear when viewed from the camera. A value of 0 corresponds to the
                                            translated point being centered in the camera's view (on the optical axis),
                                            whereas a value of 1 corresponds to the translated point being on the edge
                                            of the screen in the camera's view.

    Returns:
        np.ndarray: random translation from the camera, in the camera's frame, that's in view of the camera. Shape
                    is (3, ).
    """

    # Randomly select distance away from camera (along the optical axis)
    random_z_distance = random.uniform(min_distance, max_distance)

    # Use distance away to determine allowable range of horizontal/vertical motion that is in view of camera
    theta_x = fov_x / 2.0
    theta_y = fov_y / 2.0

    max_x = random_z_distance * math.tan(fraction_to_screen_edge * theta_x)
    max_y = random_z_distance * math.tan(fraction_to_screen_edge * theta_y)

    # Translation relative to camera in the z direction is negative due to cameras in Isaac Sim having coordinates
    # of -z out, +y up, and +x right.
    random_x = random.uniform(-max_x, max_x)
    random_y = random.uniform(-max_y, max_y)
    random_z = -random_z_distance

    return np.array([random_x, random_y, random_z])


def get_random_world_pose_in_view(
    camera_prim: Usd.Prim,
    min_distance: float,
    max_distance: float,
    fov_x: float,
    fov_y: float,
    fraction_to_screen_edge: float,
    coord_prim: Usd.Prim,
    min_rotation_range: np.ndarray,
    max_rotation_range: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Get a pose defined in the world frame that's in view of the camera.

    Args:
        camera_prim (Usd.Prim): prim path of the camera.
        min_distance (float): minimum distance away from the camera (along the optical axis) of the random
                                translation.
        max_distance (float): maximum distance away from the camera (along the optical axis) of the random
                                translation.
        fov_x (float): field of view of the camera in the x-direction in radians.
        fov_y (float): field of view of the camera in the y-direction in radians.
        fraction_to_screen_edge (float): maximum allowed fraction to the edge of the screen the translated point may
                                            appear when viewed from the camera. A value of 0 corresponds to the
                                            translated point being centered in the camera's view (on the optical axis),
                                            whereas a value of 1 corresponds to the translated point being on the edge
                                            of the screen in the camera's view.
        coord_prim (Usd.Prim): prim whose frame the orientation is defined with respect to.
        min_rotation_range (np.ndarray): minimum XYZ Euler angles of the random pose, defined with respect to the
                                            frame of the prim at coord_prim. Shape is (3, ).
        max_rotation_range (np.ndarray): maximum XYZ Euler angles of the random pose, defined with respect to the
                                            frame of the prim at coord_prim.

    Returns:
        Tuple[np.ndarray, np.ndarray]: first index is position in the world frame. Shape is (3, ). Second index is
                                        quaternion orientation in the world frame. Quaternion is scalar-first
                                        (w, x, y, z). Shape is (4, ).
    """

    random_translation_from_camera = get_random_translation_from_camera(
        min_distance, max_distance, fov_x, fov_y, fraction_to_screen_edge
    )
    random_translation_from_prim = get_translation_from_target(random_translation_from_camera, camera_prim, coord_prim)

    # Rotation ranges are expressed as Euler XYZ angles with respect to the frame of the prim at coord_prim
    random_rotation_from_prim = get_random_values_in_range(min_rotation_range, max_rotation_range)
    random_orientation_from_prim = euler_angles_to_quat(random_rotation_from_prim, degrees=True)

    translation, orientation = get_world_pose_from_relative(
        coord_prim, random_translation_from_prim, random_orientation_from_prim
    )

    return translation, orientation
