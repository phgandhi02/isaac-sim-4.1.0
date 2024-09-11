# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import typing
from typing import Any

import numpy as np
import torch
import warp as wp
from pxr import Gf
from scipy.spatial.transform import Rotation


def gf_quat_to_tensor(orientation: typing.Union[Gf.Quatd, Gf.Quatf, Gf.Quaternion], device=None) -> wp.types.array:
    """Converts a pxr Quaternion type to a torch array (scalar first).

    Args:
        orientation (typing.Union[Gf.Quatd, Gf.Quatf, Gf.Quaternion]): [description]

    Returns:
       wp.types.array: quaternion tensor
    """

    quat = torch.zeros(4, dtype=torch.float32, device=device)
    quat[1:] = torch.tensor(orientation.GetImaginary(), dtype=torch.float32, device=device)
    quat[0] = orientation.GetReal()

    quat = wp.from_torch(quat)
    return quat


def euler_angles_to_quats(
    euler_angles: wp.types.array, degrees: bool = False, extrinsic: bool = True, device=None
) -> wp.types.array:
    """Vectorized version of converting euler angles to quaternion (scalar first)

    Args:
        euler_angles (wp.types.array): euler angles with shape (N, 3)
        extrinsic (bool, optional): True if the euler angles follows the extrinsic angles
                   convention (equivalent to ZYX ordering but returned in the reverse) and False if it follows
                   the intrinsic angles conventions (equivalent to XYZ ordering).
                   Defaults to True.
        degrees (bool, optional): True if degrees, False if radians. Defaults to False.

    Returns:
        wp.types.array: quaternions representation of the angles (N, 4) - scalar first.
    """
    if extrinsic:
        order = "xyz"
    else:
        order = "XYZ"
    euler_torch = wp.to_torch(euler_angles)
    rot = Rotation.from_euler(order, euler_torch.cpu().numpy(), degrees=degrees)
    result = rot.as_quat()[:, [3, 0, 1, 2]]
    result = wp.array(result, dtype=wp.float32, device=device)

    return result


def rad2deg(radian_value: wp.types.array, device=None) -> wp.types.array:
    """_summary_

    Args:
        radian_value (wp.types.array): _description_
        device (_type_, optional): _description_. Defaults to None.

    Returns:
        wp.types.array: _description_
    """

    rad_torch = wp.to_torch(radian_value)
    rad_deg = torch.rad2deg(rad_torch).float().to(device)
    return wp.from_torch(rad_deg)


def deg2rad(degree_value: wp.types.array, device=None) -> wp.types.array:
    """_summary_

    Args:
        degree_value (torch.Tensor): _description_
        device (_type_, optional): _description_. Defaults to None.

    Returns:
        wp.types.array: _description_
    """

    degree_torch = wp.to_torch(degree_value)
    rad_torch = torch.deg2rad(degree_torch).float().to(device)
    return wp.from_torch(rad_torch)


@wp.kernel
def _xyzw2wxyz1(q: Any):
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]
    q[0] = qw
    q[1] = qx
    q[2] = qy
    q[3] = qz


wp.overload(_xyzw2wxyz1, {"q": wp.array(dtype=float)})
wp.overload(_xyzw2wxyz1, {"q": wp.indexedarray(dtype=float)})


@wp.kernel
def _xyzw2wxyz2(q: Any):
    tid = wp.tid()
    qx = q[tid, 0]
    qy = q[tid, 1]
    qz = q[tid, 2]
    qw = q[tid, 3]
    q[tid, 0] = qw
    q[tid, 1] = qx
    q[tid, 2] = qy
    q[tid, 3] = qz


wp.overload(_xyzw2wxyz2, {"q": wp.array(dtype=float, ndim=2)})
wp.overload(_xyzw2wxyz2, {"q": wp.indexedarray(dtype=float, ndim=2)})


@wp.kernel
def _xyzw2wxyz3(q: Any):
    i, j = wp.tid()
    qx = q[i, j, 0]
    qy = q[i, j, 1]
    qz = q[i, j, 2]
    qw = q[i, j, 3]
    q[i, j, 0] = qw
    q[i, j, 1] = qx
    q[i, j, 2] = qy
    q[i, j, 3] = qz


wp.overload(_xyzw2wxyz3, {"q": wp.array(dtype=float, ndim=3)})
wp.overload(_xyzw2wxyz3, {"q": wp.indexedarray(dtype=float, ndim=3)})


@wp.kernel
def _wxyz2xyzw1(q: Any):
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]
    q[0] = qx
    q[1] = qy
    q[2] = qz
    q[3] = qw


wp.overload(_wxyz2xyzw1, {"q": wp.array(dtype=float)})
wp.overload(_wxyz2xyzw1, {"q": wp.indexedarray(dtype=float)})


@wp.kernel
def _wxyz2xyzw2(q: Any):
    tid = wp.tid()
    qw = q[tid, 0]
    qx = q[tid, 1]
    qy = q[tid, 2]
    qz = q[tid, 3]
    q[tid, 0] = qx
    q[tid, 1] = qy
    q[tid, 2] = qz
    q[tid, 3] = qw


wp.overload(_wxyz2xyzw2, {"q": wp.array(dtype=float, ndim=2)})
wp.overload(_wxyz2xyzw2, {"q": wp.indexedarray(dtype=float, ndim=2)})


@wp.kernel
def _wxyz2xyzw3(q: Any):
    i, j = wp.tid()
    qw = q[i, j, 0]
    qx = q[i, j, 1]
    qy = q[i, j, 2]
    qz = q[i, j, 3]
    q[i, j, 0] = qx
    q[i, j, 1] = qy
    q[i, j, 2] = qz
    q[i, j, 3] = qw


wp.overload(_wxyz2xyzw3, {"q": wp.array(dtype=float, ndim=3)})
wp.overload(_wxyz2xyzw3, {"q": wp.indexedarray(dtype=float, ndim=3)})


def xyzw2wxyz(q):
    # TODO: warp kernels not working on cpu
    from . import move_data

    device = q.device
    q = move_data(q, device="cuda:0")
    if isinstance(q.shape, int) or len(q.shape) == 1:
        wp.launch(_xyzw2wxyz1, dim=q.shape, inputs=[q], device=q.device)
    elif len(q.shape) == 2:
        wp.launch(_xyzw2wxyz2, dim=(q.shape[0]), inputs=[q], device=q.device)
    elif len(q.shape) == 3:
        wp.launch(_xyzw2wxyz3, dim=(q.shape[0], q.shape[1]), inputs=[q], device=q.device)
    else:
        print("xyzw2wxyz does not support input >3 dimensions.")

    q = move_data(q, device=device)

    return q


def wxyz2xyzw(q):
    # TODO: warp kernels not working on cpu
    from . import move_data

    device = q.device
    q = move_data(q, device="cuda:0")
    if isinstance(q.shape, int) or len(q.shape) == 1:
        wp.launch(_wxyz2xyzw1, dim=q.shape, inputs=[q], device=q.device)
    elif len(q.shape) == 2:
        wp.launch(_wxyz2xyzw2, dim=(q.shape[0]), inputs=[q], device=q.device)
    elif len(q.shape) == 3:
        wp.launch(_wxyz2xyzw3, dim=(q.shape[0], q.shape[1]), inputs=[q], device=q.device)
    else:
        print("wxyz2xyzw does not support input >3 dimensions.")

    q = move_data(q, device=device)

    return q


PI = wp.constant(np.pi)
