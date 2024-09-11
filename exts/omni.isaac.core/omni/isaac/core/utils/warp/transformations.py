# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Any

import numpy as np
import torch
import warp as wp
from omni.isaac.core.utils.torch.rotations import gf_quat_to_tensor as torch_gf_quat_to_tensor
from omni.isaac.core.utils.torch.transformations import tf_matrices_from_poses as torch_tf_matrices_from_poses
from pxr import Gf
from scipy.spatial.transform import Rotation


@wp.kernel
def _local_to_world(
    parent_translations: wp.array(dtype=float, ndim=2),
    parent_rotations: wp.array(dtype=float, ndim=2),
    positions: Any,
    orientations: Any,
    world_pos: wp.array(dtype=float, ndim=2),
    world_rot: wp.array(dtype=float, ndim=2),
):
    tid = wp.tid()
    parent_rot = wp.quat(
        parent_rotations[tid, 0], parent_rotations[tid, 1], parent_rotations[tid, 2], parent_rotations[tid, 3]
    )
    parent_trans = wp.vec3(parent_translations[tid, 0], parent_translations[tid, 1], parent_translations[tid, 2])
    local_rot = wp.quat(orientations[tid, 1], orientations[tid, 2], orientations[tid, 3], orientations[tid, 0])
    local_pos = wp.vec3(positions[tid, 0], positions[tid, 1], positions[tid, 2])
    pos = quat_rotate(parent_rot, local_pos) + parent_trans
    world_pos[tid, 0] = pos[0]
    world_pos[tid, 1] = pos[1]
    world_pos[tid, 2] = pos[2]
    rot = parent_rot * local_rot
    world_rot[tid, 0] = rot[3]
    world_rot[tid, 1] = rot[0]
    world_rot[tid, 2] = rot[1]
    world_rot[tid, 3] = rot[2]


wp.overload(
    _local_to_world, {"positions": wp.array(dtype=float, ndim=2), "orientations": wp.array(dtype=float, ndim=2)}
)
wp.overload(
    _local_to_world,
    {"positions": wp.indexedarray(dtype=float, ndim=2), "orientations": wp.indexedarray(dtype=float, ndim=2)},
)


def get_local_from_world(parent_transforms, positions, orientations, device):
    # TODO: warp kernels not working on cpu
    ret_device = device
    positions = positions.to(device="cuda:0")
    orientations = orientations.to(device="cuda:0")
    parent_rotations = []
    parent_translations = []
    parent_transforms = parent_transforms.numpy()
    for i in range(len(parent_transforms)):
        parent_rotations.append(
            np.array(
                [
                    *Gf.Matrix4d(parent_transforms[i].tolist()).ExtractRotation().GetQuat().GetImaginary(),
                    Gf.Matrix4d(parent_transforms[i].tolist()).ExtractRotation().GetQuat().GetReal(),
                ]
            )
        )
        parent_translations.append(np.array([*Gf.Matrix4d(parent_transforms[i].tolist()).ExtractTranslation()]))

    world_pos = wp.zeros(shape=(positions.shape[0], 3), dtype=wp.float32, device="cuda:0")
    world_rot = wp.zeros(shape=(orientations.shape[0], 4), dtype=wp.float32, device="cuda:0")
    parent_translations = wp.from_numpy(np.array(parent_translations), dtype=wp.float32, device="cuda:0")
    parent_rotations = wp.from_numpy(np.array(parent_rotations), dtype=wp.float32, device="cuda:0")
    wp.launch(
        _local_to_world,
        dim=positions.shape[0],
        inputs=[parent_translations, parent_rotations, positions, orientations, world_pos, world_rot],
        device=positions.device,
    )

    world_pos = world_pos.to(device=ret_device)
    world_rot = world_rot.to(device=ret_device)

    return world_pos, world_rot


def get_world_from_local(parent_transforms, translations, orientations, device):
    calculated_translations = torch.zeros(size=(translations.shape[0], 3), dtype=torch.float32, device=device)
    calculated_orientations = torch.zeros(size=(translations.shape[0], 4), dtype=torch.float32, device=device)

    if isinstance(parent_transforms, wp.types.array):
        parent_torch = wp.to_torch(parent_transforms)
    else:
        parent_torch = parent_transforms
    if isinstance(translations, wp.types.array):
        translations_torch = wp.to_torch(translations)
    else:
        translations_torch = translations
    if isinstance(orientations, wp.types.array):
        orientations_torch = wp.to_torch(orientations)
    else:
        orientations_torch = orientations

    my_local_transforms = torch_tf_matrices_from_poses(
        translations=translations_torch, orientations=orientations_torch, device=device
    )
    # TODO: vectorize this
    for i in range(translations.shape[0]):
        world_transform = torch.matmul(parent_torch[i], my_local_transforms[i])
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(torch.transpose(world_transform, 0, 1).tolist()))
        calculated_translations[i] = torch.tensor(transform.GetTranslation(), dtype=torch.float32, device=device)
        calculated_orientations[i] = torch_gf_quat_to_tensor(transform.GetRotation().GetQuat())

    translations_wp = wp.from_torch(calculated_translations)
    orientations_wp = wp.from_torch(calculated_orientations)
    return translations_wp, orientations_wp


@wp.kernel
def _assign_pose(pose: wp.array(dtype=float, ndim=2), positions: Any, orientations: Any):
    i = wp.tid()
    pose[i, 0] = positions[i, 0]
    pose[i, 1] = positions[i, 1]
    pose[i, 2] = positions[i, 2]
    pose[i, 3] = orientations[i, 0]
    pose[i, 4] = orientations[i, 1]
    pose[i, 5] = orientations[i, 2]
    pose[i, 6] = orientations[i, 3]


wp.overload(_assign_pose, {"positions": wp.array(dtype=float, ndim=2), "orientations": wp.array(dtype=float, ndim=2)})
wp.overload(
    _assign_pose,
    {"positions": wp.indexedarray(dtype=float, ndim=2), "orientations": wp.indexedarray(dtype=float, ndim=2)},
)


def get_pose(positions, orientations, device):
    # TODO: warp kernels not working on cpu
    device = positions.device
    positions = positions.to("cuda:0")
    orientations = orientations.to("cuda:0")
    pose = wp.zeros((positions.shape[0], 7), dtype=wp.float32, device=positions.device)
    wp.launch(_assign_pose, dim=positions.shape[0], inputs=[pose, positions, orientations], device=pose.device)
    pose = pose.to(device)
    return pose


@wp.kernel
def _assign_current_pose(pose: wp.array(dtype=wp.float32, ndim=2), current_positions: Any, current_orientations: Any):
    i = wp.tid()
    pose[i, 0] = current_positions[i, 0]
    pose[i, 1] = current_positions[i, 1]
    pose[i, 2] = current_positions[i, 2]
    pose[i, 3] = current_orientations[i, 1]
    pose[i, 4] = current_orientations[i, 2]
    pose[i, 5] = current_orientations[i, 3]
    pose[i, 6] = current_orientations[i, 0]


wp.overload(
    _assign_current_pose,
    {
        "current_positions": wp.indexedarray(dtype=wp.float32, ndim=2),
        "current_orientations": wp.indexedarray(dtype=wp.float32, ndim=2),
    },
)
wp.overload(
    _assign_current_pose,
    {
        "current_positions": wp.array(dtype=wp.float32, ndim=2),
        "current_orientations": wp.array(dtype=wp.float32, ndim=2),
    },
)


@wp.kernel
def _assign_new_pose(
    pose: wp.array(dtype=wp.float32, ndim=2),
    positions: Any,
    orientations: Any,
    indices: wp.array(dtype=wp.int32),
    has_positions: int,
    has_orientations: int,
):
    i = wp.tid()
    idx = indices[i]
    if has_positions == 1:
        pose[idx, 0] = positions[i, 0]
        pose[idx, 1] = positions[i, 1]
        pose[idx, 2] = positions[i, 2]
    if has_orientations == 1:
        pose[idx, 3] = orientations[i, 1]
        pose[idx, 4] = orientations[i, 2]
        pose[idx, 5] = orientations[i, 3]
        pose[idx, 6] = orientations[i, 0]


wp.overload(
    _assign_new_pose,
    {"positions": wp.indexedarray(dtype=wp.float32, ndim=2), "orientations": wp.indexedarray(dtype=wp.float32, ndim=2)},
)
wp.overload(
    _assign_new_pose,
    {"positions": wp.array(dtype=wp.float32, ndim=2), "orientations": wp.array(dtype=wp.float32, ndim=2)},
)


def assign_pose(current_positions, current_orientations, positions, orientations, indices, device, pose):
    to_cpu = False
    if device == "cpu":
        to_cpu = True
        current_positions = current_positions.to(device="cuda:0")
        current_orientations = current_orientations.to(device="cuda:0")
        positions = positions.to(device="cuda:0")
        orientations = orientations.to(device="cuda:0")
        indices = indices.to(device="cuda:0")
        pose = pose.to(device="cuda:0")
    wp.launch(
        _assign_current_pose, dim=pose.shape[0], inputs=[pose, current_positions, current_orientations], device="cuda:0"
    )
    wp.launch(
        _assign_new_pose,
        dim=positions.shape[0],
        inputs=[
            pose,
            positions,
            orientations,
            indices,
            0 if positions is None else 1,
            0 if orientations is None else 1,
        ],
        device="cuda:0",
    )

    if to_cpu:
        pose = pose.to(device="cpu")

    return pose
