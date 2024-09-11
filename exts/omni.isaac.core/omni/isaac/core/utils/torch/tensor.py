# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import torch


def as_type(data, dtype):
    if dtype == "float32":
        return data.to(torch.float32)
    elif dtype == "bool":
        return data.to(torch.bool)
    elif dtype == "int32":
        return data.to(torch.int32)
    elif dtype == "int64":
        return data.to(torch.int64)
    elif dtype == "long":
        return data.to(torch.long)
    elif dtype == "uint8":
        return data.to(torch.uint8)
    else:
        print(f"Type {dtype} not supported.")


def convert(data, device, dtype="float32", indexed=None):
    if not isinstance(data, torch.Tensor):
        return as_type(torch.tensor(data, device=device), dtype)
    else:
        return as_type(data.to(device=device), dtype)


def create_zeros_tensor(shape, dtype, device=None):
    return as_type(torch.zeros(shape, device=device), dtype)


def create_tensor_from_list(data, dtype, device=None):
    return as_type(torch.tensor(data, device=device), dtype=dtype)


def clone_tensor(data, device):
    data = data.to(device=device)
    return torch.clone(data)


def resolve_indices(indices, count, device):
    result = indices
    if isinstance(indices, list):
        result = torch.tensor(indices, dtype=torch.long, device=device)
    if indices is None:
        result = torch.arange(count, device=device)
    return result.to(dtype=torch.long, device=device)


def move_data(data, device):
    return data.to(device=device)


def tensor_cat(data, device=None, dim=-1):
    return torch.cat(data, dim=dim)


def expand_dims(data, axis):
    return torch.unsqueeze(data, axis)


def pad(data, pad_width, mode="constant", value=None):
    if len(pad_width) == 2 and isinstance(pad_width[0], tuple):
        pad_width = pad_width[1] + pad_width[0]
    return torch.nn.functional.pad(data, pad_width, mode, value)


def tensor_stack(data, dim=0):
    return torch.stack(data, dim=dim)


def to_list(data):
    if not isinstance(data, list):
        return data.cpu().numpy().tolist()
    return data


def to_numpy(data):
    if isinstance(data, torch.Tensor):
        return data.cpu().numpy()
    return data


def assign(src, dst, indices):
    if isinstance(indices, list):
        dst[tuple(indices)] = src
    else:
        dst[indices] = src
    return dst
