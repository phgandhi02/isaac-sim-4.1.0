# Copyright (c) 2023-2024, NVIDIA CORPORATION. All rights reserved.
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


def get_type(dtype):
    if dtype == "float32":
        return wp.float32
    elif dtype == "bool":
        return wp.uint8
    elif dtype == "int32":
        return wp.int32
    elif dtype == "int64":
        return wp.int64
    elif dtype == "long":
        return wp.int64
    elif dtype == "uint8":
        return wp.uint8
    elif dtype == "bool":
        return wp.uint8  # use uint8 as bool
    else:
        print(f"Type {dtype} not supported.")


def convert(data, device, dtype="float32", indexed=False):
    arr = None
    if not isinstance(data, wp.types.array) and not isinstance(data, wp.types.indexedarray):
        arr = wp.array(data, dtype=get_type(dtype), device=device)
    else:
        arr = data.to(device)
    if indexed and not isinstance(arr, wp.types.indexedarray):
        return wp.indexedarray(arr, [None])
    else:
        return arr


def create_zeros_tensor(shape, dtype, device=None):
    return wp.zeros(shape=tuple(shape), device=device, dtype=get_type(dtype))


def create_tensor_from_list(data, dtype, device=None):
    return wp.array(data, device=device, dtype=get_type(dtype))


def clone_tensor(data, device):
    data = data.to(device)
    cloned_data = wp.zeros_like(data)
    wp.copy(cloned_data, data)
    wp.synchronize_device(device)

    return cloned_data


@wp.kernel
def _arange_k(a: wp.array(dtype=wp.int32)):
    tid = wp.tid()
    a[tid] = tid


def arange(n, device="cpu"):
    a = wp.empty(n, dtype=wp.int32, device=device)
    wp.launch(kernel=_arange_k, dim=n, inputs=[a], device=device)
    return a


global_arange = {}


def resolve_indices(indices, count, device):
    if isinstance(indices, list):
        result = wp.array(indices, dtype=wp.int32, device=device)
    elif indices is None:
        if count not in global_arange.keys():
            result = arange(count, device="cuda")  # TODO: warp kernels not working on cpu
            global_arange[count] = result
        else:
            result = global_arange[count]
    else:
        result = indices.to(device)
    result = result.to(device)
    return result


def move_data(data, device):
    if isinstance(data, wp.types.array):
        return data.to(device)
    elif isinstance(data, wp.types.indexedarray):
        indices = data.indices
        if str(device) != str(data.device):
            return wp.indexedarray(data.contiguous().to(device), indices=[None])
        else:
            return data.to(device)


def tensor_cat(data, device=None, dim=-1):
    for i, d in enumerate(data):
        if isinstance(d, wp.types.array):
            data[i] = wp.to_torch(d)
        elif isinstance(d, wp.types.indexedarray):
            data[i] = torch.tensor(d.numpy(), device=device)
    torch_cat = torch.cat(data, dim=dim)
    return wp.from_torch(torch_cat)


def expand_dims(data, axis):
    if isinstance(data, wp.types.array):
        data = wp.to_torch(data)
    data_torch = torch.unsqueeze(data, axis)
    dtype = wp.int32 if data_torch.dtype in (torch.int32, torch.long) else wp.float32
    return wp.from_torch(data_torch, dtype=dtype)


def to_list(data):
    if isinstance(data, wp.types.array):
        return data.numpy().tolist()
    elif isinstance(data, wp.types.indexedarray):
        return data.numpy().tolist()
    elif isinstance(data, torch.Tensor):
        return data.cpu().numpy().tolist()
    return data


@wp.kernel
def _assign11(src: Any, dst: wp.array(dtype=float), indices: wp.array(dtype=int)):
    tid = wp.tid()
    idx = indices[tid]
    dst[idx] = src[tid]


wp.overload(_assign11, {"src": wp.array(dtype=float)})
wp.overload(_assign11, {"src": wp.indexedarray(dtype=float)})


@wp.kernel
def _assign12(src: Any, dst: wp.array(dtype=float, ndim=2), indices: wp.array(dtype=int)):
    i, j = wp.tid()
    idx = indices[i]
    dst[idx, j] = src[i, j]


wp.overload(_assign12, {"src": wp.array(dtype=float, ndim=2)})
wp.overload(_assign12, {"src": wp.indexedarray(dtype=float, ndim=2)})


@wp.kernel
def _assign13(src: Any, dst: wp.array(dtype=float, ndim=3), indices: wp.array(dtype=int)):
    i, j, k = wp.tid()
    idx = indices[i]
    dst[idx, j, k] = src[i, j, k]


wp.overload(_assign13, {"src": wp.array(dtype=float, ndim=3)})
wp.overload(_assign13, {"src": wp.indexedarray(dtype=float, ndim=3)})


@wp.kernel
def _assign22(
    src: Any, dst: wp.array(dtype=float, ndim=2), indices1: wp.array(dtype=int), indices2: wp.array(dtype=int)
):
    i, j = wp.tid()
    idx1 = indices1[i]
    idx2 = indices2[j]
    dst[idx1, idx2] = src[i, j]


wp.overload(_assign22, {"src": wp.array(dtype=float, ndim=2)})
wp.overload(_assign22, {"src": wp.indexedarray(dtype=float, ndim=2)})


@wp.kernel
def _assign23(
    src: Any, dst: wp.array(dtype=float, ndim=3), indices1: wp.array(dtype=int), indices2: wp.array(dtype=int)
):
    i, j, k = wp.tid()
    idx1 = indices1[i]
    idx2 = indices2[j]
    dst[idx1, idx2, k] = src[i, j, k]


wp.overload(_assign23, {"src": wp.array(dtype=float, ndim=3)})
wp.overload(_assign23, {"src": wp.indexedarray(dtype=float, ndim=3)})


@wp.kernel
def _assign33(
    src: Any,
    dst: wp.array(dtype=float, ndim=3),
    indices1: wp.array(dtype=int),
    indices2: wp.array(dtype=int),
    indices3: wp.array(dtype=int),
):
    i, j, k = wp.tid()
    idx1 = indices1[i]
    idx2 = indices2[j]
    idx3 = indices3[k]
    dst[idx1, idx2, idx3] = src[i, j, k]


wp.overload(_assign33, {"src": wp.array(dtype=float, ndim=3)})
wp.overload(_assign33, {"src": wp.indexedarray(dtype=float, ndim=3)})


def assign(src, dst, indices):
    # TODO: warp kernels not working on cpu
    device = dst.device
    src = move_data(src, "cuda:0")
    dst = move_data(dst, "cuda:0")

    if len(indices) == 1 or not isinstance(indices, list):
        indices = indices.to("cuda:0")
        if len(src.shape) == 1:
            wp.launch(_assign11, dim=src.shape, inputs=[src, dst, indices], device=dst.device)
        elif len(src.shape) == 2:
            wp.launch(_assign12, dim=src.shape, inputs=[src, dst, indices], device=dst.device)
        elif len(src.shape) == 3:
            wp.launch(_assign13, dim=src.shape, inputs=[src, dst, indices], device=dst.device)
        else:
            print("assign does not support source array with >3 dimensions.")
    elif len(indices) == 2:
        indices[0] = indices[0].to("cuda:0")
        indices[1] = indices[1].to("cuda:0")
        if len(src.shape) == 2:
            wp.launch(_assign22, dim=src.shape, inputs=[src, dst, indices[0], indices[1]], device=dst.device)
        elif len(src.shape) == 3:
            wp.launch(_assign23, dim=src.shape, inputs=[src, dst, indices[0], indices[1]], device=dst.device)
        elif len(src.shape) < 2:
            print("source array must have dimension at least 2.")
        else:
            print("assign does not support source array with >3 dimensions.")
    elif len(indices) == 3:
        indices[0] = indices[0].to("cuda:0")
        indices[1] = indices[1].to("cuda:0")
        indices[2] = indices[2].to("cuda:0")
        if len(src.shape) == 3:
            wp.launch(
                _assign33, dim=src.shape, inputs=[src, dst, indices[0], indices[1], indices[2]], device=dst.device
            )
        elif len(src.shape) < 3:
            print("source array must have dimension at least 3.")
        else:
            print("assign does not support source array with >3 dimensions.")
    else:
        print("assign does not support indices >3 dimensions.")
    dst = move_data(dst, device=device)

    return dst


@wp.kernel
def _ones(a: wp.array(dtype=wp.int32)):
    tid = wp.tid()
    a[tid] = 1


def ones(n, device="cpu", dtype=wp.float32):
    a = wp.empty(n, dtype=dtype, device=device)
    wp.launch(kernel=_ones, dim=n, inputs=[a], device=device)
    return a


@wp.kernel
def clamp(data: wp.array(dtype=wp.float32, ndim=2), low: float, high: float):
    i, j = wp.tid()
    data[i, j] = wp.clamp(data[i, j], low, high)


@wp.kernel
def _finite_diff2(
    result: wp.array(dtype=wp.float32, ndim=2),
    a: wp.array(dtype=wp.float32, ndim=2),
    b: wp.array(dtype=wp.float32, ndim=2),
    dt: float,
):
    i, j = wp.tid()
    result[i, j] = (a[i, j] - b[i, j]) / dt


def finite_diff2(a, b, dt):
    result = wp.empty(a.shape, dtype=wp.float32, device=a.device)
    wp.launch(kernel=_finite_diff2, dim=a.shape, inputs=[result, a, b, dt], device=a.device)
    return result
