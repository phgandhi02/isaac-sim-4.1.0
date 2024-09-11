# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import numpy as np


def matmul(matrix_a, matrix_b):
    return np.matmul(matrix_a, matrix_b)


def sin(data):
    return np.sin(data)


def cos(data):
    return np.cos(data)


def transpose_2d(data):
    return np.transpose(data)


def inverse(data):
    return np.linalg.inv(data)
