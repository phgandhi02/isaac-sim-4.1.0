# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import Dict, Tuple

import cv2 as cv
import numpy as np
import omni
import omni.syntheticdata
from omni.isaac.core.utils.render_product import get_camera_prim_path, get_resolution
from omni.isaac.core.utils.transformations import get_relative_transform
from pxr import Gf, Usd


def read_camera_info(render_product_path: str) -> Dict:
    """Reads camera prim attributes given render product path."""
    camera_info = {}

    # Retrieve and store resolution
    width, height = get_resolution(render_product_path=render_product_path)
    camera_info["width"] = width
    camera_info["height"] = height

    # Retrieve and store camera prim object
    camera_path = get_camera_prim_path(render_product_path=render_product_path)
    camera = omni.usd.get_context().get_stage().GetPrimAtPath(camera_path)
    camera_info["prim"] = camera

    # Retrieve and store camera prim attributes
    focalLength = camera.GetAttribute("focalLength").Get()
    horizontalAperture = camera.GetAttribute("horizontalAperture").Get()
    verticalAperture = horizontalAperture * float(height / width)
    camera_info["focalLength"] = focalLength
    camera_info["horizontalAperture"] = horizontalAperture
    camera_info["verticalAperture"] = verticalAperture

    camera_info["horizontalOffset"] = camera.GetAttribute("horizontalApertureOffset").Get()
    camera_info["verticalOffset"] = camera.GetAttribute("verticalApertureOffset").Get()

    projection_type = camera.GetAttribute("cameraProjectionType").Get()
    if projection_type is None:
        projection_type = "pinhole"

    camera_info["projectionType"] = projection_type
    camera_info["cameraFisheyeParams"] = [0.0] * 19
    if projection_type != "pinhole":
        camera_info["cameraFisheyeParams"][0] = camera.GetAttribute("fthetaWidth").Get()
        camera_info["cameraFisheyeParams"][1] = camera.GetAttribute("fthetaHeight").Get()
        camera_info["cameraFisheyeParams"][2] = camera.GetAttribute("fthetaCx").Get()
        camera_info["cameraFisheyeParams"][3] = camera.GetAttribute("fthetaCy").Get()
        camera_info["cameraFisheyeParams"][4] = camera.GetAttribute("fthetaMaxFov").Get()
        camera_info["cameraFisheyeParams"][5] = camera.GetAttribute("fthetaPolyA").Get()
        camera_info["cameraFisheyeParams"][6] = camera.GetAttribute("fthetaPolyB").Get()
        camera_info["cameraFisheyeParams"][7] = camera.GetAttribute("fthetaPolyC").Get()
        camera_info["cameraFisheyeParams"][8] = camera.GetAttribute("fthetaPolyD").Get()
        camera_info["cameraFisheyeParams"][9] = camera.GetAttribute("fthetaPolyE").Get()
        camera_info["cameraFisheyeParams"][10] = camera.GetAttribute("fthetaPolyF").Get()
        camera_info["cameraFisheyeParams"][11] = camera.GetAttribute("p0").Get()
        camera_info["cameraFisheyeParams"][12] = camera.GetAttribute("p1").Get()
        camera_info["cameraFisheyeParams"][13] = camera.GetAttribute("s0").Get()
        camera_info["cameraFisheyeParams"][14] = camera.GetAttribute("s1").Get()
        camera_info["cameraFisheyeParams"][15] = camera.GetAttribute("s2").Get()
        camera_info["cameraFisheyeParams"][16] = camera.GetAttribute("s3").Get()
        camera_info["cameraFisheyeParams"][17] = camera.GetAttribute("fisheyeResolutionBudget").Get()
        camera_info["cameraFisheyeParams"][18] = camera.GetAttribute("fisheyeFrontFaceResolutionScale").Get()

    physical_distortion = camera.GetAttribute("physicalDistortionModel").Get()
    if physical_distortion is not None:
        camera_info["physicalDistortionModel"] = physical_distortion
    else:
        camera_info["physicalDistortionModel"] = "plumb_bob"

    physical_distortion_coefs = camera.GetAttribute("physicalDistortionCoefficients").Get()
    if physical_distortion_coefs is not None:
        camera_info["physicalDistortionCoefficients"] = np.asarray(physical_distortion_coefs)
    else:
        camera_info["physicalDistortionCoefficients"] = np.zeros((1, 4))

    # Compute and store camera intrinsics matrix (k)
    fx = width * focalLength / horizontalAperture
    fy = height * focalLength / verticalAperture
    cx = width * 0.5
    cy = height * 0.5
    camera_info["k"] = np.asarray([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    camera_info["r"] = np.eye(N=3, dtype=float)
    camera_info["p"] = np.concatenate((camera_info["k"], np.zeros(shape=[3, 1], dtype=float)), axis=1)

    return camera_info


def compute_relative_pose(left_camera_prim: Usd.Prim, right_camera_prim: Usd.Prim) -> Tuple[np.ndarray, np.ndarray]:

    # Compute relative transform -> translation, orientation
    relative_transform = get_relative_transform(target_prim=right_camera_prim, source_prim=left_camera_prim)
    mat = Gf.Transform()
    mat.SetMatrix(Gf.Matrix4d(np.transpose(relative_transform)))
    rotation_vec = mat.GetRotation().Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    translation = np.array(mat.GetTranslation())
    orientation = np.ndarray(shape=[3, 3], dtype=float)
    cv.Rodrigues(src=np.asarray([rotation_vec[0], rotation_vec[1], rotation_vec[2]]), dst=orientation)

    return (translation, orientation)
