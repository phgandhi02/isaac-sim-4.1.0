# Copyright (c) 2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import traceback
from typing import Dict

import carb
import cv2 as cv
import numpy as np
import omni
import omni.replicator.core as rep
import omni.syntheticdata
from omni.isaac.core.utils.render_product import get_camera_prim_path, get_resolution
from omni.isaac.core_nodes import BaseWriterNode
from omni.isaac.ros2_bridge import compute_relative_pose, read_camera_info
from pxr import Gf, Usd


class OgnROS2CameraInfoHelperInternalState(BaseWriterNode):
    def __init__(self):
        self.viewport = None
        self.viewport_name = ""
        self.rv = ""
        self.rvRight = ""
        self.resetSimulationTimeOnStop = False
        self.publishStepSize = 1

        super().__init__(initialize=False)

    def post_attach(self, writer, render_product):
        try:
            if self.rv != "":
                omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                    self.rv + "IsaacSimulationGate", {"inputs:step": self.publishStepSize}, render_product
                )
            if self.rvRight != "":
                omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                    self.rvRight + "IsaacSimulationGate", {"inputs:step": self.publishStepSize}, render_product
                )

            omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                "IsaacReadSimulationTime", {"inputs:resetOnStop": self.resetSimulationTimeOnStop}, render_product
            )
        except:
            pass


class OgnROS2CameraInfoHelper:
    @staticmethod
    def internal_state():
        return OgnROS2CameraInfoHelperInternalState()

    @staticmethod
    def add_camera_info_writer(db, frameId, topicName, camera_info: Dict, render_product_path: str):
        writer = rep.writers.get(f"ROS2PublishCameraInfo")
        writer.initialize(
            frameId=frameId,
            nodeNamespace=db.inputs.nodeNamespace,
            queueSize=db.inputs.queueSize,
            topicName=topicName,
            context=db.inputs.context,
            qosProfile=db.inputs.qosProfile,
            width=camera_info["width"],
            height=camera_info["height"],
            projectionType=camera_info["projectionType"],
            k=camera_info["k"].reshape([1, 9]),
            r=camera_info["r"].reshape([1, 9]),
            p=camera_info["p"].reshape([1, 12]),
            physicalDistortionModel=camera_info["physicalDistortionModel"],
            physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
        )
        db.per_instance_state.attach_writer(writer, render_product_path)
        return

    @staticmethod
    def compute(db) -> bool:
        if db.inputs.enabled is False:
            if db.per_instance_state.initialized is False:
                return True
            else:
                db.per_instance_state.custom_reset()
                return True

        if db.per_instance_state.initialized is False:
            db.per_instance_state.initialized = True
            # Get stage reference
            stage = omni.usd.get_context().get_stage()

            is_stereo = False
            if not db.inputs.renderProductPath:
                carb.log_warn(f"Render product {db.inputs.renderProductPath} not valid")
                db.per_instance_state.initialized = False
                return False
            if db.inputs.renderProductPathRight:
                is_stereo = True

            db.per_instance_state.resetSimulationTimeOnStop = db.inputs.resetSimulationTimeOnStop
            db.per_instance_state.publishStepSize = db.inputs.frameSkipCount + 1

            camera_info_left = read_camera_info(render_product_path=db.inputs.renderProductPath)

            if is_stereo:
                camera_info_right = read_camera_info(render_product_path=db.inputs.renderProductPathRight)

                width_left = camera_info_left["width"]
                height_left = camera_info_left["height"]
                width_right = camera_info_right["width"]
                height_right = camera_info_right["height"]
                if width_left != width_right or height_left != height_right:
                    carb.log_warn(
                        f"Mismatched stereo camera resolutions: left = [{width_left}, {height_left}], right = [{width_right}, {height_right}]"
                    )
                    return False

                translation, orientation = compute_relative_pose(
                    left_camera_prim=camera_info_left["prim"], right_camera_prim=camera_info_right["prim"]
                )

                # Compute stereo rectification parameters
                R1, R2, P1, P2, _, _, _ = cv.stereoRectify(
                    cameraMatrix1=camera_info_left["k"],
                    distCoeffs1=np.asarray(camera_info_left["physicalDistortionCoefficients"]),
                    cameraMatrix2=camera_info_right["k"],
                    distCoeffs2=np.asarray(camera_info_right["physicalDistortionCoefficients"]),
                    imageSize=(width_left, height_left),
                    R=orientation,
                    T=translation,
                )
                camera_info_left["r"] = R1
                camera_info_right["r"] = R2
                camera_info_left["p"] = P1
                camera_info_right["p"] = P2

                # Create right-side writer
                db.per_instance_state.rvRight = "PostProcessDispatchRight"
                OgnROS2CameraInfoHelper.add_camera_info_writer(
                    db,
                    topicName=db.inputs.topicNameRight,
                    frameId=db.inputs.topicNameRight,
                    camera_info=camera_info_right,
                    render_product_path=db.inputs.renderProductPathRight,
                )

            # Create left-side writer
            db.per_instance_state.rv = "PostProcessDispatch"
            OgnROS2CameraInfoHelper.add_camera_info_writer(
                db,
                topicName=db.inputs.topicName,
                frameId=db.inputs.topicName,
                camera_info=camera_info_left,
                render_product_path=db.inputs.renderProductPath,
            )

        else:
            return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnROS2CameraInfoHelperInternalState.per_instance_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
