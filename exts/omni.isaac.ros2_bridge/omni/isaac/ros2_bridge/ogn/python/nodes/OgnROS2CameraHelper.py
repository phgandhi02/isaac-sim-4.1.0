# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import traceback

import carb
import omni
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
from omni.isaac.core_nodes import BaseWriterNode, WriterRequest
from omni.isaac.ros2_bridge import read_camera_info
from omni.kit.viewport.utility import get_viewport_from_window_name
from pxr import Usd


class OgnROS2CameraHelperInternalState(BaseWriterNode):
    def __init__(self):
        self.viewport = None
        self.viewport_name = ""
        self.rv = ""
        self.resetSimulationTimeOnStop = False
        self.publishStepSize = 1

        super().__init__(initialize=False)

    def post_attach(self, writer, render_product):
        try:
            if self.rv != "":
                omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                    self.rv + "IsaacSimulationGate", {"inputs:step": self.publishStepSize}, render_product
                )

            omni.syntheticdata.SyntheticData.Get().set_node_attributes(
                "IsaacReadSimulationTime", {"inputs:resetOnStop": self.resetSimulationTimeOnStop}, render_product
            )
        except:
            pass


class OgnROS2CameraHelper:
    @staticmethod
    def internal_state():
        return OgnROS2CameraHelperInternalState()

    @staticmethod
    def compute(db) -> bool:
        if db.inputs.enabled is False:
            if db.per_instance_state.initialized is False:
                return True
            else:
                db.per_instance_state.custom_reset()
                return True

        sensor_type = db.inputs.type
        if db.per_instance_state.initialized is False:
            db.per_instance_state.initialized = True
            stage = omni.usd.get_context().get_stage()
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                if db.inputs.viewport:
                    db.log_warn(
                        "viewport input is deprecated, please use renderProductPath and the IsaacGetViewportRenderProduct to get a viewports render product path"
                    )
                    viewport_api = get_viewport_from_window_name(db.inputs.viewport)
                    if viewport_api:
                        db.per_instance_state.viewport = viewport_api
                        db.per_instance_state.viewport_name = db.inputs.viewport
                    if db.per_instance_state.viewport == None:
                        carb.log_warn("viewport name {db.inputs.viewport} not found")
                        db.per_instance_state.initialized = False
                        return False

                    viewport = db.per_instance_state.viewport
                    render_product_path = viewport.get_render_product_path()
                else:
                    render_product_path = db.inputs.renderProductPath
                    if not render_product_path:
                        carb.log_warn("Render product not valid")
                        db.per_instance_state.initialized = False
                        return False
                    if stage.GetPrimAtPath(render_product_path) is None:
                        carb.log_warn("Render product no created yet, retrying on next call")
                        db.per_instance_state.initialized = False
                        return False
                db.per_instance_state.resetSimulationTimeOnStop = db.inputs.resetSimulationTimeOnStop

                db.per_instance_state.publishStepSize = db.inputs.frameSkipCount + 1

                writer = None

                time_type = ""
                if db.inputs.useSystemTime:
                    time_type = "SystemTime"
                    if db.inputs.resetSimulationTimeOnStop:
                        carb.log_warn("System timestamp is being used. Ignoring resetSimulationTimeOnStop input")

                db.per_instance_state.rv = ""

                try:
                    if sensor_type == "rgb":

                        db.per_instance_state.rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                            sd.SensorType.Rgb.name
                        )
                        writer = rep.writers.get(db.per_instance_state.rv + f"ROS2{time_type}PublishImage")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                            qosProfile=db.inputs.qosProfile,
                        )

                    elif sensor_type == "depth":
                        db.per_instance_state.rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                            sd.SensorType.DistanceToImagePlane.name
                        )
                        writer = rep.writers.get(db.per_instance_state.rv + f"ROS2{time_type}PublishImage")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                            qosProfile=db.inputs.qosProfile,
                        )

                    elif sensor_type == "depth_pcl":

                        db.per_instance_state.rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                            sd.SensorType.DistanceToImagePlane.name
                        )

                        writer = rep.writers.get(db.per_instance_state.rv + f"ROS2{time_type}PublishPointCloud")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                            qosProfile=db.inputs.qosProfile,
                        )

                    elif sensor_type == "instance_segmentation":

                        writer = rep.writers.get(f"ROS2{time_type}PublishInstanceSegmentation")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                            qosProfile=db.inputs.qosProfile,
                        )

                    elif sensor_type == "semantic_segmentation":
                        writer = rep.writers.get(f"ROS2{time_type}PublishSemanticSegmentation")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                            qosProfile=db.inputs.qosProfile,
                        )

                    elif sensor_type == "bbox_2d_tight":
                        writer = rep.writers.get(f"ROS2{time_type}PublishBoundingBox2DTight")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                            qosProfile=db.inputs.qosProfile,
                        )

                    elif sensor_type == "bbox_2d_loose":
                        writer = rep.writers.get(f"ROS2{time_type}PublishBoundingBox2DLoose")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                            qosProfile=db.inputs.qosProfile,
                        )

                    elif sensor_type == "bbox_3d":
                        writer = rep.writers.get(f"ROS2{time_type}PublishBoundingBox3D")
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
                            context=db.inputs.context,
                            qosProfile=db.inputs.qosProfile,
                        )

                    elif sensor_type == "camera_info":
                        db.log_warn(
                            "OgnROS2CameraHelper:sensor_type == camera_info is deprecated as of Isaac Sim 4.1.0 in favour of OgnROS2CameraInfoHelper."
                        )
                        db.per_instance_state.rv = "PostProcessDispatch"
                        writer = rep.writers.get(f"ROS2{time_type}PublishCameraInfo")
                        camera_info = read_camera_info(render_product_path=db.inputs.renderProductPath)
                        writer.initialize(
                            frameId=db.inputs.frameId,
                            nodeNamespace=db.inputs.nodeNamespace,
                            queueSize=db.inputs.queueSize,
                            topicName=db.inputs.topicName,
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

                    else:
                        carb.log_error("type is not supported")
                        db.per_instance_state.initialized = False
                        return False
                    if writer is not None:
                        db.per_instance_state.append_writer(writer)
                    type_dict = {
                        "instance_segmentation": "InstanceSegmentationSD",
                        "semantic_segmentation": "SemanticSegmentationSD",
                        "bbox_2d_tight": "BoundingBox2DTightSD",
                        "bbox_2d_loose": "BoundingBox2DLooseSD",
                        "bbox_3d": "BoundingBox3DSD",
                    }
                    if sensor_type in type_dict:
                        if db.inputs.enableSemanticLabels:
                            semantic_writer = rep.writers.get(
                                type_dict[sensor_type] + f"ROS2{time_type}PublishSemanticLabels"
                            )
                            semantic_writer.initialize(
                                nodeNamespace=db.inputs.nodeNamespace,
                                queueSize=db.inputs.queueSize,
                                topicName=db.inputs.semanticLabelsTopicName,
                                context=db.inputs.context,
                                qosProfile=db.inputs.qosProfile,
                            )
                            db.per_instance_state.append_writer(semantic_writer)
                        db.per_instance_state.rv = type_dict[sensor_type]

                    db.per_instance_state.attach_writers(render_product_path)
                except Exception as e:
                    print(traceback.format_exc())
                    pass
        else:
            return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnROS2CameraHelperInternalState.per_instance_internal_state(node)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
