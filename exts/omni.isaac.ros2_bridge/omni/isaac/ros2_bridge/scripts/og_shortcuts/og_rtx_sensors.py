# Copyright (c) 2018-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


from pathlib import Path

import omni.graph.core as og
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import omni.kit.viewport.utility
import omni.ui as ui
from omni.isaac.core.utils.stage import get_next_free_path
from omni.isaac.ui.callbacks import on_docs_link_clicked, on_open_IDE_clicked
from omni.isaac.ui.style import get_style
from omni.isaac.ui.widgets import ParamWidget, SelectPrimWidget
from omni.kit.notification_manager import NotificationStatus, post_notification
from omni.kit.window.extensions import SimpleCheckBox
from pxr import OmniGraphSchema, UsdGeom


class Ros2CameraGraph:
    def __init__(self):
        self._og_path = "/Graph/ROS_Camera"
        self._camera_prim = "/OmniverseKit_Persp"  # default camera prim is the perspective camera
        self._add_to_existing_graph = False
        self._frame_id = "sim_camera"
        self._node_namespace = ""
        self._camera_info_topic = "camera_info"
        self._rgb_pub = True
        self._rgb_topic = "/rgb"
        self._depth_pub = True
        self._depth_topic = "/depth"
        self._depth_pcl_pub = False
        self._depth_pcl_topic = "/depth_pcl"
        self._instance_pub = False
        self._instance_topic = "/instance_segmentation"
        self._semantic_pub = False
        self._semantic_topic = "/semantic_segmentation"
        self._bbox2d_tight_pub = False
        self._bbox2d_tight_topic = "/bbox_2d_tight"
        self._bbox2d_loose_pub = False
        self._bbox2d_loose_topic = "/bbox_2d_loose"
        self._bbox3d_pub = False
        self._bbox3d_topic = "/bbox_3d"

    def make_graph(self):
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.stop()

        keys = og.Controller.Keys
        # if starting from a new graph, start it with just a tick and context, render product and camera info (no sim time), the rest is the same for adding to exsiting graph
        if not self._add_to_existing_graph:
            self._og_path = get_next_free_path(self._og_path, "")
            (graph_handle, nodes, _, _) = og.Controller.edit(
                {"graph_path": self._og_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("CameraInfoPublish", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("RenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("RunOnce", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                        ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ],
                    keys.SET_VALUES: [
                        ("RenderProduct.inputs:cameraPrim", self._camera_prim),
                        ("CameraInfoPublish.inputs:topicName", self._camera_info_topic),
                        ("CameraInfoPublish.inputs:type", "camera_info"),
                        ("CameraInfoPublish.inputs:frameId", self._frame_id),
                        ("CameraInfoPublish.inputs:nodeNamespace", self._node_namespace),
                        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                        ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                        ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
                        ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                        ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
                    ],
                },
            )
        else:
            graph_handle = og.get_graph_by_path(self._og_path)

        # to an existin graph
        # traverse through the graph
        all_nodes = graph_handle.get_nodes()
        tick_node = None
        context_node = None
        render_node = None
        for node in all_nodes:
            node_path = node.get_prim_path()
            node_type = node.get_type_name()
            if node_type == "omni.graph.action.OnPlaybackTick" or node_type == "omni.graph.action.OnTick":
                tick_node = node_path
            elif node_type == "omni.isaac.ros2_bridge.ROS2Context":
                context_node = node_path
            elif node_type == "omni.isaac.core_nodes.IsaacCreateRenderProduct":
                render_node_path = node_path
                render_node = node

        # if the existing graph doesn't already have a render node, or if the existing node does not use the same camera, then create a new render node and connect it to the new camera
        # TODO: so far only support if there's one existing render node. If there are multiple render nodes, it won't check if every node has unique camera prims.
        if render_node is None or render_node.get_attribute("inputs:cameraPrim").get()[0] != self._camera_prim:
            print("creating a new rendering node")
            render_node = get_next_free_path(
                self._og_path + "/RenderProduct", ""
            )  # this is actually a string path at this point, not a node prim despite the name. This is so that it's consistent with the others.
            render_node_name = Path(render_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (render_node_name, "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    ],
                    keys.SET_VALUES: [
                        (render_node_name + ".inputs:cameraPrim", self._camera_prim),
                    ],
                    keys.CONNECT: [
                        (tick_node + ".outputs:tick", render_node_name + ".inputs:execIn"),
                    ],
                },
            )
        else:
            render_node = render_node_path  # once again set render_node to the actual path, as oppose to the node_prim, just for consistency

        if self._rgb_pub:
            rgb_node = get_next_free_path(self._og_path + "/RGBPublish", "")
            rgb_node_name = Path(rgb_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (rgb_node_name, "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (rgb_node + ".inputs:topicName", self._rgb_topic),
                        (rgb_node + ".inputs:type", "rgb"),
                        (rgb_node + ".inputs:resetSimulationTimeOnStop", True),
                        (rgb_node + ".inputs:frameId", self._frame_id),
                        (rgb_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", rgb_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", rgb_node + ".inputs:renderProductPath"),
                    ],
                },
            )

            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(rgb_node + ".inputs:context"),
                )

        if self._depth_pub:
            depth_node = get_next_free_path(self._og_path + "/DepthPublish", "")
            depth_node_name = Path(depth_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (depth_node_name, "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (depth_node + ".inputs:topicName", self._depth_topic),
                        (depth_node + ".inputs:type", "depth"),
                        (depth_node + ".inputs:resetSimulationTimeOnStop", True),
                        (depth_node + ".inputs:frameId", self._frame_id),
                        (depth_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", depth_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", depth_node + ".inputs:renderProductPath"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(depth_node + ".inputs:context"),
                )

        if self._depth_pcl_pub:
            depth_pcl_node = get_next_free_path(self._og_path + "/DepthPclPublish", "")
            depth_pcl_node_name = Path(depth_pcl_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (depth_pcl_node_name, "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (depth_pcl_node + ".inputs:topicName", self._depth_pcl_topic),
                        (depth_pcl_node + ".inputs:type", "depth_pcl"),
                        (depth_pcl_node + ".inputs:resetSimulationTimeOnStop", True),
                        (depth_pcl_node + ".inputs:frameId", self._frame_id),
                        (depth_pcl_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", depth_pcl_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", depth_pcl_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", depth_pcl_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(depth_pcl_node + ".inputs:context"),
                )

        if self._instance_pub:
            instance_node = get_next_free_path(self._og_path + "/InstancePublish", "")
            instance_node_name = Path(instance_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (instance_node_name, "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (instance_node + ".inputs:topicName", self._instance_topic),
                        (instance_node + ".inputs:type", "instance_segmentation"),
                        (instance_node + ".inputs:resetSimulationTimeOnStop", True),
                        (instance_node + ".inputs:frameId", self._frame_id),
                        (instance_node + ".inputs:nodeNamespace", self._node_namespace),
                        (instance_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", instance_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", instance_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", instance_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(instance_node + ".inputs:context"),
                )

        if self._semantic_pub:
            semantic_node = get_next_free_path(self._og_path + "/SemanticPublish", "")
            semantic_node_name = Path(semantic_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (semantic_node_name, "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (semantic_node + ".inputs:topicName", self._semantic_topic),
                        (semantic_node + ".inputs:type", "semantic_segmentation"),
                        (semantic_node + ".inputs:resetSimulationTimeOnStop", True),
                        (semantic_node + ".inputs:frameId", self._frame_id),
                        (semantic_node + ".inputs:nodeNamespace", self._node_namespace),
                        (semantic_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", semantic_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", semantic_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", semantic_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(semantic_node + ".inputs:context"),
                )

        if self._bbox2d_tight_pub:
            bbox2d_tight_node = get_next_free_path(self._og_path + "/Bbox2dTightPublish", "")
            bbox2d_tight_node_name = Path(bbox2d_tight_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (bbox2d_tight_node_name, "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (bbox2d_tight_node + ".inputs:topicName", self._bbox2d_tight_topic),
                        (bbox2d_tight_node + ".inputs:type", "bbox_2d_tight"),
                        (bbox2d_tight_node + ".inputs:resetSimulationTimeOnStop", True),
                        (bbox2d_tight_node + ".inputs:frameId", self._frame_id),
                        (bbox2d_tight_node + ".inputs:nodeNamespace", self._node_namespace),
                        (bbox2d_tight_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", bbox2d_tight_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", bbox2d_tight_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", bbox2d_tight_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(bbox2d_tight_node + ".inputs:context"),
                )

        if self._bbox2d_loose_pub:
            bbox2d_loose_node = get_next_free_path(self._og_path + "/Bbox2dLoosePublish", "")
            bbox2d_loose_node_name = Path(bbox2d_loose_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (bbox2d_loose_node_name, "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (bbox2d_loose_node + ".inputs:topicName", self._bbox2d_loose_topic),
                        (bbox2d_loose_node + ".inputs:type", "bbox_2d_loose"),
                        (bbox2d_loose_node + ".inputs:resetSimulationTimeOnStop", True),
                        (bbox2d_loose_node + ".inputs:frameId", self._frame_id),
                        (bbox2d_loose_node + ".inputs:nodeNamespace", self._node_namespace),
                        (bbox2d_loose_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", bbox2d_loose_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", bbox2d_loose_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", bbox2d_loose_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(bbox2d_loose_node + ".inputs:context"),
                )

        if self._bbox3d_pub:
            bbox3d_node = get_next_free_path(self._og_path + "/Bbox3dPublish", "")
            bbox3d_node_name = Path(bbox3d_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (bbox3d_node_name, "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.SET_VALUES: [
                        (bbox3d_node + ".inputs:topicName", self._bbox3d_topic),
                        (bbox3d_node + ".inputs:type", "bbox_3d"),
                        (bbox3d_node + ".inputs:resetSimulationTimeOnStop", True),
                        (bbox3d_node + ".inputs:frameId", self._frame_id),
                        (bbox3d_node + ".inputs:nodeNamespace", self._node_namespace),
                        (bbox3d_node + ".inputs:enableSemanticLabels", True),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", bbox3d_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", bbox3d_node + ".inputs:renderProductPath"),
                        (context_node + ".outputs:context", bbox3d_node + ".inputs:context"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(bbox3d_node + ".inputs:context"),
                )

    def create_camera_graph(self):
        og_path_def = ParamWidget.FieldDef(
            name="og_path", label="Graph Path", type=ui.StringField, default=self._og_path
        )
        frame_id_def = ParamWidget.FieldDef(
            name="frame_id", label="Frame ID", type=ui.StringField, default=self._frame_id
        )
        node_namespace_def = ParamWidget.FieldDef(
            name="node_namespace", label="Node Namespace", type=ui.StringField, default=self._node_namespace
        )
        rgb_topic_def = ParamWidget.FieldDef(
            name="rgb topic", label="RGB Topic", type=ui.StringField, default=self._rgb_topic
        )
        depth_topic_def = ParamWidget.FieldDef(
            name="depth topic", label="Depth Topic", type=ui.StringField, default=self._depth_topic
        )
        depth_pcl_topic_def = ParamWidget.FieldDef(
            name="depth_pcl topic", label="Depth PCL Topic", type=ui.StringField, default=self._depth_pcl_topic
        )
        instance_topic_def = ParamWidget.FieldDef(
            name="instance topic", label="Instance Topic", type=ui.StringField, default=self._instance_topic
        )
        semantic_topic_def = ParamWidget.FieldDef(
            name="semantic topic", label="Semantic Topic", type=ui.StringField, default=self._semantic_topic
        )
        bbox2d_tight_topic_def = ParamWidget.FieldDef(
            name="bbox2d_tight topic", label="Bbox2d Tight Topic", type=ui.StringField, default=self._bbox2d_tight_topic
        )
        bbox2d_loose_topic_def = ParamWidget.FieldDef(
            name="bbox2d_loose topic", label="Bbox2d Loose Topic", type=ui.StringField, default=self._bbox2d_loose_topic
        )
        bbox3d_topic_def = ParamWidget.FieldDef(
            name="bbox3d topic", label="Bbox3d Topic", type=ui.StringField, default=self._bbox3d_topic
        )

        self._window = ui.Window("ROS2 Camera Parameters", width=400, height=650)
        with self._window.frame:
            with ui.VStack(spacing=4):
                with ui.HStack():
                    ui.Label("Add to an existing graph?", width=ui.Percent(30))
                    cb = ui.SimpleBoolModel(default_value=self._add_to_existing_graph)
                    SimpleCheckBox(self._add_to_existing_graph, self._on_use_existing_graph, model=cb)
                self.og_path_input = ParamWidget(field_def=og_path_def)
                self.camera_prim_input = SelectPrimWidget(label="Camera Prim", default=self._camera_prim)
                self.frame_id_input = ParamWidget(field_def=frame_id_def)
                self.node_namespace_input = ParamWidget(field_def=node_namespace_def)
                ui.Spacer(height=5)
                with ui.HStack():
                    ui.Label("RGB", width=ui.Percent(15))
                    cb = ui.SimpleBoolModel(default_value=self._rgb_pub)
                    SimpleCheckBox(self._rgb_pub, self._on_rgb_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.rgb_topic_input = ParamWidget(field_def=rgb_topic_def)
                with ui.HStack():
                    ui.Label("Depth", width=ui.Percent(15))
                    cb = ui.SimpleBoolModel(default_value=self._depth_pub)
                    SimpleCheckBox(self._depth_pub, self._on_depth_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.depth_topic_input = ParamWidget(field_def=depth_topic_def)
                with ui.HStack():
                    ui.Label("Depth Point Clouds", width=ui.Percent(15), word_wrap=True)
                    cb = ui.SimpleBoolModel(default_value=self._depth_pcl_pub)
                    SimpleCheckBox(self._depth_pcl_pub, self._on_depth_pcl_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.depth_pcl_topic_input = ParamWidget(field_def=depth_pcl_topic_def)
                with ui.HStack():
                    ui.Label("Instance", width=ui.Percent(15))
                    cb = ui.SimpleBoolModel(default_value=self._instance_pub)
                    SimpleCheckBox(self._instance_pub, self._on_instance_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.instance_topic_input = ParamWidget(field_def=instance_topic_def)
                with ui.HStack():
                    ui.Label("Semantic", width=ui.Percent(15), word_wrap=True)
                    cb = ui.SimpleBoolModel(default_value=self._semantic_pub)
                    SimpleCheckBox(self._semantic_pub, self._on_semantic_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.semantic_topic_input = ParamWidget(field_def=semantic_topic_def)
                with ui.HStack():
                    ui.Label("BoundingBox2D Tight", width=ui.Percent(15), word_wrap=True)
                    cb = ui.SimpleBoolModel(default_value=self._bbox2d_tight_pub)
                    SimpleCheckBox(self._bbox2d_tight_pub, self._on_bbox2d_tight_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.bbox2d_tight_topic_input = ParamWidget(field_def=bbox2d_tight_topic_def)
                with ui.HStack():
                    ui.Label("BoundingBox2D Loose", width=ui.Percent(15), word_wrap=True)
                    cb = ui.SimpleBoolModel(default_value=self._bbox2d_loose_pub)
                    SimpleCheckBox(self._bbox2d_loose_pub, self._on_bbox2d_loose_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.bbox2d_loose_topic_input = ParamWidget(field_def=bbox2d_loose_topic_def)
                with ui.HStack():
                    ui.Label("BoundingBox3D", width=ui.Percent(15), word_wrap=True)
                    cb = ui.SimpleBoolModel(default_value=self._bbox3d_pub)
                    SimpleCheckBox(self._bbox3d_pub, self._on_bbox3d_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.bbox3d_topic_input = ParamWidget(field_def=bbox3d_topic_def)

                with ui.HStack():
                    ui.Spacer(width=ui.Percent(10))
                    ui.Button("OK", height=40, width=ui.Percent(30), clicked_fn=self._on_ok)
                    ui.Spacer(width=ui.Percent(20))
                    ui.Button("Cancel", height=40, width=ui.Percent(30), clicked_fn=self._on_cancel)
                    ui.Spacer(width=ui.Percent(10))
                with ui.Frame(height=30):
                    with ui.VStack():
                        with ui.HStack():
                            ui.Label("Python Script for Graph Generation", width=ui.Percent(30))
                            ui.Button(
                                name="IconButton",
                                width=24,
                                height=24,
                                clicked_fn=lambda: on_open_IDE_clicked("", __file__),
                                style=get_style()["IconButton.Image::OpenConfig"],
                            )
                        with ui.HStack():
                            ui.Label("Documentations", width=0, word_wrap=True)
                            ui.Button(
                                name="IconButton",
                                width=24,
                                height=24,
                                clicked_fn=lambda: on_docs_link_clicked(
                                    "https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html"
                                ),
                                style=get_style()["IconButton.Image::OpenLink"],
                            )

        return self._window

    def _on_ok(self):
        self._og_path = self.og_path_input.get_value()
        self._camera_prim = self.camera_prim_input.get_value()
        self._frame_id = self.frame_id_input.get_value()
        self._node_namespace = self.node_namespace_input.get_value()
        self._rgb_topic = self.rgb_topic_input.get_value()
        self._depth_topic = self.depth_topic_input.get_value()
        self._depth_pcl_topic = self.depth_pcl_topic_input.get_value()
        self._instance_topic = self.instance_topic_input.get_value()
        self._semantic_topic = self.semantic_topic_input.get_value()
        self._bbox2d_tight_topic = self.bbox2d_tight_topic_input.get_value()
        self._bbox2d_loose_topic = self.bbox2d_loose_topic_input.get_value()
        self._bbox3d_topic = self.bbox3d_topic_input.get_value()

        param_check = self._check_params()
        if param_check:
            self.make_graph()
            self._window.visible = False
        else:
            post_notification("Parameter check failed", status=NotificationStatus.WARNING)

    def _check_params(self):
        stage = omni.usd.get_context().get_stage()

        if self._add_to_existing_graph:
            # make sure the "existing" graph exist
            og_prim = stage.GetPrimAtPath(self._og_path)
            if og_prim.IsValid() and og_prim.IsA(OmniGraphSchema.OmniGraph):
                pass
            else:
                msg = self._og_path + "is not an existing graph, check the og path"
                post_notification(msg, status=NotificationStatus.WARNING)
                return False

        # check if the camera prim is valid
        camera_prim = stage.GetPrimAtPath(self._camera_prim)
        if not camera_prim.IsValid() or not camera_prim.IsA(UsdGeom.Camera):
            msg = self._camera_prim + " is not a valid camera prim, check the camera prim"
            post_notification(msg, status=NotificationStatus.WARNING)
            return False

        return True

    def _on_cancel(self):
        self._window.visible = False

    def _on_use_existing_graph(self, check_state):
        self._add_to_existing_graph = check_state

    def _on_rgb_pub(self, check_state):
        self._rgb_pub = check_state

    def _on_depth_pub(self, check_state):
        self._depth_pub = check_state

    def _on_depth_pcl_pub(self, check_state):
        self._depth_pcl_pub = check_state

    def _on_instance_pub(self, check_state):
        self._instance_pub = check_state

    def _on_semantic_pub(self, check_state):
        self._semantic_pub = check_state

    def _on_bbox2d_tight_pub(self, check_state):
        self._bbox2d_tight_pub = check_state

    def _on_bbox2d_loose_pub(self, check_state):
        self._bbox2d_loose_pub = check_state

    def _on_bbox3d_pub(self, check_state):
        self._bbox3d_pub = check_state


class Ros2RtxLidarGraph:
    def __init__(self):
        self._og_path = "/Graph/ROS_LidarRTX"
        self._frame_id = "sim_lidar"
        self._node_namespace = ""
        self._add_to_existing_graph = False
        self._lidar_prim = ""
        self._laser_scan_pub = True
        self._laser_scan_topic = "/laser_scan"
        self._point_cloud_pub = False
        self._point_cloud_topic = "/point_cloud"

    def make_graph(self):
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.stop()

        keys = og.Controller.Keys
        # if starting from a new graph, start it with just a tick, context, and render product, (no sim time), the rest is the same for adding to exsiting graph
        if not self._add_to_existing_graph:
            self._og_path = get_next_free_path(self._og_path, "")
            (graph_handle, nodes, _, _) = og.Controller.edit(
                {"graph_path": self._og_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("RunOnce", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                        ("RenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                        ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ],
                    keys.SET_VALUES: [("RenderProduct.inputs:cameraPrim", self._lidar_prim)],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                        ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                    ],
                },
            )
        else:
            graph_handle = og.get_graph_by_path(self._og_path)

        # to an existin graph
        # traverse through the graph
        all_nodes = graph_handle.get_nodes()
        tick_node = None
        context_node = None
        render_node = None
        for node in all_nodes:
            node_path = node.get_prim_path()
            node_type = node.get_type_name()
            if node_type == "omni.graph.action.OnPlaybackTick" or node_type == "omni.graph.action.OnTick":
                tick_node = node_path
            elif node_type == "omni.isaac.ros2_bridge.ROS2Context":
                context_node = node_path
            elif node_type == "omni.isaac.core_nodes.IsaacReadSimulationTime":
                sim_time_node = node_path
            elif node_type == "omni.isaac.core_nodes.IsaacCreateRenderProduct":
                render_node = node_path

        if self._laser_scan_pub:
            laser_scan_node = get_next_free_path(self._og_path + "/LaserScanPublish", "")
            laser_scan_node_name = Path(laser_scan_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (laser_scan_node_name, "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                    ],
                    keys.SET_VALUES: [
                        (laser_scan_node + ".inputs:topicName", self._laser_scan_topic),
                        (laser_scan_node + ".inputs:type", "laser_scan"),
                        (laser_scan_node + ".inputs:frameId", self._frame_id),
                        (laser_scan_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", laser_scan_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", laser_scan_node + ".inputs:renderProductPath"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(laser_scan_node + ".inputs:context"),
                )

        if self._point_cloud_pub:
            point_cloud_node = get_next_free_path(self._og_path + "/PointCloudPublish", "")
            point_cloud_node_name = Path(point_cloud_node).name
            og.Controller.edit(
                graph_handle,
                {
                    keys.CREATE_NODES: [
                        (point_cloud_node_name, "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                    ],
                    keys.SET_VALUES: [
                        (point_cloud_node + ".inputs:topicName", self._point_cloud_topic),
                        (point_cloud_node + ".inputs:type", "point_cloud"),
                        (point_cloud_node + ".inputs:frameId", self._frame_id),
                        (point_cloud_node + ".inputs:nodeNamespace", self._node_namespace),
                    ],
                    keys.CONNECT: [
                        (render_node + ".outputs:execOut", point_cloud_node + ".inputs:execIn"),
                        (render_node + ".outputs:renderProductPath", point_cloud_node + ".inputs:renderProductPath"),
                    ],
                },
            )
            if context_node:
                og.Controller.connect(
                    og.Controller.attribute(context_node + ".outputs:context"),
                    og.Controller.attribute(point_cloud_node + ".inputs:context"),
                )

    def create_lidar_graph(self):
        og_path_def = ParamWidget.FieldDef(
            name="og_path", label="Graph Path", type=ui.StringField, default=self._og_path
        )
        frame_id_def = ParamWidget.FieldDef(
            name="frame_id", label="Frame ID", type=ui.StringField, default=self._frame_id
        )
        node_namespace_def = ParamWidget.FieldDef(
            name="node_namespace", label="Node Namespace", type=ui.StringField, default=self._node_namespace
        )
        laser_scan_topic_def = ParamWidget.FieldDef(
            name="laser_scan_topic", label="LaserScan Topic", type=ui.StringField, default=self._laser_scan_topic
        )
        point_cloud_topic_def = ParamWidget.FieldDef(
            name="point_cloud_topic", label="Point Cloud Topic", type=ui.StringField, default=self._point_cloud_topic
        )

        self._window = ui.Window("ROS2 RTX Lidar Parameters", width=400, height=350)
        with self._window.frame:
            with ui.VStack(spacing=4):
                with ui.HStack():
                    ui.Label("Add to an existing graph?", width=ui.Percent(30))
                    cb = ui.SimpleBoolModel(default_value=self._add_to_existing_graph)
                    SimpleCheckBox(self._add_to_existing_graph, self._on_use_existing_graph, model=cb)
                self.og_path_input = ParamWidget(field_def=og_path_def)
                self.lidar_prim_input = SelectPrimWidget(label="Lidar Prim", default=self._lidar_prim)
                self.frame_id_input = ParamWidget(field_def=frame_id_def)
                self.node_namespace_input = ParamWidget(field_def=node_namespace_def)
                ui.Spacer(height=5)
                with ui.HStack():
                    ui.Label("Laser Scan", width=ui.Percent(15))
                    cb = ui.SimpleBoolModel(default_value=self._laser_scan_pub)
                    SimpleCheckBox(self._laser_scan_pub, self._on_laser_scan_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.laser_scan_topic_input = ParamWidget(field_def=laser_scan_topic_def)
                with ui.HStack():
                    ui.Label("Point Cloud", width=ui.Percent(15))
                    cb = ui.SimpleBoolModel(default_value=self._point_cloud_pub)
                    SimpleCheckBox(self._point_cloud_pub, self._on_point_cloud_pub, model=cb)
                    ui.Spacer(width=ui.Percent(5))
                    self.point_cloud_topic_input = ParamWidget(field_def=point_cloud_topic_def)

                with ui.HStack():
                    ui.Spacer(width=ui.Percent(10))
                    ui.Button("OK", height=40, width=ui.Percent(30), clicked_fn=self._on_ok)
                    ui.Spacer(width=ui.Percent(20))
                    ui.Button("Cancel", height=40, width=ui.Percent(30), clicked_fn=self._on_cancel)
                    ui.Spacer(width=ui.Percent(10))
                with ui.Frame(height=30):
                    with ui.VStack():
                        with ui.HStack():
                            ui.Label("Python Script for Graph Generation", width=ui.Percent(30))
                            ui.Button(
                                name="IconButton",
                                width=24,
                                height=24,
                                clicked_fn=lambda: on_open_IDE_clicked("", __file__),
                                style=get_style()["IconButton.Image::OpenConfig"],
                            )
                        with ui.HStack():
                            ui.Label("Documentations", width=0, word_wrap=True)
                            ui.Button(
                                name="IconButton",
                                width=24,
                                height=24,
                                clicked_fn=lambda: on_docs_link_clicked(
                                    "https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html"
                                ),
                                style=get_style()["IconButton.Image::OpenLink"],
                            )

        return self._window

    def _on_ok(self):
        self._og_path = self.og_path_input.get_value()
        self._lidar_prim = self.lidar_prim_input.get_value()
        self._frame_id = self.frame_id_input.get_value()
        self._node_namespace = self.node_namespace_input.get_value()
        self._laser_scan_topic = self.laser_scan_topic_input.get_value()
        self._point_cloud_topic = self.point_cloud_topic_input.get_value()

        param_check = self._check_params()
        if param_check:
            self.make_graph()
            self._window.visible = False
        else:
            post_notification("Parameter check failed", status=NotificationStatus.WARNING)

    def _on_cancel(self):
        self._window.visible = False

    def _check_params(self):
        stage = omni.usd.get_context().get_stage()

        if self._add_to_existing_graph:
            # make sure the "existing" graph exist
            og_prim = stage.GetPrimAtPath(self._og_path)
            if og_prim.IsValid() and og_prim.IsA(OmniGraphSchema.OmniGraph):
                pass
            else:
                msg = self._og_path + "is not an existing graph, check the og path"
                post_notification(msg, status=NotificationStatus.WARNING)
                return False

        # check if the lidar prim is valid
        lidar_prim = stage.GetPrimAtPath(self._lidar_prim)
        if not lidar_prim.IsA(UsdGeom.Camera) or not lidar_prim.HasAPI(IsaacSensorSchema.IsaacRtxLidarSensorAPI):
            msg = self._lidar_prim + " is not a valid RTX lidar prim, check the lidar prim"
            post_notification(msg, status=NotificationStatus.WARNING)
            return False

        return True

    def _on_use_existing_graph(self, check_state):
        self._add_to_existing_graph = check_state

    def _on_laser_scan_pub(self, check_state):
        self._laser_scan_pub = check_state

    def _on_point_cloud_pub(self, check_state):
        self._point_cloud_pub = check_state
