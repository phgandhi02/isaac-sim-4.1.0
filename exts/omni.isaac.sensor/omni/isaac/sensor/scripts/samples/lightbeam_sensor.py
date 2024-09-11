# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import asyncio
import weakref

import carb
import omni
import omni.graph.core as og
import omni.kit.commands
import omni.physx as _physx
import omni.ui as ui
from omni.isaac.core.utils.prims import delete_prim, get_prim_at_path
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.sensor import _sensor
from omni.isaac.ui.menu import make_menu_item_description
from omni.isaac.ui.ui_utils import LABEL_WIDTH, get_style, setup_ui_headers
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

EXTENSION_NAME = "LightBeam Sensor Example"


class LightBeamSensorDemo(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._ext_id = ext_id
        self._extension_path = ext_manager.get_extension_path(ext_id)

        self._menu_items = [
            MenuItemDescription(
                name="Sensors",
                sub_menu=[make_menu_item_description(ext_id, "LightBeam", lambda a=weakref.proxy(self): a.build_ui())],
            )
        ]
        add_menu_items(self._menu_items, "Isaac Examples")
        self.meters_per_unit = 1.00
        self._window = None
        self.beam_hit_labels = []
        self.linear_depth_labels = []
        self.hit_pos_labels = []
        self.num_rays = 5

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.CLOSED):
            self.on_closed()

    def build_ui(self):
        if self._window is None:
            self._ls = _sensor.acquire_lightbeam_sensor_interface()

            self._timeline = omni.timeline.get_timeline_interface()
            self.sub = _physx.get_physx_interface().subscribe_physics_step_events(self._on_update)

            self.colors = [
                0xFFBBBBFF,
                0xFFBBFFBB,
                0xBBFFBBBB,
                0xBBAAEEFF,
                0xAABBFFEE,
                0xFFEEAABB,
                0xFFC8D5D0,
                0xFFC89BD0,
                0xFFAF9BA7,
                0xFFA4B99A,
            ]

            style = {"background_color": 0xFF888888, "color": 0xFF333333, "secondary_color": self.colors[0]}

            self._window = ui.Window(
                title=EXTENSION_NAME, width=800, height=0, visible=True, dockPreference=ui.DockPreference.LEFT_BOTTOM
            )
            with self._window.frame:
                with ui.VStack(spacing=5, height=0):

                    title = "LightBeam Sensor Example"
                    doc_link = "TBD"

                    overview = "This Example shows the output of the LightBeam sensor."
                    overview += "The LightBeam sensor constantly scans for any hits to its beams and outputs linear depth and hit position."
                    overview += "\nPress PLAY to start the simulation and visualize the light beams, left click the cube/sensor origin to drag it around."
                    overview += "\n\nPress the 'Open in IDE' button to view the source code."

                    setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

                    frame = ui.CollapsableFrame(
                        title="Sensor Readings",
                        height=0,
                        collapsed=False,
                        style=get_style(),
                        style_type_name_override="CollapsableFrame",
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    )
                    with frame:
                        with ui.VStack(style=get_style(), spacing=3):
                            for i in range(self.num_rays):
                                # Displaying light beam number and data for each light beam
                                with ui.HStack():
                                    ui.Label(
                                        f"Lightbeam {i+1}",
                                        width=LABEL_WIDTH / 2,
                                        tooltip="Light beam number",
                                        style={"secondary_color": self.colors[i]},
                                    )

                                    # Displaying beam hit status (initially empty, to be updated in _on_update)
                                    self.beam_hit_labels.append(
                                        ui.Label(
                                            "",
                                            width=LABEL_WIDTH / 1.7,
                                            tooltip="beam hit t/f",
                                            style={"secondary_color": self.colors[i]},
                                        )
                                    )

                                    # Displaying linear depth (initially empty, to be updated in _on_update)
                                    self.linear_depth_labels.append(
                                        ui.Label(
                                            "",
                                            width=LABEL_WIDTH * 1.3,
                                            tooltip="linear depth in meters",
                                            style={"secondary_color": self.colors[i]},
                                        )
                                    )

                                    # Displaying hit position with specific labels for x, y, and z (initially empty, to be updated in _on_update)
                                    self.hit_pos_labels.append(
                                        ui.Label(
                                            "",
                                            width=LABEL_WIDTH,
                                            tooltip="hit position in meters",
                                            style={"secondary_color": self.colors[i]},
                                        )
                                    )

            asyncio.ensure_future(self.create_scenario())

        self._window.visible = True

    def on_shutdown(self):
        self.on_closed()
        remove_menu_items(self._menu_items, "Isaac Examples")

    def on_closed(self):
        if self._window:
            self.sub = None
            self._timeline = None
            self._stage_event_subscription = None
            self._window.visible = False

        self._window = None

    def _on_update(self, dt):
        if self._timeline.is_playing():
            lin_depth = self._ls.get_linear_depth_data(self.sensor_path)
            hit_pos = self._ls.get_hit_pos_data(self.sensor_path)
            # cast from uint8 to bool
            beam_hit = self._ls.get_beam_hit_data(self.sensor_path).astype(bool)

            for i in range(self.num_rays):

                # Update UI labels with the new data
                self.beam_hit_labels[i].text = f"beamhit: {beam_hit[i]}"
                self.linear_depth_labels[i].text = f"linearDepth: {lin_depth[i]}"
                self.hit_pos_labels[
                    i
                ].text = f"hitPos x: {hit_pos[i][0]}, hitPos y: {hit_pos[i][1]}, hitPos z: {hit_pos[i][2]}"

    async def create_scenario(self):
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        offset = Gf.Vec3f(2.00, 0.0, 0.0)
        size = 1

        # we need a physics scene for physx raycasts
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

        self.cube_path = "/World/Cube"
        self.sensor_path = "/LightBeam_Sensor"

        # Define a light so we can see the cube better
        if get_prim_at_path("/DistantLight"):
            delete_prim("/DistantLight")
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)
        distantLight.AddRotateXYZOp().Set((-36, -36, 0))

        self.cube_geom = UsdGeom.Cube.Define(stage, self.cube_path)
        self.cube_prim = stage.GetPrimAtPath(self.cube_path)

        self.cube_geom.CreateSizeAttr(size)
        self.cube_geom.AddTranslateOp().Set(offset)
        UsdGeom.XformCommonAPI(self.cube_prim).SetScale((1.00, 1.00, 1.00))

        # In order for our cube to interact with the light beam sensor, it needs to be able to collide with our physX line traces.
        # to do this, we give our cube the collision API, and set it's material and collision group.
        UsdPhysics.CollisionAPI.Apply(self.cube_prim)

        await omni.kit.app.get_app().next_update_async()

        self.meters_per_unit = UsdGeom.GetStageMetersPerUnit(omni.usd.get_context().get_stage())

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path=self.sensor_path,
            parent=None,
            min_range=0.2,
            max_range=10.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            forward_axis=Gf.Vec3d(1, 0, 0),
            num_rays=5,
            curtain_length=0.5,
        )

        if not result:
            carb.log_error("Could not create Light Beam Sensor")
            return

        await omni.kit.app.get_app().next_update_async()

        # we want to make sure we can see the sensor we made, so we set the camera position and look target
        set_camera_view(eye=[-5.00, 5.00, 3.50], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")

        self._events = omni.usd.get_context().get_stage_event_stream()
        self._stage_event_subscription = self._events.create_subscription_to_pop(
            self._on_stage_event, name="LightBeam Sensor Sample Stage Watch"
        )

        (action_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacReadLightBeam", "omni.isaac.sensor.IsaacReadLightBeam"),
                    ("DebugDrawRayCast", "omni.isaac.debug_draw.DebugDrawRayCast"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("IsaacReadLightBeam.inputs:lightbeamPrim", self.sensor_path),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "IsaacReadLightBeam.inputs:execIn"),
                    ("IsaacReadLightBeam.outputs:execOut", "DebugDrawRayCast.inputs:exec"),
                    ("IsaacReadLightBeam.outputs:beamOrigins", "DebugDrawRayCast.inputs:beamOrigins"),
                    ("IsaacReadLightBeam.outputs:beamEndPoints", "DebugDrawRayCast.inputs:beamEndPoints"),
                    ("IsaacReadLightBeam.outputs:numRays", "DebugDrawRayCast.inputs:numRays"),
                ],
            },
        )

        await og.Controller.evaluate(action_graph)
