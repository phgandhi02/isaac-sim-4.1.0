# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.timeline
import carb
import omni.ui as ui
from omni.isaac.core.simulation_context.simulation_context import SimulationContext
from omni.isaac.core.articulations import Articulation, ArticulationGripper
from omni.isaac.core.robots import Robot, RobotView
from omni.isaac.core.prims import XFormPrim, XFormPrimView

from omni.isaac.sensor import Camera

from omni.isaac.cloner import GridCloner
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage, create_new_stage, get_current_stage
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.world import World
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.ui.element_wrappers import CollapsableFrame, StateButton
from omni.isaac.ui.element_wrappers.core_connectors import LoadButton, ResetButton
from omni.isaac.ui.ui_utils import get_style
from omni.usd import StageEventType
from pxr import Sdf, UsdLux
import torch

from .scenario import ExampleScenario
import omni.graph.core as og
# from omni.isaac.ros2_bridge._ros2_bridge import 

class UIBuilder:
	def __init__(self):
		# Frames are sub-windows that can contain multiple UI elements
		self.frames = []
		# UI elements created using a UIElementWrapper instance
		self.wrapped_ui_elements = []

		# Get access to the timeline to control stop/pause/play programmatically
		self._timeline = omni.timeline.get_timeline_interface()

		# Run initialization for the provided example
		self._on_init()

	###################################################################################
	#           The Functions Below Are Called Automatically By extension.py
	###################################################################################

	def on_menu_callback(self):
		"""Callback for when the UI is opened from the toolbar.
		This is called directly after build_ui().
		"""
		pass

	def on_timeline_event(self, event):
		"""Callback for Timeline events (Play, Pause, Stop)

		Args:
			event (omni.timeline.TimelineEventType): Event Type
		"""
		if event.type == int(omni.timeline.TimelineEventType.STOP):
			# When the user hits the stop button through the UI, they will inevitably discover edge cases where things break
			# For complete robustness, the user should resolve those edge cases here
			# In general, for extensions based off this template, there is no value to having the user click the play/stop
			# button instead of using the Load/Reset/Run buttons provided.
			self._scenario_state_btn.reset()
			self._scenario_state_btn.enabled = False

	def on_physics_step(self, step: float):
		"""Callback for Physics Step.
		Physics steps only occur when the timeline is playing

		Args:
			step (float): Size of physics step
		"""
		pass

	def on_stage_event(self, event):
		"""Callback for Stage Events

		Args:
			event (omni.usd.StageEventType): Event Type
		"""
		if event.type == int(StageEventType.OPENED):
			# If the user opens a new stage, the extension should completely reset
			self._reset_extension()

	def cleanup(self):
		"""
		Called when the stage is closed or the extension is hot reloaded.
		Perform any necessary cleanup such as removing active callback functions
		Buttons imported from omni.isaac.ui.element_wrappers implement a cleanup function that should be called
		"""
		for ui_elem in self.wrapped_ui_elements:
			ui_elem.cleanup()

	def build_ui(self):
		"""
		Build a custom UI tool to run your extension.
		This function will be called any time the UI window is closed and reopened.
		"""
		world_controls_frame = CollapsableFrame("World Controls", collapsed=False)

		with world_controls_frame:
			with ui.VStack(style=get_style(), spacing=5, height=0):
				self._load_btn = LoadButton(
					"Load Button", "LOAD", setup_scene_fn=self._setup_scene, setup_post_load_fn=self._setup_scenario
				)
				self._load_btn.set_world_settings(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
				self.wrapped_ui_elements.append(self._load_btn)

				self._reset_btn = ResetButton(
					"Reset Button", "RESET", pre_reset_fn=None, post_reset_fn=self._on_post_reset_btn
				)
				self._reset_btn.enabled = False
				self.wrapped_ui_elements.append(self._reset_btn)

		run_scenario_frame = CollapsableFrame("Run Scenario")

		with run_scenario_frame:
			with ui.VStack(style=get_style(), spacing=5, height=0):
				self._scenario_state_btn = StateButton(
					"Run Scenario",
					"RUN",
					"STOP",
					on_a_click_fn=self._on_run_scenario_a_text,
					on_b_click_fn=self._on_run_scenario_b_text,
					physics_callback_fn=self._update_scenario,
				)
				self._scenario_state_btn.enabled = False
				self.wrapped_ui_elements.append(self._scenario_state_btn)

	######################################################################################
	# Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
	######################################################################################

	def _on_init(self):
		self._articulation = None
		self._cuboid = None
		self._scenario = ExampleScenario()

	def _add_light_to_stage(self):
		"""
		A new stage does not have a light by default.  This function creates a spherical light
		"""
		sphereLight = UsdLux.SphereLight.Define(get_current_stage(), Sdf.Path("/World/SphereLight"))
		sphereLight.CreateRadiusAttr(2)
		sphereLight.CreateIntensityAttr(100000)
		XFormPrim(str(sphereLight.GetPath())).set_world_pose([6.5, 0, 12])

	def _setup_scene(self):
		"""
		This function is attached to the Load Button as the setup_scene_fn callback.
		On pressing the Load Button, a new instance of World() is created and then this function is called.
		The user should now load their assets onto the stage and add them to the World Scene.

		In this example, a new stage is loaded explicitly, and all assets are reloaded.
		If the user is relying on hot-reloading and does not want to reload assets every time,
		they may perform a check here to see if their desired assets are already on the stage,
		and avoid loading anything if they are.  In this case, the user would still need to add
		their assets to the World (which has low overhead).  See commented code section in this function.
		"""
  
		# ---------------------------------------------------------------------------- #
		#                               Setup Parameters                               #
		# ---------------------------------------------------------------------------- #

		base_env_path = "/World/env"
		robot_offset = torch.tensor([0.23385,0.30645,0.51141])
		robot_orientation = torch.tensor([1,0,0,0])
		robot_scale = torch.ones(3)
		self.robot_prim_path = base_env_path + "/ur10e"
		robot_joint_names = ["shoulder_lift_joint", "shoulder_pan_joint", "elbow_joint", "wrist_1_joint",
							 "wrist_2_joint", "wrist_3_joint", "ee_joint"]
		path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
		
		self.camera_prim_path = self.robot_prim_path + "/panda_hand/geometry/realsense/realsense_camera"


		path_to_conveyor_usd = "/home/ise.ros/Documents/AndrewC/isaacSim/conveyor.usd"
		conveyor_prim_path = base_env_path + "/env/conveyor"
		conveyor_scale = torch.tensor([1.5,0.5,0.5])
		
		path_to_teeth_usd = "/home/ise.ros/Documents/AndrewC/isaacSim/teeth_retainer.usd"
		teeth_prim_path = base_env_path + "/env/teeth"

		path_to_robotStand = get_assets_root_path() + "/Isaac/Props/Mounts/Stand/stand_instanceable.usd"
		robotStand_prim_path = base_env_path + "/stand"

		path_to_stage_scene = get_assets_root_path() + "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
		stage_scene_prim_path = "/World/stage"
		
		# ---------------------------------------------------------------------------- #
		#                                Load in objects                               #
		# ---------------------------------------------------------------------------- #
		create_new_stage()
		# Add user-loaded objects to the World
		world = World.instance()
		
		# Setup scene
		self._add_light_to_stage()
		# world.scene.add_default_ground_plane()
  
		# Load the UR10e
		# robotReference = add_reference_to_stage(path_to_robot_usd, self.robot_prim_path)
		# self.ur10 = world.scene.add(Robot(self.robot_prim_path,
		# 					  name="ur10",
		# 					  position=robot_offset,
		# 					  orientation=robot_orientation,
		# 					  scale=robot_scale,
		# 					  visible=True
		# 					  )
		# 				)
		# self.ur10.set_joint_positions(np.array([0,-71.5,64.4,-83.0,251.9,-179.3]))
		self.create_franka(self.robot_prim_path,(0.23385,0.30645+.2,0.72145))

		# Load in workstation for robot
		add_reference_to_stage(path_to_stage_scene, stage_scene_prim_path)
		add_reference_to_stage(path_to_conveyor_usd, conveyor_prim_path)
		add_reference_to_stage(path_to_robotStand, robotStand_prim_path)
		add_reference_to_stage(path_to_teeth_usd, teeth_prim_path)
		
		realsense: Camera = world.scene.add(Camera(prim_path=self.camera_prim_path))
		# realsense.set_horizontal_aperture(-.6)
		# realsense.set_vertical_aperture(-.2)
  
		world.scene.add(XFormPrim(conveyor_prim_path,
							  name="conveyor",
							  position=torch.tensor([1,0,0]),
							  orientation=torch.tensor([1,0,0,0]),
							  scale=conveyor_scale,
							  visible=True
							  )
						)
		world.scene.add(XFormPrim(teeth_prim_path,
							  name="teeth",
							  position=torch.tensor([0,0,1.673148*.6]),
							  orientation=euler_angles_to_quat(torch.tensor([130,0,0])),
							  scale=robot_scale,
							  visible=True
							  )
						)

		world.scene.add(XFormPrim(robotStand_prim_path,
							  name="stand",
							  position=torch.tensor([0.22771,0.30781+.2,0.72145]),
							  orientation=euler_angles_to_quat(torch.tensor([0,0,0])),
							  scale=torch.tensor([1,1,1]),
							  visible=True
							  )
						)
		
		# instantiate teeth in grid pattern
		num_clones = 3
		teeth_orientation_offset = euler_angles_to_quat(torch.tensor([0,0,0]))
		teeth_positions_offset = torch.tensor([0.2,0,1.67314*.6]).repeat(num_clones,1)
		random_teeth_position_offset = teeth_positions_offset + (-.05-.05)*torch.rand_like(teeth_positions_offset) + .05
		random_teeth_orientation_offset = []#torch.Tensor([1,0,0,0])
		for i in range(num_clones):
			randOrientation = torch.from_numpy(euler_angles_to_quat(100*torch.randn(3),True))
			random_teeth_orientation_offset.append(randOrientation)
   
		cloner = GridCloner(.1,num_per_row=2,stage=world.stage)
		print(len(cloner.generate_paths(teeth_prim_path,num_clones)))
		print(len(random_teeth_orientation_offset))
		teeth_positions, teeth_orientations = cloner.clone(source_prim_path=teeth_prim_path,
			prim_paths=cloner.generate_paths(teeth_prim_path,num_clones) ,
			position_offsets=random_teeth_position_offset,
			orientation_offsets=random_teeth_orientation_offset,
			base_env_path="/World/env/env",
			copy_from_source=True
		)

		self.teeths = world.scene.add(XFormPrimView(prim_paths_expr=base_env_path + "/env/teeth*",
										name="teethView"
										)
						)

		self.create_ros_action_graph(self.robot_prim_path)
		self.create_ros_camera_graph(self.camera_prim_path)
		print('Teeth positions: \n')
		print(teeth_positions)
		print('\nTeeth orientation: \n')
		print(teeth_orientations)
  
	def create_franka(self, stage_path, translation: tuple = (0, -0.64, 0)):
		from pxr import Gf
		import omni.kit.commands

		stage = World.instance().stage
		usd_path = "/Isaac/Robots/Franka/franka_alt_fingers.usd"
		asset_path = get_assets_root_path() + usd_path
		prim = stage.DefinePrim(stage_path, "Xform")
		prim.GetReferences().AddReference(asset_path)
		rot_mat = Gf.Matrix3d(Gf.Rotation((0, 0, 1), 90))
		omni.kit.commands.execute(
			"TransformPrimCommand",
			path=prim.GetPath(),
			old_transform_matrix=None,
			new_transform_matrix=Gf.Matrix4d().SetRotate(rot_mat).SetTranslateOnly(Gf.Vec3d(translation)),
		)
		pass

	def create_ros_action_graph(self, robot_stage_path):
		import usdrt.Sdf
		try:
			og.Controller.edit(
				{"graph_path": "/ActionGraph", "evaluator_name": "execution"},
				{
					og.Controller.Keys.CREATE_NODES: [
						("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
						("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
						("Context", "omni.isaac.ros2_bridge.ROS2Context"),
						("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
						("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
						("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
						("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
					],
					og.Controller.Keys.CONNECT: [
						("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
						("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
						("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
						("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
						("Context.outputs:context", "PublishJointState.inputs:context"),
						("Context.outputs:context", "SubscribeJointState.inputs:context"),
						("Context.outputs:context", "PublishClock.inputs:context"),
						("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
						("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
						("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
						(
							"SubscribeJointState.outputs:positionCommand",
							"ArticulationController.inputs:positionCommand",
						),
						(
							"SubscribeJointState.outputs:velocityCommand",
							"ArticulationController.inputs:velocityCommand",
						),
						("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
					],
					og.Controller.Keys.SET_VALUES: [
						# Setting the /Franka target prim to Articulation Controller node
						("ArticulationController.inputs:robotPath", robot_stage_path),
						("PublishJointState.inputs:topicName", "isaac_joint_states"),
						("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
						("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_stage_path)]),
					],
				},
			)
		except Exception as e:
			print(e)
		pass

	def create_ros_camera_graph(self,camera_prim_path):
		from omni.isaac.ros2_bridge.scripts.og_shortcuts.og_rtx_sensors import Ros2CameraGraph #, Ros2RtxLidarGraph
		camera_graph = Ros2CameraGraph()
		# camera_graph._og_path = camera_graph.og_path_input.get_value()
		camera_graph._camera_prim = camera_prim_path
		# camera_graph._frame_id = camera_graph.frame_id_input.get_value()
		# camera_graph._node_namespace = camera_graph.node_namespace_input.get_value()
		# camera_graph._rgb_topic = camera_graph.rgb_topic_input.get_value()
		# camera_graph._depth_topic = camera_graph.depth_topic_input.get_value()
		# camera_graph._depth_pcl_topic = camera_graph.depth_pcl_topic_input.get_value()
		# camera_graph._instance_topic = camera_graph.instance_topic_input.get_value()
		# camera_graph._semantic_topic = camera_graph.semantic_topic_input.get_value()
		# camera_graph._bbox2d_tight_topic = camera_graph.bbox2d_tight_topic_input.get_value()
		# camera_graph._bbox2d_loose_topic = camera_graph.bbox2d_loose_topic_input.get_value()
		# camera_graph._bbox3d_topic = camera_graph.bbox3d_topic_input.get_value()

		param_check = camera_graph._check_params()
		if param_check:
			camera_graph.make_graph()
		else:
			carb.log_error("Parameter check failed")

	def _setup_scenario(self):
		"""
		This function is attached to the Load Button as the setup_post_load_fn callback.
		The user may assume that their assets have been loaded by their setup_scene_fn callback, that
		their objects are properly initialized, and that the timeline is paused on timestep 0.

		In this example, a scenario is initialized which will move each robot joint one at a time in a loop while moving the
		provided prim in a circle around the robot.
		"""
		self._reset_scenario()

		# UI management
		self._scenario_state_btn.reset()
		self._scenario_state_btn.enabled = True
		self._reset_btn.enabled = True

	def _reset_scenario(self):
		self._scenario.teardown_scenario()
		self._scenario.setup_scenario()

	def _on_post_reset_btn(self):
		"""
		This function is attached to the Reset Button as the post_reset_fn callback.
		The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

		They may also assume that objects that were added to the World.Scene have been moved to their default positions.
		I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
		"""
		self._reset_scenario()

		# UI management
		self._scenario_state_btn.reset()
		self._scenario_state_btn.enabled = True

	def _update_scenario(self, step: float):
		"""This function is attached to the Run Scenario StateButton.
		This function was passed in as the physics_callback_fn argument.
		This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
		When the b_text "STOP" is pressed, the physics callback is removed.

		Args:
			step (float): The dt of the current physics step
		"""
		self._scenario.update_scenario(step)

	def _on_run_scenario_a_text(self):
		"""
		This function is attached to the Run Scenario StateButton.
		This function was passed in as the on_a_click_fn argument.
		It is called when the StateButton is clicked while saying a_text "RUN".

		This function simply plays the timeline, which means that physics steps will start happening.  After the world is loaded or reset,
		the timeline is paused, which means that no physics steps will occur until the user makes it play either programmatically or
		through the left-hand UI toolbar.
		"""
		self._timeline.play()

	def _on_run_scenario_b_text(self):
		"""
		This function is attached to the Run Scenario StateButton.
		This function was passed in as the on_b_click_fn argument.
		It is called when the StateButton is clicked while saying a_text "STOP"

		Pausing the timeline on b_text is not strictly necessary for this example to run.
		Clicking "STOP" will cancel the physics subscription that updates the scenario, which means that
		the robot will stop getting new commands and the cube will stop updating without needing to
		pause at all.  The reason that the timeline is paused here is to prevent the robot being carried
		forward by momentum for a few frames after the physics subscription is canceled.  Pausing here makes
		this example prettier, but if curious, the user should observe what happens when this line is removed.
		"""
		self._timeline.pause()

	def _reset_extension(self):
		"""This is called when the user opens a new stage from self.on_stage_event().
		All state should be reset.
		"""
		self._on_init()
		self._reset_ui()

	def _reset_ui(self):
		self._scenario_state_btn.reset()
		self._scenario_state_btn.enabled = False
		self._reset_btn.enabled = False
