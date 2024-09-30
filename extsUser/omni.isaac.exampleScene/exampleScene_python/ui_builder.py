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
		
		if SimulationContext.instance() is not None:
			self._backend = SimulationContext.instance().backend
			self._device = SimulationContext.instance().device
			self._backend_utils = SimulationContext.instance().backend_utils
		else:
			import omni.isaac.core.utils.numpy as np_utils

			self._backend = "numpy"
			self._device = None
			self._backend_utils = np_utils
		
		world_controls_frame = CollapsableFrame("World Controls", collapsed=False)

		with world_controls_frame:
			with ui.VStack(style=get_style(), spacing=5, height=0):
				self._load_btn = LoadButton(
					"Load Button", "LOAD", setup_scene_fn=self._setup_scene, setup_post_load_fn=self._setup_scenario
				)
				self._load_btn.set_world_settings(physics_dt=1 / 60.0, rendering_dt=1 / 60.0, 
												  backend=self._backend, device= self._device)
				
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
		"""_summary_
		This function is what is used to save the objects from the scene setup. 
		"""        
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
		"""_summary_
		 This function is attached to the Load Button as the setup_scene_fn callback.
		On pressing the Load Button, a new instance of World() is created and then this function is called.
		The user should now load their assets onto the stage and add them to the World Scene.

		In this example, a new stage is loaded explicitly, and all assets are reloaded.
		If the user is relying on hot-reloading and does not want to reload assets every time,
		they may perform a check here to see if their desired assets are already on the stage,
		and avoid loading anything if they are.  In this case, the user would still need to add
		their assets to the World (which has low overhead).  See commented code section in this function.
		"""
		# Load the UR10e
		num_clones = 1
		base_env_path = "/World/env"
		robot_offset = torch.tensor([0.23385,0.30645,0.51141])
		robot_orientation = torch.tensor([1,0,0,0])
		robot_scale = torch.ones(3)
		robot_prim_path = base_env_path + "/ur10e"
		robot_joint_names = ["shoulder_lift_joint", "shoulder_pan_joint", "elbow_joint", "wrist_1_joint",
							 "wrist_2_joint", "wrist_3_joint", "ee_joint"]
		path_to_robot_usd = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
		
		path_to_conveyor_usd = "/home/ise.ros/Documents/AndrewC/isaacSim/conveyor.usd"
		conveyor_prim_path = base_env_path + "/env/conveyor"
		conveyor_scale = torch.tensor([0.70154,0.3024,0.35089])
		
		path_to_teeth_usd = "/home/ise.ros/Documents/AndrewC/isaacSim/teeth_retainer.usd"
		teeth_prim_path = base_env_path + "/env/teeth"

		path_to_robotStand = get_assets_root_path() + "/Isaac/Props/Mounts/Stand/stand_instanceable.usd"
		robotStand_prim_path = base_env_path + "/stand"
		
		create_new_stage()
		# Add user-loaded objects to the World
		world = World.instance()
		
		self._add_light_to_stage()
		world.scene.add_default_ground_plane()
		robotReference = add_reference_to_stage(path_to_robot_usd, robot_prim_path)
		self.ur10 = world.scene.add(Robot(robot_prim_path,
							  name="ur10",
							  position=robot_offset,
							  orientation=robot_orientation,
							  scale=robot_scale,
							  visible=True
							  )
						)
		self.ur10.set_joint_positions(np.array([0,-71.5,64.4,-83.0,251.9,-179.3]))
		add_reference_to_stage(path_to_conveyor_usd, conveyor_prim_path)
		add_reference_to_stage(path_to_robotStand, robotStand_prim_path)
		add_reference_to_stage(path_to_teeth_usd, teeth_prim_path)
		
		# world.scene.add(XFormPrim(prim_path=conveyor_prim_path, 
		#                     scale=conveyor_scale, visible=True))
  
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
							  position=torch.tensor([0,0,0.74331]),
							  orientation=euler_angles_to_quat(torch.tensor([130,0,0])),
							  scale=robot_scale,
							  visible=True
							  )
						)

		# # instantiate the robots
		# cloner = GridCloner(2,num_per_row=1,stage=world.stage)
		# robot_prim_paths = cloner.generate_paths(robot_prim_path,num_clones)
		# robot_offsets = torch.tensor([-2,1,0]).repeat(num_clones,1)
		# robot_positions, robot_orientations = cloner.clone(source_prim_path=robot_prim_path,
		# 	prim_paths=robot_prim_paths,
		# 	position_offsets=robot_offsets,
		# 	base_env_path="/World",
		# 	copy_from_source=True
		# )

		# robots = world.scene.add(RobotView(prim_paths_expr=base_env_path + "/ur10*",
		# 						  name="ur10_robot_view"
		# 						  )
		# 				)
  
		# # instantiate the conveyor belt
		# cloner = GridCloner(2,num_per_row=1,stage=world.stage)
		# conveyor_positions_offset = torch.tensor([-2,0,0]).repeat(num_clones,1)
		# conveyor_positions, conveyor_orientations = cloner.clone(source_prim_path=conveyor_prim_path,
		# 	prim_paths= cloner.generate_paths(conveyor_prim_path,num_clones),
		# 	position_offsets=conveyor_positions_offset,
		# 	base_env_path="/World/env/env",
		# 	copy_from_source=True
		# )
  
		# conveyors = world.scene.add(XFormPrimView(prim_paths_expr=base_env_path + "/env/conveyor*",
		# 								name="conveyorView"
		# 								)
		# 				)
  
		
		# instantiate teeth
		num_clones = 20
		teeth_orientation = euler_angles_to_quat(np.array([110,0,0]))
		teeth_positions_offset = torch.tensor([0,0,0.25]).repeat(num_clones,1)
		teeth_orientation_offset = torch.tensor(teeth_orientation).repeat(num_clones,1)
		cloner = GridCloner(.1,num_per_row=4,stage=world.stage)
		teeth_positions, teeth_orientations = cloner.clone(source_prim_path=teeth_prim_path,
			prim_paths=cloner.generate_paths(teeth_prim_path,num_clones) ,
			position_offsets=random_teeth_position_offset,
			orientation_offsets=random_teeth_orientation_offset,
			base_env_path="/World/env/env",
			copy_from_source=True
		)

		teeths = world.scene.add(XFormPrimView(prim_paths_expr=base_env_path + "/env/teeth*",
										name="teethView"
										)
						)
		# print(robot_positions)
		# print(teeth_positions)
  
		# viewport_api = get_active_viewport()
		# render_product_path = viewport_api.get_render_product_path()
		# camera = Camera(
		# 		prim_path="/World/env/ur10e/tool0/Camera" ,
		# 		position=np.array([0, 0, 0]),
		# 		resolution=resolution,
		# 		orientation=euler_angles_to_quat([0, 0, 0], degrees=True),
		# 		render_product_path=render_product_path,
		# 	)
  
		# omni.kit.commands.execute(
		#         "RemoveReference", stage=world.stage, prim_path=Sdf.Path(robot_prim_path), reference=robotReference
		#     )
		# world.scene.remove_object(robot_prim_path)
  
		# world.scene.add(Robot(robot_prim_path,
		# 					  name="ur10e",
		# 					  position=robot_offset,
		# 					  orientation=robot_orientation,
		# 					  scale=robot_scale,
		# 					  visible=False
		# 					  )
		# 				)
  
		# print("printing poses")
		# print(robots.get_world_poses())
		# print(teeths.get_world_poses())
	
		# print('ext started')
		# get existing value from an attribute
		try:
			import usdrt.Sdf
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
						("ArticulationController.inputs:robotPath", robot_prim_path),
						("PublishJointState.inputs:topicName", "isaac_joint_states"),
						("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
						("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_prim_path)]),
					],
				},
			)
		except Exception as e:
			print(e)


		# print(og.get_graph_by_path("/World/env/env/conveyor_0/ConveyorTrack/ConveyorBeltGraph"))
		# print(og.get_graph_by_path("/ActionGraph"))
		# existing_text = og.Controller.attribute("/action_graph/print.inputs:text").get()
		# print("Existing Text: ", existing_text)

		# # set new value
		# og.Controller.attribute("/action_graph/print.inputs:text").set("New Texts to print")
	
		
		
		

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
		self._scenario.setup_scenario(self.ur10, self.teeths)

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