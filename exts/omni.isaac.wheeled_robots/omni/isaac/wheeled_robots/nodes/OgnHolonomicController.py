# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import numpy as np
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.ogn.OgnHolonomicControllerDatabase import OgnHolonomicControllerDatabase


class OgnHolonomicControllerInternalState(BaseResetNode):
    def __init__(self):
        self.wheel_radius = [0.0]
        self.wheel_positions = np.array([])
        self.wheel_orientations = np.array([])
        self.mecanum_angles = [0.0]
        self.wheel_axis = np.array([1.0, 0, 0])
        self.up_axis = np.array([0, 0, 1])
        self.controller_handle = None
        self.max_linear_speed = 1.0e20
        self.max_angular_speed = 1.0e20
        self.max_wheel_speed = 1.0e20
        self.linear_gain = 1.0
        self.angular_gain = 1.0
        self.node = None
        self.graph_id = None
        super().__init__(initialize=False)

    def initialize_controller(self) -> None:
        self.controller_handle = HolonomicController(
            name="holonomic_controller",
            wheel_radius=np.asarray(self.wheel_radius),
            wheel_positions=np.asarray(self.wheel_positions),
            wheel_orientations=np.asarray(self.wheel_orientations),
            mecanum_angles=np.asarray(self.mecanum_angles),
            wheel_axis=self.wheel_axis,
            up_axis=self.up_axis,
            max_linear_speed=self.max_linear_speed,
            max_angular_speed=self.max_angular_speed,
            max_wheel_speed=self.max_wheel_speed,
            linear_gain=self.linear_gain,
            angular_gain=self.angular_gain,
        )
        self.initialized = True

    def forward(self, command: np.ndarray) -> ArticulationAction:
        return self.controller_handle.forward(command)

    def custom_reset(self):
        if self.initialized:
            self.node.get_attribute("inputs:inputVelocity").set([0, 0, 0])
            self.node.get_attribute("outputs:jointVelocityCommand").set([0, 0, 0])


class OgnHolonomicController:
    """
    nodes for moving an articulated robot with joint commands
    """

    @staticmethod
    def init_instance(node, graph_instance_id):
        state = OgnHolonomicControllerDatabase.get_internal_state(node, graph_instance_id)
        state.node = node
        state.graph_id = graph_instance_id

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnHolonomicControllerDatabase.get_internal_state(node, graph_instance_id)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
            state.initialized = False

    @staticmethod
    def internal_state():
        return OgnHolonomicControllerInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state

        try:
            if not state.initialized:
                state.wheel_radius = db.inputs.wheelRadius
                state.wheel_positions = db.inputs.wheelPositions
                state.wheel_orientations = db.inputs.wheelOrientations
                state.mecanum_angles = db.inputs.mecanumAngles
                state.wheel_axis = db.inputs.wheelAxis
                state.up_axis = db.inputs.upAxis
                state.max_linear_speed = db.inputs.maxLinearSpeed
                state.max_angular_speed = db.inputs.maxAngularSpeed
                state.max_wheel_speed = db.inputs.maxWheelSpeed
                state.linear_gain = db.inputs.linearGain
                state.angular_gain = db.inputs.angularGain

                state.initialize_controller()

            joint_actions = state.forward(np.array(db.inputs.inputVelocity))

            if joint_actions.joint_velocities is not None:
                db.outputs.jointVelocityCommand = joint_actions.joint_velocities

        except Exception as error:
            db.log_warning(str(error))
            return False

        return True
