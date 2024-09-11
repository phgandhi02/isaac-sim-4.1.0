# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
import omni.graph.core as og
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core_nodes import BaseResetNode
from omni.isaac.wheeled_robots.controllers.ackermann_controller import AckermannController
from omni.isaac.wheeled_robots.ogn.OgnAckermannControllerDatabase import OgnAckermannControllerDatabase


class OgnAckermannControllerInternalState(BaseResetNode):
    def __init__(self):
        self.wheel_base = 0.0
        self.track_width = 0.0
        self.turning_wheel_radius = 0.0
        self.max_wheel_velocity = 1.0e20
        self.max_wheel_rotation_angle = 6.28
        self.use_acceleration = False
        self.invert_steering_angle = False
        self.node = None
        self.graph_id = None
        super().__init__(initialize=False)

    def initialize_controller(self) -> None:
        self.controller_handle = AckermannController(
            name="ackermann_controller",
            use_acceleration=self.use_acceleration,
            turning_wheel_radius=self.turning_wheel_radius,
            wheel_base=self.wheel_base,
            track_width=self.track_width,
            invert_steering_angle=self.invert_steering_angle,
            max_wheel_velocity=self.max_wheel_velocity,
            max_wheel_rotation_angle=self.max_wheel_rotation_angle,
        )
        self.initialized = True

    def forward(self, command: np.ndarray) -> ArticulationAction:
        return self.controller_handle.forward(command)

    def custom_reset(self):
        self.node.get_attribute("outputs:leftWheelAngle").set(0.0)
        self.node.get_attribute("outputs:rightWheelAngle").set(0.0)
        self.node.get_attribute("outputs:wheelRotationVelocity").set(0.0)


class OgnAckermannController:
    @staticmethod
    def init_instance(node, graph_instance_id):
        state = OgnAckermannControllerDatabase.get_internal_state(node, graph_instance_id)
        state.node = node
        state.graph_id = graph_instance_id

    @staticmethod
    def internal_state():
        return OgnAckermannControllerInternalState()

    @staticmethod
    def compute(db) -> bool:
        state = db.per_instance_state

        try:
            if not state.initialized:
                state.acceleration = db.inputs.acceleration
                state.wheel_base = db.inputs.wheelBase
                state.track_width = db.inputs.trackWidth
                state.turning_wheel_radius = db.inputs.turningWheelRadius
                state.max_wheel_velocity = db.inputs.maxWheelVelocity
                state.use_acceleration = db.inputs.useAcceleration
                state.invert_steering_angle = db.inputs.invertSteeringAngle
                state.max_wheel_rotation_angle = db.inputs.maxWheelRotation

                state.initialize_controller()

            actions = state.forward(
                [
                    db.inputs.steeringAngle,
                    db.inputs.speed,
                    db.inputs.currentLinearVelocity[0],
                    db.inputs.DT,
                    db.inputs.acceleration,
                ]
            )

            if actions.joint_velocities is not None:
                db.outputs.wheelRotationVelocity = actions.joint_velocities[0]

            if actions.joint_positions is not None:
                db.outputs.leftWheelAngle = actions.joint_positions[2]
                db.outputs.rightWheelAngle = actions.joint_positions[3]

        except Exception as error:
            db.log_warning(str(error))
            return False

        db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True

    @staticmethod
    def release_instance(node, graph_instance_id):
        try:
            state = OgnAckermannControllerDatabase.get_internal_state(node, graph_instance_id)
        except Exception:
            state = None
            pass

        if state is not None:
            state.reset()
            state.initialized = False
