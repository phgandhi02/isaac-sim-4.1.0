# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from math import radians

import numpy as np
import omni
import omni.graph.core as og
import omni.physics.tensors
import omni.physx as _physx
from omni.isaac.surface_gripper._surface_gripper import Surface_Gripper, Surface_Gripper_Properties
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade


class SurfaceGripperInternalState:
    def __init__(self):
        self._surface_gripper = Surface_Gripper()
        self._initialized = False
        self._sgp = Surface_Gripper_Properties()

        self._physxIFace = _physx.acquire_physx_interface()
        self._physx_subscription = None
        self._timeline = omni.timeline.get_timeline_interface()
        self._parent_node = None
        self.closed = False
        self.broken = False
        self.attempt_close = False

    def __del__(self):
        self._timeline_event_sub = None
        self._physx_subscription = None

    def _on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            print("stopping")
            self.open()
            self._initialized = False
            self._timeline_event_sub = None
            self._physx_subscription = None

    def _on_physics_step(self, step):
        self.update()

    def open(self):
        if self.update():
            self._surface_gripper.open()
            return False

    def close(self, node):
        if not self.update():
            self.closed = self._surface_gripper.close()
            if self.closed or node.get_attribute("inputs:RetryClose").get():
                stream = self._timeline.get_timeline_event_stream()
                self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)
                self._parent_node = node
        update = self.update()
        return update

    def update(self):
        if self._initialized:
            self._surface_gripper.update()
            is_closed = self._surface_gripper.is_closed()
            if not is_closed and self.attempt_close:
                self._surface_gripper.close()
                is_closed = self._surface_gripper.is_closed()
            if self.closed and not is_closed:
                self.closed = False
                self.broken = True
                if self._parent_node:
                    # print(dir(self._parent_node.get_attribute("inputs:onStep")))
                    # print(self._parent_node.get_attribute("inputs:onStep").get_upstream_connections())
                    for attr in self._parent_node.get_attribute("inputs:onStep").get_upstream_connections():
                        node = attr.get_node()
                        # print(dir(node))
                        impulse = node.get_attribute("state:enableImpulse")
                        if impulse:
                            impulse.set(True)
                            node.request_compute()
                    # self._parent_node.get_attribute("outputs:GripBroken").update_attribute_value(True)
                    # self._parent_node.get_attribute("outputs:Closed").update_attribute_value(False)
                    # self._parent_node.get_attribute("inputs:onStep").update_attribute_value(True)
                    # self._parent_node.request_compute()
            if not is_closed:
                self._timeline_event_sub = None
                self._physx_subscription = None
            return is_closed
        return False

    def initialize(self):
        if not self._initialized or not self._surface_gripper.is_closed():
            self._surface_gripper.initialize(self._sgp)
            self._initialized = True

    def open(self):
        if self._initialized:
            self._surface_gripper.open()
            self.attempt_close = False
            self.closed = False
            self._timeline_event_sub = None
            self._physx_subscription = None


class OgnSurfaceGripper:
    @staticmethod
    def internal_state():
        return SurfaceGripperInternalState()

    @staticmethod
    def compute(db) -> bool:
        if db.inputs.enabled and len(db.inputs.ParentRigidBody) > 0 and len(db.inputs.GripPosition) > 0:
            stage = omni.usd.get_context().get_stage()
            parent = stage.GetPrimAtPath(db.inputs.ParentRigidBody[0].GetString())
            if parent:

                # update internal state properties
                grip_point = stage.GetPrimAtPath(db.inputs.GripPosition[0].GetString())
                db.per_instance_state._sgp.parentPath = str(parent.GetPath())
                db.per_instance_state._sgp.d6JointPath = db.per_instance_state._sgp.parentPath + "/d6FixedJoint"
                db.per_instance_state._sgp.gripThreshold = db.inputs.GripThreshold
                db.per_instance_state._sgp.forceLimit = db.inputs.ForceLimit
                db.per_instance_state._sgp.torqueLimit = db.inputs.TorqueLimit
                db.per_instance_state._sgp.bendAngle = radians(db.inputs.BendAngle)
                db.per_instance_state._sgp.stiffness = db.inputs.Stiffness
                db.per_instance_state._sgp.damping = db.inputs.Damping
                db.per_instance_state._sgp.disableGravity = db.inputs.DisableGravity
                db.per_instance_state._sgp.retryClose = db.inputs.RetryClose
                # compute offset between parent and gripping point
                parent_pose = omni.usd.get_world_transform_matrix(parent)
                grip_pose = omni.usd.get_world_transform_matrix(grip_point)
                offset = grip_pose * parent_pose.GetInverse()
                tr = omni.physics.tensors.Transform()
                t = offset.ExtractTranslation()
                q = offset.ExtractRotationQuat()
                tr.p = [t[0], t[1], t[2]]
                qr = q.GetImaginary()
                tr.r = [qr[0], qr[1], qr[2], q.GetReal()]

                db.per_instance_state._sgp.offset = tr
                db.per_instance_state.initialize()
                db.outputs.GripBroken = db.per_instance_state.broken
                db.per_instance_state.broken = False
                db.outputs.closed = db.per_instance_state._surface_gripper.is_closed()

                if db.inputs.Close:
                    db.outputs.Closed = db.per_instance_state.close(db.node)
                if db.inputs.Open:
                    db.outputs.Closed = db.per_instance_state.open()
            db.outputs.Closed = db.per_instance_state.update()
        else:
            db.per_instance_state.open()
