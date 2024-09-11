import omni.ext
from .. import get_usd_physics_test_interface

from .tests.physicsScene import UsdPhysicsSceneTest
from .tests.collisionGroup import UsdPhysicsCollisionGroupTest
from .tests.collision import UsdPhysicsCollisionTest
from .tests.rigidBody import UsdPhysicsRigidBodyTest
from .tests.fixedJoint import UsdPhysicsFixedJointTest
from .tests.revoluteJoint import UsdPhysicsRevoluteJointTest
from .tests.prismaticJoint import UsdPhysicsPrismaticJointTest
from .tests.distanceJoint import UsdPhysicsDistanceJointTest
from .tests.sphericalJoint import UsdPhysicsSphericalJointTest
from .tests.d6Joint import UsdPhysicsD6JointTest
from .tests.articulation import UsdPhysicsArticulationTest


class PhysicsSchemaTestsExtension(omni.ext.IExt):
    def on_startup(self):
        # Init interface cache
        get_usd_physics_test_interface()

    def on_shutdown(self):
        pass
