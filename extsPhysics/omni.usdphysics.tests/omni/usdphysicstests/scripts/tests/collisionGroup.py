from .base import UsdPhysicsBaseTest


class UsdPhysicsCollisionGroupTest(UsdPhysicsBaseTest):

    async def test_collision_group(self):
        self.fail_on_log_error = True
        col_group_dict = {}
        self.expected_prims = {}

        # expected collision group
        self.expected_prims["/World/collisionGroupSpheres" + "/collisionGroup"] = col_group_dict
        await self.parse("CollisionGroup")

