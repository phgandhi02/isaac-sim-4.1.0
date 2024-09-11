import omni.kit.app


def is_rigid_body_placement_prim(prim):
    from pxr import UsdPhysics
    return is_placement_prim(prim) and prim.HasAPI(UsdPhysics.RigidBodyAPI)


def is_placement_prim(prim):
    from pxr import UsdGeom
    return prim.IsA(UsdGeom.Xformable)


async def ui_wait(updates_cnt=10):
    for _ in range(updates_cnt):
        await omni.kit.app.get_app().next_update_async()


def refresh_manipulator_selector():
    # Note: this method can be used to force-refresh the manipulator but also causes
    # an 'enable = True' to be sent (hence re-activating any manipulator, even if hidden)
    from omni.kit.manipulator.selector import get_manipulator_selector
    get_manipulator_selector("")._refresh()
