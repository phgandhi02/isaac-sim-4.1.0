import asyncio
import carb
import carb.settings
import omni.kit.commands
import omni.physxzerogravity
import omni.physxzerogravity.bindings._physXZeroGravity as pxzerog
import usdrt
from .constants import ColliderTypes
from .payload import ZeroGravityPayload
from .settings import ZeroGravitySettings
from omni.physx import get_physx_cooking_interface

CACHE_COLLIDER_BATCH_SIZE = 100


class ZeroGravitySetEnabledCommand(omni.kit.commands.Command):
    """ Enables zero gravity mode and allows physically correct placement of objects in the scene.

        Parameters:
            enabled:
                If set to true, enables zerogravity, otherwise it disables it.

        Returns:
            None.
    """
    def __init__(self, enabled):
        self._enabled = enabled

    def do(self):
        carb.settings.get_settings().set_bool(pxzerog.SETTINGS_PLACEMENT_MODE_ENABLED, self._enabled)

    def undo(self):
        carb.log_warn("Undo for this operation is not currently supported.")


class ZeroGravityClearAllCommand(omni.kit.commands.Command):
    """ Clears all zerogravity markup.

        Parameters:
            None.

        Returns:
            None.
    """
    def __init__(self):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()

    def do(self):
        self._action_bar.on_clear_all_markers()

    def undo(self):
        carb.log_warn("Undo for this operation is not currently supported.")


class ZeroGravityClearSelectedCommand(omni.kit.commands.Command):
    """ Clear zerogravity markup from the selected prim.

        Parameters:
            None.

        Returns:
            None.
    """
    def __init__(self):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()

    def do(self):
        self._action_bar.on_clear_button_clicked()

    def undo(self):
        carb.log_warn("Undo for this operation is not currently supported.")


class ZeroGravityFlushAllChanges(omni.kit.commands.Command):
    """
        Flushes all ZeroG changes and makes sure whatever was requested is applied before proceeding (blocking call)

        Parameters:
            None.

        Returns:
            None.
    """
    def __init__(self):
        assert ZeroGravityPayload.placement is not None

    def do(self):
        ZeroGravityPayload.placement._flush_all_changes()

    def undo(self):
        carb.log_warn("Undo for this operation is not supported.")


class ZeroGravityCacheAllCommand(omni.kit.commands.Command):
    """ Build all collision model forms for all UsdGeomGprims in the stage.
        Note: This is an async command, the caller needs to await the initial return
        and then call .result() on it to get the true return value of the command

        Parameters:
            None.

        Returns:
            True on success, False on failure.
    """
    def __init__(self):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()

        # prims will have colliders build and cached for all permutations defined here
        self._command_map = {
            "ZeroGravitySetSelectedStaticCommand": {
                "simplification_type": ZeroGravitySettings.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE,
                "mode": [
                    ColliderTypes.STATIC_COLLIDER_SIMPLIFICATION_TYPE_TRIANGLE_MESH,
                    ColliderTypes.STATIC_COLLIDER_SIMPLIFICATION_TYPE_MESH_SIMPLIFICATION,
                ],
            },
            "ZeroGravitySetSelectedDynamicCommand": {
                "simplification_type": ZeroGravitySettings.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE,
                "mode": [
                    ColliderTypes.DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE_CONVEX_HULL,
                    ColliderTypes.DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE_CONVEX_DECOMPOSITION,
                ],
            },
        }

    async def _cache_colliders(self, cmd):
        physx_zero_gravity = omni.physxzerogravity.get_physx_zero_gravity_interface()
        cooking_iface = get_physx_cooking_interface()

        # run the command to force colliders to be built, give it time to flush, then clear the settings
        omni.kit.commands.execute(cmd)
        await omni.kit.app.get_app().next_update_async()
        physx_zero_gravity.force_all_changes_to_be_flushed()
        await omni.kit.app.get_app().next_update_async()
        while cooking_iface.get_num_collision_tasks():
            await omni.kit.app.get_app().next_update_async()
        omni.kit.commands.execute("ZeroGravityClearSelectedCommand")
        await omni.kit.app.get_app().next_update_async()

    async def _internal_do(self):
        # make sure the basic pieces are in place
        usdrtStage = usdrt.Usd.Stage.Attach(omni.usd.get_context().get_stage_id())
        selection = omni.usd.get_context().get_selection()
        settings = carb.settings.get_settings()
        if not usdrtStage or not selection or not settings:
            carb.log_warn("Failed to get stage, selection, or settings.")
            return False

        # spin through and force batches of UsdGeomGprims to get colliders built for them
        primPaths = usdrtStage.GetPrimsWithTypeName("UsdGeomGprim")
        numPrimPaths = len(primPaths)
        if numPrimPaths == 0:
            return True

        # cache original values so they can be restored after the command runs
        orig_selected_paths = selection.get_selected_prim_paths()
        orig_static_mode = settings.get_as_int(ZeroGravitySettings.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE)
        orig_dynamic_mode = settings.get_as_int(ZeroGravitySettings.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE)

        # ensure ZG is enabled and sweep mode is not
        omni.kit.commands.execute("ZeroGravitySetEnabledCommand", enabled=True)
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=False)
        await omni.kit.app.get_app().next_update_async()

        idx = 0
        for cmd in self._command_map:
            simplification_type = self._command_map[cmd]["simplification_type"]
            for mode in self._command_map[cmd]["mode"]:
                # update the settings used by the UI when building colliders for selected prims
                settings.set(simplification_type, mode)

                while idx < numPrimPaths:
                    # convert from SdfPath to string for set_selected_prim_paths()
                    selected_paths = [path.GetString() for path in primPaths[idx:idx + CACHE_COLLIDER_BATCH_SIZE]]
                    idx += CACHE_COLLIDER_BATCH_SIZE

                    # the logic operates on selected prims, so set the selection and call the process function
                    selection.set_selected_prim_paths(selected_paths, True)
                    await omni.kit.app.get_app().next_update_async()
                    await self._cache_colliders(cmd)

        # restore the cached values
        selection.set_selected_prim_paths(orig_selected_paths, True)
        settings.set(ZeroGravitySettings.SETTINGS_STATIC_COLLIDER_SIMPLIFICATION_TYPE, orig_static_mode)
        settings.set(ZeroGravitySettings.SETTINGS_DYNAMIC_COLLIDER_SIMPLIFICATION_TYPE, orig_dynamic_mode)

        return True

    def do(self):
        # work is done async over multiple frames
        return asyncio.ensure_future(self._internal_do())


class ZeroGravitySetSelectedStaticCommand(omni.kit.commands.Command):
    """ Applies static zerogravity markup to the selected prim.

        Parameters:
            None.

        Returns:
            None.
    """
    def __init__(self):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()

    def do(self):
        self._action_bar.on_static_button_clicked()

    def undo(self):
        carb.log_warn("Undo for this operation is not currently supported.")


class ZeroGravitySetSelectedDynamicCommand(omni.kit.commands.Command):
    """ Applies dynamic zerogravity markup to the selected prim.

        Parameters:
            None.

        Returns:
            None.
    """
    def __init__(self):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()

    def do(self):
        self._action_bar.on_dynamic_button_clicked()

    def undo(self):
        carb.log_warn("Undo for this operation is not currently supported.")


class ZeroGravitySetDroppingCommand(omni.kit.commands.Command):
    """ Enables or disables dropping mode for selected zerogravity dynamic marked prims.

        Parameters:
            dropping:
                If set to true, selected zerogravity dynamic markup assets will start falling down.
                Set to false, disables zerogravity dropping mode.

        Returns:
            None.
    """
    def __init__(self, dropping):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()
        self._dropping = dropping

    def do(self):
        self._action_bar.set_dropping(self._dropping)

    def undo(self):
        carb.log_warn("Undo for this operation is not currently supported.")


class ZeroGravitySetSweepModeCommand(omni.kit.commands.Command):
    """ Enables or disables automatic sweeping mode for zerogravity prims and automatically marks prims around the
        selected prims as static or dynamic.

        Parameters:
            sweep_mode:
                If set to true, enables sweep mode. Otherwise disables it.

        Returns:
            None.
    """
    def __init__(self, sweep_mode):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()
        self._sweep_mode = sweep_mode

    def do(self):
        self._action_bar.set_sweep_mode(self._sweep_mode)

    def undo(self):
        self._action_bar.set_sweep_mode(not self._sweep_mode)


class ZeroGravityMarkSweepItemsDynamicCommand(omni.kit.commands.Command):
    """ Sets whether to use dynamic markers for zerogravity automatic sweep mode.

        Parameters:
            use_dynamic_markers_for_swept_items:
                If set to true, prims nearby to the selection will be marked as dynamic. Otherwise as static.

        Returns:
            None.
    """
    def __init__(self, use_dynamic_markers_for_swept_items):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()
        self._use_dynamic_markers_for_swept_items = use_dynamic_markers_for_swept_items

    def do(self):
        self._action_bar.set_swept_items_dynamic(self._use_dynamic_markers_for_swept_items)

    def undo(self):
        self._action_bar.set_swept_items_dynamic(not self._use_dynamic_markers_for_swept_items)


class ZeroGravitySweepAreaVisualizeCommand(omni.kit.commands.Command):
    """ Show or hide a sweep area bounding box visualization.

        Parameters:
            visualize_aabb:
                If set to true, a bounding box for the current sweep area will be shown in the viewport.

        Returns:
            None.
    """
    def __init__(self, visualize_aabb):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()
        self._visualize_aabb = visualize_aabb

    def do(self):
        self._action_bar.sweep_area_visualize(self._visualize_aabb)

    def undo(self):
        self._action_bar.sweep_area_visualize(not self._visualize_aabb)

class ZeroGravityActivateSimreadyDnDIntegrationCommand(omni.kit.commands.Command):
    """ Enable or disable ZeroGravity integration with Simready drag and drop.

        Parameters:
            activate_on_drop:
                If set to true, zerogravity will automatically activate when dropping Simready assets.

        Returns:
            None.
    """
    def __init__(self, activate_on_drop):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()
        self._activate_on_drop = activate_on_drop

    def do(self):
        self._action_bar.zerog_simready_dnd_integration_enable(self._activate_on_drop)

    def undo(self):
        self._action_bar.zerog_simready_dnd_integration_enable(not self._activate_on_drop)


class ZeroGravityRestoreAllTransformsCommand(omni.kit.commands.Command):
    """ Restores all original transforms undoing zerogravity applied transforms.

        Parameters:
            None.

        Returns:
            None.
    """
    def __init__(self):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()

    def do(self):
        self._action_bar.on_restore_all_transforms()

    def undo(self):
        carb.log_warn("Undo for this operation is not currently supported.")


class ZeroGravityWaitForSimulationStepCompletedCommand(omni.kit.commands.Command):
    """ Blocks until this simulation step is completed. Call this before reading out simulation statistics
        (e.g. in unit tests) otherwise you'll read garbage data still in simulation and get errors.

        Parameters:
            None.

        Returns:
            None.
    """
    def __init__(self):
        assert ZeroGravityPayload.placement is not None
        self._action_bar = ZeroGravityPayload.placement.get_action_bar()

    def do(self):
        self._action_bar.on_wait_for_simulation_step_completed()

    def undo(self):
        carb.log_warn("Undo for this operation is not currently supported.")


def register_commands():
    ZeroGravityPayload.cmds = omni.kit.commands.register_all_commands_in_module(__name__)
