# Copyright (c) 2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import json
import os
import string

import carb
import omni.kit.app
import omni.kit.commands
import omni.kit.undo
import omni.physxzerogravity.bindings._physXZeroGravity as pxzerog
import omni.usd
from omni.kit.viewport.window.dragdrop.scene_drop_delegate import SceneDropDelegate
from pxr import Gf, Tf, Usd, UsdPhysics

SIMREADY_DRAG_PREFIX = "SimReady::"


# A ZeroGravity drag and drop handler for SimReady assets (i.e. assets dropped from the simready browser)
# so that we can activate ZeroGravity immediately for dropped simready assets and have physically correct placement
class ZeroGravityDndHelper(SceneDropDelegate):
    def __init__(self, preview_setting: str = None, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # To intercept dragged Simready assets we use a rather involved system: we register a Usd.Notice.ObjectsChanged
        # handler and we intercept the modified prim by comparing its default sdf path name (we rely on the fact that
        # this stuff is all serial - no other changes should occur inbetween), then we deregister the handler and
        # get the final prim path that we can use to activate zerog on it
        self.__drag_to_check = False
        self.__notice_listener = None
        self._variants = None
        self.__variantset_listener = None
        self.__resolve_path = None
        self.__url_prim_name = None
        self._activate_zg_sub = None
        self._settings = carb.settings.get_settings()
        self._zerog_simready_dnd_enabled = self._settings.get_as_bool(pxzerog.SETTINGS_ZEROG_SIMREADY_DND_ENABLED)
        self._settings_subs = []
        self._settings_subs.append(omni.kit.app.SettingChangeSubscription(
            pxzerog.SETTINGS_ZEROG_SIMREADY_DND_ENABLED, self._on_zerog_simready_dnd_enabled_setting_changed
        ))

    def __del__(self):
        self._settings_subs = []

    def _on_zerog_simready_dnd_enabled_setting_changed(self, item, event_type):
        """ Event handler for zerogravity drag and drop of simready assets enabled setting changed"""
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._zerog_simready_dnd_enabled = self._settings.get_as_bool(pxzerog.SETTINGS_ZEROG_SIMREADY_DND_ENABLED)

    def reset_state(self):
        ret = super().reset_state()
        self.__drag_to_check = False
        if self.__notice_listener:
            self.__notice_listener.Revoke()
            self.__notice_listener = None
        if self.__variantset_listener:
            self.__variantset_listener.Revoke()
            self.__variantset_listener = None
        self.__url_prim_name = None
        self.__resolve_path = None
        return ret

    # Method to allow subclassers to test url and keep all other default behavior
    def ignore_url(self, url: str):
        # Validate it's a USD file and there is a Stage or UsdContet to use
        if not omni.usd.is_usd_readable_filetype(url):
            return True

        # Early out for other .usda handlers that claim the protocol
        if super().is_ignored_protocol(url):
            return True

        if super().is_ignored_extension(url):
            return True

        return False

    def get_raw_data(self, drop_data: dict) -> dict:
        return json.loads(drop_data.get("mime_data")[len(SIMREADY_DRAG_PREFIX):])

    # Helper methods to inspect drop_data. Returns a tuple consisting of the asset URL and the requested variant (if any)
    def get_url_and_variant(self, drop_data: dict):
        mime_data = drop_data.get('mime_data')
        url = None
        variant = None
        rawdata_dict = self.get_raw_data(drop_data)
        if mime_data.startswith(SIMREADY_DRAG_PREFIX):
            url = rawdata_dict["url"]
        else:
            url = mime_data

        if "variants" in rawdata_dict:
            variant = rawdata_dict["variants"]

        return (url, variant)

    # let this handler also handle the file being dropped together with the other handlers
    def accepted(self, drop_data: dict):
        # Reset super's state
        self.reset_state()

        # Check if this functionality is disabled
        if not self._zerog_simready_dnd_enabled:
            return False

        # Note that _variants should always be a valid dictionary
        (url, self._variants) = self.get_url_and_variant(drop_data)
        if self.ignore_url(url):
            return False

        usd_context, stage = self.get_context_and_stage(drop_data)
        if not usd_context:
            return False

        # Validate not dropping a cyclical reference
        if stage:
            root_layer = stage.GetRootLayer()
            if url == self.normalize_sdf_path(root_layer.realPath):
                return False

            # valid drop - we'll check this path in the usd changed handler
            self.__drag_to_check = True
            url = self.get_url(drop_data)
            self.__url_prim_name = str(Tf.MakeValidIdentifier(os.path.basename(os.path.splitext(url)[0])))
            self.__notice_listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_objects_changed, None)

        return True

    # Listener to check for USD changed objects from simready assets
    # that we're tracking and get the right path where they've been dropped
    def _on_objects_changed(self, notice, sender):
        if not self.__url_prim_name:
            return

        if self.__resolve_path:
            return


        cleaned = self.__url_prim_name.rstrip(string.digits)
        for path in notice.GetResyncedPaths():
            if path.IsPrimPath():
                if cleaned in str(path):
                    self.__resolve_path = path
                    if self.__drag_to_check:
                        # ZG drag and drop handler processing dropped file

                        if "PhysicsVariant" in self._variants and self._variants["PhysicsVariant"] != "":
                            # We have a Physics variant - we will defer activating ZG to when the right variant asset is
                            # spawned and to when the Physics schemas are applied
                            self.__variantset_listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_objects_changed_vset, None)
                        else:
                            # No Physics variant detected - we can activate ZG immediately
                            if self._activate_zg_sub is None:
                                self._activate_zg_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._activate_zg_subscription, name="zerogravity dnd handler - activate zerogravity")

                    self.__drag_to_check = False

    # Variant sets are applied later (e.g. a variant with a rigid body API for an asset): this listener
    # waits for a structural change in the tracked prim, checks if the physically-enabled variant was enabled
    # and activates ZG later. This is necessary to avoid activating ZG *before* the right asset variant has been deployed.
    def _on_objects_changed_vset(self, notice, sender):
        if not self.__resolve_path:
            return

        if not self._variants:
            return

        for path in notice.GetResyncedPaths():
            if path.IsPrimPath() and path == self.__resolve_path:
                # Analyzing our tracked prim for potential Physics variant changes..
                stage = omni.usd.get_context().get_stage()
                prim = stage.GetPrimAtPath(path)
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    # We finally have the final variant-enabled prim with the RB API, activate ZG at next update loop
                    # we don't do it here for performance reasons (keep notice handlers lean)
                    if self._activate_zg_sub is None:
                        self._activate_zg_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._activate_zg_subscription, name="zerogravity dnd handler - activate zerogravity")
                    self.__variantset_listener.Revoke()
                    self.__variantset_listener = None
                    return

    def add_drop_marker(self, drop_data: dict, world_space_pos: Gf.Vec3d):
        # we dont want any drop marker here
        return

    # Called when a drop is finalized - we activate zerog here on that prim!
    def dropped(self, drop_data: dict):
        super().dropped(drop_data)

    def _activate_zg_subscription(self, event):
        self._activate_zg_sub = None  # oneshot
        omni.kit.commands.execute("ZeroGravitySetEnabledCommand", enabled=True)
        omni.kit.commands.execute("ZeroGravitySetSweepModeCommand", sweep_mode=True)
        omni.usd.get_context().get_selection().set_selected_prim_paths([str(self.__resolve_path)], True)

    def cancel(self, drop_data):
        self.reset_state()
