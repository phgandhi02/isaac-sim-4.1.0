import omni.ext
import carb
import carb.profiler
from omni import ui
from omni.physxpvd.bindings import _physxPvd
import omni.usd
from pprint import pprint
from datetime import datetime
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf, UsdLux
import os, sys
import ctypes
from enum import IntEnum
from omni.timeline import get_timeline_interface
from omni.kit.widget.stage.stage_icons import StageIcons
import omni.kit.app
import asyncio

import omni.kit.tool.asset_importer as ai
from typing import List, Union, Dict

from omni.physxuicommon import windowmenuitem
from omni.physx.scripts.utils import safe_import_tests

from omni.kit.widget.settings import SettingsWidgetBuilder
from omni.kit.widget.settings.settings_widget import SettingType, create_setting_widget, create_setting_widget_combo
from omni.kit.viewport.utility import frame_viewport_selection, get_active_viewport
from omni.kit.window.filepicker import FilePickerDialog

from omni.physxpvd.scripts.property_widget.property_widget_omni_pvd import PropertyWidgetOmniPvd
from omni.physxpvd.scripts.property_widget.prim_handlers.default import prim_handler as default_prim_handler

from omni.physx import get_physx_interface
from omni.physx.bindings._physx import SimulationEvent

import omni.kit.viewport.utility as vp_utils

from omni.physxpvd.scripts.omniusd_to_physxusd.omniusd_to_physxusd import ConvertOmniPvdToPhysXUSD

from omni.physx.bindings._physx import (
    SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY,
    SETTING_OMNIPVD_IS_OVD_STAGE,
    SETTING_OMNIPVD_IS_RECORDING,
    SETTING_OMNIPVD_ENABLED
)
from omni.physxpvd.bindings._physxPvd import (
    SETTING_OMNIPVD_IMPORTED_OVD,
    SETTING_OMNIPVD_OVD_FOR_BAKING,
    SETTING_OMNIPVD_USD_CACHE_DIRECTORY,
    SETTING_OMNIPVD_PHYSX_USD_DIRECTORY,
    SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY,
    SETTING_OMNIPVD_INVALIDATE_CACHE,
    SETTING_OMNIPVD_GIZMO_CONTACT_VIZMODE,
    SETTING_OMNIPVD_GIZMO_CENTER_OF_MASS_VIZMODE,
    SETTING_OMNIPVD_GIZMO_JOINT_VIZMODE,
    SETTING_OMNIPVD_GIZMO_BOUNDING_BOX_VIZMODE,
    SETTING_OMNIPVD_GIZMO_COORDINATE_SYSTEM_VIZMODE,
    SETTING_OMNIPVD_GIZMO_VELOCITY_VIZMODE,
    SETTING_OMNIPVD_GIZMO_TRANSPARENCY_VIZMODE,
    SETTING_OMNIPVD_GIZMO_GLOBAL_SCALE,
    SETTING_OMNIPVD_GIZMO_CONTACT_SCALE,
    SETTING_OMNIPVD_GIZMO_CENTER_OF_MASS_SCALE,
    SETTING_OMNIPVD_GIZMO_JOINT_SCALE,
    SETTING_OMNIPVD_GIZMO_COORDINATE_SYSTEM_SCALE,
    SETTING_OMNIPVD_GIZMO_VELOCITY_SCALE,
    SETTING_OMNIPVD_GIZMO_TRANSPARENCY_SCALE
)

from omni.kit.widget.settings.settings_widget import SettingType
from omni.kit.widget.settings.settings_model import SettingModel

DISABLED_STYLE = {"color": 0xff888888}

################################################################################
# Mirrors this from the C++ code
################################################################################
#struct OmniPVDDebugGizmoType
#{
#    enum Enum
#    {
#        eBoundingBox,
#        eContact,
#        eCenterOfMass,
#        eVelocity,
#        eCoordinateSystem,
#        eJoint,
#        eTransparency,
#        eNbrEnums
#    };
#};
################################################################################

class OmniPVDGizmoType(IntEnum): 
    eBoundingBox = 0
    eContact = 1
    eCenterOfMass = 2
    eVelocity = 3
    eCoordinateSystem = 4
    eJoint = 5
    eTransparency = 6

import datetime

def cleanup_fp_dialog(fp_dialog):
    async def cleanup_async(fp_dialog):
        await omni.kit.app.get_app().next_update_async()
        fp_dialog.destroy()
    asyncio.ensure_future(cleanup_async(fp_dialog))

def set_usd_cache_dir():
    def on_choose(fp_dialog, filename, dirpath):
        fp_dialog.hide()
        if (dirpath):
            if not dirpath.endswith("/"):
                dirpath += "/"
            carb.settings.get_settings().set(SETTING_OMNIPVD_USD_CACHE_DIRECTORY, dirpath)                
        cleanup_fp_dialog(fp_dialog)

    item_filter_options = ["OVD Files (*.ovd)"]
    fp_dialog = FilePickerDialog(
        "Select the USD Cache Directory",
        apply_button_label="Select Current",
        click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
        item_filter_options=item_filter_options,
    )
    fp_dialog.show()

def getStageUpAxisAsInt():
    if not omni.usd.get_context():
        return False, -1
    stage = omni.usd.get_context().get_stage()
    if not stage:
        return False, -1
    up_axis_from_stage = UsdGeom.GetStageUpAxis(stage)
    if (up_axis_from_stage == "Y"):
        return True, 0
    if (up_axis_from_stage == "Z"):
        return True, 1
    return False, -1

def _build_checkbox(*args, **kwargs):
    with ui.VStack():
        ui.Spacer()
        cb = ui.CheckBox(*args, **kwargs, height=0)
        ui.Spacer()
    return cb

class Item(ui.AbstractItem):
    def __init__(self, prim):
        super().__init__()
        self._pvd_node = OmniPvdNode(prim, self)

class OmniPvdNode():
    def __init__(self, prim, item):
        prim_path = str(prim.GetPath())
        self._prim_name = prim_path.split("/")[-1]
        self._node_name = prim_path
        self._all_children = None
        self._found_in_search = False
        self._item = item
        self._highlighted =  False

    def get_time(self):
        timeline = get_timeline_interface()
        return timeline.get_current_time() * timeline.get_time_codes_per_seconds()
   
    def _build_all_children(self, stage):
        if not self._all_children:
            if stage:
                ancestorPrim = stage.GetPrimAtPath(self._node_name)
                if ancestorPrim:
                    self._all_children = []
                    display_predicate = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
                    children_iterator = ancestorPrim.GetFilteredChildren(display_predicate)
                    for child_prim in children_iterator:
                        if child_prim.HasAttribute("omni:pvdi:class"):
                            self._all_children.append( Item(child_prim) )

    def get_child_item(self, prim_name, stage):
        if not self._all_children:
            self._build_all_children(stage)
        for child_item in self._all_children:
            # the pvd_node contains the "payload data"
            pvd_node = child_item._pvd_node
            if pvd_node._prim_name == prim_name:
                return child_item
        return None

    def get_current_visibile_children(self, stage):
        if not self._all_children:
            self._build_all_children(stage)
        visible_children = []
        if stage:
            for childItem in self._all_children:
                prim = stage.GetPrimAtPath(childItem._pvd_node._node_name)
                if prim:
                    # if the prim is below /shared, don't check for visibility
                    if not childItem._pvd_node._node_name.startswith('/Shared'):
                        visibility_attribute = prim.GetAttribute('visibility')
                        if not visibility_attribute:
                            continue
                        if visibility_attribute.Get(self.get_time()) != 'inherited':
                            continue
                    visible_children.append(childItem)
        return visible_children                

class Model(ui.AbstractItemModel):
    def __init__(self):
        super().__init__()
        self.reset()

    def reset(self):
        self._pvd_node = None
        self._found_pvd_nodes = []

    def create_item_chain_and_expand(self, split_prim_names, stage, tree, is_for_search = True):
        if not self._pvd_node:
            self.get_item_children(None)
        if not self._pvd_node:
            return
        ancestor_pvd_node = self._pvd_node
        last_prim_idx = len(split_prim_names) - 1
        for idx, prim_name in enumerate(split_prim_names):
            child_item = ancestor_pvd_node.get_child_item(prim_name, stage)
            # if this is the last prim, set the child leaf node to found_in_search to be True
            if child_item:
                if idx == last_prim_idx:
                    if is_for_search:
                        child_item._pvd_node._found_in_search = True
                        self._found_pvd_nodes.append(child_item._pvd_node)
                    else:
                        self._expanded_select_items.append(child_item)
                    self._item_changed(child_item)
                else:
                    tree.set_expanded(child_item, True, False)
                ancestor_pvd_node = child_item._pvd_node                    
            else:
                # something went wrong, exit the loop
                return

    def get_item_children(self, item):
        visible_children = []
        if omni.usd.get_context():
            stage = omni.usd.get_context().get_stage()
            if stage:
                if item is not None:
                    visible_children = item._pvd_node.get_current_visibile_children(stage)
                else:
                    if not self._pvd_node:
                        rootPrim = stage.GetPrimAtPath("/")
                        self._pvd_node = OmniPvdNode(rootPrim, None)
                    visible_children = self._pvd_node.get_current_visibile_children(stage)
        return visible_children

    def get_item_value_model_count(self, item):
        return 2

    def get_item_value_model(self, item, column_id):
        return item.name_model

class OmniPVDNodeDelegate(ui.AbstractItemDelegate):
    def __init__(self):
        super().__init__()
        self._actortype_map = dict([
            ('eRIGID_STATIC', 'PxRigidStatic'),
            ('eRIGID_DYNAMIC', 'PxRigidDynamic'),
            ('eARTICULATION_LINK', 'PxArticulationLink'),
            ('eSOFTBODY', 'PxSoftBody'),
            ('eFEMCLOTH', 'PxFEMCloth'),
            ('ePBD_PARTICLESYSTEM', 'PxPBDParticleSystem'),
            ('eFLIP_PARTICLESYSTEM', 'PxFLIPParticleSystem'),
            ('eMPM_PARTICLESYSTEM', 'PxMPMParticleSystem'),
            ('PxHairSystem', 'eHAIRSYSTEM')
            ])

        self._jointtype_map = dict([
            ('eSPHERICAL', 'PxSphericalJoint'),
            ('eREVOLUTE', 'PxRevoluteJoint'),
            ('ePRISMATIC', 'PxPrismaticJoint'),
            ('eFIXED', 'PxFixedJoint'),
            ('eDISTANCE', 'PxDistanceJoint'),
            ('eD6', 'PxD6Joint'),
            ('eCONTACT', 'PxContactJoint'),
            ('eGEAR', 'PxGearJoint'),
            ('eRACK_AND_PINION', 'PxRackAndPinionJoint')
            ])

        self.reset()

    def reset(self):        
        self._ctx = None
        self._stage = None
        self._tree = None
        self._search_string = ""
        self._current_idx = -1

    def _set_stage_vars(self):
        if not omni.usd.get_context():
            self.reset()
            return
        self._ctx = omni.usd.get_context()
        self._stage = self._ctx.get_stage()

    def get_time(self):
        timeline = get_timeline_interface()
        return timeline.get_current_time() * timeline.get_time_codes_per_seconds()

    def build_branch(self, model, item, column_idx, level, expanded):
        if column_idx!=0:
            return
        if item is not None:
            padding = "    " * level
            if model.get_item_children(item):
                with ui.HStack():
                    if model.can_item_have_children(item):
                        sign = "-" if expanded else "+"
                        if item._pvd_node._found_in_search:
                            ui.Label(f"{padding}{sign}  ", style_type_name_override="TreeView.Item", style={"color": ui.color.white})
                        else:
                            ui.Label(f"{padding}{sign}  ", style_type_name_override="TreeView.Item")
            else:
                with ui.HStack():
                    if item._pvd_node._found_in_search:
                        ui.Label(f"{padding}  ", style_type_name_override="TreeView.Item", style={"color": ui.color.white})
                    else:
                        ui.Label(f"{padding}  ", style_type_name_override="TreeView.Item")

    def build_widget(self, model, item, column_idx, level, expanded):
        if item is None:
            return
        if not self._stage:
            self._set_stage_vars()
        if not self._stage:
            return
        prim = self._stage.GetPrimAtPath(item._pvd_node._node_name)
        if not prim:
            return
        if column_idx == 0:
            if prim.HasAttribute("omni:pvd:name"):
                object_name_attr = prim.GetAttribute("omni:pvd:name")
                object_name = object_name_attr.Get(self.get_time())
            else:
                if prim.HasAttribute("omni:pvdi:name"):
                    object_name = prim.GetAttribute("omni:pvdi:name").Get()
                else:
                    object_name = item._pvd_node._prim_name
            if prim.HasAttribute("omni:pvdi:uid"):
                uid = prim.GetAttribute("omni:pvdi:uid").Get()
                if uid>0:
                    object_name = "(" + str(uid) + ")" + object_name
            if item._pvd_node._found_in_search:
                ui.Label(f"{object_name}", style_type_name_override="TreeView.Item", style={"color": ui.color.white})
            else:
                ui.Label(f"{object_name}", style_type_name_override="TreeView.Item")
        else:
            omniPvdClass = ""
            if prim.HasAttribute("omni:pvdi:class"):
                omniPvdClass = prim.GetAttribute("omni:pvdi:class").Get()
                if omniPvdClass.startswith("Px"):
                    if prim.HasAttribute("omni:pvd:type"):
                        object_type = prim.GetAttribute("omni:pvd:type").Get(self.get_time())
                        if object_type:
                            if omniPvdClass == "PxActor":
                                omniPvdClass = self._actortype_map[object_type]
                            elif omniPvdClass == "PxJoint":
                                omniPvdClass = self._jointtype_map[object_type]
                else:
                    omniPvdClass = ""
            if item._pvd_node._found_in_search:
                ui.Label(f"{omniPvdClass}", alignment = omni.ui.Alignment.LEFT,  style_type_name_override="TreeView.Item", style={"color": ui.color.white})
            else:
                ui.Label(f"{omniPvdClass}", alignment = omni.ui.Alignment.LEFT,  style_type_name_override="TreeView.Item")

    # For all prims that correspond to the search criteria, create the items for each prim (and its ancestors)
    # and expand up until including the found prims ancestor item
    def iterate_search_on_prim(self, prim, search_string, display_predicate):
        if prim:            
            if prim.HasAttribute("omni:pvdi:class"):
                visibility_attribute = prim.GetAttribute('visibility')
                if not visibility_attribute:
                    return
                if visibility_attribute.Get(self.get_time()) != 'inherited':
                    return

                # This should rather be in a pre-allocated Item or PVDNode
                # the whole strategy on traversing the USD stage makes less and less sense
                # as these "hidden" costs, such as re-generation of the search name, start to amass
                if prim.HasAttribute("omni:pvd:name"):
                    object_name_attr = prim.GetAttribute("omni:pvd:name")
                    object_name = object_name_attr.Get(self.get_time())
                else:
                    if prim.HasAttribute("omni:pvdi:name"):
                        object_name = prim.GetAttribute("omni:pvdi:name").Get()
                    else:
                        object_name = prim.GetName()
                if prim.HasAttribute("omni:pvdi:uid"):
                    uid = prim.GetAttribute("omni:pvdi:uid").Get()
                    if uid>0:
                        object_name = object_name + "(" + str(uid) + ")"

                # The test below should be on the OmniPVD object name, which can be any string
                if search_string in object_name:
                    path_str = prim.GetPath().pathString
                    split_prim_names = path_str.split("/")
                    # filter out the empty elements : typically the first one "/World/box" -> ["", "World", "box"]
                    split_prim_names = list(filter(None, split_prim_names))
                    self._model.create_item_chain_and_expand(split_prim_names, self._stage, self._tree)                    
                # Also check the children
                children_iterator = prim.GetFilteredChildren(display_predicate)
                for child_prim in children_iterator:
                    self.iterate_search_on_prim(child_prim, search_string, display_predicate)

    def reset_nodes(self, pvd_node):
        if not pvd_node:
            return
        if pvd_node._found_in_search:
            pvd_node._found_in_search = False
            pvd_node._highlighted = False
            self._model._item_changed(pvd_node._item)
        if pvd_node._all_children:
            for item in pvd_node._all_children:
                pvd_node_of_item = item._pvd_node
                self.reset_nodes(pvd_node_of_item)        

    def on_search(self, model):
        self._search_string = f"{model.as_string}"
        self._set_stage_vars()
        self._current_idx = -1
        if not self._ctx:
            return
        self.reset_nodes(self._model._pvd_node)
        if not self._search_string.strip():
            self._model._found_pvd_nodes = []
            return
        if not self._stage:
            return
        self._model._found_pvd_nodes = []
        rootPrim = self._stage.GetPrimAtPath("/")
        display_predicate = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)        
        if rootPrim:            
            children_iterator = rootPrim.GetFilteredChildren(display_predicate)
            for child_prim in children_iterator:
                self.iterate_search_on_prim(child_prim, self._search_string, display_predicate)
        if len(self._model._found_pvd_nodes)>0:
            self._current_idx = 0
            self.on_selection_of_item([self._model._found_pvd_nodes[self._current_idx]._item])
        else:
            self._current_idx = -1
        self._model._item_changed(None)

    ################################################################################
    # on_selection_of_item gets called when something is selected in either the USD Stage
    # view or the OVD Tree view
    ################################################################################
    def on_selection_of_item(self, items):
        if not self:
            return
        ctx = omni.usd.get_context()
        if not ctx:
            return
        stage = ctx.get_stage()
        if not stage:
            return
        if not carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_IS_OVD_STAGE):
            return
        selected_paths = []
        for item in items:
            selected_paths.append(item._pvd_node._node_name)
        if (len(selected_paths) > 0):
            self._set_stage_vars()
            if self._window.focused:
                self._ctx.get_selection().set_selected_prim_paths(selected_paths, True)
            self._tree.selection = items
        else:
            if hasattr(self, '_ctx'):
                if not self._ctx:
                    return
            #self._set_stage_vars()
            if self._window.focused:
                self._ctx.get_selection().set_selected_prim_paths(selected_paths, True)
            self._tree.selection = []

    def _iter_next_phase_1(self):
        if self._current_idx == -1:
            return False
        self._model._found_pvd_nodes[self._current_idx]._highlighted = False
        item = self._model._found_pvd_nodes[self._current_idx]._item
        self._model._item_changed(item)
        return True

    def _iter_next_phase_2(self):
        self._model._found_pvd_nodes[self._current_idx]._highlighted = True
        item = self._model._found_pvd_nodes[self._current_idx]._item
        self._model._item_changed(item)
        items_selected = [item]
        self.on_selection_of_item(items_selected)

    def _on_prev_button_pressed(self):
        if self._iter_next_phase_1():
            self._current_idx = self._current_idx - 1
            if self._current_idx < 0:
                self._current_idx = len(self._model._found_pvd_nodes)-1
            self._iter_next_phase_2()

    def _on_next_button_pressed(self):
        if self._iter_next_phase_1():
            self._current_idx = self._current_idx + 1
            if self._current_idx > (len(self._model._found_pvd_nodes)-1):
                self._current_idx = 0
            self._iter_next_phase_2()

    def _on_collapse_all_button_pressed(self):
        self._set_stage_vars()
        if not self._ctx:
            return
        self.reset_nodes(self._model._pvd_node)

        root_node = self._model._pvd_node
        if not root_node:
            return
        for item in root_node._all_children:
            self._tree.set_expanded(item, False, True)

    def build_header(self, column_idx):
        with ui.VStack():
            with ui.HStack():
                if column_idx == 0:
                    ui.Label("Name", tooltip="Header")
                else:
                    ui.Label("Type", tooltip="Header", alignment = omni.ui.Alignment.LEFT, height=0)

################################################################################
# The OmniPVD Object Tree window
################################################################################
class OmniPVdObjectTreeWindow(ui.Window):
    def get_time(self):
        timeline = get_timeline_interface()
        return timeline.get_current_time() * timeline.get_time_codes_per_seconds()

    def mybuild(self):
        with self.frame:
            if not hasattr(self, '_style'):
                self._style = carb.settings.get_settings().get_as_string("/persistent/app/window/uiStyle") or "NvidiaDark"
                if self._style == "NvidiaLight":
                    self._style_data = {
                        "Button.Image::filter": {"image_url": StageIcons().get("filter"), "color": 0xFF535354},
                        "Button.Image::options": {"image_url": StageIcons().get("options"), "color": 0xFF535354},
                        "Button.Image::visibility": {"image_url": StageIcons().get("eye_on")},
                        "Button.Image::visibility:checked": {"image_url": StageIcons().get("eye_off")},
                        "Button.Image::visibility:disabled": {"color": 0x608A8777},
                        "Button.Image::visibility:selected": {"color": 0xFF23211F},
                        "Button::filter": {"background_color": 0x0, "margin": 0},
                        "Button::options": {"background_color": 0x0, "margin": 0},
                        "Button::visibility": {"background_color": 0x0, "margin": 0, "margin_width": 1},
                        "Button::visibility:checked": {"background_color": 0x0},
                        "Button::visibility:hovered": {"background_color": 0x0},
                        "Button::visibility:pressed": {"background_color": 0x0},
                        "Field": {"background_color": 0xFF535354, "color": 0xFFCCCCCC},
                        "Label::search": {"color": 0xFFACACAC},
                        "Menu.CheckBox": {"background_color": 0x0, "margin": 0},
                        "Menu.CheckBox::drag": {
                            "image_url": StageIcons().get("drag"),
                            "color": 0xFF505050,
                            "alignment": ui.Alignment.CENTER,
                        },
                        "Menu.CheckBox.Image": {"image_url": StageIcons().get("check_off"), "color": 0xFF8A8777},
                        "Menu.CheckBox.Image:checked": {"image_url": StageIcons().get("check_on")},
                        "ScrollingFrame": {"secondary_color": 0xFF444444},
                        "TreeView": {
                            "background_color": 0xFFE0E0E0,
                            "background_selected_color": 0x109D905C,
                            "secondary_color": 0xFFACACAC,
                        },
                        "TreeView.ScrollingFrame": {"background_color": 0xFFE0E0E0},
                        "TreeView.Header": {"color": 0xFFCCCCCC},
                        "TreeView.Header::background": {
                            "background_color": 0xFF535354,
                            "border_color": 0xFF707070,
                            "border_width": 0.5,
                        },
                        "TreeView.Header::columnname": {"margin": 3},
                        "TreeView.Header::visibility_header": {"image_url": StageIcons().get("eye_header")},
                        "TreeView.Image::object_icon_grey": {"color": 0x80FFFFFF},
                        "TreeView.Item": {"color": 0xFF535354, "font_size": 16},
                        "TreeView.Item::object_name": {"margin": 3},
                        "TreeView.Item::object_name_grey": {"color": 0xFFACACAC},
                        "TreeView.Item::object_name_missing": {"color": 0xFF6F72FF},
                        "TreeView.Item:selected": {"color": 0xFF2A2825},
                        "TreeView:selected": {"background_color": 0x409D905C},
                        "TreeView:drop": {"background_color": 0x409D905C},
                    }
                else:
                    self._style_data = {
                        "Button.Image::filter": {"image_url": StageIcons().get("filter"), "color": 0xFF8A8777},
                        "Button.Image::options": {"image_url": StageIcons().get("options"), "color": 0xFF8A8777},
                        "Button.Image::visibility": {"image_url": StageIcons().get("eye_on"), "color": 0xFF8A8777},
                        "Button.Image::visibility:checked": {"image_url": StageIcons().get("eye_off")},
                        "Button.Image::visibility:disabled": {"color": 0x608A8777},
                        "Button.Image::visibility:selected": {"color": 0xFF23211F},
                        "Button::filter": {"background_color": 0x0, "margin": 0},
                        "Button::options": {"background_color": 0x0, "margin": 0},
                        "Button::visibility": {"background_color": 0x0, "margin": 0, "margin_width": 1},
                        "Button::visibility:checked": {"background_color": 0x0},
                        "Button::visibility:hovered": {"background_color": 0x0},
                        "Button::visibility:pressed": {"background_color": 0x0},
                        "Label::search": {"color": 0xFF808080, "margin_width": 4},
                        "Menu.CheckBox": {"background_color": 0x0, "margin": 0},
                        "Menu.CheckBox::drag": {
                            "image_url": StageIcons().get("drag"),
                            "color": 0xFF505050,
                            "alignment": ui.Alignment.CENTER,
                        },
                        "Menu.CheckBox.Image": {"image_url": StageIcons().get("check_off"), "color": 0xFF8A8777},
                        "Menu.CheckBox.Image:checked": {"image_url": StageIcons().get("check_on")},
                        "TreeView": {
                            "background_color": 0xFF23211F,
                            "background_selected_color": 0x664F4D43,
                            "secondary_color": 0xFF403B3B,
                        },
                        "TreeView.ScrollingFrame": {"background_color": 0xFF23211F},
                        "TreeView.Header": {"background_color": 0xFF343432, "color": 0xFFCCCCCC, "font_size": 12},
                        "TreeView.Header::visibility_header": {"image_url": StageIcons().get("eye_header")},
                        "TreeView.Image::object_icon_grey": {"color": 0x80FFFFFF},
                        "TreeView.Image:disabled": {"color": 0x60FFFFFF},
                        "TreeView.Item": {"color": 0xFF8A8777},
                        "TreeView.Item:disabled": {"color": 0x608A8777},
                        "TreeView.Item::object_name_grey": {"color": 0xFF4D4B42},
                        "TreeView.Item::object_name_missing": {"color": 0xFF6F72FF},
                        "TreeView.Item:selected": {"color": 0xFF23211F},
                        "TreeView:selected": {"background_color": 0xFF8A8777},
                        "TreeView:drop": {"background_color": 0xFF8A8777},
                    }
                self.frame.set_style(self._style_data)

            if not hasattr(self, '_model'):
                self._model = Model()
            else:
                self._model.reset()
            if not hasattr(self, '_delegate'):
                self._delegate = OmniPVDNodeDelegate()
            else:
                self._delegate.reset()

            ################################################################################
            # Window handle is needed to know if the OVD Tree window is in focuse from the
            # on_selection event handled by the delegate
            ################################################################################
            self._delegate._window = self

            with ui.VStack():
                with ui.HStack(height=0):
                    ui.Label("Search", tooltip="Search", width=80)
                    sf_filter = ui.StringField()
                    sf_filter.model.add_end_edit_fn(self._delegate.on_search)
                    sf_filter.model.set_value(self._delegate._search_string)
                with ui.HStack(height=0):
                    self._delegate._collapse_all_button = ui.Button("Collapse All", width=20, height=0)
                    self._delegate._collapse_all_button.set_clicked_fn(self._delegate._on_collapse_all_button_pressed)

                    self._delegate._prev_button = ui.Button("Previous", width=20, height=0)
                    self._delegate._prev_button.set_clicked_fn(self._delegate._on_prev_button_pressed)

                    self._delegate._next_button = ui.Button("Next", width=20, height=0)
                    self._delegate._next_button.set_clicked_fn(self._delegate._on_next_button_pressed)    
                with ui.ScrollingFrame(
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    style_type_name_override="TreeView",
                ):                
                    self._tree = ui.TreeView(self._model, delegate = self._delegate, root_visible = False, header_visible = True, style={"margin": 0.5},  columns_resizable = True, style_type_name_override="TreeView", column_widths=[ui.Fraction(3.5), ui.Fraction(1)])
                    self._model._tree = self._tree
                    self._delegate._model = self._model
                    self._delegate._tree = self._tree
                    self._tree.set_selection_changed_fn(self._delegate.on_selection_of_item)

    def _setVisibleChildren(self, prim):
        if not prim:
            return
        if prim.HasAttribute("omni:pvdi:viz"):
            if not prim.GetAttribute("omni:pvdi:viz").Get(self.cached_time):
                UsdGeom.Imageable(prim).MakeInvisible()
            else:
                UsdGeom.Imageable(prim).MakeVisible()
                children_iterator = prim.GetChildren()
                for child_prim in children_iterator:
                    self._setVisibleChildren(child_prim)
        else:
            children_iterator = prim.GetChildren()
            for child_prim in children_iterator:
                self._setVisibleChildren(child_prim)

    def _updatedFromEvent(self):
        if not hasattr(self, "_model"):
            return
        if not carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_IS_OVD_STAGE):
            return
        usd_context = omni.usd.get_context()
        if not usd_context:
            return
        stage = usd_context.get_stage()
        if stage:
            stage.SetInterpolationType(Usd.InterpolationTypeHeld)
            self.cached_time = self.get_time()
            with Sdf.ChangeBlock():
                self._setVisibleChildren(stage.GetPrimAtPath("/Scenes"))
                self._setVisibleChildren(stage.GetPrimAtPath("/Shared"))
            self._model._expanded_select_items = []
            selected_prim_paths = usd_context.get_selection().get_selected_prim_paths()
            for path in selected_prim_paths:
                split_prim_names = path.split("/")
                split_prim_names = list(filter(None, split_prim_names))
                self._model.create_item_chain_and_expand(split_prim_names, stage, self._tree, False)
            self._tree.selection = self._model._expanded_select_items
        self._model._item_changed(None)

    def _on_timeline_event(self, e):
        current_time = e.payload["currentTime"]
        if current_time != self._current_time:
            self._current_time = current_time
            self._updatedFromEvent()

    def __init__(self):
        super().__init__("OmniPVD Object Tree", ui.DockPreference.RIGHT_TOP, width=640, height=480)
        self.deferred_dock_in("Stage", ui.DockPolicy.DO_NOTHING)
        self._usd_context = omni.usd.get_context()

        self.frame.set_build_fn(self.mybuild)
        self.frame.rebuild()

        @carb.profiler.profile
        def on_stage_event(event):
            if (not self._usd_context):
                return
            if (not self._usd_context.get_stage()):
                return
            if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
                if hasattr(self, '_model'):
                    self._model._expanded_select_items = []
                    if carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_IS_OVD_STAGE):
                        stage = self._usd_context.get_stage()
                        if stage:
                            selected_prim_paths = self._usd_context.get_selection().get_selected_prim_paths()
                            for path in selected_prim_paths:
                                split_prim_names = path.split("/")
                                split_prim_names = list(filter(None, split_prim_names))
                                self._model.create_item_chain_and_expand(split_prim_names, stage, self._tree, False)
                    self._tree.selection = self._model._expanded_select_items
            elif event.type == int(omni.usd.StageEventType.OPENED):
                self.frame.rebuild()
                self._updatedFromEvent()

        event_stream = self._usd_context.get_stage_event_stream()
        self._stage_event_sub = event_stream.create_subscription_to_pop(on_stage_event)

        timeline = omni.timeline.get_timeline_interface()
        self._current_time = timeline.get_current_time()
        stream = timeline.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)


    def on_shutdown(self):
        self._stage_event_sub = None
        self._timeline_subscription = None

    def traverse_stage(self):
        return

def build_section(name, build_func):
    with ui.CollapsableFrame(name, height=0):
        with ui.HStack():
            ui.Spacer(width=20)
            with ui.VStack():
                build_func()

################################################################################
# OVD File->Import dialogue
################################################################################
class OVDImporter(ai.AbstractImporterDelegate):
    def __init__(self, ovd_importer) -> None:
        super().__init__()
        self._name = "OVD OmniPVD Importer"
        self._filters = [".*\\.ovd$"]
        self._descriptions = ["OVD Files (*.ovd)"]
        self._default_color = [200, 200, 200]
        # used for icon paths
        self._ovdImporter = ovd_importer

    def destroy(self):
        self._ovdImporter = None
        return

    @property
    def name(self) -> str:
        return self._name

    @property
    def filter_regexes(self) -> List[str]:
        return self._filters

    @property
    def filter_descriptions(self) -> List[str]:
        return self._descriptions

    def build_options(self, paths: List[str]) -> None:
        return True

    async def convert_assets(self, paths: List[str]) -> Dict[str, Union[str, None]]:        
        converted_assets = {}
        full_path = paths[0]
        file_name = os.path.basename(full_path)
        file_dir = os.path.dirname(full_path)
        file_dir += "/"
        self._ovdImporter._import_ovd_file_from_path(file_name, file_dir)
        return converted_assets

class PhysxPvdExtension(omni.ext.IExt):
    instance = None

    def __init__(self):
        self._propertyWidgetOmniPvd = None
        super().__init__()

    def modTime(self, fileName):
        concatPath = self._dirPath + fileName
        return os.path.getmtime(concatPath)

    def _getLatestFileByModificationTime(self, directory):
        def is_valid_file(filename):                    
            return filename != 'tmp.ovd' and filename.endswith('.ovd')                        

        self._dirPath = directory
        filePath = None
        if not os.path.exists(directory):
            return filePath
        try:
            my_list = next(os.walk(directory))[2]
            if my_list:
                my_list = list(filter(is_valid_file, my_list))
                if my_list:
                    filePath = max(my_list, key=self.modTime)
        except StopIteration:
            pass
        return filePath

    def _build_omnipvd_ui(self):
        def define_recording_dir():
            def on_choose(fp_dialog, filename, dirpath):
                fp_dialog.hide()
                if (dirpath):
                    if not dirpath.endswith("/"):
                        dirpath += "/"
                    # Sets the OVD recording directory for Omni Physics simulations
                    carb.settings.get_settings().set(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY, dirpath)
                cleanup_fp_dialog(fp_dialog)

            item_filter_options = ["OVD Files (*.ovd)"]
            fp_dialog = FilePickerDialog(
                "Select the OVD Recording Directory",
                apply_button_label="Select Current",
                click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
                item_filter_options=item_filter_options,
            )
            fp_dialog.show()


        # Set the default OVD recording directory lazily
        ovdRecordingDir = carb.settings.get_settings().get(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY)
        if not ovdRecordingDir:
            ovdRecordingDir = carb.tokens.get_tokens_interface().resolve("${omni_documents}") + "/omni_pvd/out/"
            if not os.path.exists(ovdRecordingDir):
                os.makedirs(ovdRecordingDir)
            carb.settings.get_settings().set(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY, ovdRecordingDir)

        omnipvd_settings = [
            [SettingType.BOOL, "Recording Enabled", SETTING_OMNIPVD_ENABLED]
        ]

        def build_inner():
            for setting in omnipvd_settings:
                with ui.HStack(height=23):
                    ui.Spacer(width=5)
                    ui.Label(setting[1], width=160)
                    if setting[0] == SettingType.BOOL:
                        cb = _build_checkbox(SettingModel(setting[2]))
                        if setting[2] == SETTING_OMNIPVD_ENABLED:
                            self._omniPvdEnabledCheckbox = cb
                    elif setting[0] == SettingType.STRING:
                        widget = ui.StringField(SettingModel(setting[2]), height=20)
            with ui.HStack(height=23):
                button = ui.Button("Set Recording Directory", height=30, width=160)
                button.set_clicked_fn(define_recording_dir)
                ui.Spacer(width=5)
                with ui.VStack(height=23):
                    ui.Spacer(height=4)
                    widget = ui.StringField(SettingModel(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY), height=22, enabled = False)
                    widget.enabled = False
                    widget.set_style(DISABLED_STYLE)
        build_inner()

        # Disallow switching omniPVD on and off while sim is running or the stage is an OmniPVD stage
        def on_simulation_event_omnipvd_enable(event):
            if event.type == int(SimulationEvent.RESUMED):
                self._omniPvdEnabledCheckbox.enabled = False
            elif event.type == int(SimulationEvent.STOPPED):
                if not carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_IS_OVD_STAGE):
                    self._omniPvdEnabledCheckbox.enabled = True

        def on_stage_event_omnipvd_enable(event):
            if event.type == int(omni.usd.StageEventType.OPENED):
                # check the stage if OmniPVD stage?
                is_OVD_stage = False
                ctx = omni.usd.get_context()
                if ctx:
                    stage = ctx.get_stage()
                    if stage:
                        prim = stage.GetPrimAtPath("/scenes")
                        if not prim:
                            prim = stage.GetPrimAtPath("/shared")
                        if not prim:
                            prim = stage.GetPrimAtPath("/Scenes")
                        if not prim:
                            prim = stage.GetPrimAtPath("/Shared")
                        if prim:
                            if prim.HasAttribute("omni:pvdi:class"):
                                is_OVD_stage = True
                if not is_OVD_stage:
                    self._omniPvdEnabledCheckbox.enabled = True
                    carb.settings.get_settings().set(SETTING_OMNIPVD_IS_OVD_STAGE, False)
                else:
                    self._omniPvdEnabledCheckbox.enabled = False
                    carb.settings.get_settings().set(SETTING_OMNIPVD_IS_OVD_STAGE, True)

        self._omniPvdEnabledCheckbox.enabled = not get_physx_interface().is_running()

        physxInterface = get_physx_interface()
        self._omniPvdEventSubcription = physxInterface.get_simulation_event_stream_v2().create_subscription_to_pop(
            on_simulation_event_omnipvd_enable
        )


        usd_context = omni.usd.get_context()
        if usd_context:
            event_stream = usd_context.get_stage_event_stream()
            if event_stream:
                self._stage_event_sub_ovd = event_stream.create_subscription_to_pop(on_stage_event_omnipvd_enable)

    def _getLatestDirByDirName(self, directory):
        maxInt = 0
        try:
            my_list = next(os.walk(directory))[1]
            if my_list:
                filtered_list = list(filter(lambda x:x.isnumeric(), my_list))
                if filtered_list:
                    filtered_ints = map(lambda x:int(x), filtered_list)
                    if filtered_ints:
                        maxInt = max(filtered_ints)
        except StopIteration:
            maxInt = 0
        return maxInt + 1

    # Scan the directory for all directories that consist of a number and
    # keep the largest number. Put the new directory to be created to be +1 of that largest
    # number.
    def _getNextIntFromDirectories(self, directory):
        maxInt = 0
        try:
            my_list = next(os.walk(directory))[1]
            if my_list:
                filtered_list = list(filter(lambda x:x.isnumeric(), my_list))
                if filtered_list:
                    filtered_ints = map(lambda x:int(x), filtered_list)
                    if filtered_ints:
                        maxInt = max(filtered_ints)
        except StopIteration:
            maxInt = 0
        return maxInt + 1

    def _open_latest_stage(self, outputBasePath):
        lastInt = self._getNextIntFromDirectories(outputBasePath) - 1
        if (lastInt>0):
            outputStagePath = outputBasePath + str(lastInt) + "/";
            # now check if the stage is usd or usda
            stageFile = outputStagePath + "stage.usda"
            if not os.path.exists(stageFile):
                stageFile = outputStagePath + "stage.usd"
            if os.path.exists(stageFile):
                omni.usd.get_context().open_stage(stageFile)

    def _button_stage_traversal(self):        
        toPhysxUSDConverter = ConvertOmniPvdToPhysXUSD()
        outputBasePath = carb.settings.get_settings().get(SETTING_OMNIPVD_PHYSX_USD_DIRECTORY)
        nextInt = self._getNextIntFromDirectories(outputBasePath)
        outputStagePath = outputBasePath + str(nextInt);
        if (self._omniPvdUSDType == 0):
            outputStagePath += "/stage.usd"
        else:
            outputStagePath += "/stage.usda"
        toPhysxUSDConverter.convert(outputStagePath, False)
        toPhysxUSDConverter = None

    def _button_open_latest_usd_stage(self):
        outputBasePath = carb.settings.get_settings().get(SETTING_OMNIPVD_USD_CACHE_DIRECTORY)
        self._open_latest_stage(outputBasePath)

    def _button_open_latest_physx_stage(self):
        outputBasePath = carb.settings.get_settings().get(SETTING_OMNIPVD_PHYSX_USD_DIRECTORY)
        self._open_latest_stage(outputBasePath)

    def _getUSDCacheDirName(self, dirPath, fileName):
        mtime = os.path.getmtime(dirPath + fileName)
        mtimeString = f"{mtime}"
        mtimeString = mtimeString.replace(".","_")
        cacheDirName = mtimeString + "_" + fileName.replace(".","_")
        return cacheDirName

    def _open_cached_stage(self, stageFile):
        if os.path.exists(stageFile):
            omni.usd.get_context().open_stage(stageFile)
            stage = omni.usd.get_context().get_stage()
            if not stage:
                return False

            domeLight = UsdLux.DomeLight.Define(stage, Sdf.Path("/DomeLight"))
            domeLight.CreateIntensityAttr(500)
            domeLight.CreateSpecularAttr().Set(0.0)

            distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
            distantLight.CreateIntensityAttr(500)
            distantLight.CreateAngleAttr(0.53)
            distantLight.CreateSpecularAttr().Set(0.0)
            
            stage.SetEditTarget(stage.GetSessionLayer())

            cam = UsdGeom.Camera.Define(stage, "/OmniverseKit_Persp")
            if cam:
                # Fix for the camera clipping range using the physx tolerances scale
                clipRangeWasSet = false = False
                physxInstancePrim = stage.GetPrimAtPath("/Scenes/PxPhysics/PxPhysics_1")
                if physxInstancePrim:
                    if physxInstancePrim.HasAttribute("omni:pvd:tolerancesScale"):
                        tolerancesScale = physxInstancePrim.GetAttribute("omni:pvd:tolerancesScale").Get(Usd.TimeCode.EarliestTime())
                        if tolerancesScale:
                            metersPerStageUnit = 1 / tolerancesScale[0]
                            cam.CreateClippingRangeAttr(Gf.Vec2f(0.01/metersPerStageUnit, 1000000))
                            clipRangeWasSet = True
                            # Set the default global gizmo slider scale, which works like a multiplier
                            carb.settings.get_settings().set(SETTING_OMNIPVD_GIZMO_GLOBAL_SCALE, 1.0/metersPerStageUnit)
                if not clipRangeWasSet:
                    cam.CreateClippingRangeAttr(Gf.Vec2f(0.000001, 1000000))

            stage.SetEditTarget(stage.GetRootLayer())

            resolution = vp_utils.get_active_viewport().resolution
            viewport_api = get_active_viewport()
            cam_path = viewport_api.camera_path
            omni.kit.commands.execute(
                'FramePrimsCommand',
                prim_to_move= cam_path,
                prims_to_frame = ["/Scenes"],
                time_code=Usd.TimeCode.Default(),
                aspect_ratio=resolution[0]/resolution[1],
                zoom=0.25
                )
            return True
        return False

    def _import_ovd_file_from_path(self, fileName, dirPath):
        # added as a security when invalidating the cache and file locks are still being held for the previous cached file
        # when re-loading the same stage twice for example
        omni.usd.get_context().close_stage()
        carb.settings.get_settings().set(SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY, dirPath)
        ################################################################################
        # Convert the fileName into the cacheFileDir
        ################################################################################
        cacheFileDir = self._getUSDCacheDirName(dirPath, fileName)
        cacheDir = carb.settings.get_settings().get(SETTING_OMNIPVD_USD_CACHE_DIRECTORY)
        fullCacheDirPath = cacheDir + cacheFileDir + "/"
        cacheStagePath = fullCacheDirPath + "stage.usda"
        ovdFile = dirPath + fileName
        useCacheFile = not carb.settings.get_settings().get_as_bool(SETTING_OMNIPVD_INVALIDATE_CACHE)
        if useCacheFile:
            if self._open_cached_stage(cacheStagePath):
                carb.settings.get_settings().set(SETTING_OMNIPVD_IMPORTED_OVD, ovdFile)
                foundAxis, upAxis = getStageUpAxisAsInt()
                if foundAxis:
                    self._axisComboBox.model.get_item_value_model().set_value(upAxis)
                window = ui.Workspace.get_window("OmniPVD Object Tree")
                if window:
                    window.visible = True
                    window.focus()
                return
        if not os.path.exists(fullCacheDirPath):
            os.makedirs(fullCacheDirPath)
        ################################################################################
        # if stage.usda does NOT exist in the directory
        #   convert ovd to usda into the cache dir (or not might do it in memory)
        ################################################################################        
        self._physxPvdInterface.ovd_to_usd(ovdFile, fullCacheDirPath, self._omniPvdUpAxis, self._omniPvdUSDType)

        self._open_cached_stage(cacheStagePath)
        carb.settings.get_settings().set(SETTING_OMNIPVD_IMPORTED_OVD, ovdFile)
        foundAxis, upAxis = getStageUpAxisAsInt()
        if foundAxis:
            self._axisComboBox.model.get_item_value_model().set_value(upAxis)
        window = ui.Workspace.get_window("OmniPVD Object Tree")
        if window:
            window.visible = True
            window.focus()

        
    def _import_ovd_file(self):
        def on_choose(fp_dialog, fileName, dirPath):
            fp_dialog.hide()
            if dirPath and fileName:
                if not dirPath.endswith("/"):
                    dirPath += "/"
                fullPath = dirPath + fileName
                if os.path.exists(fullPath):
                    self._import_ovd_file_from_path(fileName, dirPath)
            cleanup_fp_dialog(fp_dialog)

        ################################################################################
        # Get the OVD recording directory
        ################################################################################
        kitOvdRecordingDir = carb.settings.get_settings().get(SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY)
        if not kitOvdRecordingDir:
            kitOvdRecordingDir = carb.settings.get_settings().get(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY)
        defaultPath = ""
        if kitOvdRecordingDir:
            fileName = self._getLatestFileByModificationTime(kitOvdRecordingDir)
            if fileName:
                defaultPath = kitOvdRecordingDir + fileName
        if defaultPath:
            if not kitOvdRecordingDir.endswith("/"):
                kitOvdRecordingDir += "/"
            self._import_ovd_file_from_path(fileName, kitOvdRecordingDir)
        else:
            item_filter_options = ["OVD Files (*.ovd)", "All Files (*)"]
            fp_dialog = FilePickerDialog(
                "Import OVD File",
                apply_button_label="Import",
                click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
                item_filter_options=item_filter_options,
            )
            fp_dialog.show(path = defaultPath)

    def get_ovd_for_baking(self):
        def modTime(fileName):
            concatPath = kitOvdRecordingDir + fileName
            return os.path.getmtime(concatPath)

        def _getLatestFileByModificationTime(directory):
            def is_valid_file(filename):                    
                return filename != 'tmp.ovd' and filename.endswith('.ovd')                        

            kitOvdRecordingDir = directory
            filePath = None
            if not os.path.exists(directory):
                return filePath
            try:
                my_list = next(os.walk(directory))[2]
                if my_list:
                    my_list = list(filter(is_valid_file, my_list))
                    if my_list:
                        filePath = max(my_list, key=modTime)
            except StopIteration:
                pass
            return filePath

        def on_choose(fp_dialog, filename, dirpath):
            fp_dialog.hide()
            if (dirpath and filename):
                if not dirpath.endswith("/"):
                    dirpath += "/"
                # Set the OmniPVD output dir here, needs new var, outputdir or similar
                carb.settings.get_settings().set(SETTING_OMNIPVD_OVD_FOR_BAKING, dirpath + filename)
                daString = carb.settings.get_settings().get(SETTING_OMNIPVD_OVD_FOR_BAKING)
                self._physxPvdInterface.ovd_to_usd_over(daString)
                
            cleanup_fp_dialog(fp_dialog)

        kitOvdRecordingDir = carb.settings.get_settings().get(SETTING_OMNIPVD_OVD_RECORDING_DIRECTORY)
        defaultPath = ""
        if kitOvdRecordingDir:
            fileName = _getLatestFileByModificationTime(kitOvdRecordingDir)
            if fileName:
                defaultPath = kitOvdRecordingDir + fileName

        item_filter_options = ["OVD Files (*.ovd)"]
        fp_dialog = FilePickerDialog(
            "Latest OVD Recording",
            apply_button_label="Select Current",
            click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
            item_filter_options=item_filter_options,
        )
        fp_dialog.show(path = defaultPath)

    def _add_over_layer_to_stage(self):
        self.get_ovd_for_baking()

    def _up_axis_changed(self, model, item):
        self._omniPvdUpAxis = model.get_item_value_model().as_int
        if not omni.usd.get_context():
            return
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return
        if (self._omniPvdUpAxis == 0):
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        else:
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    def _usd_type_changed(self, model, item):
        self._omniPvdUSDType = model.get_item_value_model().as_int

    def _input_file_changed(model):
        carb.settings.get_settings().set(SETTING_OMNIPVD_OVD_FOR_BAKING, model.as_string)

    ################################################################################
    # UI gizmo visualization functions
    ################################################################################
    def _build_usdtform(self):
        def define_import_dir():
            def on_choose(fp_dialog, filename, dirpath):
                fp_dialog.hide()
                if (dirpath):
                    if not dirpath.endswith("/"):
                        dirpath += "/"
                    # Sets the OVD import directory
                    carb.settings.get_settings().set(SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY, dirpath)
                cleanup_fp_dialog(fp_dialog)

            item_filter_options = ["OVD Files (*.ovd)"]
            fp_dialog = FilePickerDialog(
                "Select the OVD Import Directory",
                apply_button_label="Select Current",
                click_apply_handler=lambda filename, dirname: on_choose(fp_dialog, filename, dirname),
                item_filter_options=item_filter_options,
            )
            fp_dialog.show()

        with ui.HStack():
            self._transformToOmniPvdUSDButton = ui.Button("Load Latest", tooltip="Load the last modified OVD file in the import directory", width = 160, height = 20)
            self._transformToOmniPvdUSDButton.set_clicked_fn(self._import_ovd_file)
            ui.Spacer(width=5)
            with ui.VStack():
                ui.Spacer(height=3)
                widget, model = create_setting_widget(SETTING_OMNIPVD_IMPORTED_OVD, SettingType.STRING, height = 15, enabled = False)
                widget.enabled = False
                widget.set_style(DISABLED_STYLE)
        with ui.HStack(height=23):
            button = ui.Button("Set Import Directory", tooltip="Select the import directory from where Load Latest will scan for the last modified OVD file. If an OVD file is recorded, the import directory is automatically set to the recording directory.", height=30, width=160)
            button.set_clicked_fn(define_import_dir)
            ui.Spacer(width=5)
            with ui.VStack(height=23):
                ui.Spacer(height=4)
                widget = ui.StringField(SettingModel(SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY), height=22, enabled = False)
                widget.enabled = False
                widget.set_style(DISABLED_STYLE)
            #ui.Spacer(height=5)
        with ui.HStack():
            ui.Spacer(width=3)
            with ui.VStack():
                ui.Spacer(height=3)
                with ui.HStack():
                    ################################################################################
                    # Up axis
                    ################################################################################
                    axisTypes = ["Y up axis", "Z up axis"]
                    axisFound, upAxis = getStageUpAxisAsInt()
                    if axisFound:
                        self._omniPvdUpAxis = upAxis
                    self._axisComboBox = omni.ui.ComboBox(self._omniPvdUpAxis, *axisTypes, tooltip="Change the up axis of the Stage", width = 155, height = 20)
                    self._axisComboBox.model.add_item_changed_fn(self._up_axis_changed)

                    ui.Spacer(width=10)
                    ui.Label("Invalidate Cache", width=100)
                    self._cacheBox = _build_checkbox(SettingModel(SETTING_OMNIPVD_INVALIDATE_CACHE))

        with ui.HStack():
            self._my_button = ui.Button("Set Cache Directory", tooltip="Select where the USD Stage of the imported OVD file will be saved", width = 160, height = 20 )
            self._my_button.set_clicked_fn(set_usd_cache_dir)
            ui.Spacer(width=5)
            widget, model = create_setting_widget(SETTING_OMNIPVD_USD_CACHE_DIRECTORY, SettingType.STRING, height = 15)
            widget.enabled = False
            widget.set_style(DISABLED_STYLE)
        ui.Spacer(height=5)
        

    def _build_omnipvd_gizmos(self):
        ################################################################################
        # Set contacts/bounding boxes/center of mass (None/Selected/Unselected/Everything)
        ################################################################################
        vizModes = ["None", "Selected", "All"]
        vizModesHide = ["None", "Selected"]

        #ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # All
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("All", width = 150, height = 20)
            ui.Spacer(width=5+100+20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_GLOBAL_SCALE, SettingType.FLOAT, 0.0, 1000.0, width = 80, height = 15)
        ui.Spacer(height=10)
        ui.Separator(height=10)
        with ui.HStack():
            ################################################################################
            # Velocities
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Hide", width = 150, height = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_TRANSPARENCY_VIZMODE, vizModesHide)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Contacts
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Contacts", width = 150, height = 20)
            ui.Spacer(width=5)
            #workaround for create_setting_widget_combo not taking layout parameters
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_CONTACT_VIZMODE, vizModes)
            ui.Spacer(width=20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_CONTACT_SCALE, SettingType.FLOAT, 0.0, 100.0, width = 80, height = 15)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Bounding Boxes
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Bounding Boxes", width = 150, height = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_BOUNDING_BOX_VIZMODE, vizModes)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Coordinate System
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Center of Mass", width = 150, height = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_CENTER_OF_MASS_VIZMODE, vizModes)
            ui.Spacer(width=20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_CENTER_OF_MASS_SCALE, SettingType.FLOAT, 0.0, 100.0, width = 80, height = 15)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Coordinate System
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Coordinate System", width = 150, height = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_COORDINATE_SYSTEM_VIZMODE, vizModes)
            ui.Spacer(width=20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_COORDINATE_SYSTEM_SCALE, SettingType.FLOAT, 0.0, 100.0, width = 80, height = 15)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Velocities
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Velocities", width = 150, height = 20)
            ui.Spacer(width=5)
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_VELOCITY_VIZMODE, vizModes)
            ui.Spacer(width=20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_VELOCITY_SCALE, SettingType.FLOAT, 0.0, 100.0, width = 80, height = 15)
        ui.Spacer(height=5)
        with ui.HStack():
            ################################################################################
            # Joints
            ################################################################################
            ui.Spacer(width=5)
            ui.Label("Joints", width = 150, height = 20)
            ui.Spacer(width=5)
            #workaround for create_setting_widget_combo not taking layout parameters
            with ui.Frame(width = 100, height = 20):
                create_setting_widget_combo(SETTING_OMNIPVD_GIZMO_JOINT_VIZMODE, vizModes)
            ui.Spacer(width=20)
            ui.Label("Scale", width = 40, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_GIZMO_JOINT_SCALE, SettingType.FLOAT, 0.0, 100.0, width = 80, height = 15)
        ui.Spacer(height=5)

    def _build_physxusdtform(self):
        ################################################################################
        # Transform current animation frame in OmniPvd USD into a PhysX USD
        ################################################################################
        ui.Spacer(height=5)
        with ui.HStack():
            self._my_button = ui.Button("Export OVD Frame to PhysX USD", tooltip="Saves the the current animation frame in a PhysX USD format. Can only do so if the current Stage is an imported OVD file.",  width = 160, height = 20 )
            self._my_button.set_clicked_fn(self._button_stage_traversal)
            ui.Spacer(width=5)
        ui.Spacer(height=5)
        with ui.HStack():
            ui.Spacer(width=5)
            ui.Label("Output PhysX USD Directory", width = 150, height = 20)
            ui.Spacer(width=5)
            create_setting_widget(SETTING_OMNIPVD_PHYSX_USD_DIRECTORY, SettingType.STRING, height = 15)
        with ui.HStack():
            self._my_button = ui.Button("Open Latest PhysX USD Stage", width = 110, height = 20 )
            self._my_button.set_clicked_fn(self._button_open_latest_physx_stage)
        ui.Spacer(height=5)

    def _build_physxoverlay(self):
        ################################################################################
        # Add an over layer to the current Stage using and OmniPvd OVD file as input
        ################################################################################
        ui.Spacer(height=5)
        with ui.HStack():
            self._my_button = ui.Button("Bake PhysX Transforms into the Active Edit Layer", width = 160, height = 20 )
            self._my_button.set_clicked_fn(self._add_over_layer_to_stage)
        ui.Spacer(height=5)
        with ui.HStack():
            ui.Spacer(width=5)
            ui.Label("Overlay OVD", width = 150, height = 20)
            ui.Spacer(width=5)
            widget, model = create_setting_widget(SETTING_OMNIPVD_OVD_FOR_BAKING, SettingType.STRING, height = 15)
        ui.Spacer(height=5)

    def _make_my_window(self):
        self._omniPvdUpAxis = 0
        self._omniPvdUSDType = 1

        self._window_example = ui.Window("OmniPVD", width=100, dockPreference=omni.ui.DockPreference.RIGHT_BOTTOM)
        self._window_example.deferred_dock_in("Content", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

        ################################################################################
        # Chose input file
        ################################################################################
        from omni.kit.widget.settings.settings_widget import SettingType, create_setting_widget

        outDirectory = carb.settings.get_settings().get(SETTING_OMNIPVD_USD_CACHE_DIRECTORY)
        if not outDirectory:
            outDirectory = carb.tokens.get_tokens_interface().resolve("${omni_documents}") + "/omni_pvd/out/omnivpd_usd/"
            if not os.path.exists(outDirectory):
                os.makedirs(outDirectory)
            carb.settings.get_settings().set(SETTING_OMNIPVD_USD_CACHE_DIRECTORY, outDirectory)
        outDirectory = carb.settings.get_settings().get(SETTING_OMNIPVD_PHYSX_USD_DIRECTORY)
        if not outDirectory:
            outDirectory = carb.tokens.get_tokens_interface().resolve("${omni_documents}") + "/omni_pvd/out/omnivpd_physx_usd/"
            if not os.path.exists(outDirectory):
                os.makedirs(outDirectory)
            carb.settings.get_settings().set(SETTING_OMNIPVD_PHYSX_USD_DIRECTORY, outDirectory)

        with self._window_example.frame:
            with ui.ScrollingFrame(
                                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED):
                with ui.VStack(height = 0):
                    build_section("Load OVD", self._build_usdtform)
                    ui.Spacer(height=5)
                    build_section("Record OVD", self._build_omnipvd_ui)
                    ui.Spacer(height=5)
                    build_section("Gizmos", self._build_omnipvd_gizmos)
                    ui.Spacer(height=5)
                    #build_section("Export Frame to PhysX USD", self._build_physxusdtform)
                    #ui.Spacer(height=5)
                    build_section("Overlay OVD on an Active Edit Layer", self._build_physxoverlay)

        return self._window_example

    def on_startup(self, ext_id):
        ################################################################################
        # Preload library plugin dependencies, to make them discoverable
        ################################################################################
        if sys.platform == "win32":
            ext_folder = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../.."))
            omnipvd_dll_path = os.path.join(ext_folder, "bin/PVDRuntime_64.dll")
            ctypes.WinDLL(omnipvd_dll_path)

        ################################################################################
        # Acquire the instance reference pointer (for the sake of testing)
        ################################################################################
        PhysxPvdExtension.instance = self
        self._physxPvdInterface = _physxPvd.acquire_physx_pvd_interface()

        ################################################################################
        # Create the OmniPVD import/conversion menu
        ################################################################################
        self._menu = windowmenuitem.WindowMenuItem("OmniPVD", lambda: self._make_my_window(), True)

        ################################################################################
        # Create the OmniPVD Object Tree
        ################################################################################
        self._treeView = windowmenuitem.WindowMenuItem("OmniPVD Object Tree view", lambda: OmniPVdObjectTreeWindow(), True)

        ################################################################################
        # Create the property widget
        ################################################################################
        if not self._propertyWidgetOmniPvd:
            prim_handlers = [
                default_prim_handler,
            ]
            self._propertyWidgetOmniPvd = PropertyWidgetOmniPvd(prim_handlers)
            from omni.kit.property.physx import register_widget
            register_widget(self._propertyWidgetOmniPvd.name, self._propertyWidgetOmniPvd)
            self._propertyWidgetOmniPvd.on_startup()

        ################################################################################
        # Create the OVD file import menu and register it with Kit
        ################################################################################
        self._importer = OVDImporter(self)
        omni.kit.tool.asset_importer.register_importer(self._importer)

    def on_shutdown(self):
        ################################################################################
        # Unregister the property widget
        ################################################################################
        if self._propertyWidgetOmniPvd:
            self._propertyWidgetOmniPvd.on_shutdown()
            from omni.kit.property.physx import unregister_widget
            unregister_widget(self._propertyWidgetOmniPvd.name)
            self._propertyWidgetOmniPvd = None

        ################################################################################
        # Release the pvd interface
        ################################################################################
        _physxPvd.release_physx_pvd_interface(self._physxPvdInterface)
        self._physxPvdInterface = None

        ################################################################################
        # Remove the OVD asset importer from the Kit file menu
        ################################################################################
        omni.kit.tool.asset_importer.remove_importer(self._importer)
        self._importer.destroy()
        self._importer = None

        ################################################################################
        # Shut down the conversion menu
        ################################################################################
        self._menu.on_shutdown()
        self._menu = None

        ################################################################################
        # Remove the reference to the OmniPVD extension
        ################################################################################
        PhysxPvdExtension.instance = None

        ################################################################################
        # Shut down the OmniPVD Object Tree View
        ################################################################################
        if (self._treeView):
            self._treeView.on_shutdown()
            self._treeView = None

safe_import_tests("omni.physxpvd.scripts.tests")
