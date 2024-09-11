import os
import webbrowser
import carb
import carb.settings
import omni.kit.ui
import omni.kit.widget
import omni.usd
import omni.kit.menu
import omni.kit.menu.utils
import omni.kit.context_menu
from omni.kit.menu.utils import MenuItemDescription
from pxr import UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema
from omni.physx.scripts import utils, particleUtils
import omni.physxcommands as commands
from omni.kit.window.popup_dialog.options_dialog import OptionsDialog
import omni.kit.actions.core
import omni.kit.notification_manager as nm
from .physxAttachmentsDialog import AttachmentsDialog


def get_prims(payload):
    return payload.get("prim_list", [])


def can_show_all(condition_fn):
    return [lambda payload: len(get_prims(payload)) > 0 and all([condition_fn(prim) for prim in get_prims(payload)])]


def can_show_any(condition_fn):
    return [lambda payload: len(get_prims(payload)) > 0 and any([condition_fn(prim) for prim in get_prims(payload)])]


JOINT_NAMES = ["D6", "Fixed", "Revolute", "Prismatic", "Spherical", "Distance", "Gear", "Rack and Pinion"]
JOINT_TYPES = [n.title().replace(" ", "") for n in JOINT_NAMES]
PHYSICS_GLYPH_FILE = "menu_physics.svg"

# The Physics submenu when you right-click on an object in the viewport and choose 'Create'
class PhysicsMenu:
    instance = None

    def __init__(self):
        self._usd_context = omni.usd.get_context()
        self._selected_prim_paths = []
        self._developer_mode = carb.settings.get_settings().get("physics/developmentMode")
        if self._developer_mode:
            print("Physics Development Mode Active")
        self._menu_ext_dicts = {"Create": [], "Add": [], "Remove": []}
        self._refresh_menu_fn = {
            "Create": self._refresh_create_menu,
            "Add": self._refresh_additional_menu,
            "Remove": self._refresh_additional_menu,
        }

    def on_startup(self):
        self._menu_items = []
        self._create_menu_item = None

        def on_stage_event(event):
            if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
                self._selected_prim_paths = self._usd_context.get_selection().get_selected_prim_paths()
                omni.kit.menu.utils.refresh_menu_items("Create")

        event_stream = self._usd_context.get_stage_event_stream()
        self._stage_event_sub = event_stream.create_subscription_to_pop(on_stage_event)

        self._refresh_create_menu()
        self._refresh_additional_menu()

        PhysicsMenu.instance = self

    def on_shutdown(self):
        self._menu_items = []
        if self._create_menu_item is not None:
            omni.kit.menu.utils.remove_menu_items([self._create_menu_item], "Create")
            self._create_menu_item = None
        self._stage_event_sub = None
        self._selected_prim_paths = []

        self._refresh_menu_fn.clear()
        self._refresh_menu_fn = None

        self._remove_context_menus()

        PhysicsMenu.instance = None

    @staticmethod
    def _get_submenu(submenu):
        if PhysicsMenu.instance is None:
            carb.log_warn("PhysicsMenu: instance is not alive, did you forget a dependency?")
            return None, None

        menu_ext = PhysicsMenu.instance._menu_ext_dicts.get(submenu)
        if menu_ext is None:
            carb.log_warn(f"PhysicsMenu: context submenu '{submenu}' does not exist!")
            return None, None

        return PhysicsMenu.instance, menu_ext

    @staticmethod
    def add_context_menu(submenu, menu_dict):
        self, menu_ext = PhysicsMenu._get_submenu(submenu)
        if menu_ext is None:
            return

        menu_ext.append(menu_dict)
        self._refresh_menu_fn[submenu]()

    @staticmethod
    def remove_context_menu(submenu, menu_dict):
        self, menu_ext = PhysicsMenu._get_submenu(submenu)
        if menu_ext is None:
            return

        try:
            menu_ext.remove(menu_dict)
            self._refresh_menu_fn[submenu]()
        except ValueError:
            carb.log_warn(f"PhysicsMenu: '{menu_dict} was not found in the context_menu")

    def _refresh_create_menu(self):
        create_menu_dict = {
            "name": {
                "Physics":
                [
                    {
                        "name": "Physics Scene",
                        "onclick_fn": lambda *_: self._on_create_physics_scene()
                    },
                    {
                        "name": "Ground Plane",
                        "onclick_fn": lambda *_: self._on_create_ground_plane()
                    },
                    {
                        "name": "Physics Material",
                        "onclick_fn": lambda *_: self._on_create_physics_material()
                    },
                    {
                        "name": "Collision Group",
                        "onclick_fn": lambda *_: self._on_create_collision_group()
                    },
                    {
                        "name": {
                            "Joint":
                            [
                                {
                                    "name": f"{joint_name} Joint",
                                    # this works in ContextMenu.add_create_menu
                                    "enabled_fn": [lambda *_: self._show_joints()],
                                    "onclick_fn": lambda *_, joint_type=joint_type: self._on_joint_create_click(joint_type)
                                } for joint_name, joint_type in zip(JOINT_NAMES, JOINT_TYPES)
                            ],
                        },
                        # this works in omni.kit.menu.utils.add_menu_items
                        "show_fn": [lambda *_: True],
                        "enabled_fn": [lambda *_: self._show_joints()],
                    },
                    {
                        "name": "Attachment",
                        "show_fn": [lambda *_: self._show_attachment()],
                        "onclick_fn": lambda *_: self._on_create_attachment()
                    },
                    {
                        "name": "Particle System",
                        "onclick_fn": lambda *_: self._on_add_particle_system()
                    },
                    *self._menu_ext_dicts["Create"],
                ]
            },
            "glyph": PHYSICS_GLYPH_FILE,
        }

        self._viewport_create_menu = omni.kit.context_menu.add_menu(create_menu_dict, "CREATE", "omni.kit.window.viewport")
        self._stage_create_menu = omni.kit.context_menu.add_menu(create_menu_dict, "CREATE", "omni.kit.widget.stage")

        # NOTE: rewrite all to actions when the other menus support them too ...
        ext_id = "omni.physxui"
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_all_actions_for_extension(ext_id)

        def convert_to_action(item):
            fn = item.get("onclick_fn")
            name = item.get("name")
            if not fn or not name:
                return

            action_registry.register_action(
                ext_id,
                name,
                fn,
                display_name=f"Create->Physics->{name}",
                description=name,
                tag="Physics Menu Actions"
            )

            item["onclick_action"] = (ext_id, name)

        def create_submenu(item):
            item_dict = item["name"]
            name = next(iter(item_dict))
            subitems = []
            for subitem in item_dict[name]:
                if isinstance(subitem["name"], dict):
                    subitems.append(create_submenu(subitem))
                else:
                    convert_to_action(subitem)
                    subitems.append(MenuItemDescription(
                        name=subitem.get("name"),
                        enable_fn=subitem.get("enabled_fn"),
                        glyph=subitem.get("glyph"),
                        show_fn=subitem.get("show_fn"),
                        onclick_action=subitem.get("onclick_action"),
                    ))
            return MenuItemDescription(
                name=name,
                enable_fn=item.get("enabled_fn"),
                glyph=item.get("glyph"),
                sub_menu=subitems
            )

        if self._create_menu_item is not None:
            omni.kit.menu.utils.remove_menu_items([self._create_menu_item], "Create")

        self._create_menu_item = create_submenu(create_menu_dict)
        self._create_menu_item.appear_after = "Material"
        omni.kit.menu.utils.add_menu_items([self._create_menu_item], "Create")

    def _refresh_additional_menu(self):
        add_menu_dict = {
            "name": {
                "Apply Preset":
                [
                    *self._menu_ext_dicts["Add"],
                ],
            },
        }

        remove_menu_dict = {
            "name": {
                "Remove Preset":
                [
                    *self._menu_ext_dicts["Remove"],
                ],
            },
        }

        additional_menu_dict = {
            "name": {
                "Physics": [
                    add_menu_dict,
                    remove_menu_dict
                ]
            },
            "glyph": PHYSICS_GLYPH_FILE,
        }

        self._viewport_menu = omni.kit.context_menu.add_menu(additional_menu_dict, "MENU", "omni.kit.window.viewport")
        self._stage_menu = omni.kit.context_menu.add_menu(additional_menu_dict, "MENU", "omni.kit.widget.stage")

    def _remove_context_menus(self):
        self._viewport_create_menu = None
        self._stage_create_menu = None
        self._viewport_menu = None
        self._stage_menu = None

    def _on_joint_create_click(self, joint_type):
        no_joints_created_warning = "No joints were created. You must select 1 or 2 xformable prims to create joint on or between them."

        stage = self._usd_context.get_stage()
        new_joints = []

        selected_prim_paths = self._selected_prim_paths
        selected_len = len(selected_prim_paths)
        if selected_len == 1:
            (ret, new_joints) = commands.CreateJointsCommand.execute(stage, joint_type, [selected_prim_paths[0]], False)
        elif selected_len == 2:
            from_prim = stage.GetPrimAtPath(selected_prim_paths[0])
            to_prim = stage.GetPrimAtPath(selected_prim_paths[1])
            (ret, new_joint) = commands.CreateJointCommand.execute(stage, joint_type, from_prim, to_prim)
            if new_joint is not None and new_joint.IsValid():
                new_joints.append(new_joint)
        else:
            carb.log_warn(no_joints_created_warning)

        if len(new_joints) > 0:
            paths = []
            for joint in new_joints:
                paths.append(joint.GetPath().pathString)
            self._usd_context.get_selection().set_selected_prim_paths(paths, True)
        else:
            carb.log_warn(no_joints_created_warning)

    def _show_joints(self):
        selected_prim_paths = self._usd_context.get_selection().get_selected_prim_paths()

        if not len(selected_prim_paths) in [1, 2]:
            return False

        stage = self._usd_context.get_stage()
        if stage is None:
            return False

        retall = all([stage.GetPrimAtPath(path).IsA(UsdGeom.Xformable) for path in selected_prim_paths])
        return retall

    def _on_create_physics_scene(self):
        stage = self._usd_context.get_stage()
        path = omni.usd.get_stage_next_free_path(stage, "/PhysicsScene", True)
        commands.AddPhysicsSceneCommand.execute(stage, path)

    def _on_create_ground_plane(self):
        stage = self._usd_context.get_stage()
        upAxis = UsdGeom.GetStageUpAxis(stage)
        scaleFactor = utils.getUnitScaleFactor(stage)
        commands.AddGroundPlaneCommand.execute(stage, "/GroundPlane", upAxis, 25.0 * scaleFactor, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    def _on_create_physics_material(self):
        def create_material(dialog):
            stage = self._usd_context.get_stage()
            choice = dialog.get_values()
            path = omni.usd.get_stage_next_free_path(stage, "/PhysicsMaterial", True)
            omni.kit.undo.begin_group()
            if choice[0]:
                commands.AddRigidBodyMaterialCommand.execute(stage, path)
            if choice[1]:
                commands.AddDeformableBodyMaterialCommand.execute(stage, path)
            if choice[2]:
                commands.AddPBDMaterialCommand.execute(stage, path)
            omni.kit.undo.end_group()
            dialog.hide()

        dialog = OptionsDialog(
            title="Test",
            ok_handler=create_material,
            field_defs=[
                OptionsDialog.FieldDef(0, "Rigid Body Material", False),
                OptionsDialog.FieldDef(1, "Deformable Body Material", False),
                OptionsDialog.FieldDef(2, "PBD Particle Material", False),
            ])

        dialog.show()

    def _on_create_collision_group(self):
        stage = self._usd_context.get_stage()
        path = omni.usd.get_stage_next_free_path(stage, "/CollisionGroup", True)
        commands.AddCollisionGroupCommand.execute(stage, path)

    def _get_selected_attachables(self, stage):
        particlecloths = set()
        deformablebodies = set()
        deformablesurfaces = set()
        hair = set()
        rigids = set()

        for selected_path in self._usd_context.get_selection().get_selected_prim_paths():
            prim = stage.GetPrimAtPath(selected_path)
            if PhysxSchema.PhysxParticleClothAPI(prim):
                particlecloths.add(Sdf.Path(selected_path))
            elif PhysxSchema.PhysxDeformableBodyAPI(prim):
                deformablebodies.add(Sdf.Path(selected_path))
            elif PhysxSchema.PhysxDeformableSurfaceAPI(prim):
                deformablesurfaces.add(Sdf.Path(selected_path))
            elif PhysxSchema.PhysxHairAPI(prim):
                hair.add(Sdf.Path(selected_path))
            elif UsdPhysics.RigidBodyAPI(prim) or UsdPhysics.CollisionAPI(prim):
                rigids.add(Sdf.Path(selected_path))

        return list(particlecloths), list(deformablebodies), list(deformablesurfaces), list(hair), list(rigids)

    def _on_create_attachment(self):
        stage = self._usd_context.get_stage()
        particlecloths, deformablebodies, deformablesurfaces, hair, rigids = self._get_selected_attachables(stage)
        deformables = particlecloths + deformablebodies + deformablesurfaces + hair
        target_paths = deformables + rigids

        if len(target_paths) > 2:
            # Handle multiple attachments creation      
            app = AttachmentsDialog(particlecloths, deformablebodies, deformablesurfaces, hair, rigids)
        else:
            if len(target_paths) < 1:
                nm.post_notification(
                    "No attachment was created. At least one prim needs to be selected to create an attachment.",
                    duration=5,
                    status=nm.NotificationStatus.WARNING)
                return

            if len(target_paths) == 1 and len(deformables) == 0:
                nm.post_notification(
                    "No attachment was created. One or more deformable or particle cloth or deformable surface prims need to be selected.",
                    duration=5,
                    status=nm.NotificationStatus.WARNING)
                return

            if len(particlecloths) > 1:
                nm.post_notification(
                    "No attachment was created. Attachments between two prims with particle cloth component are not supported.",
                    duration=5,
                    status=nm.NotificationStatus.WARNING)
                return

            if len(deformablesurfaces) > 1:
                nm.post_notification(
                    "No attachment was created. Attachments between two prims with deformable surface component are not supported.",
                    duration=5,
                    status=nm.NotificationStatus.WARNING)
                return

            if len(hair) > 1:
                nm.post_notification(
                    "No attachment was created. Attachments between two prims with hair simulation are not supported.",
                    duration=5,
                    status=nm.NotificationStatus.WARNING)
                return

            if len(rigids) > 1:
                nm.post_notification(
                    "No attachment was created. To connect two rigid bodies use joints.",
                    duration=5,
                    status=nm.NotificationStatus.WARNING)
                return

            parent_path = deformables[0]
            att_path = parent_path.AppendElementString("attachment")
            att_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, str(att_path), False))
            if len(target_paths) < 2:
                target_paths.append(None)

            omni.kit.undo.begin_group()
            omni.kit.commands.execute("CreatePhysicsAttachmentCommand", target_attachment_path=att_path, actor0_path=target_paths[0], actor1_path=target_paths[1])
            omni.kit.undo.end_group()


    def _show_attachment(self):
        selected_prim_paths = self._usd_context.get_selection().get_selected_prim_paths()

        stage = self._usd_context.get_stage()
        if stage is None:
            return False

        particlecloths, deformablebodies, deformablesurfaces, hair, rigids = self._get_selected_attachables(stage)
        attachables = particlecloths
        attachables.extend(deformablebodies)
        attachables.extend(deformablesurfaces)
        attachables.extend(hair)
        attachables.extend(rigids)

        return attachables

    def _on_add_particle_system(self):
        stage = self._usd_context.get_stage()
        particle_system_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, str(particleUtils.get_default_particle_system_path(stage)), True))
        omni.kit.commands.execute("AddParticleSystem", target_particle_system_path=particle_system_path)
