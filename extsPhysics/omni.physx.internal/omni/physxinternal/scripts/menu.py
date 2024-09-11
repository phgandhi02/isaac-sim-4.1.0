from functools import partial
import omni.kit.ui
import omni.ui as ui
import omni.usd
import omni.kit.undo
from . import physxInternalUtils
from . physxInternalUtils import TetrahedralMeshType
from omni.physxui import PhysicsMenu, can_show_any
from pxr import UsdGeom, UsdSkel, Usd, UsdPhysics, PhysxSchema, Sdf
from omni.kit.window.popup_dialog import MessageDialog
from omni.kit.widget.stage import StageWidget
from omni.kit.property.usd.relationship import SelectionWatch
from omni.physx.scripts import utils
import typing
import weakref
from omni.kit.window.popup_dialog.options_dialog import OptionsDialog
import omni.physx.bindings._physx as pxb
import carb


def _show_tetmesh_error_dialog():
    _popup = MessageDialog(
        width=600,
        message="No valid tetmesh could be generated given the skin mesh. Please try to improve the quality of the skin mesh by making sure that it does not contain an excessive amount of triangles, does not self intersect and is watertight. Furthermore triangles in skin meshes that are close to equilateral lead to better tetmeshes.",
        ok_handler=lambda dialog: on_okay_clicked(dialog),
        ok_label="Ok",
        title="Meshing Error",
        disable_cancel_button=True
    )
    _popup.show()
    return


def on_okay_clicked(dialog: MessageDialog):
    dialog.hide()


def post_toast(msg):
    carb.log_warn(msg)


class PhysxInternalMenu:
    def __init__(self):
        self._physxProps = None

    def on_startup(self):
        self._usd_context = omni.usd.get_context()
        self._selection = self._usd_context.get_selection()

        create_menu_items = [
            {"name": "Tetrahedral Mesh", "onclick_fn": lambda *_: self._on_create_tetrahedral_mesh_dialog()},
            {"name": "Custom Deformable Body", "onclick_fn": lambda *_: self._on_create_custom_deformablebody_dialog()},
        ]

        self._menu_items = {
            "Create": create_menu_items,
        }

        for submenu, items in self._menu_items.items():
            for item in items:
                PhysicsMenu.add_context_menu(submenu, item)

    def get_deps(self):
        return "omni.physx"

    @staticmethod
    def _on_create_tetrahedral_mesh(stage, source_meshes, tetrahedral_mesh_type, voxel_resolution, simplify, simpl_accuracy, simpl_min_triangles, simpl_max_triangles):
        for source_mesh in source_meshes:
            target_mesh_path = physxInternalUtils.create_tetrahedral_mesh_path(stage, Sdf.Path(source_mesh), tetrahedral_mesh_type)
            if tetrahedral_mesh_type is TetrahedralMeshType.CONFORMING:
                success = omni.kit.commands.execute("CreateConformingTetrahedralMesh", target_mesh_path=target_mesh_path, source_mesh_path=source_mesh, simplify = simplify, simpl_accuracy=simpl_accuracy, simpl_min_triangles=simpl_min_triangles, simpl_max_triangles=simpl_max_triangles)
                if not success[1]:
                    _show_tetmesh_error_dialog()
            elif tetrahedral_mesh_type is TetrahedralMeshType.VOXEL:
                success = omni.kit.commands.execute("CreateVoxelTetrahedralMesh", target_mesh_path=target_mesh_path, source_mesh_path=source_mesh, voxel_resolution=voxel_resolution, simplify = simplify, simpl_accuracy=simpl_accuracy, simpl_min_triangles=simpl_min_triangles, simpl_max_triangles=simpl_max_triangles)
                if not success[1]:
                    _show_tetmesh_error_dialog()
            omni.usd.get_context().get_selection().set_prim_path_selected(str(target_mesh_path), True, True, True, True)


    def _on_create_collision_from_skel_mesh(self, menu, value):
        # Collect selected skeletal mesh
        stage = self._usd_context.get_stage()

        skel_bindings = []

        for selected_path in self._selection.get_selected_prim_paths():
            mesh = UsdGeom.Mesh.Get(stage, selected_path)
            if not mesh:
                continue

            skel_binding = UsdSkel.BindingAPI.Get(stage, selected_path)

            if skel_binding:
                skel_bindings.append(skel_binding)

        if len(skel_bindings) <= 0:
            raise Exception("Skeletal Mesh Collision - At less one skeletal mesh must be selected.")

        # Create dialog to collect parameters
        ui = omni.kit.ui
        self._skel_mesh_collision_wnd = None
        self._skel_mesh_collision_wnd = ui.Popup("Skeletal Mesh Collision", modal=True)
        layout = self._skel_mesh_collision_wnd.layout
        max_vert = layout.add_child(ui.FieldInt("Max Vertices Per Convex", 16))
        max_convex = layout.add_child(ui.FieldInt("Max Convexes Per Joint", 1))
        resolution = layout.add_child(ui.FieldInt("Resolution", 10000))
        button_row = layout.add_child(ui.RowLayout())
        yes = button_row.add_child(ui.Button("OK"))
        cancel = button_row.add_child(ui.Button("Cancel"))

        def on_yes(widget):
            # Create collisions for each skeletal mesh
            for skel_binding in skel_bindings:
                self._create_collision_from_skel_mesh(skel_binding, max_convex.value, max_vert.value, resolution.value)

            # Destroy dialog
            self._skel_mesh_collision_wnd = None

        def on_cancel(widget):
            self._skel_mesh_collision_wnd = None

        yes.set_clicked_fn(on_yes)
        cancel.set_clicked_fn(on_cancel)

    def _create_collision_from_skel_mesh(self, skel_binding, max_convex, max_vert, resolution):
        # Create convex mesh for joints
        geom_plugin = _geometry.acquire_geometry()

        parent_name = "collision"
        print("Generating collisions from " + str(skel_binding.GetPath()))
        geom_plugin.createConvexHullFromSkeletalMesh(
            omni.usd.get_context().get_stage_id(),
            str(skel_binding.GetPath()),
            parent_name,
            max_convex,
            max_vert,
            resolution,
            True,
        )
        # Apply kinematic actor schema to each convex mesh
        parent_path = skel_binding.GetPath().AppendElementString(parent_name)
        parent = skel_binding.GetPrim().GetStage().GetPrimAtPath(parent_path)

        for prim in Usd.PrimRange(parent):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            # Setup physics properties to make it a kinematic object
            physics_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            physics_api.CreateKinematicEnabledAttr(True)

            meshCollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(prim)
            meshCollisionAPI.CreateApproximationAttr("convexHull")

            UsdPhysics.CollisionAPI.Apply(prim)

            skel_rel = prim.GetRelationship("xformConstraint:skeleton")
            skel_prim = None
            if skel_rel:
                skel_paths = skel_rel.GetTargets()
                if len(skel_paths) > 0:
                    skel_prim = prim.GetStage().GetPrimAtPath(skel_paths[0])
            joint_attr = prim.GetAttribute("xformConstraint:joint")
            joint = joint_attr.Get()

            xform_constraint_utility.create_xform_constraint_joint(skel_prim, joint, prim)
        print("Generated collisions from " + str(skel_binding.GetPath()))

    def on_shutdown(self):
        for submenu, items in self._menu_items.items():
            for item in items:
                PhysicsMenu.remove_context_menu(submenu, item)


    class CreateTetrahedralMeshDialog:
        def __init__(self, stage, source_meshes_selection, cb_create_tetrahedral_mesh):
            self._stage = stage
            self._source_meshes = source_meshes_selection
            self._types = [TetrahedralMeshType.CONFORMING, TetrahedralMeshType.VOXEL]
            self._types_text = [TetrahedralMeshType.CONFORMING, TetrahedralMeshType.VOXEL]
            self._type = TetrahedralMeshType.CONFORMING
            self._voxel_resolution = 10
            self._simplify = False
            self._accuracy = 0.55
            self._min_triangles = 1000
            self._max_triangles = 20000

            self._cb_create_tetrahedral_mesh = cb_create_tetrahedral_mesh

            self._create_tetrahedral_mesh_wnd = omni.ui.Window("Create Tetrahedral Mesh", visible=True, height=0, dockPreference=omni.ui.DockPreference.DISABLED)
            self._create_tetrahedral_mesh_wnd.flags = (omni.ui.WINDOW_FLAGS_NO_COLLAPSE
                                                       | omni.ui.WINDOW_FLAGS_NO_RESIZE
                                                       | omni.ui.WINDOW_FLAGS_NO_SCROLLBAR
                                                       | omni.ui.WINDOW_FLAGS_MODAL)

            with self._create_tetrahedral_mesh_wnd.frame:
                with omni.ui.VStack():
                    omni.ui.Spacer(height=10)
                    with omni.ui.HStack():
                        omni.ui.Spacer(width=10)
                        # @sschirm: tooltips not working
                        type_text = """Type of tetrahedral mesh:
                                       "conforming": create space filling tetrahedrons inside closed triangle source mesh, with surface tetrahedrons aligning with triangles.
                                       "voxel": create tetrahedrons on a voxel grid such as to embed all tetrahedrons of the corresponding conforming tetrahedral mesh."""
                        omni.ui.Label("Type", tooltip=type_text, word_wrap=True, height=0)
                        omni.ui.Spacer(width=10)
                        combo_box = omni.ui.ComboBox(0, *self._types_text)
                        combo_box.model.add_item_changed_fn(self.cb_type_changed)
                        omni.ui.Spacer(width=10)
                    omni.ui.Spacer(height=10)
                    with omni.ui.HStack():
                        omni.ui.Spacer(width=10)
                        label = omni.ui.Label("Voxel Resolution", tooltip="Number of voxels along largest dimension of mesh bounding box", word_wrap=True, height=0)
                        omni.ui.Spacer(width=10)
                        int_drag = omni.ui.IntDrag(min=1, max=20, step=1)
                        int_drag.model.add_value_changed_fn(self.cb_voxel_resolution_changed)
                        int_drag.model.set_value(self._voxel_resolution)
                        self._voxel_int_drag = int_drag
                        self._voxel_label = label
                        omni.ui.Spacer(width=10)
                    omni.ui.Spacer(height=10)
                    with omni.ui.HStack():
                        omni.ui.Spacer(width=10)
                        label = omni.ui.Label("Simplify", tooltip="Enables collision mesh simplification", word_wrap=True, height=0)
                        omni.ui.Spacer(width=10)
                        check_box = omni.ui.CheckBox()
                        check_box.model.add_value_changed_fn(self._simplify_changed)
                        check_box.model.set_value(self._simplify)
                        self._simplify_check_box = check_box
                        self._simplify_label = label
                        omni.ui.Spacer(width=10)
                    omni.ui.Spacer(height=10)
                    with omni.ui.HStack():
                        omni.ui.Spacer(width=10)
                        label = omni.ui.Label("Accuracy", tooltip="Specifies the max allowed deviation from the input mesh", word_wrap=True, height=0)
                        omni.ui.Spacer(width=10)
                        double_drag = omni.ui.FloatDrag(min=0, max=1, step=0.001)
                        double_drag.model.add_value_changed_fn(self._accuracy_changed)
                        double_drag.model.set_value(self._accuracy)
                        self._accuracy_double_drag = double_drag
                        self._accuracy_label = label
                        omni.ui.Spacer(width=10)
                    omni.ui.Spacer(height=10)
                    with omni.ui.HStack():
                        omni.ui.Spacer(width=10)
                        label = omni.ui.Label("Min Triangle Count", tooltip="The minimal number of triangles the simplified mesh should have", word_wrap=True, height=0)
                        omni.ui.Spacer(width=10)
                        int_drag = omni.ui.IntDrag(min=100, max=100000, step=100)
                        int_drag.model.add_value_changed_fn(self._min_triangles_changed)
                        int_drag.model.set_value(self._min_triangles)
                        self._min_triangles_int_drag = int_drag
                        self._min_triangles_label = label
                        omni.ui.Spacer(width=10)
                    omni.ui.Spacer(height=10)
                    with omni.ui.HStack():
                        omni.ui.Spacer(width=10)
                        label = omni.ui.Label("Max Triangle Count", tooltip="The maximal number of triangles the simplified mesh should have", word_wrap=True, height=0)
                        omni.ui.Spacer(width=10)
                        int_drag = omni.ui.IntDrag(min=100, max=100000, step=100)
                        int_drag.model.add_value_changed_fn(self._max_triangles_changed)
                        int_drag.model.set_value(self._max_triangles)
                        self._max_triangles_int_drag = int_drag
                        self._max_triangles_label = label
                        omni.ui.Spacer(width=10)
                    omni.ui.Spacer(height=20)
                    with omni.ui.HStack(width=200):
                        omni.ui.Spacer(width=10)
                        ok_button = omni.ui.Button("OK", height=30)
                        ok_button.set_clicked_fn(self.cb_ok_fn)
                        omni.ui.Spacer(width=10)
                        cancel_button = omni.ui.Button("Cancel", height=30)
                        cancel_button.set_clicked_fn(self.cb_cancel_fn)
                    omni.ui.Spacer(height=10)
                self.cb_type_changed(combo_box.model, None)

        def cb_type_changed(self, model, _):
            sel = model.get_item_value_model().as_int
            self._type = self._types[sel]
            ENABLED_STYLE = {"color": 0xffcccccc}
            DISABLED_STYLE = {"color": 0xff888888}
            if self._type == TetrahedralMeshType.VOXEL:
                self._voxel_int_drag.enabled = True
                self._voxel_label.set_style(ENABLED_STYLE)
                self._voxel_int_drag.set_style(ENABLED_STYLE)
            else:
                self._voxel_int_drag.enabled = False
                self._voxel_label.set_style(DISABLED_STYLE)
                self._voxel_int_drag.set_style(DISABLED_STYLE)


        def cb_voxel_resolution_changed(self, model):
            self._voxel_resolution = model.get_value_as_int()

        def _accuracy_changed(self, model):
            self._accuracy = model.get_value_as_float()

        def _min_triangles_changed(self, model):
            self._min_triangles = model.get_value_as_int()

        def _max_triangles_changed(self, model):
            self._max_triangles = model.get_value_as_int()

        def _simplify_changed(self, model):
            self._simplify = model.get_value_as_bool()

        def cb_ok_fn(self):
            self._create_tetrahedral_mesh_wnd.visible = False
            self._cb_create_tetrahedral_mesh(self._stage, self._source_meshes, self._type, self._voxel_resolution, self._simplify, self._accuracy, self._min_triangles, self._max_triangles)

        def cb_cancel_fn(self):
            self._create_tetrahedral_mesh_wnd.visible = False

    def _on_create_tetrahedral_mesh_dialog(self):
        stage = self._usd_context.get_stage()

        prim_paths = self._selection.get_selected_prim_paths()
        if len(prim_paths) == 0:
            post_toast("No meshes selected.")
            return

        source_meshes = []
        for path in prim_paths:
            prim = stage.GetPrimAtPath(path)
            if prim.IsA(UsdGeom.Mesh) or prim.IsA(PhysxSchema.TetrahedralMesh):
                source_meshes.append(path)

        if len(source_meshes) == 0:
            post_toast("None of the selected primitives can be used as a source mesh for tetrahedral mesh generation.")
            return

        self._create_tetrahedral_mesh_dialog = self.CreateTetrahedralMeshDialog(stage, source_meshes, self._on_create_tetrahedral_mesh)

    def _on_create_custom_deformablebody_dialog(self):
        stage = self._usd_context.get_stage()
        prim_paths = self._usd_context.get_selection().get_selected_prim_paths()

        tetrahedral_meshes = []
        usdgeom_mesh = Sdf.Path()

        # todo preist: Limit to max 1 mesh and 2 tetmeshes here?
        for path in prim_paths:
            prim = stage.GetPrimAtPath(path)
            if prim.IsA(PhysxSchema.TetrahedralMesh):
                tetrahedral_meshes.append(path)
            if prim.IsA(UsdGeom.Mesh):
                usdgeom_mesh = path

        self._create_custom_deformablebody_dialog = CreateCustomDeformableBodyDialog(
            usdgeom_mesh_path=usdgeom_mesh, tetrahedral_mesh_paths=tetrahedral_meshes, cb_create_deformablebody=self._on_create_deformablebody)

    @staticmethod
    def _on_create_deformablebody(skin_mesh_path: Sdf.Path, collision_mesh_path: Sdf.Path, simulation_mesh_path: Sdf.Path):
        stage = omni.usd.get_context().get_stage()

        skin_prim = None
        collision_prim = None
        simulation_prim = None
        conflict_msg = "The primitive is not valid or a conflicting API already exists in the tree for the specified "
        if skin_mesh_path:
            skin_prim = stage.GetPrimAtPath(skin_mesh_path)
            if not skin_prim or pxb.hasconflictingapis_PhysxDeformableBodyAPI(skin_prim, False):
                post_toast(conflict_msg + "skin primitive")
                return
        else:
            post_toast("No appropriate skin mesh path")
            return

        if collision_mesh_path:
            collision_prim = stage.GetPrimAtPath(collision_mesh_path)
            if not collision_prim or pxb.hasconflictingapis_PhysxDeformableBodyAPI(collision_prim, False):
                post_toast(conflict_msg + "collision primitive")
                return
            if not collision_prim.IsA(PhysxSchema.TetrahedralMesh):
                post_toast("No appropriate collision tetrahedral mesh")
                return

        if simulation_mesh_path:
            simulation_prim = stage.GetPrimAtPath(simulation_mesh_path)
            if not simulation_prim or pxb.hasconflictingapis_PhysxDeformableBodyAPI(simulation_prim, False):
                post_toast(conflict_msg + "simulation primitive")
                return
            if not simulation_prim.IsA(PhysxSchema.TetrahedralMesh):
                post_toast("No appropriate simulation tetrahedral mesh")
                return

        if skin_prim:
            if not skin_prim.IsA(UsdGeom.Mesh):
                post_toast("No appropriate skin mesh")
                return
        else:
            if not collision_prim:
                post_toast("No appropriate collision mesh to generate skin mesh from")
                return

        omni.kit.undo.begin_group()
        if not skin_prim:
            if collision_mesh_path:
                omni.kit.commands.execute("CreateSkinMeshFromTetrahedralMesh",
                                            target_skin_mesh_path=skin_mesh_path,
                                            source_tetrahedral_mesh_path=collision_mesh_path)

        success = omni.kit.commands.execute("AddDeformableBodyComponent",
                                            skin_mesh_path=skin_mesh_path,
                                            collision_mesh_path=collision_mesh_path,
                                            simulation_mesh_path=simulation_mesh_path)
        omni.kit.undo.end_group()

        if success[1]:
            omni.usd.get_context().get_selection().set_prim_path_selected(str(skin_mesh_path), True, True, True, True)
        else:
            _show_tetmesh_error_dialog()


class CreateCustomDeformableBodyDialog:
    def __init__(self, usdgeom_mesh_path: Sdf.Path, tetrahedral_mesh_paths: typing.List[Sdf.Path], cb_create_deformablebody):
        """
        Creates dialogue for user to adjust selection (if any) of tet-meshes used for deformable body creation.

        Args:
            usdgeom_mesh_path: Sdf.Path to skin mesh being simulated as a deformable body.
            tetrahedral_mesh_paths: List of user-selected TetrahedralMesh Sdf.Paths used for collision or simulation meshes.

                Valid configurations:
                1. usdgeom_mesh_path provided, len(tetrahedral_mesh_paths) in [0, 1, 2]
                2. usdgeom_mesh_path == None, len(tetrahedral_mesh_paths) in [1, 2]
            cb_create_deformablebody: Callback for creating softbody from user-dialogue mesh-path combination.
        """

        self.ENABLED_STYLE = {"color": 0xffcccccc}
        self.DISABLED_STYLE = {"color": 0xff888888}

        # prim pickers:
        stage = omni.usd.get_context().get_stage()
        self._skin_mesh_picker = PrimPicker(stage, self.cb_skin_mesh_changed, [UsdGeom.Mesh], filter_lambda=None)
        self._coll_tet_picker = PrimPicker(stage, self.cb_collision_mesh_changed, [PhysxSchema.TetrahedralMesh], filter_lambda=None)
        self._sim_tet_picker = PrimPicker(stage, self.cb_simulation_mesh_changed, [PhysxSchema.TetrahedralMesh], filter_lambda=None)

        # create and reset button and mesh attributes
        self._reset_buttons()
        self._reset_meshes()

        # figure out a collision mesh candidate:
        for meshPath in tetrahedral_mesh_paths:
            if "conforming" in meshPath:
                self._collision_mesh = meshPath

        # if there is no conforming, pick the first tetmesh provided:
        if not self._collision_mesh:
            if len(tetrahedral_mesh_paths) > 0:
                self._collision_mesh = tetrahedral_mesh_paths[0]

        # can only have a sim mesh if there is a collision mesh
        if self._collision_mesh:
            # pick a voxel candidate first:
            for meshPath in tetrahedral_mesh_paths:
                if "voxel" in meshPath and meshPath != self._collision_mesh:
                    self._simulation_mesh = meshPath
            # otherwise search for first possible fallback:
            if not self._simulation_mesh:
                for meshPath in tetrahedral_mesh_paths:
                    if meshPath != self._collision_mesh:
                        self._simulation_mesh = meshPath
                        break

        # set skin mesh to input
        self.cb_skin_mesh_changed(usdgeom_mesh_path)

        self._cb_create_deformablebody = cb_create_deformablebody

        self._create_softbody_wnd = omni.ui.Window("Create Custom Deformable Body", visible=True, height=0, dockPreference=omni.ui.DockPreference.DISABLED)
        self._create_softbody_wnd.flags = (omni.ui.WINDOW_FLAGS_NO_COLLAPSE
                                           | omni.ui.WINDOW_FLAGS_NO_RESIZE
                                           | omni.ui.WINDOW_FLAGS_NO_SCROLLBAR
                                           | omni.ui.WINDOW_FLAGS_MODAL)

        # ui dims:
        labelWidth = 60
        removeButtonWidth = 20
        spacingPixels = 10

        with self._create_softbody_wnd.frame:
            with omni.ui.VStack(style={"Button:disabled": {"background_color": 0xFF595959}}):
                omni.ui.Spacer(height=spacingPixels)
                with omni.ui.HStack(spacing=spacingPixels):
                    omni.ui.Label("Skin Mesh", word_wrap=True, height=0, width=labelWidth)
                    self._skin_mesh_button = omni.ui.Button("", clicked_fn=self._skin_mesh_picker.show)
                    self._skin_mesh_remove_button = omni.ui.Button(  # remove button
                        "-", clicked_fn=partial(self.cb_skin_mesh_changed, path=None), width=removeButtonWidth)
                omni.ui.Spacer(height=spacingPixels)
                with omni.ui.HStack(spacing=spacingPixels):
                    omni.ui.Label("Collision Mesh", word_wrap=True, width=labelWidth)
                    self._coll_mesh_button = omni.ui.Button("", clicked_fn=self._coll_tet_picker.show)
                    self._coll_mesh_remove_button = omni.ui.Button(  # remove button
                        "-", clicked_fn=partial(self.cb_collision_mesh_changed, path=None), width=removeButtonWidth)
                omni.ui.Spacer(height=spacingPixels)

                omni.ui.Spacer(height=spacingPixels)
                with omni.ui.HStack(spacing=spacingPixels):
                    omni.ui.Label("Simulation Mesh", word_wrap=True, width=labelWidth)
                    self._sim_mesh_button = omni.ui.Button("", clicked_fn=self._sim_tet_picker.show)
                    self._sim_mesh_remove_button = omni.ui.Button(  # remove button
                        "-", clicked_fn=partial(self.cb_simulation_mesh_changed, path=None), width=removeButtonWidth)
                omni.ui.Spacer(height=2 * spacingPixels)
                omni.ui.Spacer(height=10)

                with omni.ui.HStack():
                    self._ok_button = omni.ui.Button("OK", height=30)
                    self._ok_button.set_clicked_fn(self.cb_ok_fn)
                    cancel_button = omni.ui.Button("Cancel", height=30)
                    cancel_button.set_clicked_fn(self.cb_cancel_fn)
                omni.ui.Spacer(height=10)

        self._validate_mesh_paths()


    def _validate_mesh_paths(self):
        """
        Implements checking valid path configurations based on user selections in dialogue.
        """
        if self._skin_mesh_button is None:
            # if buttons are not shown, there is no dialogue open and this should not run
            return

        # reset to default labels
        self._skin_mesh_button.text = "Select Mesh"
        self._coll_mesh_button.text = "Select optional TetrahedralMesh"
        self._sim_mesh_button.text = "Select optional TetrahedralMesh"
        self._ok_button.enabled = True

        # invalid config:
        if not self._skin_mesh and not self._collision_mesh:
            self._skin_mesh_button.text = "Select Mesh"  # also a valid starting point
            self._ok_button.enabled = False

        # default or minimal or user-friendly default:
        elif not self._skin_mesh:
            # must have collision mesh:
            assert self._collision_mesh
            self._coll_mesh_button.text = self._collision_mesh
            if not self._simulation_mesh:
                self._sim_mesh_button.text = "Generated from collision"
                # enable simplification settings
                self._enable_col_mesh_gui_settings()
            else:
                self._sim_mesh_button.text = self._simulation_mesh

        # there is a skin mesh:
        else:
            self._skin_mesh_button.text = self._skin_mesh
            # user-friendly from skin mesh
            if not self._collision_mesh:
                self._coll_mesh_button.text = "Generated from skin"
                self._sim_mesh_button.text = "Generated from skin"
            else:
                self._coll_mesh_button.text = self._collision_mesh
                # stop-gap with or without sim mesh:
                if self._simulation_mesh:
                    self._sim_mesh_button.text = self._simulation_mesh
                else:
                    self._sim_mesh_button.text = "Generated from collision"

        # update button enable/disable
        # can only add/have a sim mesh if there is a collision mesh:
        self._sim_mesh_button.enabled = bool(self._collision_mesh)
        if not self._collision_mesh:
            self._simulation_mesh = Sdf.Path()
        self._skin_mesh_remove_button.enabled = bool(self._skin_mesh) and bool(self._collision_mesh)
        self._coll_mesh_remove_button.enabled = bool(self._collision_mesh)
        self._sim_mesh_remove_button.enabled = bool(self._simulation_mesh)

    def cb_collision_mesh_changed(self, path: Sdf.Path):
        self._collision_mesh = path
        self._validate_mesh_paths()

    def cb_simulation_mesh_changed(self, path: Sdf.Path):
        self._simulation_mesh = path
        self._validate_mesh_paths()

    def cb_skin_mesh_changed(self, path: Sdf.Path):
        if not path and bool(self._collision_mesh):
            collision_path = Sdf.Path(self._collision_mesh)
            surface_path = collision_path.ReplaceName(collision_path.name + '_surface')
            self._skin_mesh = str(surface_path)
        else:
            self._skin_mesh = path
        self._validate_mesh_paths()

    def cb_ok_fn(self):
        self._reset_buttons()
        self._create_softbody_wnd.visible = False
        self._cb_create_deformablebody(self._skin_mesh, self._collision_mesh, self._simulation_mesh)

    def cb_cancel_fn(self):
        self._create_softbody_wnd.visible = False
        self._reset_meshes()
        self._reset_buttons()

    def _reset_buttons(self):
        self._skin_mesh_button = None
        self._coll_mesh_button = None
        self._sim_mesh_button = None
        self._skin_mesh_remove_button = None
        self._coll_mesh_remove_button = None
        self._sim_mesh_remove_button = None
        self._ok_button = None

    def _reset_meshes(self):
        self._collision_mesh = Sdf.Path()
        self._simulation_mesh = Sdf.Path()
        self._skin_mesh = Sdf.Path()


class PrimPicker:
    def __init__(self, stage, on_select_fn, filter_type_list, filter_lambda=None, **kwargs):
        self._weak_stage = weakref.ref(stage)
        self._on_select_fn = on_select_fn
        self._selected_path = Sdf.Path()
        self._window_title = "Select a " + " or ".join([x.__name__ for x in filter_type_list])
        self._window_title = kwargs.pop("window_title", self._window_title)
        self._window_width = kwargs.pop("window_width", 400)
        self._window_height = kwargs.pop("window_height", 400)

        def on_window_visibility_changed(visible):
            if not visible:
                self._stage_widget.open_stage(None)
            else:
                # Only attach the stage when picker is open. Otherwise the Tf notice listener in StageWidget kills perf
                self._stage_widget.open_stage(self._weak_stage())

        self._window = ui.Window(
            self._window_title,
            width=self._window_width,
            height=self._window_height,
            visible=False,
            flags=ui.WINDOW_FLAGS_MODAL,
            visibility_changed_fn=on_window_visibility_changed,
        )
        with self._window.frame:
            with ui.VStack():
                with ui.Frame():
                    self._stage_widget = StageWidget(None, columns_enabled=["Type"])
                    self._selection_watch = SelectionWatch(
                        stage=stage,
                        on_selection_changed_fn=self._on_selection_changed,
                        filter_type_list=filter_type_list,
                        filter_lambda=filter_lambda,
                    )
                    self._stage_widget.set_selection_watch(self._selection_watch)

                    if len(filter_type_list):
                        self._stage_widget._filter_by_type(filter_type_list, True)
                    if filter_lambda is not None:
                        self._stage_widget._filter_by_lambda({"relpicker_filter": filter_lambda}, True)

                def on_select(weak_self):
                    weak_self = weak_self()
                    if not weak_self:
                        return
                    weak_self._window.visible = False
                    self._on_select_fn(self._selected_path)

                with ui.VStack(
                    height=0, style={"Button.Label:disabled": {"color": 0xFF606060}}
                ):  # TODO consolidate all styles
                    self._label = ui.Label("Selected Path:\n\tNone")
                    self._button = ui.Button(
                        "Select", height=10, clicked_fn=partial(on_select, weak_self=weakref.ref(self)), enabled=False
                    )

    def clean(self):
        self._window.set_visibility_changed_fn(None)
        self._window = None
        self._selection_watch = None
        self._stage_widget.open_stage(None)
        self._stage_widget.destroy()
        self._stage_widget = None

    def show(self):
        self._selection_watch.reset(1)
        self._window.visible = True

    def _on_selection_changed(self, paths):
        if len(paths) > 0:
            self._selected_path = paths[0]
        else:
            self._selected_path = None
        if self._button:
            self._button.enabled = self._selected_path is not None
        if self._label:
            if self._selected_path is not None:
                label_text = f"Selected Path: {self._selected_path}"
                self._label.text = label_text
            else:
                self._label.text = ""
