from typing import List

import carb
import omni
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade


class MeshMerger(object):
    def __init__(self, stage):
        self._clear_parent_xform = False
        self._combine_materials = False
        self._deactivate_source = False
        self._materials_destination = ""
        self._output_mesh = ""
        self._selected_objects = []
        self._stage = stage

        self._total_meshes = 0
        self._total_subsets = 0
        self._total_materials = 0
        self._meshes_to_merge = []
        self._created_materials = []

    @property
    def total_meshes(self):
        return self._total_meshes

    @property
    def total_subsets(self):
        return self._total_subsets

    @property
    def total_materials(self):
        return self._total_materials

    @property
    def meshes_to_merge(self):
        return self._meshes_to_merge

    @property
    def clear_parent_xform(self):
        return self._clear_parent_xform

    @clear_parent_xform.setter
    def clear_parent_xform(self, value):
        self._clear_parent_xform = value

    @property
    def deactivate_source(self):
        return self._deactivate_source

    @deactivate_source.setter
    def deactivate_source(self, value):
        self._deactivate_source = value

    @property
    def selected_objects(self):
        return self._selected_objects

    @property
    def combine_materials(self):
        return self._combine_materials

    @combine_materials.setter
    def combine_materials(self, value):
        self._combine_materials = value

    @property
    def materials_destination(self):
        return self._materials_destination

    @materials_destination.setter
    def materials_destination(self, value):
        self._materials_destination = value

    @property
    def output_mesh(self):
        return self._output_mesh

    @output_mesh.setter
    def output_mesh(self, value):

        self._output_mesh = omni.usd.get_stage_next_free_path(self._stage, value, False)

    def fix_material_sources(self, mat):
        shader_path = mat.GetPrim().GetChildren()[0].GetPath()

        material_outputs = mat.GetOutputs()

        # List to store all connected sources
        connected_sources = []

        # Iterate over each output connection
        for output in material_outputs:
            # Get the source connected to this output
            connected_source = output.GetRawConnectedSourcePaths()
            for i in range(len(connected_source)):
                # Append the connected source to the list
                connected_source[i] = Sdf.Path(f"{shader_path}.{connected_source[i].name}")
                connected_sources.append(connected_source[i])
            output.ClearSources()
            for path in connected_sources:
                output.ConnectToSource(path)

    def update_selection(self, selection, stage=None):

        self._selected_objects = selection
        if stage:
            self._stage = stage
        total_meshes = 0
        total_subsets = 0
        self._total_meshes = 0
        self._total_subsets = 0
        self._total_materials = 0
        self._meshes_to_merge = []
        for prim in self.selected_objects:
            curr_prim = self._stage.GetPrimAtPath(prim)
            materials = {}
            primrange = [child_prim for child_prim in Usd.PrimRange(curr_prim, Usd.TraverseInstanceProxies())] + [
                curr_prim
            ]
            for child_prim in primrange:
                imageable = UsdGeom.Imageable(child_prim)
                if imageable:
                    visible = imageable.ComputeVisibility(Usd.TimeCode.Default())
                    if (
                        child_prim.IsA(UsdGeom.Mesh)
                        and visible != UsdGeom.Tokens.invisible
                        and imageable.GetPurposeAttr().Get() in ["default", "render"]
                    ):
                        self._meshes_to_merge.append(child_prim)
                        usdMesh = UsdGeom.Mesh(child_prim)
                        mat, rel = UsdShade.MaterialBindingAPI(usdMesh).ComputeBoundMaterial()
                        mat_path = str(mat.GetPath())
                        if self.combine_materials:
                            mat_path = "{}/{}".format(
                                self.materials_destination,
                                mat_path.rsplit("/", 1)[-1],
                            )

                        if not rel:
                            mat_path = "/None"
                        if rel:
                            materials[mat_path] = 1
                        subsets = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(child_prim))
                        if len(subsets):
                            total_subsets = total_subsets + len(subsets)
                            for s in subsets:
                                mat, rel = UsdShade.MaterialBindingAPI(s).ComputeBoundMaterial()
                                mat_path = str(mat.GetPath())
                                if self.combine_materials:
                                    mat_path = "{}/{}".format(
                                        self.materials_destination,
                                        mat_path.rsplit("/", 1)[-1],
                                    )
                                if not rel:
                                    mat_path = "/None"
                                materials[mat_path] = 1

                        total_meshes = total_meshes + 1

            # print(*materials, sep = "\n")

            self._total_meshes = total_meshes
            self._total_subsets = total_subsets
            self._total_materials = len(materials)

    def merge_meshes(self):
        meshes = []
        curr_prim = self._stage.GetPrimAtPath(self.selected_objects[0])
        prim_transform = omni.usd.get_world_transform_matrix(curr_prim, Usd.TimeCode.Default())

        if self.combine_materials and not self._stage.GetPrimAtPath(self.materials_destination):
            self._stage.DefinePrim(self.materials_destination, "Scope")

        for prim in self._meshes_to_merge:
            if prim:
                usdMesh = UsdGeom.Mesh(prim)
                mesh = {}
                mesh["points"] = usdMesh.GetPointsAttr().Get()
                world_mtx = omni.usd.get_world_transform_matrix(prim, Usd.TimeCode.Default())
                if self.clear_parent_xform:
                    world_mtx = world_mtx
                else:
                    world_mtx = world_mtx * prim_transform.GetInverse()
                world_rot = world_mtx.ExtractRotation()
                mesh["points"][:] = [world_mtx.TransformAffine(x) for x in mesh["points"]]
                mesh["normals"] = usdMesh.GetNormalsAttr().Get()
                mesh["attr_normals"] = usdMesh.GetPrim().GetAttribute("primvars:normals").Get()
                mesh["attr_normals_indices"] = usdMesh.GetPrim().GetAttribute("primvars:normals:indices").Get()
                if not mesh["attr_normals"]:
                    mesh["attr_normals"] = []
                if not mesh["attr_normals_indices"]:
                    mesh["attr_normals_indices"] = []
                if mesh["normals"]:
                    mesh["normals"][:] = [world_rot.TransformDir(x).GetNormalized() for x in mesh["normals"]]
                else:
                    mesh["normals"] = []
                    carb.log_warn(f"mesh doesn't contain normals: ({prim.GetName()})")
                if mesh["attr_normals"]:
                    mesh["attr_normals"][:] = [world_rot.TransformDir(x) for x in mesh["attr_normals"]]
                mesh["vertex_counts"] = usdMesh.GetFaceVertexCountsAttr().Get()
                mesh["vertex_indices"] = usdMesh.GetFaceVertexIndicesAttr().Get()
                mesh["name"] = prim.GetName()
                mat, rel = UsdShade.MaterialBindingAPI(usdMesh).ComputeBoundMaterial()
                if mat and rel:
                    mat_path = str(mat.GetPath())
                    if self.combine_materials:
                        _mat_path = "{}/{}".format(self.materials_destination, mat_path.rsplit("/", 1)[-1])
                        if not self._stage.GetPrimAtPath(_mat_path):
                            omni.kit.commands.execute(
                                "CopyPrimCommand", path_from=str(mat.GetPath()), path_to=_mat_path
                            )
                            self._created_materials.append(_mat_path)
                            mat = UsdShade.Material(self._stage.GetPrimAtPath(_mat_path))
                            self.fix_material_sources(mat)
                        mat_path = _mat_path
                else:
                    mat_path = "/None"
                # if rel:
                #     mesh["mat"] = str(mat.GetPath())
                # else:
                mesh["mat"] = mat_path
                subsets = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
                mesh["subset"] = []
                for s in subsets:
                    mat, rel = UsdShade.MaterialBindingAPI(s).ComputeBoundMaterial()
                    if mat and rel:
                        mat_path = str(mat.GetPath())
                        if self.combine_materials:
                            _mat_path = "{}/{}".format(self.materials_destination, mat_path.rsplit("/", 1)[-1])
                            if not self._stage.GetPrimAtPath(_mat_path):
                                omni.kit.commands.execute(
                                    "CopyPrimCommand", path_from=str(mat.GetPath()), path_to=_mat_path
                                )
                                self._created_materials.append(_mat_path)
                                mat = UsdShade.Material(self._stage.GetPrimAtPath(_mat_path))
                                self.fix_material_sources(mat)
                            mat_path = _mat_path
                        if not rel:
                            mat_path = "/None"
                        mesh["subset"].append((mat_path, s.GetIndicesAttr().Get()))
                meshes.append(mesh)

        carb.log_info(f"Merging: {self._total_meshes} meshes")
        all_points = []
        all_normals = []
        all_normals_attr = []
        all_normals_indices = []
        all_vertex_counts = []
        all_vertex_indices = []
        all_mats = {}
        index_offset = 0
        normals_offset = 0
        index = 0
        range_offset = 0
        for mesh in meshes:
            all_points.extend(mesh["points"])
            all_normals.extend(mesh["normals"])
            all_normals_attr.extend(mesh["attr_normals"])
            mesh["attr_normals_indices"][:] = [x + normals_offset for x in mesh["attr_normals_indices"]]
            all_normals_indices.extend(mesh["attr_normals_indices"])
            if mesh["normals"]:
                mesh["normals"][:] = [world_rot.TransformDir(x).GetNormalized() for x in mesh["normals"]]
            all_vertex_counts.extend(mesh["vertex_counts"])
            mesh["vertex_indices"][:] = [x + index_offset for x in mesh["vertex_indices"]]
            all_vertex_indices.extend(mesh["vertex_indices"])
            # all_st.extend(mesh["st"])
            index_offset = index_offset + len(meshes[index]["points"])
            normals_offset = normals_offset + len(mesh["attr_normals_indices"])
            # print("Offset", index_offset)
            index = index + 1
            # create the material entry
            if len(mesh["subset"]) == 0:
                if mesh["mat"] not in all_mats:
                    all_mats[mesh["mat"]] = []
                all_mats[mesh["mat"]].extend([*range(range_offset, range_offset + len(mesh["vertex_counts"]), 1)])
            else:
                for subset in mesh["subset"]:
                    if subset[0] not in all_mats:
                        all_mats[subset[0]] = []
                    all_mats[subset[0]].extend([*(x + range_offset for x in subset[1])])
            range_offset = range_offset + len(mesh["vertex_counts"])
        curr_prim = self._stage.GetPrimAtPath(self.selected_objects[0])
        merged_path = "/Merged/" + str(curr_prim.GetName())
        merged_path = omni.usd.get_stage_next_free_path(self._stage, merged_path, False)
        carb.log_info(f"Merging to path: {merged_path}")
        merged_mesh = UsdGeom.Mesh.Define(self._stage, merged_path)
        xform = UsdGeom.Xformable(merged_mesh)
        xform_op_t = xform.AddXformOp(UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, "")
        xform_op_r = xform.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
        if not self.clear_parent_xform:
            xform_op_t.Set(prim_transform.ExtractTranslation())
            q = prim_transform.ExtractRotation().GetQuaternion()
            xform_op_r.Set(Gf.Quatd(q.GetReal(), q.GetImaginary()))
        # xform_op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
        # if not self.parent_xform.get_value_as_bool():
        # xform_op.Set(prim_transform)
        # merged_mesh.CreateSubdivisionSchemeAttr("none")
        # merged_mesh.CreateTriangleSubdivisionRuleAttr("smooth")
        merged_mesh.CreatePointsAttr(all_points)
        if all_normals:
            merged_mesh.CreateNormalsAttr(all_normals)
            merged_mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        merged_mesh.CreateSubdivisionSchemeAttr("none")
        merged_mesh.CreateFaceVertexCountsAttr(all_vertex_counts)
        merged_mesh.CreateFaceVertexIndicesAttr(all_vertex_indices)
        if all_normals_attr:
            normals_attr = merged_mesh.GetPrim().CreateAttribute(
                "primvars:normals", Sdf.ValueTypeNames.Float3Array, False
            )
            normals_attr.Set(all_normals_attr)
            normals_attr.SetMetadata("interpolation", "vertex")
            merged_mesh.GetPrim().CreateAttribute("primvars:normals:indices", Sdf.ValueTypeNames.IntArray, False).Set(
                all_normals_indices
            )
        extent = merged_mesh.ComputeExtent(all_points)
        merged_mesh.CreateExtentAttr().Set(extent)
        # texCoord = merged_mesh.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying)
        # texCoord.Set(all_st)
        # print(all_mats)
        for name, counts in sorted(all_mats.items(), key=lambda a: a[0].rsplit("/", 1)[-1]):
            subset_name = merged_path + "/{}".format(name.rsplit("/", 1)[-1])
            geomSubset = UsdGeom.Subset.Define(
                self._stage, omni.usd.get_stage_next_free_path(self._stage, subset_name, False)
            )
            geomSubset.CreateElementTypeAttr("face")
            geomSubset.CreateFamilyNameAttr("materialBind")
            # print(mesh["vertex_indices"])
            geomSubset.CreateIndicesAttr(counts)
            if name != "/None":
                material = UsdShade.Material.Get(self._stage, name)
                binding_api = UsdShade.MaterialBindingAPI(geomSubset)
                binding_api.Bind(material)

        if self.deactivate_source:
            for source in self.selected_objects:
                prim = self._stage.GetPrimAtPath(source)
                prim.SetActive(False)

    def reactivate_sources(self):
        if self.deactivate_source:
            for source in self.selected_objects:
                prim = self._stage.GetPrimAtPath(source)
                prim.SetActive(True)

    def remove_created_materials(self):
        for mat in self._created_materials:
            self._stage.RemovePrim(mat)
        self._created_materials = []
