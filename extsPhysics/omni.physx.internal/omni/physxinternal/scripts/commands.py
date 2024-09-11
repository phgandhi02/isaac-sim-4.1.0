import carb
import omni.usd
import omni.kit
from omni.kit.commands import Command, execute
from omni.kit.usd_undo import UsdLayerUndo
from pxr import UsdGeom, Usd, Sdf, Gf, PhysxSchema
from omni.physx import get_physx_cooking_interface
from omni.physx.scripts import physicsUtils, deformableUtils
import fnmatch
import typing
from omni.physxcommands import UnapplyAPISchemaCommand, RemoveAttributeCommand, DeletePrimsCommand
from omni.physx.scripts.pythonUtils import autoassign
from omni.convexdecomposition.bindings import _convexdecomposition
import carb.settings
from omni.physx.bindings._physx import SETTING_VISUALIZATION_GAP
from omni.physxcommands import PhysicsCommand


def _getSchemaRegistryProperties(api_name):
    primDef = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(api_name)
    return [primDef.GetSchemaPropertySpec(x) for x in primDef.GetPropertyNames()]

class CreateSkinMeshFromTetrahedralMeshCommand(Command):

    @autoassign
    def __init__(self, target_skin_mesh_path: Sdf.Path = Sdf.Path(), source_tetrahedral_mesh_path: Sdf.Path = Sdf.Path()):
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        if not self.check_inputs():
            return False

        self._usd_undo.reserve(self._target_skin_mesh_path)
        mesh = deformableUtils.create_skin_mesh_from_tetrahedral_mesh(self._target_skin_mesh_path, self._stage, self._source_tetrahedral_mesh_path)
        return mesh != None

    def undo(self):
        self._usd_undo.undo()

    def check_inputs(self) -> bool:
        if not self._target_skin_mesh_path:
            carb.log_error(type(self).__name__ + ": must provide target_skin_mesh_path.")
            return False
        self._target_skin_mesh_path = Sdf.Path(self._target_skin_mesh_path)

        if self._stage.GetPrimAtPath(self._target_skin_mesh_path):
            carb.log_error(type(self).__name__ + ": target mesh already exists.")
            return False

        if not self._source_tetrahedral_mesh_path:
            carb.log_error(type(self).__name__ + ": must provide source_mesh_path.")
            return False
        self._source_tetrahedral_mesh_path = Sdf.Path(self._source_tetrahedral_mesh_path)

        mesh_prim = self._stage.GetPrimAtPath(self._source_tetrahedral_mesh_path)
        if not mesh_prim or (not mesh_prim.IsA(PhysxSchema.TetrahedralMesh)):
            carb.log_error(type(self).__name__ + ": source mesh must be a PhysxSchema.TetrahedralMesh.")
            return False

        # all passed
        return True


class CreateTetrahedralMeshCommandBase(Command):

    @autoassign
    def __init__(self, command_name: str, target_mesh_path: Sdf.Path, source_mesh_path: Sdf.Path,
                 create_tet_mesh: typing.Callable[[str, Usd.Prim, int], deformableUtils.TetMeshData], voxel_resolution: int, simplify: bool = False, simpl_accuracy: float = 0.55, simpl_min_triangles: int = 1000, simpl_max_triangles: int = 20000):
        self._stage = omni.usd.get_context().get_stage()
        self._usd_undo = UsdLayerUndo(self._stage.GetEditTarget().GetLayer())
        super().__init__()

    def do(self):
        
        if not self.check_inputs():
            return False

        self._usd_undo.reserve(self._target_mesh_path)

        source_prim = self._stage.GetPrimAtPath(self._source_mesh_path)
        target_mesh = PhysxSchema.TetrahedralMesh.Define(self._stage, self._target_mesh_path)
        if not target_mesh:
            carb.log_error(self._command_name + ": target PhysxSchemaTetrahedralMesh creation failed.")
            self.undo()
            return False

        physicsUtils.copy_transform_as_scale_orient_translate(UsdGeom.Xformable(source_prim), target_mesh)

        tet_mesh_data, hexahedral_resolution = self._create_tet_mesh(self._command_name, source_prim, self._voxel_resolution, self._simplify, self._simpl_accuracy, self._simpl_min_triangles, self._simpl_max_triangles)
        if not tet_mesh_data:
            self.undo()
            return False

        if tet_mesh_data.is_valid():
            target_mesh.GetPointsAttr().Set(tet_mesh_data.points)
            target_mesh.GetIndicesAttr().Set(tet_mesh_data.indices)
            gap = carb.settings.get_settings().get(SETTING_VISUALIZATION_GAP)
            target_mesh.GetPrim().CreateAttribute("visualizationGap", Sdf.ValueTypeNames.Float).Set(gap)
            if hexahedral_resolution > 0:
                target_mesh.GetPrim().CreateAttribute("physxTetrahedralMesh:hexahedralResolution", Sdf.ValueTypeNames.Int).Set(self._voxel_resolution)
            return True

        self.undo()
        return False

    def undo(self):
        self._usd_undo.undo()

    def check_inputs(self) -> bool:
        if not self._target_mesh_path:
            carb.log_error("CreateConformingTetrahedralMeshCommand must provide target_mesh_path.")
            return False
        self._target_mesh_path = Sdf.Path(self._target_mesh_path)

        if self._stage.GetPrimAtPath(self._target_mesh_path):
            carb.log_error(self._command_name + ": target mesh already exists.")
            return False

        if not self._source_mesh_path:
            carb.log_error(self._command_name + ": must provide source_mesh_path.")
            return False
        self._source_mesh_path = Sdf.Path(self._source_mesh_path)

        mesh_prim = self._stage.GetPrimAtPath(self._source_mesh_path)
        if not mesh_prim or (not mesh_prim.IsA(UsdGeom.Mesh) and not mesh_prim.IsA(PhysxSchema.TetrahedralMesh)):
            carb.log_error(self._command_name + ": source mesh must be either a UsdGeom.Mesh or a PhysxSchema.TetrahedralMesh.")
            return False

        # all passed
        return True


class CreateConformingTetrahedralMeshCommand(CreateTetrahedralMeshCommandBase):
    """ Creates a PhysxSchema.TetrahedralMesh from a source mesh of either UsdGeom.Mesh or PhysxSchema.TetrahedralMesh type.

        Parameters:
            target_mesh_path:   Target path for new PhysxSchema.TetrahedralMesh.
            source_mesh_path:   Source path to UsdGeom.Mesh or PhysxSchema.TetrahedralMesh prim.

        Returns:
            True / False that indicates success of command execution.

        The conforming tetrahedral mesh is defined as a tetrahedral mesh whose surface triangles align with the closed source
        triangle mesh and whose internal vertices lie on the inside of the closed source triangle mesh.
    """

    def __init__(self, target_mesh_path: Sdf.Path, source_mesh_path, simplify=False, simpl_accuracy=0.55, simpl_min_triangles=1000, simpl_max_triangles=20000):
        super().__init__(type(self).__name__, target_mesh_path, source_mesh_path, self.create_tet_mesh_data, 0, simplify, simpl_accuracy, simpl_min_triangles, simpl_max_triangles)

    def do(self):
        return super().do()

    def undo(self):
        super().undo()

    @staticmethod
    def create_tet_mesh_data(command_name: str, source_prim: Usd.Prim, voxel_resolution: int, simplify: bool = False, simpl_accuracy: float = 0.55, simpl_min_triangles: int = 1000, simpl_max_triangles: int = 20000):
        tet_mesh_data = deformableUtils.TetMeshData()
        if source_prim.IsA(UsdGeom.Mesh):
            source_tri_mesh = UsdGeom.Mesh(source_prim)
            source_points = source_tri_mesh.GetPointsAttr().Get() 
            source_indices = deformableUtils.triangulate_mesh(source_tri_mesh)            
            
            if simplify:
                convexdecomposition = _convexdecomposition.acquire_convexdecomposition_interface()
                result = convexdecomposition.simplify_triangle_mesh_float(source_points, source_indices, simpl_accuracy, simpl_min_triangles, simpl_max_triangles)
                points = result["points"]
                indices = result["indices"]

                max_edge_length = physicsUtils.compute_bounding_box_diagonal(points) * 0.1
                result = get_physx_cooking_interface().compute_edge_length_limited_triangle_mesh(points, indices, max_edge_length)
                points = result["points"]
                indices = result["indices"]

                data = get_physx_cooking_interface().compute_conforming_tetrahedral_mesh(points, indices)
            else:                
                data = get_physx_cooking_interface().compute_conforming_tetrahedral_mesh(source_points, source_indices)
                
            tet_mesh_data.from_dict(data)
            if not tet_mesh_data.is_valid():
                carb.log_error(command_name + ": compute_conforming_tetrahedral_mesh failed for source asset: " + str(source_prim.GetPath()))
                return None, 0
        else:
            source_tet_mesh = PhysxSchema.TetrahedralMesh(source_prim)
            source_points = source_tet_mesh.GetPointsAttr().Get()
            source_indices = source_tet_mesh.GetIndicesAttr().Get()
            # first extract surface triangle mesh
            tri_points, tri_indices = deformableUtils.extractTriangleSurfaceFromTetra(source_points, source_indices)

            data = get_physx_cooking_interface().compute_conforming_tetrahedral_mesh(tri_points, tri_indices)
            tet_mesh_data.from_dict(data)
            if not tet_mesh_data.is_valid():
                carb.log_error(command_name + ": compute_conforming_tetrahedral_mesh failed for source asset: " + str(source_prim.GetPath()))
                return None, 0

        return tet_mesh_data, 0


class CreateVoxelTetrahedralMeshCommand(CreateTetrahedralMeshCommandBase):
    """ Creates a PhysxSchema.TetrahedralMesh from a source mesh of either UsdGeom.Mesh or PhysxSchema.TetrahedralMesh type.

        Parameters:
            target_mesh_path:   Target path for new PhysxSchema.TetrahedralMesh.
            source_mesh_path:   Source path to UsdGeom.Mesh or PhysxSchema.TetrahedralMesh prim.
            voxel_resolution:   Number of voxels along longest dimension of axis aligned bounding box of source mesh.
                                Only considered if type is 'voxel'. Must be > 0.
        Returns:
            True / False that indicates success of command execution.

        The voxel tetrahedral mesh is made by voxelizing the source tetrahedra on a regular grid. The resulting
        voxel tetrahedral mesh embeds all tetrahedra of the source mesh.

        The provided voxel resolution may be lowered automatically in order to match a lower resolution detected in the
        source mesh. This may help to avoid softbody convergence issues with high-resolution tetrahedra embedding low
        resolution collision meshes.
    """

    def __init__(self, target_mesh_path: Sdf.Path, source_mesh_path: Sdf.Path, voxel_resolution: int, simplify: bool = False, simpl_accuracy: float = 0.55, simpl_min_triangles: int = 1000, simpl_max_triangles: int = 20000):
        super().__init__(type(self).__name__, target_mesh_path, source_mesh_path, self.create_tet_mesh_data, voxel_resolution, simplify, simpl_accuracy, simpl_min_triangles, simpl_max_triangles)

    def do(self):
        return super().do()

    def undo(self):
        super().undo()

    @staticmethod
    def create_tet_mesh_data(command_name: str, source_prim: Usd.Prim, voxel_resolution: int, simplify: bool = False, simpl_accuracy: float = 0.55, simpl_min_triangles: int = 1000, simpl_max_triangles: int = 20000):
        source_to_world = UsdGeom.Xformable(source_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        scale = Gf.Vec3f(Gf.Transform(source_to_world).GetScale())
        tet_mesh_data = deformableUtils.TetMeshData()
        if source_prim.IsA(UsdGeom.Mesh):
            source_tri_mesh = UsdGeom.Mesh(source_prim)
            source_points = source_tri_mesh.GetPointsAttr().Get() 
            source_indices = deformableUtils.triangulate_mesh(source_tri_mesh)

            if simplify:
                convexdecomposition = _convexdecomposition.acquire_convexdecomposition_interface()
                result = convexdecomposition.simplify_triangle_mesh_float(source_points, source_indices, simpl_accuracy, simpl_min_triangles, simpl_max_triangles)
                points = result["points"]
                indices = result["indices"]
                                
                max_edge_length = physicsUtils.compute_bounding_box_diagonal(points) * 0.1
                result = get_physx_cooking_interface().compute_edge_length_limited_triangle_mesh(points, indices, max_edge_length);
                points = result["points"]
                indices = result["indices"]

                data = get_physx_cooking_interface().compute_conforming_tetrahedral_mesh(points, indices)
            else:
                data = get_physx_cooking_interface().compute_conforming_tetrahedral_mesh(source_points, source_indices)
            tet_mesh_data.from_dict(data)
            if not tet_mesh_data.is_valid():
                carb.log_error(command_name + ": compute_conforming_tetrahedral_mesh failed for source asset: " + str(source_prim.GetPath()))
                return None, 0
            else:
                data = get_physx_cooking_interface().compute_voxel_tetrahedral_mesh(tet_mesh_data.points, tet_mesh_data.indices, scale, voxel_resolution)
                tet_mesh_data.from_dict(data)
                if not tet_mesh_data.is_valid():
                    carb.log_error(command_name + ": compute_voxel_tetrahedral_mesh failed for source asset: " + str(source_prim.GetPath()))
                    return None, 0
        else:
            source_tet_mesh = PhysxSchema.TetrahedralMesh(source_prim)
            source_points = source_tet_mesh.GetPointsAttr().Get()
            source_indices = source_tet_mesh.GetIndicesAttr().Get()
            data = get_physx_cooking_interface().compute_voxel_tetrahedral_mesh(source_points, source_indices, scale, voxel_resolution)
            tet_mesh_data.from_dict(data)
            if not tet_mesh_data.is_valid():
                carb.log_error(command_name + ": compute_voxel_tetrahedral_mesh failed for source asset: " + str(source_prim.GetPath()))
                return None, 0
        return tet_mesh_data, voxel_resolution
