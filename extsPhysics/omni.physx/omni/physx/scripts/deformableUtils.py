import typing
import carb
from . import physicsUtils, utils
from pxr import UsdGeom, Usd, Sdf, PhysxSchema, UsdPhysics, Gf, UsdShade
from omni.physx import get_physx_cooking_interface


class TetMeshData:
    def __init__(self):
        self.points = []
        self.indices = []
        self.embedding = []

    def from_dict(self, data: typing.Dict[str, typing.List]):
        self.points = [Gf.Vec3f(v.x, v.y, v.z) for v in data["points"]]
        self.indices = [index for index in data["indices"]]
        if "embedding" in data:
            self.embedding = [index for index in data["embedding"]]

    def from_mesh(self, mesh: PhysxSchema.TetrahedralMesh):
        self.points = mesh.GetPointsAttr().Get()
        self.indices = mesh.GetIndicesAttr().Get()

    def is_valid(self) -> bool:
        return self.points and self.indices


def add_physx_deformable_body(  # TODO PREIST: Get defaults from schema metadata instead of hardcoding here
    stage,
    prim_path: Sdf.Path,
    collision_rest_points: typing.List[Gf.Vec3f] = None,
    collision_indices: typing.List[int] = None,
    kinematic_enabled : bool = False,
    collision_simplification : bool = True,
    collision_simplification_remeshing: bool = True,
    collision_simplification_remeshing_resolution: int = 0,
    collision_simplification_target_triangle_count: int = 0,
    collision_simplification_force_conforming: bool = False,
    simulation_rest_points: typing.List[Gf.Vec3f] = None,
    simulation_indices: typing.List[int] = None,
    embedding: typing.List[int] = None,
    simulation_hexahedral_resolution: int = 10,
    solver_position_iteration_count: int = None,
    vertex_velocity_damping: float = None,
    sleep_damping: float = None,
    sleep_threshold: float = None,
    settling_threshold: float = None,
    self_collision: bool = None,
    self_collision_filter_distance: float = None,
) -> bool:

    """DEPRECATED: Will be replaced by new deformable implementation in future release.
    Applies the deformable body API to an Xform at prim_path on stage.

    Args:
        stage:                                          The stage
        prim_path:                                      Path to UsdGeom.Mesh 'skin mesh' to which the PhysxSchema.PhysXDeformableBodyAPI is applied to.
        collision_rest_points:                          List of vertices of the collision tetrahedral mesh at rest.
                                                        If a collision mesh is provided, the simulation mesh needs to be provided too.
                                                        If no collision mesh is provided, it will be computed implicitly based on the simplification parameter.
        collision_indices:                              List of indices of the collision tetrahedral mesh.
        kinematic_enabled:                              Enables kinematic body.
        collision_simplification:                       Boolean flag indicating if simplification should be applied to the mesh before creating a
                                                        softbody out of it. Is ignored if simulation mesh has been provided.
        collision_simplification_remeshing:             Boolean flag indicating if the simplification should be based on remeshing.
                                                        Ignored if collision_simplification equals False.
        collision_simplification_remeshing_resolution:  The resolution used for remeshing. A value of 0 indicates that a heuristic is used to determine
                                                        the resolution. Ignored if collision_simplification_remeshing is False.
        collision_simplification_target_triangle_count: The target triangle count used for the simplification. A value of 0 indicates
                                                        that a heuristic based on the simulation_hexahedral_resolution is to determine the target count.
                                                        Ignored if collision_simplification equals False.
        collision_simplification_force_conforming:      Boolean flag indicating that the tretrahedralizer used to generate the collision mesh should produce
                                                        tetrahedra that conform to the triangle mesh. If False the implementation chooses the tretrahedralizer
                                                        used.
        simulation_rest_points:                         List of vertices of the simulation tetrahedral mesh at rest.
                                                        If a simulation mesh is provided, the collision mesh needs to be provided too.
                                                        If no simulation mesh is provided it will be computed implicitly based on simulation_hexahedral_resolution.
        simulation_indices:                             List of indices of the simulation tetrahedral mesh.
        embedding:                                      Optional embedding information mapping collision points to containing simulation tetrahedra.
        simulation_hexahedral_resolution:               Target resolution of voxel simulation mesh. Is ignored if simulation mesh has been provided.
        ...:                                            See USD schema for documentation

    Returns:
        True / False that indicates success of schema application
    """

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        carb.log_warn("No valid primitive prim_path provided")
        return False

    # check if it is a rigid body:
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        carb.log_warn(
            "PhysxSchema.PhysxDeformableBodyAPI cannot be applied to a primitive with UsdPhysics.RigidBodyAPI"
        )
        return False

    # check if it is a UsdGeom.Mesh
    if not prim.IsA(UsdGeom.Mesh):
        carb.log_warn("PhysxSchema.PhysxDeformableBodyAPI can only be applied to a UsdGeom.Mesh")
        return False

    # check collision mesh
    if collision_rest_points:
        if len(collision_rest_points) < 4:
            carb.log_warn("collision_rest_points is invalid")
            return False
        if not collision_indices:
            carb.log_warn("collision mesh invalid")
            return False
        if not simulation_rest_points:
            carb.log_warn("collision mesh is invalid without simulation mesh")
            return False

    if collision_indices:
        if len(collision_indices) < 4 or len(collision_indices) % 4 != 0:
            carb.log_warn("collision_indices is invalid")
            return False
        if not collision_rest_points:
            carb.log_warn("collision mesh invalid")
            return False

    # check simulation mesh
    if simulation_rest_points:
        if len(simulation_rest_points) < 4:
            carb.log_warn("simulation_rest_points is invalid")
            return False
        if not simulation_indices:
            carb.log_warn("simulation mesh invalid")
            return False
        if not collision_rest_points:
            carb.log_warn("simulation mesh is invalid without collision mesh")
            return False

    if simulation_indices:
        if len(simulation_indices) < 4 or len(simulation_indices) % 4 != 0:
            carb.log_warn("simulation_indices is invalid")
            return False
        if not simulation_rest_points:
            carb.log_warn("simulation mesh invalid")
            return False

    if embedding:
        if len(embedding) != len(collision_rest_points):
            carb.log_warn("embedding is invalid")
            return False
        if not simulation_rest_points:
            carb.log_warn("embedding is invalid without simulation mesh")
            return False

    # warnings
    if kinematic_enabled and (collision_rest_points or simulation_rest_points):
        carb.log_warn("provided custom collision or simulation mesh: unable to enable kinematic on time varying skin mesh")
        kinematic_enabled = False

    if kinematic_enabled and collision_simplification_remeshing is not None and collision_simplification_remeshing == True:
        carb.log_warn("enable kinematic: remeshing disabled")
        collision_simplification_remeshing = False

    if kinematic_enabled and collision_simplification_force_conforming is not None and collision_simplification_force_conforming == False:
        carb.log_warn("enable kinematic: force conforming enabled")
        collision_simplification_force_conforming = True

    # apply APIs and create attributes
    deformable_body_api = PhysxSchema.PhysxDeformableBodyAPI.Apply(prim)
    deformable_api = PhysxSchema.PhysxDeformableAPI(deformable_body_api)

    if solver_position_iteration_count is not None:
        deformable_api.CreateSolverPositionIterationCountAttr().Set(solver_position_iteration_count)
    if vertex_velocity_damping is not None:
        deformable_api.CreateVertexVelocityDampingAttr().Set(vertex_velocity_damping)
    if sleep_damping is not None:
        deformable_api.CreateSleepDampingAttr().Set(sleep_damping)
    if sleep_threshold is not None:
        deformable_api.CreateSleepThresholdAttr().Set(sleep_threshold)
    if settling_threshold is not None:
        deformable_api.CreateSettlingThresholdAttr().Set(settling_threshold)
    if self_collision is not None:
        deformable_api.CreateSelfCollisionAttr().Set(self_collision)
    if self_collision_filter_distance is not None:
        deformable_api.CreateSelfCollisionFilterDistanceAttr().Set(self_collision_filter_distance)

    if collision_indices:
        deformable_body_api.CreateCollisionIndicesAttr().Set(collision_indices)
    if collision_rest_points:
        deformable_body_api.CreateCollisionRestPointsAttr().Set(collision_rest_points)
    if simulation_indices:
        deformable_api.CreateSimulationIndicesAttr().Set(simulation_indices)
    if simulation_rest_points:
        deformable_body_api.CreateSimulationRestPointsAttr().Set(simulation_rest_points)

    # Custom attributes
    if not simulation_rest_points and not collision_rest_points:
        prim.CreateAttribute("physxDeformable:kinematicEnabled", Sdf.ValueTypeNames.Bool).Set(kinematic_enabled)

    if not simulation_rest_points:
        prim.CreateAttribute("physxDeformable:simulationHexahedralResolution", Sdf.ValueTypeNames.UInt).Set(
            simulation_hexahedral_resolution
        )
        prim.CreateAttribute("physxDeformable:numberOfTetsPerHex", Sdf.ValueTypeNames.UInt).Set(
            5
        )

    if not collision_rest_points:
        prim.CreateAttribute("physxDeformable:collisionSimplification", Sdf.ValueTypeNames.Bool).Set(
            collision_simplification
        )
        prim.CreateAttribute("physxDeformable:collisionSimplificationRemeshing", Sdf.ValueTypeNames.Bool).Set(
            collision_simplification_remeshing
        )
        prim.CreateAttribute("physxDeformable:collisionSimplificationRemeshingResolution", Sdf.ValueTypeNames.UInt).Set(
            collision_simplification_remeshing_resolution
        )
        prim.CreateAttribute("physxDeformable:collisionSimplificationTargetTriangleCount", Sdf.ValueTypeNames.UInt).Set(
            collision_simplification_target_triangle_count
        )
        prim.CreateAttribute("physxDeformable:collisionSimplificationForceConforming", Sdf.ValueTypeNames.Bool).Set(
            collision_simplification_force_conforming
        )

    if embedding:
        prim.CreateAttribute("physxDeformable:collisionVertexToSimulationTetIndices", Sdf.ValueTypeNames.IntArray).Set(
            embedding
        )

    # turn on ccd (In the schema, it is off by default)
    deformable_api.CreateEnableCCDAttr().Set(True)

    PhysxSchema.PhysxCollisionAPI.Apply(prim)

    return True


def add_physx_deformable_surface(  # TODO PREIST: Get defaults from schema metadata instead of hardcoding here
    stage,
    prim_path: Sdf.Path,
    simulation_rest_points: typing.List[Gf.Vec3f] = None,
    simulation_indices: typing.List[int] = None,
    bending_stiffness_scale: float = None,
    solver_position_iteration_count: int = None,
    vertex_velocity_damping: float = None,
    sleep_damping: float = None,
    sleep_threshold: float = None,
    settling_threshold: float = None,
    self_collision: bool = None,
    self_collision_filter_distance: float = None
) -> bool:

    """DEPRECATED: Will be replaced by new deformable implementation in future release.
    Applies the deformable surface API to an Xform at prim_path on stage.

    Args:
        stage:                                          The stage
        prim_path:                                      Path to UsdGeom.Mesh 'skin mesh' to which the PhysxSchema.PhysXDeformableBodyAPI is applied to.
        simulation_rest_points:                         List of vertices of the simulation mesh at rest.
        simulation_indices:                             List of indices of the simulation mesh.
        ...:                                            See USD schema for documentation

    Returns:
        True / False that indicates success of schema application
    """

    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        carb.log_warn("No valid primitive prim_path provided")
        return False

    # check if it is a rigid body:
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        carb.log_warn(
            "PhysxSchema.PhysxDeformableSurfaceAPI cannot be applied to a primitive with UsdPhysics.RigidBodyAPI"
        )
        return False

    # check if it is a UsdGeom.Mesh
    if not prim.IsA(UsdGeom.Mesh):
        carb.log_warn("PhysxSchema.PhysxDeformableSurfaceAPI can only be applied to a UsdGeom.Mesh")

    # check simulation mesh
    if simulation_rest_points:
        if len(simulation_rest_points) < 3:
            carb.log_warn("simulation_rest_points is invalid")
            return False
        if not simulation_indices:
            carb.log_warn("simulation mesh invalid")
            return False

    if simulation_indices:
        if len(simulation_indices) < 3 or len(simulation_indices) % 3 != 0:
            carb.log_warn("simulation_indices is invalid")
            return False

    # apply APIs and create attributes
    deformable_surface_api = PhysxSchema.PhysxDeformableSurfaceAPI.Apply(prim)
    deformable_api = PhysxSchema.PhysxDeformableAPI(deformable_surface_api)
    if bending_stiffness_scale is not None:
        deformable_surface_api.CreateBendingStiffnessScaleAttr().Set(bending_stiffness_scale)

    if solver_position_iteration_count is not None:
        deformable_api.CreateSolverPositionIterationCountAttr().Set(solver_position_iteration_count)
    if vertex_velocity_damping is not None:
        deformable_api.CreateVertexVelocityDampingAttr().Set(vertex_velocity_damping)
    if sleep_damping is not None:
        deformable_api.CreateSleepDampingAttr().Set(sleep_damping)
    if sleep_threshold is not None:
        deformable_api.CreateSleepThresholdAttr().Set(sleep_threshold)
    if settling_threshold is not None:
        deformable_api.CreateSettlingThresholdAttr().Set(settling_threshold)
    if self_collision is not None:
        deformable_api.CreateSelfCollisionAttr().Set(self_collision)
    if self_collision_filter_distance is not None:
        deformable_api.CreateSelfCollisionFilterDistanceAttr().Set(self_collision_filter_distance)

    if simulation_indices:
        deformable_api.CreateSimulationIndicesAttr().Set(simulation_indices)
    if simulation_rest_points:
        deformable_api.CreateRestPointsAttr().Set(simulation_rest_points)

    PhysxSchema.PhysxCollisionAPI.Apply(prim)

    return True


def add_deformable_body_material(
    stage,
    path,
    damping_scale=None,
    density=None,
    dynamic_friction=None,
    elasticity_damping=None,
    poissons_ratio=None,
    youngs_modulus=None,
):
    """DEPRECATED: Will be replaced by new deformable implementation in future release.
    Applies the PhysxSchema.PhysxDeformableSurfaceMaterialAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to UsdShade.Material to which the material API should be applied to.
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    if not utils.ensureMaterialOnPath(stage, path):
        return False

    material = PhysxSchema.PhysxDeformableBodyMaterialAPI.Apply(stage.GetPrimAtPath(path))

    if damping_scale is not None:
        material.CreateDampingScaleAttr().Set(damping_scale)
    if density is not None:
        material.CreateDensityAttr().Set(density)
    if dynamic_friction is not None:
        material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    if elasticity_damping is not None:
        material.CreateElasticityDampingAttr().Set(elasticity_damping)
    if poissons_ratio is not None:
        material.CreatePoissonsRatioAttr().Set(poissons_ratio)
    if youngs_modulus is not None:
        material.CreateYoungsModulusAttr().Set(youngs_modulus)

    return True


def add_deformable_surface_material(
    stage,
    path,
    density=None,
    dynamic_friction=None,
    poissons_ratio=None,
    thickness=None,
    youngs_modulus=None,
):
    """DEPRECATED: Will be replaced by new deformable implementation in future release.
    Applies the PhysxSchema.PhysxDeformableSurfaceMaterialAPI to the prim at path on stage.

    Args:
        stage:                          The stage
        path:                           Path to UsdShade.Material to which the material API should be applied to.
        ... schema attributes:          See USD schema for documentation

    Returns:
        True if the API apply succeeded.
    """
    if not utils.ensureMaterialOnPath(stage, path):
        return False

    material = PhysxSchema.PhysxDeformableSurfaceMaterialAPI.Apply(stage.GetPrimAtPath(path))

    if density is not None:
        material.CreateDensityAttr().Set(density)
    if dynamic_friction is not None:
        material.CreateDynamicFrictionAttr().Set(dynamic_friction)
    if poissons_ratio is not None:
        material.CreatePoissonsRatioAttr().Set(poissons_ratio)
    if thickness is not None:
        material.CreateThicknessAttr().Set(thickness)
    if youngs_modulus is not None:
        material.CreateYoungsModulusAttr().Set(youngs_modulus)

    return True


def extractTriangleSurfaceFromTetra(tetra_points, tetra_indices):

    # extract all triangles
    triangles = [(-1, -1, -1)] * len(tetra_indices)  # tetra has as many triangles as vertices
    for t in range(0, len(tetra_indices) // 4):
        (v0, v1, v2, v3) = (
            tetra_indices[t * 4],
            tetra_indices[t * 4 + 1],
            tetra_indices[t * 4 + 2],
            tetra_indices[t * 4 + 3],
        )
        triangles[t * 4 + 0] = (v0, v1, v2)
        triangles[t * 4 + 1] = (v1, v3, v2)
        triangles[t * 4 + 2] = (v0, v3, v1)
        triangles[t * 4 + 3] = (v0, v2, v3)

    # extract surface triangles
    surface_triangles_dict = {}
    for i, t in enumerate(triangles):
        vs = sorted([t[0], t[1], t[2]])
        key = (vs[0], vs[1], vs[2])
        if key in surface_triangles_dict:
            del surface_triangles_dict[key]
        else:
            surface_triangles_dict[key] = t

    surface_triangles = list(surface_triangles_dict.values())

    points = []
    indices = []
    tetra_points_to_points = [-1] * len(tetra_points)
    for t in surface_triangles:
        (v0, v1, v2) = t
        if tetra_points_to_points[v0] < 0:
            tetra_points_to_points[v0] = len(points)
            points.append(tetra_points[v0])
        if tetra_points_to_points[v1] < 0:
            tetra_points_to_points[v1] = len(points)
            points.append(tetra_points[v1])
        if tetra_points_to_points[v2] < 0:
            tetra_points_to_points[v2] = len(points)
            points.append(tetra_points[v2])

        indices.extend([tetra_points_to_points[v0], tetra_points_to_points[v1], tetra_points_to_points[v2]])

    return points, indices


def create_skin_mesh_from_tetrahedral_mesh(
    target_path: Sdf.Path, stage: Usd.Stage, source_tetrahedal_mesh_path: Sdf.Path
) -> UsdGeom.Mesh:
    """DEPRECATED: Will be replaced by new deformable implementation in future release."""

    skin_mesh = UsdGeom.Mesh.Define(stage, target_path)
    if skin_mesh:
        collision_mesh = PhysxSchema.TetrahedralMesh(stage.GetPrimAtPath(source_tetrahedal_mesh_path))
        tet_points = collision_mesh.GetPointsAttr().Get()
        tet_indices = collision_mesh.GetIndicesAttr().Get()
        tri_points, tri_indices = extractTriangleSurfaceFromTetra(tet_points, tet_indices)
        skin_mesh.GetPointsAttr().Set(tri_points)
        skin_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        skin_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
        skin_mesh.GetSubdivisionSchemeAttr().Set("none")
        physicsUtils.copy_transform_as_scale_orient_translate(collision_mesh, skin_mesh)
    return skin_mesh


def triangulate_mesh(mesh: UsdGeom.Mesh) -> typing.List[int]:
    # indices and faces converted to triangles
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    faces = mesh.GetFaceVertexCountsAttr().Get()

    triangles = []
    if not indices or not faces:
        return triangles

    indices_offset = 0

    for face_count in faces:
        start_index = indices[indices_offset]
        for face_index in range(face_count - 2):
            index1 = indices_offset + face_index + 1
            index2 = indices_offset + face_index + 2
            triangles.append(start_index)
            triangles.append(indices[index1])
            triangles.append(indices[index2])
        indices_offset += face_count

    return triangles


def compute_conforming_tetrahedral_mesh(
    triangle_mesh_points: typing.List[Gf.Vec3f], triangle_mesh_indices: typing.List[int]
) -> typing.Tuple[typing.List[Gf.Vec3f], typing.List[int]]:

    data = get_physx_cooking_interface().compute_conforming_tetrahedral_mesh(
        triangle_mesh_points, triangle_mesh_indices
    )
    conforming_tet_points = [Gf.Vec3f(v.x, v.y, v.z) for v in data["points"]]
    conforming_tet_indices = [index for index in data["indices"]]
    return conforming_tet_points, conforming_tet_indices


def compute_voxel_tetrahedral_mesh(
    tet_mesh_points: typing.List[Gf.Vec3f], tet_mesh_indices: typing.List[int], scale: Gf.Vec3f, resolution: int
) -> typing.Tuple[typing.List[Gf.Vec3f], typing.List[int]]:

    carbScale = carb.Float3(scale[0], scale[1], scale[2])
    data = get_physx_cooking_interface().compute_voxel_tetrahedral_mesh(
        tet_mesh_points, tet_mesh_indices, carbScale, resolution
    )
    voxel_tet_points = [Gf.Vec3f(v.x, v.y, v.z) for v in data["points"]]
    voxel_tet_indices = [index for index in data["indices"]]
    return voxel_tet_points, voxel_tet_indices


def create_triangle_mesh_square(dimx: int, dimy: int, scale: float = 1.0):
    """Creates points and vertex data for a regular-grid flat triangle mesh square.

    Args:
        dimx:                       Mesh-vertex resolution in X
        dimy:                       Mesh-vertex resolution in Y
        scale:                      Uniform scale applied to vertices

    Returns:
        points, indices:            The vertex and index data
    """

    points = [Gf.Vec3f(0.0)] * (dimx + 1) * (dimy + 1)
    indices = [-1] * (dimx * dimy) * 2 * 3

    for y in range(dimy + 1):
        for x in range(dimx + 1):
            points[y * (dimx + 1) + x] = Gf.Vec3f(x, y, 0.0)

    offset = 0
    for y in range(dimy):
        for x in range(dimx):
            v0 = y * (dimx + 1) + x
            v1 = y * (dimx + 1) + x + 1
            v2 = (y + 1) * (dimx + 1) + x
            v3 = (y + 1) * (dimx + 1) + x + 1
            if (x % 2 == 0) != (y % 2 == 0):
                indices[offset] = v0
                indices[offset + 1] = v1
                indices[offset + 2] = v2
                indices[offset + 3] = v1
                indices[offset + 4] = v3
                indices[offset + 5] = v2
            else:
                indices[offset] = v0
                indices[offset + 1] = v1
                indices[offset + 2] = v3
                indices[offset + 3] = v0
                indices[offset + 4] = v3
                indices[offset + 5] = v2
            offset = offset + 6

    for i in range(len(points)):
        p = points[i]
        points[i] = Gf.Vec3f(p[0] / dimx, p[1] / dimy, p[2]) - Gf.Vec3f(0.5, 0.5, 0.0)
        points[i] = Gf.Vec3f(scale * points[i][0], scale * points[i][1], scale * points[i][2])

    return points, indices
