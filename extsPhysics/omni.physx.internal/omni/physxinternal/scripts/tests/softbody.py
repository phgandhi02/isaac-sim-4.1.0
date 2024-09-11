import carb
import unittest
import omni.kit.test
import omni.kit.commands
from pxr import Gf, Sdf, UsdGeom, Usd, UsdLux, PhysxSchema
import omni.physx
from omni.physxcommands import AddGroundPlaneCommand
from omni.physxtests import utils
from omni.physx import get_physx_interface, get_physx_cooking_interface
from omni.physx.scripts import deformableUtils, physicsUtils
from .. physxInternalUtils import TetrahedralMeshType
from omni.physxtests.utils.physicsBase import PhysicsBaseAsyncTestCase
import omni.usd


class PhysXSoftbodyInternalTest(PhysicsBaseAsyncTestCase):
    @classmethod
    def setUpClass(self):
        # init for attributes that are not stage-dependent:
        # default number of places to check float equality:
        self._places = 3
        # carb settings and bloky dev mode:
        self._settings = carb.settings.get_settings()
        self._prim_type_list = ['Cone', 'Cube', 'Cylinder', 'Sphere', 'Torus']

    # runs before each test case
    async def setUp(self):
        await super().setUp()
        self._baseWasSetup = False

    # runs after each test case and runs base_terminate if setup was called:
    async def tearDown(self):
        if self._baseWasSetup:
            await self.base_terminate()
        omni.kit.commands.execute("SelectPrims", old_selected_paths=[], new_selected_paths=[], expand_in_stage=False)
        await super().tearDown()

    async def base_setup(self):
        self._baseWasSetup = True
        self.fail_on_log_error = True
        self._stage = await utils.new_stage_setup()
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        self._upAxis = UsdGeom.GetStageUpAxis(self._stage)
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()

        # add physics scene
        omni.kit.commands.execute("AddPhysicsScene", stage=self._stage, path='/World/PhysicsScene')

        # create a material (make a bit squishier for clearer deformation results)
        self._deformable_body_material_path = '/World/DeformableBodyMaterial'
        omni.kit.commands.execute("AddDeformableBodyMaterial",
                                  stage=self._stage, path=self._deformable_body_material_path,
                                  youngsModulus=5000.0)

    async def base_terminate(self):
        pass

    def add_groundplane(self):
        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane',
                                      self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

    @staticmethod
    async def _delay_update_asyncs(num_cycles: int = 10):
        for _ in range(num_cycles):
            await omni.kit.app.get_app().next_update_async()

    async def _runAddDeformableBodyComponentCommand(self, skin_mesh_path: Sdf.Path=Sdf.Path(), collision_mesh_path: Sdf.Path=Sdf.Path(), simulation_mesh_path: Sdf.Path=Sdf.Path()) -> bool:
        # make path for deformablebody
        self.assertTrue(bool(skin_mesh_path))

        # create softbody:
        success = omni.kit.commands.execute(
            "AddDeformableBodyComponent",
            skin_mesh_path=skin_mesh_path,
            collision_mesh_path=collision_mesh_path,
            simulation_mesh_path=simulation_mesh_path)

        # set deformable body material
        physicsUtils.add_physics_material_to_prim(self._stage, self._stage.GetPrimAtPath(skin_mesh_path), self._deformable_body_material_path)

        # this is a workaround for hang in logger while async cooking, would be nice to get at the bottom of this
        # tests that check for the cooked meshes should still call cook_deformable_body_mesh separately
        get_physx_cooking_interface().cook_deformable_body_mesh(str(skin_mesh_path))
        return success

    def _test_float_equal(self, floatA: float, floatB: float, places: int = None):
        if places is None:
            places = self._places
        self.assertAlmostEqual(floatA, floatB, places=places)

    def _assert_transform_close(self, transform: Gf.Matrix4d, reference_transform: Gf.Matrix4d):
        for va, vb in zip(transform, reference_transform):
            for a, b in zip(va, vb):
                self._test_float_equal(a, b)

    @staticmethod
    def set_prim_translation(prim: Usd.Prim, translateVec: Gf.Vec3d):
        translate_mtx = Gf.Matrix4d().SetTranslate(translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=translate_mtx)

    def _check_collision_mesh(self, soft_body_path: Sdf.Path):
        prim = self._stage.GetPrimAtPath(soft_body_path)
        deformable_body = PhysxSchema.PhysxDeformableBodyAPI(prim)
        indicesAttr = deformable_body.GetCollisionIndicesAttr()
        self.assertTrue(indicesAttr.HasAuthoredValue())
        indices = indicesAttr.Get()
        self.assertTrue(bool(indices) and len(indices) > 0 and len(indices) % 4 == 0)
        restPointsAttr = deformable_body.GetCollisionRestPointsAttr();
        self.assertTrue(restPointsAttr.HasAuthoredValue())
        restPoints = restPointsAttr.Get()
        self.assertTrue(bool(restPoints) and len(restPoints) >= 4)
        pointsAttr = deformable_body.GetCollisionPointsAttr()
        if pointsAttr.HasAuthoredValue():
            points = pointsAttr.Get()
            self.assertTrue(bool(points) and len(points) == len(restPoints))

    def _create_mesh_prims(self, prim_type_list: list) -> list:
        mesh_list = []
        for prim_type in prim_type_list:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + prim_type, True))
            omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type)
            mesh = UsdGeom.Mesh.Get(self._stage, path)
            self.assertTrue(mesh)
            mesh_list.append(mesh)
        return mesh_list

    async def _create_mesh_primitives(self, prim_type_list, starting_height=60.0):
        mesh_list = self._create_mesh_prims(prim_type_list)

        height = starting_height
        offset = 150
        origin = Gf.Vec3d(-offset * 3 / 2, height, -offset * 3 / 2)
        for i in range(3):
            for j in range(3):
                index = i * 3 + j
                if index < len(mesh_list):
                    self.set_prim_translation(mesh_list[index].GetPrim(), origin + Gf.Vec3d(i * offset, 0, j * offset))

        return mesh_list

    @staticmethod
    def _get_time_step():
        return 1.0 / 60.0

    def _start(self):
        physx_interface = get_physx_interface()
        physx_interface.start_simulation()

    def _step(self, numSteps):
        physx_interface = get_physx_interface()
        time = 0.0
        dtime = self._get_time_step()
        for i in range(numSteps):
            physx_interface.update_simulation(dtime, time)
            physx_interface.update_transformations(True, True, True, False)
            time = time + dtime

    def _reset(self):
        physx_interface = get_physx_interface()
        physx_interface.reset_simulation()


    @unittest.skip("https://jirasw.nvidia.com/browse/OM-124833")
    async def test_create_tetrahedral_mesh(self):
        await self.base_setup()
        prim_type_list = ['Torus', 'Cylinder', 'Sphere', 'Cube']
        mesh_list = await self._create_mesh_primitives(prim_type_list)
        method_list = [TetrahedralMeshType.CONFORMING, TetrahedralMeshType.VOXEL]

        for method in method_list:
            for j, mesh in enumerate(mesh_list):
                target_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(mesh.GetPath()) + "_" + method, False))

                if method is TetrahedralMeshType.CONFORMING:
                    omni.kit.commands.execute("CreateConformingTetrahedralMesh", target_mesh_path=Sdf.Path(target_mesh_path), source_mesh_path=mesh.GetPath())
                else:
                    omni.kit.commands.execute("CreateVoxelTetrahedralMesh", target_mesh_path=Sdf.Path(target_mesh_path), source_mesh_path=mesh.GetPath(), voxel_resolution=10)

                tetrahedral_mesh = PhysxSchema.TetrahedralMesh(self._stage.GetPrimAtPath(target_mesh_path))
                self.assertTrue(tetrahedral_mesh is not None)

                indices = tetrahedral_mesh.GetIndicesAttr().Get()
                points = tetrahedral_mesh.GetPointsAttr().Get()

                self.assertTrue(len(indices) > 0)
                self.assertTrue(len(points) > 0)

        self._reset()

    @unittest.skip("https://jirasw.nvidia.com/browse/OM-124833")
    async def test_create_tetrahedral_transform_setup(self):
        await self.base_setup()
        prim_type_list = ['Torus', 'Cylinder']
        mesh_list = await self._create_mesh_primitives(prim_type_list)

        torusMesh = mesh_list[0]
        torusMesh.ClearXformOpOrder()
        torusMesh.GetPrim().RemoveProperty("xformOp:scale")
        torusMesh.SetResetXformStack(True)
        torusMesh.AddRotateXOp().Set(10)
        torusMesh.AddTranslateOp().Set(Gf.Vec3f(1, 2, 3))
        torusMesh.AddScaleOp().Set(Gf.Vec3f(1.1, 1.2, 1.3))

        cylinderMesh = mesh_list[1]
        cylinderMesh.ClearXformOpOrder()
        cylinderMesh.GetPrim().RemoveProperty("xformOp:scale")
        cylinderMesh.AddTranslateOp().Set(Gf.Vec3f(4, 5, 6))
        cylinderMesh.AddRotateXOp().Set(20)
        cylinderMesh.AddScaleOp().Set(Gf.Vec3f(1.4, 1.5, 1.6))

        tetrahedral_meshes = []
        for j, mesh in enumerate(mesh_list):
            target_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(mesh.GetPath()) + "_conforming", False))
            omni.kit.commands.execute("CreateConformingTetrahedralMesh", target_mesh_path=Sdf.Path(target_mesh_path), source_mesh_path=mesh.GetPath())

            #read delay, to see whether this fixes new crash.
            await self._delay_update_asyncs(10)

            tetrahedral_meshes.append(PhysxSchema.TetrahedralMesh(self._stage.GetPrimAtPath(target_mesh_path)))
            self.assertTrue(tetrahedral_meshes[-1] is not None)

        for mesh, tetmesh in zip(mesh_list, tetrahedral_meshes):
            meshReset = mesh.GetResetXformStack()
            tetMeshReset = tetmesh.GetResetXformStack()
            self.assertEqual(meshReset, tetMeshReset)
            stack = tetmesh.GetOrderedXformOps()
            opNames = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
            for op, opName in zip(stack, opNames):
                self.assertEqual(op.GetName(), opName)
            meshTransform = mesh.GetLocalTransformation()
            tetMeshTransform = tetmesh.GetLocalTransformation()
            self._assert_transform_close(meshTransform, tetMeshTransform)

        self._reset()

    async def test_set_simulation_mesh_hexahedral_flag(self):
        await self.base_setup()
        mesh, = await self._create_mesh_primitives(['Cube'])

        #create conforming tetrahedral mesh (physxTetrahedralMesh:hexahedralResolution should not be set or 0 for non hexahedral meshes)
        target_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(mesh.GetPath()) + "_conforming", False))
        omni.kit.commands.execute("CreateConformingTetrahedralMesh",
            target_mesh_path=Sdf.Path(target_mesh_path), source_mesh_path=mesh.GetPath())
        conforming_tet_mesh_prim = self._stage.GetPrimAtPath(target_mesh_path)
        hexResolutionAttr = conforming_tet_mesh_prim.GetAttribute("physxTetrahedralMesh:hexahedralResolution")
        self.assertTrue(not hexResolutionAttr or hexResolutionAttr.Get() == 0)

        #create voxel tetrahedral mesh (physxTetrahedralMesh:hexahedralResolution should be set to greater 0)
        target_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(mesh.GetPath()) + "_voxel", False))
        omni.kit.commands.execute("CreateVoxelTetrahedralMesh",
            target_mesh_path=Sdf.Path(target_mesh_path), source_mesh_path=mesh.GetPath(), voxel_resolution=2)
        voxel_tet_mesh_prim = self._stage.GetPrimAtPath(target_mesh_path)
        hexResolutionAttr = voxel_tet_mesh_prim.GetAttribute("physxTetrahedralMesh:hexahedralResolution")
        self.assertTrue(hexResolutionAttr and hexResolutionAttr.Get() > 0)

        #create softbody from skin mesh cube. (physxDeformable:simulationHexahedralResolution should be set to greater 0)
        omni.kit.commands.execute("AddDeformableBodyComponent", skin_mesh_path=mesh.GetPath())
        hexResolutionAttr = mesh.GetPrim().GetAttribute("physxDeformable:simulationHexahedralResolution")
        self.assertTrue(hexResolutionAttr and hexResolutionAttr.Get() > 0)

        #create softbody from conforming for sim mesh (physxDeformable:simulationHexahedralResolution should not be set or 0)
        target_skin_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(conforming_tet_mesh_prim.GetPath()) + "_skin", False))
        deformableUtils.create_skin_mesh_from_tetrahedral_mesh(target_skin_mesh_path, self._stage, conforming_tet_mesh_prim.GetPath())
        omni.kit.commands.execute("AddDeformableBodyComponent",
            skin_mesh_path=target_skin_mesh_path,
            collision_mesh_path=conforming_tet_mesh_prim.GetPath(),
            simulation_mesh_path=conforming_tet_mesh_prim.GetPath())
        hexResolutionAttr = self._stage.GetPrimAtPath(target_skin_mesh_path).GetAttribute("physxDeformable:simulationHexahedralResolution")
        self.assertTrue(not hexResolutionAttr or hexResolutionAttr.Get() == 0)

        #create softbody from voxel for sim mesh (physxDeformable:simulationHexahedralResolution should not be set or 0)
        #this behavior has changed, as now physxDeformable:simulationHexahedralResolution is not written whenever any simulation mesh is specified.
        target_skin_mesh_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, str(conforming_tet_mesh_prim.GetPath()) + "_skin", False))
        deformableUtils.create_skin_mesh_from_tetrahedral_mesh(target_skin_mesh_path, self._stage, conforming_tet_mesh_prim.GetPath())
        omni.kit.commands.execute("AddDeformableBodyComponent",
            skin_mesh_path=target_skin_mesh_path,
            collision_mesh_path=conforming_tet_mesh_prim.GetPath(),
            simulation_mesh_path=voxel_tet_mesh_prim.GetPath())
        hexResolutionAttr = self._stage.GetPrimAtPath(target_skin_mesh_path).GetAttribute("physxDeformable:simulationHexahedralResolution")
        self.assertTrue(not hexResolutionAttr or hexResolutionAttr.Get() == 0)
