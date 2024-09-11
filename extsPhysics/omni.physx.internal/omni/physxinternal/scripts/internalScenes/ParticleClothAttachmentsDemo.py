import os
import omni.kit.commands
import omni.physxdemos as demo

from omni.physx.scripts import physicsUtils, deformableUtils
from pxr import UsdGeom, UsdLux, Sdf, Gf, Vt, UsdPhysics, Usd, PhysxSchema
from omni.physxcommands import AddGroundPlaneCommand, SetRigidBodyCommand, SetStaticColliderCommand
from omni.physx import get_physx_interface, get_physx_cooking_interface


class ParticleClothAttachmentsDemo(demo.Base):
    title = "Particle Cloth Attachments"
    category = demo.Categories.INTERNAL
    short_description = "Particle Cloth Attachments"
    description = "This snippet shows the different types of particle cloth attachments."


    @staticmethod
    def _set_prim_translation(prim: Usd.Prim, translateVec: Gf.Vec3d, scaleVec: Gf.Vec3d=Gf.Vec3d(1.0,1.0,1.0)):
        translate_mtx = Gf.Matrix4d().SetScale(scaleVec)
        translate_mtx.SetTranslateOnly(translateVec)
        omni.kit.commands.execute("TransformPrim", path=prim.GetPath(), new_transform_matrix=translate_mtx)


    def _create_mesh_prims(self, prim_type_list: list) -> list:
        mesh_list = []
        for prim_type in prim_type_list:
            path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/" + prim_type, True))
            omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type=prim_type)
            mesh = UsdGeom.Mesh.Get(self._stage, path)
            mesh_list.append(mesh)
        return mesh_list


    def _create_mesh_primitives(self, prim_type_list, start_height=0.0, start_x=0.0, offset=100.0, scale=Gf.Vec3d(1.0,1.0,1.0)):
        mesh_list = self._create_mesh_prims(prim_type_list)

        origin = Gf.Vec3d(start_x, start_height, 0.0)
        for index, prim_type in enumerate(prim_type_list):
            self._set_prim_translation(mesh_list[index].GetPrim(), origin + Gf.Vec3d(index * offset, 0.0, 0.0), scale)

        return mesh_list


    def _runAddDeformableBodyComponentCommand(self, skin_mesh_path: Sdf.Path=Sdf.Path(), collision_mesh_path: Sdf.Path=Sdf.Path(), simulation_mesh_path: Sdf.Path=Sdf.Path()) -> bool:
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


    def create(self, stage):
        self._stage = stage

        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        self._upAxis = UsdGeom.GetStageUpAxis(self._stage)
        self._defaultPrimPath = self._stage.GetDefaultPrim().GetPath()

        # viewer:
        self._viewerPosition = Gf.Vec3d(29.72, 774.76, 1229.86)
        self._viewerTarget = Gf.Vec3d(21.76, -243.60, -1066.56)

        # set cam:
        customLayerData = {
            "cameraSettings": {
                "Perspective": {"position": self._viewerPosition, "radius": 500, "target": self._viewerTarget},
                "boundCamera": "/OmniverseKit_Persp",
            }
        }
        stage.SetMetadata("customLayerData", customLayerData)

        # add physics scene
        omni.kit.commands.execute("AddPhysicsScene", stage=self._stage, path='/World/PhysicsScene')

        # create a material (make a bit squishier for clearer deformation results)
        self._deformable_body_material_path = '/World/DeformableBodyMaterial'
        omni.kit.commands.execute("AddDeformableBodyMaterial",
                                  stage=self._stage, path=self._deformable_body_material_path,
                                  youngsModulus=5000.0)

        AddGroundPlaneCommand.execute(self._stage, '/CollisionPlane',
                                      self._upAxis, 1000.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        cubePrim0 = physicsUtils.add_cube(stage, "/Cube", 100.0, Gf.Vec3f(-200.0, 500, 0.0))
        SetRigidBodyCommand.execute(cubePrim0.GetPath(), "", True)

        cubePrim1 = physicsUtils.add_cube(stage, "/Cube_01", 100.0, Gf.Vec3f(200.0, 500, 0.0))
        SetStaticColliderCommand.execute(cubePrim1.GetPath(), "")

        rigid_actor_list=[cubePrim0, cubePrim1]

        clothMesh=[None,None]
        for i in range(2):

            # Create grid mesh
            clothMeshPath = omni.usd.get_stage_next_free_path(stage, "/cloth" + str(i), True)
            clothMesh[i] = UsdGeom.Mesh.Define(stage, clothMeshPath)
            tri_points, tri_indices = deformableUtils.create_triangle_mesh_square(16, 16, 100.0)
            clothMesh[i].GetPointsAttr().Set(tri_points)
            clothMesh[i].GetFaceVertexIndicesAttr().Set(tri_indices)
            clothMesh[i].GetFaceVertexCountsAttr().Set([3]*(len(tri_indices)//3))
            color_rgb = [0.0, 0.0, 0.0]
            color_rgb[i % 3] = 1.0
            color = Vt.Vec3fArray([Gf.Vec3f(color_rgb[0], color_rgb[1], color_rgb[2])])
            clothMesh[i].CreateDisplayColorAttr(color)

            omni.kit.commands.execute("AddParticleClothComponent", prim_path=clothMesh[i].GetPrim().GetPath())
            auto_cloth_api = PhysxSchema.PhysxAutoParticleClothAPI(clothMesh[i].GetPrim())
            auto_cloth_api.GetSpringStretchStiffnessAttr().Set(1000000)
            auto_cloth_api.GetSpringBendStiffnessAttr().Set(1000000)
            auto_cloth_api.GetSpringShearStiffnessAttr().Set(1000000)

            clothMesh[i].AddTranslateOp().Set(Gf.Vec3f(-200.0 + (i * 400.0), 300.0, 0.0))
            clothMesh[i].AddScaleOp().Set(Gf.Vec3f(1.0, 3.5, 1.0))
            target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, clothMeshPath + "/Attachment", True))
            omni.kit.commands.execute("CreatePhysicsAttachment",
                                      target_attachment_path=target_attachment_path,
                                      actor0_path=clothMesh[i].GetPath(),
                                      actor1_path=rigid_actor_list[i].GetPath())

        mesh_list = self._create_mesh_primitives(['Sphere', 'Sphere'], start_height=100.0, start_x=-200.0, offset=400.0)
        for index, m in enumerate(mesh_list):
            self._runAddDeformableBodyComponentCommand(skin_mesh_path=m.GetPath())
            target_attachment_path = Sdf.Path(omni.usd.get_stage_next_free_path(self._stage, "/Attachment", True))
            omni.kit.commands.execute("CreatePhysicsAttachment",
                                      target_attachment_path=target_attachment_path,
                                      actor0_path=mesh_list[index].GetPath(),
                                      actor1_path=clothMesh[index].GetPath())
