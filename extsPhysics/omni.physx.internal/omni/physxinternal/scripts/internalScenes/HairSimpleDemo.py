import math
from pxr import UsdGeom, Gf, UsdPhysics, UsdShade, PhysxSchema
import omni.physxdemos as demo
from omni.physx.scripts import physicsUtils


class HairSimpleDemo(demo.Base):
    title = "Hair Simple"
    category = demo.Categories.INTERNAL
    short_description = "Hair simulation scene setup"
    description = "This snippet sets up a few hair strands and attaches them to a sphere."

    def create(self, stage):
        default_prim_path, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage, zoom=0.1)
        scale = 10
        sphereCenter = Gf.Vec3f(0, 0, scale * 2)

        # Create a kinematic sphere, which will serve as the head
        sphere_prim = physicsUtils.add_rigid_sphere(stage, default_prim_path + "/head", radius=scale, position=sphereCenter)
        UsdPhysics.RigidBodyAPI(sphere_prim).CreateKinematicEnabledAttr().Set(True)

        # Create hair geometry: Use Fibonacci sphere algorithm to determine root positions
        curves = UsdGeom.BasisCurves.Define(stage, default_prim_path + "/hair")
        hair_prim = curves.GetPrim()
        points = []
        vtxCounts = []
        strandWidth = scale * 0.02
        numStrands = 512
        numVertsPerStrand = 64
        vertexDist = scale * 0.02
        overlap = vertexDist / 2  # with sphere prim, needed for auto-attachment

        phi = math.pi * (3.0 - math.sqrt(5.0))
        for strand in range(numStrands):
            z = 1.0 - strand / (numStrands - 1)
            radius = math.sqrt(1.0 - z * z)
            theta = phi * strand
            x = math.cos(theta) * radius
            y = math.sin(theta) * radius
            rootPos = scale * Gf.Vec3f(x, y, z)

            for vtx in range(numVertsPerStrand):
                dir = (rootPos + 0.1 * vtx * Gf.Vec3f(0, 0, -1)).GetNormalized()
                points.append(sphereCenter + rootPos + (vtx * vertexDist - overlap) * dir)
            vtxCounts.append(numVertsPerStrand)
        curves.CreateCurveVertexCountsAttr().Set(vtxCounts)
        curves.CreatePointsAttr().Set(points)
        curves.GetTypeAttr().Set("linear")
        curves.CreateWidthsAttr().Set([strandWidth] * len(points))

        # Apply HairAPI for simulation
        physxHairApi = PhysxSchema.PhysxHairAPI.Apply(curves.GetPrim())
        physxHairApi.CreateSegmentLengthAttr().Set(vertexDist * numVertsPerStrand / 16)
        physxHairApi.CreateGlobalShapeComplianceAtRootAttr().Set(2000.0)
        physxHairApi.CreateGlobalShapeComplianceStrandAttenuationAttr().Set(1.2)

        # Add Hair Material and bind it to the hair prim
        materialPath = default_prim_path + "/hairMaterial"
        material = UsdShade.Material.Define(stage, materialPath)
        hairMaterial = PhysxSchema.PhysxHairMaterialAPI.Apply(material.GetPrim())
        hairMaterial.GetCurveBendStiffnessAttr().Set(0.0)
        bindingAPI = UsdShade.MaterialBindingAPI.Apply(physxHairApi.GetPrim())
        bindingAPI.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")

        # Create attachment between hair and head
        attachment_path = hair_prim.GetPath().AppendElementString("attachment")
        attachment = PhysxSchema.PhysxPhysicsAttachment.Define(stage, attachment_path)
        attachment.GetActor0Rel().SetTargets([hair_prim.GetPath()])
        attachment.GetActor1Rel().SetTargets([sphere_prim.GetPath()])
        PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())
