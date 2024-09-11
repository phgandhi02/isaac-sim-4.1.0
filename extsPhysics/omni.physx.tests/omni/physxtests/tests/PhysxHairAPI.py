from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
import omni
from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase, TestCategory
from omni.physxcommands import AddHairCommand, RemoveHairCommand
from omni.physxtests.tests import PhysicsUtils
import unittest
import math


class PhysxHairCoreTest(PhysicsKitStageAsyncTestCase):
    category = TestCategory.Core

    async def test_apply_unapply_hair_via_cmd(self):
        stage = await self.new_stage()

        numStrands = 10
        numVertsPerStrand = 16
        segmentLength = 1.0
        strandOffset = Gf.Vec3f(1.0, 0.0, 0.0)
        vertexOffset = Gf.Vec3f(0.0, -segmentLength, 0.0)
        curves = PhysicsUtils.create_hair_basiscurves(stage, "/hair", numStrands=numStrands, numVertsPerStrand=numVertsPerStrand,
                                                      strandOffset=strandOffset, vertexOffset=vertexOffset)
        self.assertFalse(curves.GetPrim().HasAPI(PhysxSchema.PhysxHairAPI))

        self.assertTrue(AddHairCommand(curves.GetPath()).do())
        self.assertTrue(curves.GetPrim().HasAPI(PhysxSchema.PhysxHairAPI))

        self.assertTrue(RemoveHairCommand(curves.GetPath()).do())
        self.assertFalse(curves.GetPrim().HasAPI(PhysxSchema.PhysxHairAPI))

    async def test_apply_unapply_hair_via_undo(self):
        stage = await self.new_stage()

        numStrands = 10
        numVertsPerStrand = 16
        segmentLength = 1.0
        strandOffset = Gf.Vec3f(1.0, 0.0, 0.0)
        vertexOffset = Gf.Vec3f(0.0, -segmentLength, 0.0)
        curves = PhysicsUtils.create_hair_basiscurves(stage, "/hair", numStrands=numStrands, numVertsPerStrand=numVertsPerStrand,
                                                      strandOffset=strandOffset, vertexOffset=vertexOffset)
        self.assertFalse(curves.GetPrim().HasAPI(PhysxSchema.PhysxHairAPI))

        cmd = AddHairCommand(curves.GetPath())
        self.assertTrue(cmd.do())

        self.assertTrue(curves.GetPrim().HasAPI(PhysxSchema.PhysxHairAPI))

        cmd.undo()
        self.assertFalse(curves.GetPrim().HasAPI(PhysxSchema.PhysxHairAPI))

    async def test_world_attachment(self):
        stage = await self.new_stage()

        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.CreateSolverTypeAttr("PGS")

        # Set up the hairsystem
        numStrands = 10
        numVertsPerStrand = 16
        segmentLength = 1.0
        strandOffset = Gf.Vec3f(1.0, 0.0, 0.0)
        vertexOffset = Gf.Vec3f(0.0, -segmentLength, 0.0)
        curves = PhysicsUtils.create_hair_basiscurves(stage, "/hair", numStrands=numStrands, numVertsPerStrand=numVertsPerStrand,
                                                      strandOffset=strandOffset, vertexOffset=vertexOffset)

        AddHairCommand(curves.GetPath()).do()
        physxHairApi = PhysxSchema.PhysxHairAPI(curves.GetPrim())
        self.assertTrue(physxHairApi is not None)

        # Set up the attachment
        target_attachment_path = curves.GetPath().AppendElementString("worldAttachment")
        target_attachment_path = Sdf.Path(
            omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False)
        )
        omni.kit.commands.execute(
            "CreatePhysicsAttachment",
            target_attachment_path=target_attachment_path,
            actor0_path=curves.GetPath(),
            actor1_path=None,
        )

        strandRootPositionsStart = []
        for strand in range(numStrands):
            strandRootPositionsStart.append(curves.GetPointsAttr().Get()[strand * numVertsPerStrand])

        await self.step(num_steps=5, dt=1/60)

        # Test target position of strand roots. Due to world attachment, they should not have fallen
        for strand in range(numStrands):
            strandRootPos = curves.GetPointsAttr().Get()[strand * numVertsPerStrand]
            error = strandRootPos - (strandRootPositionsStart[strand])
            self.assertAlmostEqual(error.GetLength(), 0.0)

    async def test_rigid_attachment(self):
        stage = await self.new_stage()

        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.CreateSolverTypeAttr("PGS")

        dt = 1.0 / 60
        numSteps = 5

        # Set up the kinematic cube
        cube = UsdGeom.Cube.Define(stage, "/cube")
        cube.CreateSizeAttr().Set(100)  # make sure it's large enough for auto-attachments
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        physicsAPI.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        # Motion of the cube
        positionDefault = Gf.Vec3d(0.0, 0.0, 0.0)
        position = Gf.Vec3d(0.0, 10.0, 0.0)
        positionEnd = Gf.Vec3d(0.0, 100.0, 0.0)
        xformable = UsdGeom.Xformable(cube.GetPrim())
        translateOp = xformable.AddTranslateOp()
        translateOp.Set(positionDefault)
        translateOp.Set(time=0, value=position)
        translateOp.Set(time=numSteps * dt, value=positionEnd)

        # Set up the hairsystem
        numStrands = 10
        numVertsPerStrand = 16
        segmentLength = 1.0
        strandOffset = Gf.Vec3f(1.0, 0.0, 0.0)
        vertexOffset = Gf.Vec3f(0.0, -segmentLength, 0.0)
        curves = PhysicsUtils.create_hair_basiscurves(stage, "/hair", numStrands=numStrands, numVertsPerStrand=numVertsPerStrand,
                                                      strandOffset=strandOffset, vertexOffset=vertexOffset)

        AddHairCommand(curves.GetPath()).do()
        physxHairApi = PhysxSchema.PhysxHairAPI(curves.GetPrim())
        self.assertTrue(physxHairApi is not None)

        PhysxSchema.PhysxDeformableAPI(physxHairApi).CreateSelfCollisionAttr().Set(False)

        # Set up the attachment
        target_attachment_path = curves.GetPath().AppendElementString("rigidAttachment")
        target_attachment_path = Sdf.Path(
            omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False)
        )
        omni.kit.commands.execute(
            "CreatePhysicsAttachment",
            target_attachment_path=target_attachment_path,
            actor0_path=curves.GetPath(),
            actor1_path=cube.GetPath(),
        )

        strandRootPositionsStart = []
        for strand in range(numStrands):
            strandRootPositionsStart.append(curves.GetPointsAttr().Get()[strand * numVertsPerStrand])

        cubePosStart = cube.GetPrim().GetAttribute("xformOp:translate").Get(0.0)
        self.assertAlmostEqual(cubePosStart, position)

        await self.step(num_steps=numSteps, dt=dt)

        endTime = dt * numSteps
        cubePosEnd = cube.GetPrim().GetAttribute("xformOp:translate").Get(endTime)
        self.assertAlmostEqual(cubePosEnd, positionEnd)
        cubePosDelta = cubePosEnd - cubePosStart

        # Test target position of strand roots
        for strand in range(numStrands):
            strandRootPos = curves.GetPointsAttr().Get()[strand * numVertsPerStrand]
            error = strandRootPos - (strandRootPositionsStart[strand] + Gf.Vec3f(cubePosDelta))
            self.assertAlmostEqual(error.GetLength(), 0.0, delta=1e-6)

    # Test that we can attach to a rigid without collider as long as the min/max hair roots are specified
    # This is useful for attaching to skeletal joints (omniJoint)
    async def test_rigid_attachment_noCollider(self):
        stage = await self.new_stage()

        xform = UsdGeom.Xform.Define(stage, "/xform")
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        physicsAPI.CreateKinematicEnabledAttr().Set(True)

        positionDefault = Gf.Vec3d(0.0, 0.0, 10.0)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        translateOp = xformable.AddTranslateOp()
        translateOp.Set(positionDefault)
        # NOTE: No collider or shape here.

        # Set up the hairsystem
        numStrands = 8
        numVertsPerStrand = 16
        segmentLength = 1.0
        strandOffset = Gf.Vec3f(1.0, 0.0, 0.0)
        vertexOffset = Gf.Vec3f(0.0, -segmentLength, 0.0)
        curves = PhysicsUtils.create_hair_basiscurves(stage, "/hair", numStrands=numStrands, numVertsPerStrand=numVertsPerStrand,
                                                      strandOffset=strandOffset, vertexOffset=vertexOffset)

        AddHairCommand(curves.GetPath()).do()
        physxHairApi = PhysxSchema.PhysxHairAPI(curves.GetPrim())
        self.assertTrue(physxHairApi is not None)

        PhysxSchema.PhysxDeformableAPI(physxHairApi).CreateSelfCollisionAttr().Set(False)

        # Set up the attachment
        target_attachment_path = curves.GetPath().AppendElementString("rigidAttachment")
        target_attachment_path = Sdf.Path(
            omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False)
        )
        omni.kit.commands.execute(
            "CreatePhysicsAttachment",
            target_attachment_path=target_attachment_path,
            actor0_path=curves.GetPath(),
            actor1_path=xform.GetPath(),
        )

        strandRootPositionsStart = []
        curvePoints = curves.GetPointsAttr().Get()
        for strand in range(numStrands):
            strandRootPositionsStart.append(curvePoints[strand * numVertsPerStrand])

        dt = 1.0 / 60
        numSteps = 5
        await self.step(num_steps=numSteps, dt=dt)
        
        # Test target position of strand roots
        curvePoints = curves.GetPointsAttr().Get()
        for strand in range(numStrands):
            strandRootPos = curvePoints[strand * numVertsPerStrand]
            error = strandRootPos - strandRootPositionsStart[strand]
            self.assertAlmostEqual(error.GetLength(), 0.0, delta=0.1)

    # Test that hair can be simulated on a mesh
    async def test_geom_mesh(self):
        stage = await self.new_stage()

        mesh = PhysicsUtils.create_hair_mesh(stage, "/hair")

        AddHairCommand(mesh.GetPath()).do()
        physxHairApi = PhysxSchema.PhysxHairAPI(mesh.GetPrim())
        self.assertTrue(physxHairApi is not None)

        origPositions = mesh.GetPointsAttr().Get()

        numSteps = 5
        dt = 1/60
        await self.step(num_steps=numSteps, dt=dt)

        newPositions = mesh.GetPointsAttr().Get()

        #expect everything has moved
        for origPos, newPos in zip(origPositions, newPositions):
            delta = (origPos - newPos).GetLength()
            self.assertNotAlmostEqual(delta, 0.0)

    # Test that hair can be simulated on a mesh subset and that parts NOT part of the subset don't move
    @unittest.skip("Flaky test: https://nvidia-omniverse.atlassian.net/browse/OM-86278")
    async def test_geom_mesh_subset(self):
        stage = await self.new_stage()

        mesh, subset = PhysicsUtils.create_hair_mesh_subset(stage, "/hair")

        AddHairCommand(subset.GetPath()).do()
        physxHairApi = PhysxSchema.PhysxHairAPI(subset.GetPrim())
        self.assertTrue(physxHairApi is not None)

        origPositions = mesh.GetPointsAttr().Get()

        numSteps = 5
        dt = 1/60
        await self.step(num_steps=numSteps, dt=dt)

        newPositions = mesh.GetPointsAttr().Get()

        #expect only first half of vertices have moved
        numVertices = len(origPositions)
        for i, (origPos, newPos) in enumerate(zip(origPositions, newPositions)):
            delta = (origPos - newPos).GetLength()
            if i < numVertices / 2:
                self.assertNotAlmostEqual(delta, 0.0)
            else:
                self.assertAlmostEqual(delta, 0.0)

    async def test_transform_and_scale_hair(self):
        # Ensure that transform (including scale) on the hair prim is properly picked up and does not cause a jump

        stage = await self.new_stage()

        scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        scene.CreateGravityMagnitudeAttr().Set(0.0)

        # Arbitrary xform with translation, scale, and rotation
        xform = UsdGeom.Xform.Define(stage, "/xform")
        translateOp = xform.AddTranslateOp()
        scaleOp = xform.AddScaleOp()
        rotateXyzOp = xform.AddRotateXYZOp()
        translateOp.Set(Gf.Vec3d(2.0, -2.0, 1.0))
        # scaleOp.Set(Gf.Vec3d(2.0, 3.0, 4.0)) # does not work yet: OM-74465
        rotateXyzOp.Set(Gf.Vec3d(20, 30, 40))

        # Set up the hairsystem under the xform
        numStrands = 4
        numVertsPerStrand = 8
        segmentLength = 1.0
        strandOffset = Gf.Vec3f(1.0, 0.0, 0.0)
        vertexOffset = Gf.Vec3f(0.0, -segmentLength, 0.0)
        curves = PhysicsUtils.create_hair_basiscurves(stage, "/xform/hair", numStrands=numStrands, numVertsPerStrand=numVertsPerStrand,
                                                      strandOffset=strandOffset, vertexOffset=vertexOffset)
        AddHairCommand(curves.GetPath()).do()
        self.assertTrue(curves.GetPrim().HasAPI(PhysxSchema.PhysxHairAPI))

        initPos = curves.GetPointsAttr().Get()

        await self.step(num_steps=2, dt=1/60)

        finalPos = curves.GetPointsAttr().Get()

        for iPos, fPos in zip(initPos, finalPos):
            error = (iPos-fPos).GetLength()
            self.assertAlmostEqual(error, 0.0, delta=0.2)

    async def test_transform_and_scale_rigid_attachment(self):
        # Ensure that a transform (including scale) on the attachment rigid does not cause a jump

        stage = await self.new_stage()

        # Arbitrary xform with translation, scale, and rotation
        xform = UsdGeom.Xform.Define(stage, "/xform")
        xform.AddTranslateOp().Set(Gf.Vec3d(2.0, -2.0, 1.0))
        xform.AddRotateXYZOp().Set(Gf.Vec3d(20, 30, 40))
        xform.AddScaleOp().Set(Gf.Vec3d(2.0, 3.0, 4.0))

        # Set up rigid attachment body under the xform
        cube = UsdGeom.Cube.Define(stage, "/xform/cube")
        cube.CreateSizeAttr().Set(100)  # make sure it's large enough for auto-attachments
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        physicsAPI.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim()) # needed for auto-attachments
        massAPI = UsdPhysics.MassAPI.Apply(physicsAPI.GetPrim())
        massAPI.GetCenterOfMassAttr().Set(Gf.Vec3d(1.0, 0.0, 0.0))

        # Arbitrary xform with translation and rotation for the hair (no scale!)
        xform = UsdGeom.Xform.Define(stage, "/xform2")
        xform.AddTranslateOp().Set(Gf.Vec3d(-2.0, 2.0, 1.0))
        xform.AddRotateXYZOp().Set(Gf.Vec3d(-20, 30, -20))

        # Set up the hairsystem (no scale or transform)
        numStrands = 4
        numVertsPerStrand = 8
        segmentLength = 1.0
        strandOffset = Gf.Vec3f(1.0, 0.0, 0.0)
        vertexOffset = Gf.Vec3f(0.0, -segmentLength, 0.0)
        curves = PhysicsUtils.create_hair_basiscurves(stage, "/xform2/hair", numStrands=numStrands, numVertsPerStrand=numVertsPerStrand,
                                                      strandOffset=strandOffset, vertexOffset=vertexOffset)
        AddHairCommand(curves.GetPath()).do()
        self.assertTrue(curves.GetPrim().HasAPI(PhysxSchema.PhysxHairAPI))
        physxHairApi = PhysxSchema.PhysxHairAPI(curves.GetPrim())
        physxHairApi.CreateExternalCollisionAttr().Set(False)

        # Set up the attachment
        target_attachment_path = curves.GetPath().AppendElementString("rigidAttachment")
        target_attachment_path = Sdf.Path(
            omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False)
        )
        omni.kit.commands.execute(
            "CreatePhysicsAttachment",
            target_attachment_path=target_attachment_path,
            actor0_path=curves.GetPath(),
            actor1_path=cube.GetPath(),
        )

        # simulate and check
        initPos = curves.GetPointsAttr().Get()

        await self.step(num_steps=2, dt=1/60)

        finalPos = curves.GetPointsAttr().Get()

        for iPos, fPos in zip(initPos, finalPos):
            error = (iPos-fPos).GetLength()
            self.assertAlmostEqual(error, 0.0, delta=0.05)

    async def test_mirrored_rigid_attachment(self):
        stage = await self.new_stage()

        scene0 = UsdPhysics.Scene.Define(stage, "/physicsScene0")
        scene1 = UsdPhysics.Scene.Define(stage, "/physicsScene1")
        physxScene0API = PhysxSchema.PhysxSceneAPI.Apply(scene0.GetPrim())
        physxScene1API = PhysxSchema.PhysxSceneAPI.Apply(scene1.GetPrim())
        physxScene0API.CreateSolverTypeAttr("PGS")
        physxScene1API.CreateSolverTypeAttr("PGS")

        dt = 1.0 / 60
        numSteps = 5

        # Set up the kinematic cube with two simulation owners
        cube = UsdGeom.Cube.Define(stage, "/cube")
        cube.CreateSizeAttr().Set(100)  # make sure it's large enough for auto-attachments
        rigidApi = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        rigidApi.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        rigidApi.GetSimulationOwnerRel().SetTargets([scene0.GetPrim().GetPrimPath(), scene1.GetPrim().GetPrimPath()])

        # Motion of the cube
        positionDefault = Gf.Vec3d(0.0, 0.0, 0.0)
        position = Gf.Vec3d(0.0, 10.0, 0.0)
        positionEnd = Gf.Vec3d(0.0, 100.0, 0.0)
        xformable = UsdGeom.Xformable(cube.GetPrim())
        endTime = dt * numSteps
        translateOp = xformable.AddTranslateOp()
        translateOp.Set(positionDefault)
        translateOp.Set(time=0, value=position)
        translateOp.Set(time=endTime, value=positionEnd)

        # Set up the hairsystem on scene 1
        numStrands = 10
        numVertsPerStrand = 16
        segmentLength = 1.0
        strandOffset = Gf.Vec3f(1.0, 0.0, 0.0)
        vertexOffset = Gf.Vec3f(0.0, -segmentLength, 0.0)
        curves = PhysicsUtils.create_hair_basiscurves(stage, "/hair", numStrands=numStrands, numVertsPerStrand=numVertsPerStrand,
                                                      strandOffset=strandOffset, vertexOffset=vertexOffset)

        AddHairCommand(curves.GetPath()).do()
        physxHairApi = PhysxSchema.PhysxHairAPI(curves.GetPrim())
        self.assertTrue(physxHairApi is not None)
        PhysxSchema.PhysxDeformableAPI(physxHairApi).GetSimulationOwnerRel().SetTargets([scene1.GetPrim().GetPrimPath()])
        PhysxSchema.PhysxDeformableAPI(physxHairApi).CreateSelfCollisionAttr().Set(False)

        # Set up the attachment
        target_attachment_path = curves.GetPath().AppendElementString("rigidAttachment")
        target_attachment_path = Sdf.Path(
            omni.usd.get_stage_next_free_path(self._stage, str(target_attachment_path), False)
        )
        omni.kit.commands.execute(
            "CreatePhysicsAttachment",
            target_attachment_path=target_attachment_path,
            actor0_path=curves.GetPath(),
            actor1_path=cube.GetPath(),
        )

        strandRootPositionsStart = []
        for strand in range(numStrands):
            strandRootPositionsStart.append(curves.GetPointsAttr().Get()[strand * numVertsPerStrand])

        cubePosStart = cube.GetPrim().GetAttribute("xformOp:translate").Get(0.0)
        self.assertAlmostEqual(cubePosStart, position)

        await self.step(num_steps=numSteps, dt=dt)

        cubePosEnd = cube.GetPrim().GetAttribute("xformOp:translate").Get(endTime)
        self.assertAlmostEqual(cubePosEnd, positionEnd)
        cubePosDelta = cubePosEnd - cubePosStart

        # Test target position of strand roots
        for strand in range(numStrands):
            strandRootPos = curves.GetPointsAttr().Get()[strand * numVertsPerStrand]
            error = strandRootPos - (strandRootPositionsStart[strand] + Gf.Vec3f(cubePosDelta))
            self.assertAlmostEqual(error.GetLength(), 0.0, delta=1e-6)

    # Ensure that skinning curved strands properly resets to initial conditions
    # Similar setup to HairSimpleDemo
    async def test_curved_strands_skinning_reset_consistency(self):
        stage = await self.new_stage()

        scale = 10
        sphereCenter = Gf.Vec3f(0, 0, scale * 2)

        # Create a kinematic sphere, which will serve as the head
        xform = UsdGeom.Xform.Define(stage, "/xform")
        xform.AddTranslateOp().Set(sphereCenter)

        # Set up rigid attachment body under the xform
        sphere = UsdGeom.Sphere.Define(stage, "/xform/sphere")
        sphere.CreateRadiusAttr(scale)
        sphere.CreateExtentAttr([(-scale, -scale, -scale), (scale, scale, scale)])
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
        physicsAPI.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim()) # needed for auto-attachments

        # Create hair geometry: Use Fibonacci sphere algorithm to determine root positions
        curves = UsdGeom.BasisCurves.Define(stage, "/hair")
        hair_prim = curves.GetPrim()
        points = []
        vtxCounts = []
        strandWidth = scale * 0.02
        numStrands = 512
        numVertsPerStrand = 40
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
        materialPath = "/hairMaterial"
        material = UsdShade.Material.Define(stage, materialPath)
        hairMaterial = PhysxSchema.PhysxHairMaterialAPI.Apply(material.GetPrim())
        hairMaterial.GetCurveBendStiffnessAttr().Set(0.0)
        bindingAPI = UsdShade.MaterialBindingAPI.Apply(physxHairApi.GetPrim())
        bindingAPI.Bind(material, UsdShade.Tokens.weakerThanDescendants, "physics")

        # Create attachment between hair and head
        attachment_path = hair_prim.GetPath().AppendElementString("attachment")
        attachment = PhysxSchema.PhysxPhysicsAttachment.Define(stage, attachment_path)
        attachment.GetActor0Rel().SetTargets([hair_prim.GetPath()])
        attachment.GetActor1Rel().SetTargets([sphere.GetPath()])
        PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())

        pos_before = curves.GetPointsAttr().Get()
        await self.step(num_steps=2, dt=1/60, stop_timeline_after=False)
        physx_interface = omni.physx.get_physx_interface()
        physx_interface.reset_simulation()
        pos_after = curves.GetPointsAttr().Get()

        self.assertEqual(len(pos_before), len(pos_after))

        for iPos, fPos in zip(pos_before, pos_after):
            error = (iPos-fPos).GetLength()
            self.assertEqual(error, 0.0)
