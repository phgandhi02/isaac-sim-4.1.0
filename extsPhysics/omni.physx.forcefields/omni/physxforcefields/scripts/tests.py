import math
import os
import random
import unittest
from pxr import Gf, Sdf, Usd
from pxr import UsdGeom, UsdUtils, UsdPhysics
from pxr import PhysxSchema, PhysicsSchemaTools, ForceFieldSchema

from omni.physxtests.utils.physicsBase import PhysicsKitStageAsyncTestCase

import omni.physx.scripts.physicsUtils as physicsUtils
import omni.kit

import omni.graph.core as og


gPhysXInterface = None
gPhysXSceneQueryInterface = None
gPhysXUnitTestInterface = None
gPhysXForceFieldsInterface = None


def setPhysxInterface(physxInterface):
    global gPhysXInterface
    gPhysXInterface = physxInterface

def clearPhysxInterface():
    global gPhysXInterface
    gPhysXInterface = None


def setPhysxSimInterface(physxSimInterface):
    global gPhysXSimInterface
    gPhysXSimInterface = physxSimInterface

def clearPhysxSimInterface():
    global gPhysXSimInterface
    gPhysXSimInterface = None


def setPhysxSceneQueryInterface(physxSceneQueryInterface):
    global gPhysXSceneQueryInterface
    gPhysXSceneQueryInterface = physxSceneQueryInterface

def clearPhysxSceneQueryInterface():
    global gPhysXSceneQueryInterface
    gPhysXSceneQueryInterface = None


def setPhysxUnitTestInterface(physxUnitTestInterface):
    global gPhysXUnitTestInterface
    gPhysXUnitTestInterface = physxUnitTestInterface

def clearPhysxUnitTestInterface():
    global gPhysXUnitTestInterface
    gPhysXUnitTestInterface = None


def setPhysxForceFieldsInterface(physxForceFieldsInterface):
    global gPhysXForceFieldsInterface
    gPhysXForceFieldsInterface = physxForceFieldsInterface

def clearPhysxForceFieldsInterface():
    global gPhysXForceFieldsInterface
    gPhysXForceFieldsInterface = None


class PhysXForceFieldsTestBase:
        
    def _get_time_step(self):
        return 1.0 / 60.0

    def _prepare_for_simulation(self):
        return

    def _simulate_one_frame(self, elapsedTime=None):
        return

    def _simulate_one_frame_with_prep(self):
        self._prepare_for_simulation()
        self._simulate_one_frame()

    def _simulate(self, secondsToRun):
        return

    def _simulate_with_prep(self, secondsToRun):
        self._prepare_for_simulation()
        self._simulate(secondsToRun)

    def _write_back_transforms(self, updateToFastCache, updateToUsd, updateVelocitiesToUsd):
        global gPhysXInterface
        gPhysXInterface.update_transformations(updateToFastCache, updateToUsd, updateVelocitiesToUsd)

    def _get_delta_angle_radians(self, dirA, dirB):
        angle = math.acos(dirA.GetDot(dirB))
        return angle

    def _get_delta_angle_degree(self, dirA, dirB):
        angle = (self._get_delta_angle_radians(dirA, dirB) * 180) / math.pi
        return angle

    def _rotate_dir(self, dir, transform):
        # dir is expected to be a GfVec3
        # transform is expected to be a GfMatrix4
        quat = transform.ExtractRotation()
        rotatedDir = quat.TransformDir(dir)
        return rotatedDir

    def _local_dir_to_world(self, dir, xformable):
        # dir is expected to be a GfVec3
        return self._rotate_dir(dir, xformable.ComputeLocalToWorldTransform(0))

    def _get_sim_stats(self):
        global gPhysXUnitTestInterface
        return gPhysXUnitTestInterface.get_physics_stats()

    def _check_sim_stats(
        self,
        simStats,
        numDynRig,
        numStatRig,
        numKinRig,
        numSphereShapes,
        numBoxShapes,
        numCapsShapes,
        numCylShapes,
        numConvexShapes,
        numTriShapes,
        numPlShapes,
        numConeShapes,
    ):
        self.assertEqual(simStats["numDynamicRigids"], numDynRig)
        self.assertEqual(simStats["numStaticRigids"], numStatRig)
        self.assertEqual(simStats["numKinematicBodies"], numKinRig)
        self.assertEqual(simStats["numSphereShapes"], numSphereShapes)
        self.assertEqual(simStats["numBoxShapes"], numBoxShapes)
        self.assertEqual(simStats["numCapsuleShapes"], numCapsShapes)
        self.assertEqual(simStats["numCylinderShapes"], numCylShapes)
        self.assertEqual(simStats["numConvexShapes"], numConvexShapes)
        self.assertEqual(simStats["numTriMeshShapes"], numTriShapes)
        self.assertEqual(simStats["numPlaneShapes"], numPlShapes)
        self.assertEqual(simStats["numConeShapes"], numConeShapes)


class PhysXForceFieldsTestKitStage(PhysicsKitStageAsyncTestCase, PhysXForceFieldsTestBase):

    def _prepare_for_simulation(self):
        global gPhysXInterface
        global gPhysXForceFieldsInterface
        
        gPhysXInterface.start_simulation()

        stage = self.get_stage()
        stageId = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        gPhysXForceFieldsInterface.attach_stage(stageId)

    def _simulate_one_frame(self, elapsedTime=None):
        if (elapsedTime is None):
            timeStep = self._get_time_step()
        else:
            timeStep = elapsedTime
        currentTime = 0.0

        global gPhysXInterface
        gPhysXInterface.update_simulation(timeStep, currentTime)

    def _simulate(self, secondsToRun):
        timeStep = self._get_time_step()
        targetIterationCount = math.ceil(secondsToRun / timeStep)
        currentTime = 0.0

        global gPhysXForceFieldsInterface
        global gPhysXInterface

        for i in range(targetIterationCount):
            gPhysXForceFieldsInterface.update(timeStep, currentTime)
            gPhysXInterface.update_simulation(timeStep, currentTime)

            currentTime += timeStep

    def _end_simulation(self):
        gPhysXForceFieldsInterface.detach_stage()

    async def _open_usd(self, filename):
        data_path = "../../../../data/tests"
        schema_folder = os.path.abspath(os.path.normpath(os.path.join(__file__, data_path)))
        schema_folder = schema_folder.replace("\\", "/") + "/"

        await omni.usd.get_context().open_stage_async(schema_folder + filename + ".usda")

        usd_context = omni.usd.get_context()
        self.assertIn(filename, usd_context.get_stage_url())

    async def _load_usd_file(
        self,
        fileName,
        numDynRig,
        numStatRig,
        numKinRig,
        numSphereShapes,
        numBoxShapes,
        numCapsShapes,
        numCylShapes,
        numConvexShapes,
        numTriShapes,
        numPlShapes,
        numConeShapes,
    ):
        await self._open_usd(fileName)

        self.set_stage(omni.usd.get_context().get_stage())  # else preparation fails as get_stage() returns None
        self._simulate_one_frame_with_prep()
        simStats = self._get_sim_stats()

        self._check_sim_stats(
            simStats,
            numDynRig,
            numStatRig,
            numKinRig,
            numSphereShapes,
            numBoxShapes,
            numCapsShapes,
            numCylShapes,
            numConvexShapes,
            numTriShapes,
            numPlShapes,
            numConeShapes,
        )
 
    async def test_create_all_force_fields(self):
        stage = await self.new_stage()

        primPathName = "/World/ForceFieldPrim"

        # Create the force field prim
        xformPrim = UsdGeom.Xform.Define(stage, primPathName)
        xformPrim.AddTranslateOp().Set(Gf.Vec3f(0.0, 300.0, 0.0))
        prim = xformPrim.GetPrim()

        dragApi = ForceFieldSchema.PhysxForceFieldDragAPI.Apply(prim, "drag")
        forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(dragApi, "drag")
        dragApi.CreateMinimumSpeedAttr(10.0)
        dragApi.CreateLinearAttr(1.0e6)
        dragApi.CreateSquareAttr(0.0)
        forceFieldApi.CreateEnabledAttr(True)
        forceFieldApi.CreatePositionAttr(Gf.Vec3f(1.0, 2.0, 2.0))
        forceFieldApi.CreateRangeAttr(Gf.Vec2f(10.0, 20.0))

        linearApi = ForceFieldSchema.PhysxForceFieldLinearAPI.Apply(prim, "linear")
        forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(linearApi, "linear")
        linearApi.CreateDirectionAttr(Gf.Vec3f(0.0, 1.0, 0.0))
        linearApi.CreateConstantAttr(1.0)
        linearApi.CreateLinearAttr(2.0)
        linearApi.CreateInverseSquareAttr(3.0)
        forceFieldApi.CreateEnabledAttr(True)
        forceFieldApi.CreatePositionAttr(Gf.Vec3f(1.0, 2.0, 3.0))
        forceFieldApi.CreateRangeAttr(Gf.Vec2f(10.0, 20.0))

        noiseApi = ForceFieldSchema.PhysxForceFieldNoiseAPI.Apply(prim, "noise")
        forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(noiseApi, "noise")
        noiseApi.CreateDragAttr(1.0)
        noiseApi.CreateAmplitudeAttr(Gf.Vec3f(1.0, 2.0, 3.0))
        noiseApi.CreateFrequencyAttr(Gf.Vec3f(4.0, 5.0, 6.0))
        forceFieldApi.CreateEnabledAttr(True)
        forceFieldApi.CreatePositionAttr(Gf.Vec3f(1.0, 2.0, 3.0))
        forceFieldApi.CreateRangeAttr(Gf.Vec2f(10.0, 20.0))

        planarApi = ForceFieldSchema.PhysxForceFieldPlanarAPI.Apply(prim, "planar")
        forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(planarApi, "planar")
        planarApi.CreateNormalAttr(Gf.Vec3f(0.0, 0.0, 1.0))
        planarApi.CreateConstantAttr(1.0)
        planarApi.CreateLinearAttr(2.0)
        planarApi.CreateInverseSquareAttr(3.0)
        forceFieldApi.CreateEnabledAttr(True)
        forceFieldApi.CreatePositionAttr(Gf.Vec3f(1.0, 2.0, 3.0))
        forceFieldApi.CreateRangeAttr(Gf.Vec2f(10.0, 20.0))

        sphericalApi = ForceFieldSchema.PhysxForceFieldSphericalAPI.Apply(prim, "spherical")
        forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(sphericalApi, "spherical")
        sphericalApi.CreateConstantAttr(1.0)
        sphericalApi.CreateLinearAttr(2.0)
        sphericalApi.CreateInverseSquareAttr(3.0)
        forceFieldApi.CreateEnabledAttr(True)
        forceFieldApi.CreatePositionAttr(Gf.Vec3f(1.0, 2.0, 3.0))
        forceFieldApi.CreateRangeAttr(Gf.Vec2f(10.0, 20.0))

        conicalApi = ForceFieldSchema.PhysxForceFieldConicalAPI.Apply(prim, "conical")
        forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(conicalApi, "conical")
        conicalApi.CreateConstantAttr(1.0)
        conicalApi.CreateLinearAttr(2.0)
        conicalApi.CreateInverseSquareAttr(3.0)
        conicalApi.CreateDirectionAttr(Gf.Vec3f(3.0, 2.0, 1.0))
        conicalApi.CreateAngleAttr(4.0)
        conicalApi.CreateLinearFalloffAttr(0.5)
        conicalApi.CreatePowerFalloffAttr(6.0)
        forceFieldApi.CreateEnabledAttr(True)
        forceFieldApi.CreatePositionAttr(Gf.Vec3f(1.0, 2.0, 3.0))
        forceFieldApi.CreateRangeAttr(Gf.Vec2f(10.0, 20.0))

        spinApi = ForceFieldSchema.PhysxForceFieldSpinAPI.Apply(prim, "spin")
        forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(spinApi, "spin")
        spinApi.CreateSpinAxisAttr(Gf.Vec3f(0.0, 0.0, 1.0))
        spinApi.CreateConstantAttr(1.0)
        spinApi.CreateLinearAttr(2.0)
        spinApi.CreateInverseSquareAttr(3.0)
        forceFieldApi.CreateEnabledAttr(True)
        forceFieldApi.CreatePositionAttr(Gf.Vec3f(1.0, 2.0, 3.0))
        forceFieldApi.CreateRangeAttr(Gf.Vec2f(10.0, 20.0))

        windApi = ForceFieldSchema.PhysxForceFieldWindAPI.Apply(prim, "wind")
        forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(windApi, "wind")
        windApi.CreateDragAttr(1.0)
        windApi.CreateAverageSpeedAttr(1.0)
        windApi.CreateSpeedVariationAttr(2.0)
        windApi.CreateSpeedVariationFrequencyAttr(3.0)
        windApi.CreateAverageDirectionAttr(Gf.Vec3f(0.0, 0.0, 1.0))
        windApi.CreateDirectionVariationAttr(Gf.Vec3f(0.707, 0.707, 0.0))
        windApi.CreateDirectionVariationFrequencyAttr(Gf.Vec3f(1.0, 2.0, 3.0))
        forceFieldApi.CreateEnabledAttr(True)
        forceFieldApi.CreatePositionAttr(Gf.Vec3f(1.0, 2.0, 3.0))
        forceFieldApi.CreateRangeAttr(Gf.Vec2f(10.0, 20.0))

        # Ensure all of the schemas were applied
        schemas = prim.GetAppliedSchemas()

        found = [False] * 7

        for s in schemas:
            instance = s[s.find(":") + 1:]

            if s.find("PhysxForceFieldDragAPI") >= 0:
                found[0] = True
                dragTest = ForceFieldSchema.PhysxForceFieldDragAPI.Get(prim, instance)
                forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(dragTest, instance)
                self.assertEqual(dragTest.GetMinimumSpeedAttr().Get(), 10.0)
                self.assertEqual(dragTest.GetLinearAttr().Get(), 1.0e6)
                self.assertEqual(dragTest.GetSquareAttr().Get(), 0.0)
                self.assertEqual(forceFieldApi.GetEnabledAttr().Get(), True)
                self.assertEqual(forceFieldApi.GetPositionAttr().Get(), Gf.Vec3f(1.0, 2.0, 2.0))
                self.assertEqual(forceFieldApi.GetRangeAttr().Get(), Gf.Vec2f(10.0, 20.0))
            elif s.find("PhysxForceFieldLinearAPI") >= 0:
                found[1] = True
                linearApi = ForceFieldSchema.PhysxForceFieldLinearAPI.Get(prim, instance)
                forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(linearApi, instance)
                self.assertEqual(linearApi.CreateDirectionAttr().Get(), Gf.Vec3f(0.0, 1.0, 0.0))
                self.assertEqual(linearApi.CreateConstantAttr().Get(), 1.0)
                self.assertEqual(linearApi.CreateLinearAttr().Get(), 2.0)
                self.assertEqual(linearApi.CreateInverseSquareAttr().Get(), 3.0)
                self.assertEqual(forceFieldApi.CreateEnabledAttr().Get(), True)
                self.assertEqual(forceFieldApi.CreatePositionAttr().Get(), Gf.Vec3f(1.0, 2.0, 3.0))
                self.assertEqual(forceFieldApi.CreateRangeAttr().Get(), Gf.Vec2f(10.0, 20.0))                
            elif s.find("PhysxForceFieldNoiseAPI") >= 0:
                found[2] = True
                noiseApi = ForceFieldSchema.PhysxForceFieldNoiseAPI.Get(prim, instance)
                forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(noiseApi, instance)
                self.assertEqual(noiseApi.CreateDragAttr().Get(), 1.0)
                self.assertEqual(noiseApi.CreateAmplitudeAttr().Get(), Gf.Vec3f(1.0, 2.0, 3.0))
                self.assertEqual(noiseApi.CreateFrequencyAttr().Get(), Gf.Vec3f(4.0, 5.0, 6.0))
                self.assertEqual(forceFieldApi.CreateEnabledAttr().Get(), True)
                self.assertEqual(forceFieldApi.CreatePositionAttr().Get(), Gf.Vec3f(1.0, 2.0, 3.0))
                self.assertEqual(forceFieldApi.CreateRangeAttr().Get(), Gf.Vec2f(10.0, 20.0))                
            elif s.find("PhysxForceFieldPlanarAPI") >= 0:
                found[3] = True
                planarApi = ForceFieldSchema.PhysxForceFieldPlanarAPI.Get(prim, instance)
                forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(planarApi, instance)
                self.assertEqual(planarApi.CreateNormalAttr().Get(), Gf.Vec3f(0.0, 0.0, 1.0))
                self.assertEqual(planarApi.CreateConstantAttr().Get(), 1.0)
                self.assertEqual(planarApi.CreateLinearAttr().Get(), 2.0)
                self.assertEqual(planarApi.CreateInverseSquareAttr().Get(), 3.0)
                self.assertEqual(forceFieldApi.CreateEnabledAttr().Get(), True)
                self.assertEqual(forceFieldApi.CreatePositionAttr().Get(), Gf.Vec3f(1.0, 2.0, 3.0))
                self.assertEqual(forceFieldApi.CreateRangeAttr().Get(), Gf.Vec2f(10.0, 20.0))                
            elif s.find("PhysxForceFieldSphericalAPI") >= 0:
                found[4] = True
                sphericalApi = ForceFieldSchema.PhysxForceFieldSphericalAPI.Get(prim, instance)
                forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(sphericalApi, instance)
                self.assertEqual(sphericalApi.CreateConstantAttr().Get(), 1.0)
                self.assertEqual(sphericalApi.CreateLinearAttr().Get(), 2.0)
                self.assertEqual(sphericalApi.CreateInverseSquareAttr().Get(), 3.0)
                self.assertEqual(forceFieldApi.CreateEnabledAttr().Get(), True)
                self.assertEqual(forceFieldApi.CreatePositionAttr().Get(), Gf.Vec3f(1.0, 2.0, 3.0))
                self.assertEqual(forceFieldApi.CreateRangeAttr().Get(), Gf.Vec2f(10.0, 20.0))
            elif s.find("PhysxForceFieldConicalAPI") >= 0:
                found[4] = True
                conicalApi = ForceFieldSchema.PhysxForceFieldConicalAPI.Get(prim, instance)
                forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(conicalApi, instance)
                self.assertEqual(conicalApi.CreateConstantAttr().Get(), 1.0)
                self.assertEqual(conicalApi.CreateLinearAttr().Get(), 2.0)
                self.assertEqual(conicalApi.CreateInverseSquareAttr().Get(), 3.0)
                self.assertEqual(conicalApi.CreateDirectionAttr().Get(), Gf.Vec3f(3.0, 2.0, 1.0))
                self.assertEqual(conicalApi.CreateAngleAttr().Get(), 4.0)
                self.assertEqual(conicalApi.CreateLinearFalloffAttr().Get(), 0.5)
                self.assertEqual(conicalApi.CreatePowerFalloffAttr().Get(), 6.0)
                self.assertEqual(forceFieldApi.CreateEnabledAttr().Get(), True)
                self.assertEqual(forceFieldApi.CreatePositionAttr().Get(), Gf.Vec3f(1.0, 2.0, 3.0))
                self.assertEqual(forceFieldApi.CreateRangeAttr().Get(), Gf.Vec2f(10.0, 20.0))
            elif s.find("PhysxForceFieldSpinAPI") >= 0:
                found[5] = True
                spinApi = ForceFieldSchema.PhysxForceFieldSpinAPI.Get(prim, instance)
                forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(spinApi, instance)
                self.assertEqual(spinApi.CreateSpinAxisAttr().Get(), Gf.Vec3f(0.0, 0.0, 1.0))
                self.assertEqual(spinApi.CreateConstantAttr().Get(), 1.0)
                self.assertEqual(spinApi.CreateLinearAttr().Get(), 2.0)
                self.assertEqual(spinApi.CreateInverseSquareAttr().Get(), 3.0)
                self.assertEqual(forceFieldApi.CreateEnabledAttr().Get(), True)
                self.assertEqual(forceFieldApi.CreatePositionAttr().Get(), Gf.Vec3f(1.0, 2.0, 3.0))
                self.assertEqual(forceFieldApi.CreateRangeAttr().Get(), Gf.Vec2f(10.0, 20.0))
            elif s.find("PhysxForceFieldWindAPI") >= 0:
                found[6] = True
                windApi = ForceFieldSchema.PhysxForceFieldWindAPI.Get(prim, instance)
                forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(windApi, instance)
                self.assertEqual(windApi.CreateDragAttr().Get(), 1.0)
                self.assertEqual(windApi.CreateAverageSpeedAttr().Get(), 1.0)
                self.assertEqual(windApi.CreateSpeedVariationAttr().Get(), 2.0)
                self.assertEqual(windApi.CreateSpeedVariationFrequencyAttr().Get(), 3.0)
                self.assertEqual(windApi.CreateAverageDirectionAttr().Get(), Gf.Vec3f(0.0, 0.0, 1.0))
                self.assertEqual(windApi.CreateDirectionVariationAttr().Get(), Gf.Vec3f(0.707, 0.707, 0.0))
                self.assertEqual(windApi.CreateDirectionVariationFrequencyAttr().Get(), Gf.Vec3f(1.0, 2.0, 3.0))
                self.assertEqual(forceFieldApi.CreateEnabledAttr().Get(), True)
                self.assertEqual(forceFieldApi.CreatePositionAttr().Get(), Gf.Vec3f(1.0, 2.0, 3.0))
                self.assertEqual(forceFieldApi.CreateRangeAttr().Get(), Gf.Vec2f(10.0, 20.0))

        for f in found:
            self.assertEqual(f, True)
        
        await self.new_stage()

    async def test_all_force_fields_work(self):

        for j in range(2):
            for i in range(7):        
                stage = await self.new_stage()

                scenePathName = "/World/Scene"
                groundPathName = "/Ground"
                primPathName = "/World/ForceFieldPrim"
                boxPathName = "/Box"
                
                # Physics scene
                up = Gf.Vec3f(0.0)
                up[1] = 1.0
                gravityDirection = -up
                gravityMagnitude = 1000.0

                scene = UsdPhysics.Scene.Define(stage, scenePathName)
                scene.CreateGravityDirectionAttr(gravityDirection)
                scene.CreateGravityMagnitudeAttr(gravityMagnitude)

                # Plane
                physicsUtils.add_ground_plane(stage, groundPathName, "Y", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

                # Create the force field prim
                xformPrim = UsdGeom.Xform.Define(stage, primPathName)
                xformPrim.AddTranslateOp().Set(Gf.Vec3f(0.0, 500.0, 0.0))
                prim = xformPrim.GetPrim()

                # Add the collection 
                collectionAPI = Usd.CollectionAPI.Apply(prim, ForceFieldSchema.Tokens.forceFieldBodies)
                collectionAPI.CreateIncludesRel().AddTarget(stage.GetDefaultPrim().GetPath())

                # Box
                boxSize = Gf.Vec3f(100.0)
                boxPosition = Gf.Vec3f(0.0)
                boxPath = boxPathName
                boxPosition[0] = 0.0
                boxPosition[1] = boxSize[1]
                boxPosition[2] = 0.0
                boxPrim = physicsUtils.add_rigid_box(stage, boxPath, position=boxPosition, size=boxSize)

                secondsToRun = 3.0

                # Reset the box position and orientation.
                boxPathName = "/World/Box"
                boxPrim.GetAttribute("xformOp:translate").Set(boxPosition)
                boxPrim.GetAttribute("xformOp:orient").Set(Gf.Quatf(1.0))

                # rays per square centimeter
                surfaceSampleDensity = 0.00001

                if i == 0:
                    # Make sure the spherical force field pulls the box up.
                    sphericalApi = ForceFieldSchema.PhysxForceFieldSphericalAPI.Apply(prim, "spherical")
                    sphericalApi.CreateConstantAttr(0.0)
                    sphericalApi.CreateLinearAttr(-10000.0)
                    sphericalApi.CreateInverseSquareAttr(0.0)
                    forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(sphericalApi, "spherical")
                    forceFieldApi.CreateEnabledAttr(True)
                    forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
                    forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

                    if j == 0:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(0.0)
                    else:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(surfaceSampleDensity)

                    secondsToRun = 0.5

                elif i == 1:
                    # Make sure the spherical force field pulls the box up.
                    sphericalApi = ForceFieldSchema.PhysxForceFieldSphericalAPI.Apply(prim, "spherical")
                    sphericalApi.CreateConstantAttr(0.0)
                    sphericalApi.CreateLinearAttr(-1000.0)
                    sphericalApi.CreateInverseSquareAttr(0.0)
                    forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(sphericalApi, "spherical")
                    forceFieldApi.CreateEnabledAttr(True)
                    forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
                    forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

                    if j == 0:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(0.0)
                    else:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(surfaceSampleDensity)

                    # Add a drag force field and make sure the box stops moving.
                    dragApi = ForceFieldSchema.PhysxForceFieldDragAPI.Apply(prim, "drag")
                    dragApi.CreateMinimumSpeedAttr(0.0)
                    dragApi.CreateLinearAttr(0.0)
                    dragApi.CreateSquareAttr(10.0)
                    forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(dragApi, "drag")
                    forceFieldApi.CreateEnabledAttr(True)
                    forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
                    forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

                    if j == 0:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(0.0)
                    else:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(surfaceSampleDensity)

                    secondsToRun = 5.0

                elif i == 2:
                    # Make sure the linear force field repels the box.
                    linearApi = ForceFieldSchema.PhysxForceFieldLinearAPI.Apply(prim, "linear")
                    linearApi.CreateDirectionAttr(Gf.Vec3f(0.0, 1.0, 0.0))
                    linearApi.CreateConstantAttr(0.0)
                    linearApi.CreateLinearAttr(-1000.0)
                    linearApi.CreateInverseSquareAttr(0.0)
                    forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(linearApi, "linear")
                    forceFieldApi.CreateEnabledAttr(True)
                    forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
                    forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0)) 

                    if j == 0:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(0.0)
                    else:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(surfaceSampleDensity)

                    # Move the box away from the center
                    boxPrim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(500.0, 50.0, 0.0))

                    secondsToRun = 0.6

                elif i == 3:
                    # Make sure the linear force field repels the box.
                    linearApi = ForceFieldSchema.PhysxForceFieldLinearAPI.Apply(prim, "linear")
                    linearApi.CreateDirectionAttr(Gf.Vec3f(0.0, 1.0, 0.0))
                    linearApi.CreateConstantAttr(0.0)
                    linearApi.CreateLinearAttr(-1000.0)
                    linearApi.CreateInverseSquareAttr(0.0)
                    forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(linearApi, "linear")
                    forceFieldApi.CreateEnabledAttr(True)
                    forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
                    forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0)) 

                    if j == 0:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(0.0)
                    else:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(surfaceSampleDensity)

                    # Move the box away from the center
                    boxPrim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(500.0, 50.0, 0.0))

                    # Add some spin to move the box around the line. 
                    spinApi = ForceFieldSchema.PhysxForceFieldSpinAPI.Apply(prim, "spin")
                    spinApi.CreateSpinAxisAttr(Gf.Vec3f(0.0, 1.0, 0.0))
                    spinApi.CreateConstantAttr(1e5)
                    spinApi.CreateLinearAttr(0.0)
                    spinApi.CreateInverseSquareAttr(0.0)
                    forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(spinApi, "spin")
                    forceFieldApi.CreateEnabledAttr(True)
                    forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
                    forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0)) 

                    if j == 0:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(0.0)
                    else:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(surfaceSampleDensity)

                    # Move the box away from the center
                    boxPrim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(200.0, 50.0, 0.0))

                    secondsToRun = 3.6

                elif i == 4:
                    # Levitate the box off the ground.
                    planarApi = ForceFieldSchema.PhysxForceFieldPlanarAPI.Apply(prim, "planar")
                    planarApi.CreateNormalAttr(Gf.Vec3f(0.0, 1.0, 0.0))
                    planarApi.CreateConstantAttr(0.0)
                    planarApi.CreateLinearAttr(0.0)
                    planarApi.CreateInverseSquareAttr(1e9)
                    forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(planarApi, "planar")
                    forceFieldApi.CreateEnabledAttr(True)
                    forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
                    forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0)) 

                    if j == 0:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(0.0)
                    else:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(surfaceSampleDensity)

                    # Move the box away from the center
                    boxPrim.GetAttribute("xformOp:translate").Set(Gf.Vec3f(0.0, 600.0, 0.0))

                    secondsToRun = 1.0

                elif i == 5:
                    # Shake the box and look for motion.
                    noiseApi = ForceFieldSchema.PhysxForceFieldNoiseAPI.Apply(prim, "noise")
                    noiseApi.CreateDragAttr(1000.0)
                    noiseApi.CreateAmplitudeAttr(Gf.Vec3f(5000.0, 0.0, 5000.0))
                    noiseApi.CreateFrequencyAttr(Gf.Vec3f(0.5, 0.0, 0.5))
                    forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(noiseApi, "noise")
                    forceFieldApi.CreateEnabledAttr(True)
                    forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
                    forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

                    if j == 0:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(0.0)
                    else:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(surfaceSampleDensity)

                    secondsToRun = 3.0

                elif i == 6:
                    windApi = ForceFieldSchema.PhysxForceFieldWindAPI.Apply(prim, "wind")
                    windApi.CreateDragAttr(1000.0)
                    windApi.CreateAverageSpeedAttr(500.0)
                    windApi.CreateSpeedVariationAttr(2.0)
                    windApi.CreateSpeedVariationFrequencyAttr(3.0)
                    windApi.CreateAverageDirectionAttr(Gf.Vec3f(0.0, 0.0, 1.0))
                    windApi.CreateDirectionVariationAttr(Gf.Vec3f(0.707, 0.707, 0.0))
                    windApi.CreateDirectionVariationFrequencyAttr(Gf.Vec3f(2.0, 2.0, 2.0))
                    forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(windApi, "wind")
                    forceFieldApi.CreateEnabledAttr(True)
                    forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
                    forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

                    if j == 0:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(0.0)
                    else:
                        forceFieldApi.CreateSurfaceSampleDensityAttr(surfaceSampleDensity)

                    secondsToRun = 3.0

                # Run the simulation
                self._prepare_for_simulation()

                timeStep = self._get_time_step()
                targetIterationCount = math.ceil(secondsToRun / timeStep)

                currentTime = 0.0
                position = Gf.Vec3f(0.0)
                velocity = Gf.Vec3f(0.0)
                maxPosition = Gf.Vec3f(0.0)

                global gPhysXForceFieldsInterface
                global gPhysXInterface

                for iteration in range(targetIterationCount):
                    gPhysXForceFieldsInterface.update(timeStep, currentTime)
                    gPhysXInterface.update_simulation(timeStep, currentTime)

                    currentTime += timeStep

                    # The force fields extension enables the fabric, which prevents the transform from being
                    # written back to USD. The only way to get the physics state data is through get_rigidbody_transformation.
                    lastPosition = position

                    transform = gPhysXInterface.get_rigidbody_transformation(boxPathName)
                    position = transform["position"]

                    velocity[0] = (position[0] - lastPosition[0]) / timeStep
                    velocity[1] = (position[1] - lastPosition[1]) / timeStep
                    velocity[2] = (position[2] - lastPosition[2]) / timeStep

                    maxPosition[0] = max(maxPosition[0], abs(position[0]))
                    maxPosition[1] = max(maxPosition[1], abs(position[1]))
                    maxPosition[2] = max(maxPosition[2], abs(position[2]))

                #print("position")
                #print(position)
                #print("velocity")
                #print(velocity)
                #print("maxPosition")
                #print(maxPosition)

                if i == 0:
                    self.assertGreater(position[1], 100.0)
                elif i == 1:
                    self.assertLess(abs(velocity[1]), 10.0)
                elif i == 2:
                    self.assertLess(position[0], 100.0)
                elif i == 3:
                    radius = math.sqrt(position[0] * position[0] + position[2] * position[2])
                    self.assertGreater(radius, 100.0)
                elif i == 4:
                    self.assertGreater(position[1], 500.0)
                elif i == 5:
                    radius = math.sqrt(maxPosition[0] * maxPosition[0] + maxPosition[2] * maxPosition[2])
                    self.assertGreater(radius, 100.0)
                elif i == 6:
                    self.assertGreater(position[2], 200.0)

                self._end_simulation()

            await self.new_stage()
                
    async def test_force_field_on_kinematic(self):

        for i in range(2):        
            stage = await self.new_stage()

            scenePathName = "/World/Scene"
            groundPathName = "/Ground"
            primPathName = "/World/ForceFieldPrim"
            boxPathName = "/Box"
            
            # Physics scene
            up = Gf.Vec3f(0.0)
            up[1] = 1.0
            gravityDirection = -up
            gravityMagnitude = 1000.0

            scene = UsdPhysics.Scene.Define(stage, scenePathName)
            scene.CreateGravityDirectionAttr(gravityDirection)
            scene.CreateGravityMagnitudeAttr(gravityMagnitude)

            # Plane
            physicsUtils.add_ground_plane(stage, groundPathName, "Y", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

            # Create the force field prim
            xformPrim = UsdGeom.Xform.Define(stage, primPathName)
            xformPrim.AddTranslateOp().Set(Gf.Vec3f(0.0, 500.0, 0.0))
            prim = xformPrim.GetPrim()

            # Add the collection 
            collectionAPI = Usd.CollectionAPI.Apply(prim, ForceFieldSchema.Tokens.forceFieldBodies)
            collectionAPI.CreateIncludesRel().AddTarget(stage.GetDefaultPrim().GetPath())

            # Box
            boxSize = Gf.Vec3f(100.0)
            boxPosition = Gf.Vec3f(0.0)
            boxPath = boxPathName
            boxPosition[0] = 0.0
            boxPosition[1] = boxSize[1]
            boxPosition[2] = 0.0
            boxPrim = physicsUtils.add_rigid_box(stage, boxPath, position=boxPosition, size=boxSize)

            secondsToRun = 3.0

            # Reset the box position and orientation.
            boxPathName = "/World/Box"
            boxPrim.GetAttribute("xformOp:translate").Set(boxPosition)
            boxPrim.GetAttribute("xformOp:orient").Set(Gf.Quatf(1.0))
            
            bodyAPI = UsdPhysics.RigidBodyAPI(boxPrim)

            if i == 1:
                bodyAPI.CreateKinematicEnabledAttr().Set(True)

            # Make sure the spherical force field pulls the box up.
            sphericalApi = ForceFieldSchema.PhysxForceFieldSphericalAPI.Apply(prim, "spherical")
            sphericalApi.CreateConstantAttr(0.0)
            sphericalApi.CreateLinearAttr(-10000.0)
            sphericalApi.CreateInverseSquareAttr(0.0)
            forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(sphericalApi, "spherical")
            forceFieldApi.CreateEnabledAttr(True)
            forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
            forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

            # Add a drag force field and make sure the box stops moving.
            dragApi = ForceFieldSchema.PhysxForceFieldDragAPI.Apply(prim, "drag")
            dragApi.CreateMinimumSpeedAttr(0.0)
            dragApi.CreateLinearAttr(0.0)
            dragApi.CreateSquareAttr(10.0)
            forceFieldApi = ForceFieldSchema.PhysxForceFieldAPI(dragApi, "drag")
            forceFieldApi.CreateEnabledAttr(True)
            forceFieldApi.CreatePositionAttr(Gf.Vec3f(0.0, 0.0, 0.0))
            forceFieldApi.CreateRangeAttr(Gf.Vec2f(-1.0, -1.0))

            secondsToRun = 0.5

            # Run the simulation
            self._prepare_for_simulation()

            timeStep = self._get_time_step()
            targetIterationCount = math.ceil(secondsToRun / timeStep)

            currentTime = 0.0
            position = Gf.Vec3f(0.0)
            velocity = Gf.Vec3f(0.0)
            maxPosition = Gf.Vec3f(0.0)

            global gPhysXForceFieldsInterface
            global gPhysXInterface

            for iteration in range(targetIterationCount):
                gPhysXForceFieldsInterface.update(timeStep, currentTime)
                gPhysXInterface.update_simulation(timeStep, currentTime)

                currentTime += timeStep

                # The force fields extension enables the fabric, which prevents the transform from being
                # written back to USD. The only way to get the physics state data is through get_rigidbody_transformation.
                lastPosition = position

                transform = gPhysXInterface.get_rigidbody_transformation(boxPathName)
                position = transform["position"]

                velocity[0] = (position[0] - lastPosition[0]) / timeStep
                velocity[1] = (position[1] - lastPosition[1]) / timeStep
                velocity[2] = (position[2] - lastPosition[2]) / timeStep

                maxPosition[0] = max(maxPosition[0], abs(position[0]))
                maxPosition[1] = max(maxPosition[1], abs(position[1]))
                maxPosition[2] = max(maxPosition[2], abs(position[2]))

            #print("position")
            #print(position)

            if i == 0:
                self.assertGreater(position[1], 300.0)
            else:
                self.assertLess(position[1], 120.0)

            self._end_simulation()

        await self.new_stage()

    async def setup_ogn_test(self, sphereName):
        stage = await self.new_stage()

        # Create a sphere to test on.
        defaultPrimPath = str(stage.GetDefaultPrim().GetPath())

        UsdPhysics.Scene.Define(stage, defaultPrimPath + "/physicsScene")

        # setup a rigid body
        radius = 20.0
        position = Gf.Vec3f(0.0, 20.0, 0.0)
        spherePrimPath = defaultPrimPath + "/" + sphereName
        physicsUtils.add_rigid_sphere(stage, spherePrimPath, radius, position)
        
        return spherePrimPath

    async def test_ogn_spherical_force_field(self):
        spherePrimPath = await self.setup_ogn_test("sphere")

        ogController = og.Controller()
        keys = ogController.Keys
        (graph, nodes, _, _) = ogController.edit(
            {
                "graph_path": "/TestGraph"
            },
            {
                keys.CREATE_NODES: [
                    ("sphericalForceField", "omni.physx.forcefields.ForceFieldSpherical"),
                    ("dragForceField", "omni.physx.forcefields.ForceFieldDrag"),
                    ("findPrims", "omni.graph.nodes.FindPrims"),
                    ("tick", "omni.graph.action.OnTick")
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "sphericalForceField.inputs:execution"),
                    ("findPrims.outputs:primPaths", "sphericalForceField.inputs:primPaths"),
                    ("findPrims.outputs:primPaths", "dragForceField.inputs:primPaths")
                ],
                keys.SET_VALUES: [
                    ("findPrims.inputs:namePrefix", "sphere"),
                    ("findPrims.inputs:rootPrimPath", "/World"),
                    ("sphericalForceField.inputs:linear", -4000.0),
                    ("dragForceField.inputs:square", 1.0)
                ]
            }
        )

        omni.timeline.get_timeline_interface().play()

        for i in range(40):
            await omni.kit.app.get_app().next_update_async()
            await ogController.evaluate(graph)

            transform = gPhysXInterface.get_rigidbody_transformation(spherePrimPath)
            position = transform["position"]
            #print(position[1])

        # stop, check for reset and prep for next run
        omni.timeline.get_timeline_interface().stop()

        self.assertGreater(position[1], -40.0)
        
        self._end_simulation()
        await self.new_stage()

    async def test_ogn_planar_force_field(self):
        spherePrimPath = await self.setup_ogn_test("sphere")

        ogController = og.Controller()
        keys = ogController.Keys
        (graph, nodes, _, _) = ogController.edit(
            {
                "graph_path": "/TestGraph"
            },
            {
                keys.CREATE_NODES: [
                    ("planarForceField", "omni.physx.forcefields.ForceFieldPlanar"),
                    ("dragForceField", "omni.physx.forcefields.ForceFieldDrag"),
                    ("findPrims", "omni.graph.nodes.FindPrims"),
                    ("tick", "omni.graph.action.OnTick")
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "planarForceField.inputs:execution"),
                    ("findPrims.outputs:primPaths", "planarForceField.inputs:primPaths"),
                    ("findPrims.outputs:primPaths", "dragForceField.inputs:primPaths")
                ],
                keys.SET_VALUES: [
                    ("findPrims.inputs:namePrefix", "sphere"),
                    ("findPrims.inputs:rootPrimPath", "/World"),
                    ("planarForceField.inputs:inverseSquare", 1.0e8),
                    ("planarForceField.inputs:normal", [0.0, 1.0, 0.0]),
                    ("dragForceField.inputs:square", 1.0)
                ]
            }
        )

        omni.timeline.get_timeline_interface().play()

        for i in range(40):
            await omni.kit.app.get_app().next_update_async()
            await ogController.evaluate(graph)

            transform = gPhysXInterface.get_rigidbody_transformation(spherePrimPath)
            position = transform["position"]
            # print(position[1])

        # stop, check for reset and prep for next run
        omni.timeline.get_timeline_interface().stop()

        self.assertGreater(position[1], 0.0)

        self._end_simulation()
        await self.new_stage()
        
    @unittest.skip("OM-109660")
    async def test_ogn_ring_force_field(self):
        spherePrimPath = await self.setup_ogn_test("sphere")

        ogController = og.Controller()
        keys = ogController.Keys
        (graph, nodes, _, _) = ogController.edit(
            {
                "graph_path": "/TestGraph"
            },
            {
                keys.CREATE_NODES: [
                    ("ringForceField", "omni.physx.forcefields.ForceFieldRing"),
                    ("dragForceField", "omni.physx.forcefields.ForceFieldDrag"),
                    ("findPrims", "omni.graph.nodes.FindPrims"),
                    ("tick", "omni.graph.action.OnTick")
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "ringForceField.inputs:execution"),
                    ("findPrims.outputs:primPaths", "ringForceField.inputs:primPaths"),
                    ("findPrims.outputs:primPaths", "dragForceField.inputs:primPaths")
                ],
                keys.SET_VALUES: [
                    ("findPrims.inputs:namePrefix", "sphere"),
                    ("findPrims.inputs:rootPrimPath", "/World"),
                    ("ringForceField.inputs:linear", -1000.0),
                    ("ringForceField.inputs:spinConstant", -10000.0),
                    ("ringForceField.inputs:position", [200.0, 0.0, 0.0]),
                    ("ringForceField.inputs:normalAxis", [0.0, 1.0, 0.0]),
                    ("ringForceField.inputs:radius", 50.0),
                    ("dragForceField.inputs:square", 0.1)
                ]
            }
        )

        omni.timeline.get_timeline_interface().play()

        for i in range(40):
            await omni.kit.app.get_app().next_update_async()
            await ogController.evaluate(graph)

            transform = gPhysXInterface.get_rigidbody_transformation(spherePrimPath)
            position = transform["position"]
            # print(position[1])

        # stop, check for reset and prep for next run
        omni.timeline.get_timeline_interface().stop()

        self.assertGreater(position[1], -100.0)

        self._end_simulation()
        await self.new_stage()

    @unittest.skip("OM-109660")
    async def test_ogn_linear_force_field(self):
        spherePrimPath = await self.setup_ogn_test("sphere")

        ogController = og.Controller()
        keys = ogController.Keys
        (graph, nodes, _, _) = ogController.edit(
            {
                "graph_path": "/TestGraph"
            },
            {
                keys.CREATE_NODES: [
                    ("linearForceField", "omni.physx.forcefields.ForceFieldLinear"),
                    ("dragForceField", "omni.physx.forcefields.ForceFieldDrag"),
                    ("findPrims", "omni.graph.nodes.FindPrims"),
                    ("tick", "omni.graph.action.OnTick")
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "linearForceField.inputs:execution"),
                    ("findPrims.outputs:primPaths", "linearForceField.inputs:primPaths"),
                    ("findPrims.outputs:primPaths", "dragForceField.inputs:primPaths")
                ],
                keys.SET_VALUES: [
                    ("findPrims.inputs:namePrefix", "sphere"),
                    ("findPrims.inputs:rootPrimPath", "/World"),
                    ("linearForceField.inputs:linear", -10000.0),
                    ("linearForceField.inputs:direction", [1.0, 0.0, 0.0]),
                    ("dragForceField.inputs:square", 1.0)
                ]
            }
        )

        omni.timeline.get_timeline_interface().play()

        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            await ogController.evaluate(graph)

            transform = gPhysXInterface.get_rigidbody_transformation(spherePrimPath)
            position = transform["position"]
            # print(position[1])

        # stop, check for reset and prep for next run
        omni.timeline.get_timeline_interface().stop()

        self.assertGreater(position[1], -20.0)

        self._end_simulation()
        await self.new_stage()

    @unittest.skip("OM-109660")
    async def test_ogn_wind_force_field(self):
        spherePrimPath = await self.setup_ogn_test("sphere")

        ogController = og.Controller()
        keys = ogController.Keys
        (graph, nodes, _, _) = ogController.edit(
            {
                "graph_path": "/TestGraph"
            },
            {
                keys.CREATE_NODES: [
                    ("windForceField", "omni.physx.forcefields.ForceFieldWind"),
                    ("findPrims", "omni.graph.nodes.FindPrims"),
                    ("tick", "omni.graph.action.OnTick")
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "windForceField.inputs:execution"),
                    ("findPrims.outputs:primPaths", "windForceField.inputs:primPaths")
                ],
                keys.SET_VALUES: [
                    ("findPrims.inputs:namePrefix", "sphere"),
                    ("findPrims.inputs:rootPrimPath", "/World"),
                    ("windForceField.inputs:drag", 10.0),
                    ("windForceField.inputs:averageSpeed", 1000.0),
                    ("windForceField.inputs:averageDirection", [1.0, 0.0, 0.0])
                ]
            }
        )

        omni.timeline.get_timeline_interface().play()

        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            await ogController.evaluate(graph)

            transform = gPhysXInterface.get_rigidbody_transformation(spherePrimPath)
            position = transform["position"]
            # print(position[0])

        # stop, check for reset and prep for next run
        omni.timeline.get_timeline_interface().stop()

        self.assertGreater(position[0], 30.0)
        
        self._end_simulation()
        await self.new_stage()

    async def test_ogn_conical_force_field(self):
        spherePrimPath = await self.setup_ogn_test("sphere")

        # Plane
        groundPathName = "/Ground"
        stage = self.get_stage()
        physicsUtils.add_ground_plane(stage, groundPathName, "Y", 750.0, Gf.Vec3f(0.0), Gf.Vec3f(0.5))

        ogController = og.Controller()
        keys = ogController.Keys
        (graph, nodes, _, _) = ogController.edit(
            {
                "graph_path": "/TestGraph"
            },
            {
                keys.CREATE_NODES: [
                    ("conicalForceField", "omni.physx.forcefields.ForceFieldConical"),
                    ("findPrims", "omni.graph.nodes.FindPrims"),
                    ("tick", "omni.graph.action.OnTick")
                ],
                keys.CONNECT: [
                    ("tick.outputs:tick", "conicalForceField.inputs:execution"),
                    ("findPrims.outputs:primPaths", "conicalForceField.inputs:primPaths")
                ],
                keys.SET_VALUES: [
                    ("findPrims.inputs:namePrefix", "sphere"),
                    ("findPrims.inputs:rootPrimPath", "/World"),
                    ("conicalForceField.inputs:angle", 60.0),
                    ("conicalForceField.inputs:constant", 100000.0),
                    ("conicalForceField.inputs:position", [500.0, 500.0, 0.0]),
                    ("conicalForceField.inputs:direction", [0.0, -1.0, 0.0])
                ]
            }
        )

        omni.timeline.get_timeline_interface().play()

        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            await ogController.evaluate(graph)

            transform = gPhysXInterface.get_rigidbody_transformation(spherePrimPath)
            position = transform["position"]
            # print(position[0])

        # stop, check for reset and prep for next run
        omni.timeline.get_timeline_interface().stop()

        self.assertLess(position[0], -100.0)

        self._end_simulation()
        await self.new_stage()