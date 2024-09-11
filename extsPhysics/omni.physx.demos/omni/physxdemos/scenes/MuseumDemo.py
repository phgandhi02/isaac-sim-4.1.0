from omni.physx.scripts import physicsUtils
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics, PhysxSchema, PhysicsSchemaTools, UsdUtils, Sdf, Usd
import omni.physxdemos as demo
from omni.physx import get_physx_simulation_interface, get_physx_scene_query_interface
import carb.input
import omni.appwindow
import omni.timeline
import carb.windowing # todo: not needed?
from omni import ui
from carb.input import MouseEventType, EVENT_TYPE_ALL
import omni.physx.scripts.utils as core_utils
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.kit.viewport.utility as vp_utils
from omni.debugdraw import get_debug_draw_interface
from omni.physxui import get_physicsui_instance
from omni.physx import get_physx_interface
from .KeyboardHelpers import *
import random
import math

import cProfile
import pstats
import io
from pstats import SortKey
import traceback

class EmptyDemo(): # demo.Base
    title = ""
    category = demo.Categories.BASICS
    short_description = ""
    description = ""

    def create(self, stage):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage, hasTable = False)

class DemoItem:
    def __init__(self, demoInstance, index, demoName, rootPos, params, roomTemplateStr):
        self.demoInstance = demoInstance
        self.index = index
        self.demoName = demoName
        self.rootPos = rootPos
        self.params = params
        self.roomTemplateStr = roomTemplateStr
        self.isVisible = False

class SortedItem:
    def __init__(self, index, indX, indY, distance, rootPos):
        self.index = index
        self.indX = indX
        self.indY = indY
        self.distance = distance,
        self.rootPos = rootPos

def get_cct_api():
    try:
        import omni.physxcct.scripts.utils as cct_utils
        return cct_utils
    except ModuleNotFoundError:
        pass
    return None

class MuseumDemo(demo.AsyncDemoBase):
    title = "A Demo Museum"
    category = demo.Categories.BASICS
    short_description = "A collection of demos that can be simulated independently"
    description = "Controls -- Spacebar: start/stop the museum demo; F: play/pause the current museum exhibit; R: reset the current museum exhibit; A,S,D,W: movement; E: jump"

    def startProf(self):
        self.ob.enable()

    def stopProf(self):
        self.ob.disable()
        sec = io.StringIO()
        sortby = SortKey.CUMULATIVE
        ps = pstats.Stats(self.ob, stream=sec).sort_stats(sortby)
        ps.print_stats()
        print("seconds:", sec.getvalue())

    def __init__(self):
        super().__init__(enable_tensor_api=False, enable_fabric=False)

    def create(self, stage, mouse_sensitivity, gamepad_sensitivity):
        self._cct_utils = get_cct_api()
        stage.SetTimeCodesPerSecond(60.0)

        self._firstTime = True

        if self._cct_utils:
            self._disableCCT = False
        else :
            self._disableCCT = True

        self._onlyNearestLight = True
        self._maxLoadingDemos = 15 # maximum number of demos to load in one frame
        self._minLoadingDemos = 0 # minimum number of demos to load in one frame
        self._maxLoadedDemos = 30 # maximum number of demos to have loaded at a given time
        self._maxDisplayedDemos = 25 # maximum number of demos to have displayed at a given time

        self._demoSize = 1200.0
        # self._curThread = None
        self._sortedArr = None
        self._lastCrosshairBoxPos = Gf.Vec3d(-99999.0)
        self._lastCrosshairBoxPos2 = Gf.Vec3d(-99999.0)
        self._infoBox = None
        self._currentText = []
        self._dirtyText = True
        self._lastSortedIndex = -1
        self._wasShutdown = False
        self._isSimulating = False
        self._targetLightPos = Gf.Vec3f(0.0)
        self._doReset = False
        self._hasSetCamTarget = False
        self._museumScale = 1

        settings = carb.settings.get_settings()
        settings.set("/app/player/playSimulations", False)

        self.ob = cProfile.Profile()

        self._stage = stage
        self._stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self._defaultPrimPath, scene = demo.setup_physics_scene(self, stage, primPathAsString = False)
        self._mainScenePathAsInt = PhysicsSchemaTools.sdfPathToInt(self._defaultPrimPath.AppendPath("physicsScene"))

        self._sub_keyboard = init_keyboard(self._on_keyboard_event)

        boxSize = 20.0
        boxPos = Gf.Vec3f(0.0)
        self._roomInstancer = demo.get_room_instancer(stage)
        self._isMuseum = True

        # not strictly needed, but keeps scene ordering cleaner to create this scope in advance
        material_scope_path = self._defaultPrimPath.AppendPath("Looks")
        UsdGeom.Scope.Define(stage, material_scope_path)

        self.demo_camera = self._defaultPrimPath.AppendPath("Camera")
        self._cam = UsdGeom.Camera.Define(stage, self.demo_camera)
        self._cam.CreateFocalLengthAttr().Set(6.0)
        imageable = UsdGeom.Imageable.Get(self._stage, self.demo_camera)
        imageable.GetVisibilityAttr().Set("invisible")

        self.generateDictionary()
        self.assignGrid()
        self.assignLights()
        self.loadGrid(stage)
        self.initCharacterController(stage, mouse_sensitivity, gamepad_sensitivity)

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input == carb.input.KeyboardInput.F:
                self._isSimulating = not self._isSimulating
                self._dirtyText = True
                return False
            if event.input == carb.input.KeyboardInput.R:
                self._doReset = True
                self._dirtyText = True
                return False
        return True

    def assignLights(self):
        self._scaleFactor = 1.0 / (UsdGeom.GetStageMetersPerUnit(self._stage) * 100.0)
        RoomHelper = demo.get_room_helper_class()
        targetLightPos = Gf.Vec3f(-35.0 * self._museumScale, -50.0 * self._museumScale, 800.0)
        self._sphereLight, self._currentLightPos = RoomHelper.get_lights(self._stage, self._defaultPrimPath, self, self._isMuseum, self._scaleFactor, True, targetLightPos)

    def setSceneTargets(self, prim):
        if not prim:
            return

        didAssign = False
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigidBodyAPI = UsdPhysics.RigidBodyAPI.Get(self._stage, prim.GetPrimPath())
            rigidBodyAPI.GetSimulationOwnerRel().AddTarget(self._curScenePath)
            didAssign = True
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collisionAPI = UsdPhysics.CollisionAPI.Get(self._stage, prim.GetPrimPath())
            collisionAPI.GetSimulationOwnerRel().AddTarget(self._curScenePath)
            didAssign = True

        if didAssign:
            return # only need to apply rigid body / collision to root note

        children = prim.GetChildren()
        for i in children:
            self.setSceneTargets(i)

    def generateCategorizedDemoList(self):
        self._categoryDictionary = {}
        for sceneClass in demo.Manager.instance._scenes.values():
            demoName = sceneClass.__module__.rsplit('.', 1)[1]
            demoCategory = sceneClass.category
            if demoCategory not in self._categoryDictionary:
                self._categoryDictionary[demoCategory] = []
            self._categoryDictionary[demoCategory].append(demoName)

        for key in self._categoryDictionary:
            val = self._categoryDictionary[key]
            print("######## " + key + " ########")
            print("[")
            for i in val:
                print("\"" + i + "\",")
            print("],")

    def generateDictionary(self):
        self._demoDictionary = {"EmptyDemo": EmptyDemo}

        for sceneClass in demo.Manager.instance._scenes.values(): 
            if "internalScenes" not in sceneClass.__module__:
                demoName = sceneClass.__module__.rsplit('.', 1)[1]
                self._demoDictionary[demoName] = sceneClass

    def makeItem(self, demoName, params = {}, appendToBaseGrid = True):
        obj = {
            "demoName": demoName,
            "params": params,
            "roomTemplateStr": "default"
        }
        if appendToBaseGrid:
            self._gridBaseFlat.append(obj)

        return obj

    def createRandomGrid(self, sizeX = 20, sizeY = 20):
        self._maxLoadingDemos = 15 # maximum number of demos to load in one frame
        self._minLoadingDemos = 0 # minimum number of demos to load in one frame
        self._maxLoadedDemos = 30 # maximum number of demos to have loaded at a given time
        self._maxDisplayedDemos = 25 # maximum number of demos to have displayed at a given time
        self._totalRooms = sizeX * sizeY
        for j in range(sizeY):
            self._grid.append([])
            for i in range(sizeX):
                if (j == 0) and (i == 0):
                    self._grid[j].append(self._gridBase[0][0])
                else:
                    item = random.choice(self._gridBaseFlat)
                    self._grid[j].append(item)

    def createDemoGrid(self):
        counter = 0

        sizeX = math.ceil(len(self._gridBaseFlat)/2)
        sizeY = 3

        self._museumScale = demo.get_room_helper_class().museumScale
        self._maxLoadingDemos = 100 # maximum number of demos to load in one frame
        self._minLoadingDemos = 0 # minimum number of demos to load in one frame
        self._maxLoadedDemos = 100 # maximum number of demos to have loaded at a given time
        self._maxDisplayedDemos = 100 # maximum number of demos to have displayed at a given time
        self._demoSize = 660*self._museumScale
        self._totalRooms = sizeX * sizeY

        

        for j in range(sizeY):
            self._grid.append([])
            for i in range(sizeX):

                item = None
                if (j == 1): # hallway
                    item = self._emptyDemo
                else:
                    if counter == len(self._gridBaseFlat):
                        continue
                    item = self._gridBaseFlat[counter]
                    counter += 1

                # wall and pillar configuration:
                # + -------- x -
                #  1   N   0   y
                #      0       |
                #  W 3   2 E   |
                #      1       |
                #  3   S   2   +

                isEastEnd = (i == (sizeX - 1))
                isWestEnd = (i == 0)

                if isEastEnd:
                    postfix = "east"
                elif isWestEnd:
                    postfix = "west"
                else:
                    postfix = "center"

                if (j == 2): # north rooms
                    template = "museum_room_north_" + postfix
                elif (j == 1): # hallway
                    template = "museum_hallway_" + postfix
                elif (j == 0): # south rooms
                    template = "museum_room_south_" + postfix

                if item:
                    obj = { # deep copy of object
                        "demoName": item["demoName"],
                        "params": item["params"],
                        "roomTemplateStr": template
                    }
                    self._grid[j].append(obj)

                

    def assignGrid(self):
        self._grid = []
        self._gridBaseFlat = []

        self._emptyDemo = self.makeItem("EmptyDemo", appendToBaseGrid = False)

        self._gridBase = [

            ######## Basics ########
            [
            self.makeItem("BoxOnPlaneDemo"),
            self.makeItem("MultiShapeBodyOnPlaneDemo"),
            # self.makeItem("MuseumDemo"),
            self.makeItem("VariousShapesOnTrimeshDemo"),
            ],

            ######## Rigid Bodies ########
            [
            self.makeItem("BoxOnPlaneInstancedDemo"),
            self.makeItem("CenterOfMassDemo"),
            self.makeItem("ForceDemo", {"Enable_Flow_Flame": False, "manual_animation":True}), # works but flow flame does not appear currently
            # self.makeItem("KaplaArenaDemo"), # works but relatively slow to load
            # self.makeItem("KaplaTowerDemo"), # works but relatively slow to load
            self.makeItem("KinematicBodyDemo", {"manual_animation": True}),
            # self.makeItem("RigidBodyCCDDemo", {"manual_animation": True}), # does not currently animate (?), relatively slow to load
            ],

            # ######## Materials ########
            [
            self.makeItem("MaterialDensityDemo"),
            self.makeItem("MaterialFrictionDemo"),
            self.makeItem("MaterialRestitutionDemo", {"default_restitution": 0.5}),
            # self.makeItem("TriangleMeshMultiMaterialDemo"), # works on first load, callback fails on contiguous loads - this is a known bug that needs to be fixed
            ],

            # ######## Simulation Partitioning ########
            [
            # self.makeItem("CollisionGroupsDemo"), # slightly slow to load
            self.makeItem("FilteringPairDemo"),
            # self.makeItem("MultipleScenesDemo"), # slightly slow to load
            ],

            # ######## Contacts ########
            [
            self.makeItem("CompliantContactsDemo"),
            self.makeItem("ContactReportDemo", {"FrictionAnchors": True}),
            # self.makeItem("ContactReportImmediateAPIDemo"), callback does not report contacts for some reason
            # self.makeItem("ContactSlopCoefficientDemo"), # does not use demo room currently
            self.makeItem("ConveyorBeltDemo"),
            self.makeItem("CustomConeDemo"),
            self.makeItem("CustomCylinderDemo"),
            ],

            # ######## Triggers ########
            [
            self.makeItem("TriggerDemo"),
            self.makeItem("TriggerStateAPIDemo"),
            self.makeItem("TriggerConveyorDemo"),
            self.makeItem("TriggerShapeSortingDemo"),
            ],

            # ######## Scene Query ########
            [
            # self.makeItem("ConvexMeshDataDemo"), # does not use z as up axis
            # self.makeItem("OverlapAnyDemo"), # does not currently use demo room
            self.makeItem("OverlapMeshDemo"),
            # self.makeItem("OverlapMultipleDemo", {"manual_animation": True}), # works but slow to load
            self.makeItem("OverlapShapeDemo"),
            self.makeItem("RaycastsDemo", {"manual_animation": True}), # works but slow to load
            # self.makeItem("SweepsDemo", {"manual_animation": True}), # works but slow to load
            ],

            # ######## Joints ########
            [
            self.makeItem("D6JointDemo",{"Trans_X":True,"Trans_Y":True,"Trans_Z":True,"Rot_X":False,"Rot_Y":False,"Rot_Z":True}),
            self.makeItem("DistanceJointDemo"),
            self.makeItem("GearJointDemo"),
            self.makeItem("JointBreakDemo"),
            self.makeItem("JointStateReadDemo", {"D6Joint":False, "Prismatic":False}),
            self.makeItem("PrismaticJointDemo"),
            self.makeItem("RackAndPinionJointDemo"),
            self.makeItem("RevoluteJointDemo"),
            # self.makeItem("RigidBodyRopeDemo"), # works but somewhat slow to load
            self.makeItem("SphericalJointDemo"),
            ],

            ######## Articulations ########
            # [
            # self.makeItem("ArticulationDemo"), # wrong unit scale
            # self.makeItem("FixedTendonDemo", {"Use_Limits":False, "Disable_Motion":False}), # does not use z as up axis
            # self.makeItem("SpatialTendonActuationDemo", {"Enable_Motion":True}), # wrong unit scale
            # self.makeItem("SpatialTendonDemo", {"Parents":True}), # does not use z as up axis
            # ],

            # # ######## Deformables ########
            # [
            # self.makeItem("DeformableBodyAttachmentsDemo"),
            # self.makeItem("DeformableBodySimpleDemo"),
            # self.makeItem("TeddyOnIceDemo"), # does not use z as up axis
            # ],

            ######## Particles ########
            # [
            # self.makeItem("ClothDeckChairDemo"), # does not use z as up axis
            # self.makeItem("FluidBallEmitterDemo", {"Single_Particle_Set":True,"Use_Instancer":True}), # wrong unit scale
            # self.makeItem("FluidIsosurfaceGlassBoxDemo"), # invalid offset on liquid, should be fixable
            # self.makeItem("ParticleClothDemo"), # does not use z as up axis
            # self.makeItem("ParticleInflatableDemo"), # does not use z as up axis
            # self.makeItem("ParticlePostProcessingDemo", {"Anisotropy": False, "Smoothing": False, "Isosurface": False}), # does not use z as up axis
            # self.makeItem("ParticleSamplerDemo", {"Sample_Volume":True,"Particle_Contact_Offset":1.0}), # does not use z as up axis
            # ],

            ######## Complex Showcases ########
            [
            # self.makeItem("AnalogDigitalClockDemo"), # does not use z as up axis
            # self.makeItem("ChairStackingDemo"),
            # self.makeItem("EggSmashDemo"),
            # self.makeItem("FrankaDeformableDemo"),
            # self.makeItem("HoneyDemo"),
            # self.makeItem("LegoTechnicBuggyRigid"),
            # self.makeItem("MixerDemo"),
            # self.makeItem("NutsAndBoltsDemo"),
            # self.makeItem("ImmediateComplexDepenetrationDemo"),
            ],

        ]

        # self._grid = [[self.makeItem("BoxOnPlaneDemo")]]
        if self._isMuseum:
            self.grid = self.createDemoGrid()
        else:
            self.grid = self.createRandomGrid()



    def loadGrid(self, stage):

        self._demoArray = []
        self._sortedArr = []
        i = 0
        for indY, gridRow in enumerate(self._grid):
            self._demoArray.append([])
            for indX, gridItem in enumerate(gridRow):
                rootPos = Gf.Vec3d(indX * -self._demoSize, indY * -self._demoSize, 0.0)
                self._demoArray[-1].append(DemoItem(
                    None,
                    i,
                    gridItem["demoName"],
                    rootPos,
                    gridItem["params"],
                    gridItem["roomTemplateStr"]
                ))
                self._sortedArr.append(SortedItem(i,indX,indY,0,rootPos))
                i += 1

    def unloadDemo(self, demoItem):
        demoPath = self._defaultPrimPath.AppendPath("demos/" + demoItem.demoName + str(demoItem.index))
        # print("unload", demoPath)
        demoPrim = self._stage.GetPrimAtPath(demoPath)
        if getattr(demoItem.demoInstance, "on_shutdown", None):
            demoItem.demoInstance.on_shutdown()
        self._stage.RemovePrim(demoPath)
        demoItem.demoInstance = None

    def loadDemo(self, demoItem : DemoItem):
        stage = self._stage

        demoInstance = self._demoDictionary[demoItem.demoName]()
        demoInstance.demoParams = {
            "rootPath": "demos/" + demoItem.demoName + str(demoItem.index),
            "rootPosition": demoItem.rootPos,
            "roomInstancer": self._roomInstancer,
            "groundPlaneSize": self._demoSize,
            "roomTemplateStr": demoItem.roomTemplateStr,
            "isMuseum": self._isMuseum
        }
        if getattr(demoInstance, "on_startup", None):
            demoInstance.on_startup()

        demoInstance.create(stage, **demoItem.params)

        # set physics scene targets
        demoRootPrim = stage.GetPrimAtPath(demoInstance._room._defaultPrimPath)
        demoInstance.scenePath = demoInstance._room._defaultPrimPath.AppendPath("physicsScene")
        demoInstance.scenePathAsInt = PhysicsSchemaTools.sdfPathToInt(demoInstance.scenePath)
        self._curScenePath = demoInstance.scenePath
        self.setSceneTargets(demoRootPrim)
        
        # only add the static room scenes to the main physics scene so that the
        # cct can collide with them and they can also simulate independently
        self._curScenePath = self._defaultPrimPath.AppendPath("physicsScene")
        roomRootPrim = stage.GetPrimAtPath(demoInstance._room._defaultPrimPath.AppendPath("roomScene"))
        self.setSceneTargets(roomRootPrim)

        demoInstance.active = False
        demoInstance.runningSteps = 0.0

        if not self._hasSetCamTarget:
            self._hasSetCamTarget = True
            demoInstance._room.set_camera_target(self.demo_camera, self._cam, 0.25)

        return demoInstance

    def draw_info_text(self):
        sortedItem = self._sortedArr[0]
        curItem = self._demoArray[sortedItem.indY][sortedItem.indX]
        if not curItem.demoInstance:
            return

        if self._dirtyText or curItem.demoInstance._room._dirtyText:
            self._dirtyText = False
            is_playing = omni.timeline.get_timeline_interface().is_playing()

            if is_playing:
                self._currentText = [
                    curItem.demoInstance.title + " (playing)" if self._isSimulating else curItem.demoInstance.title + " (paused)",
                    curItem.demoInstance.short_description,
                    "Press 'F' to " + ("pause" if self._isSimulating else "play") + " the current exhibit, 'R' to reset it"
                ]
                for i in curItem.demoInstance._room._text_array:
                    self._currentText.append(i)
            else:
                self._currentText = []
            
            if not self._infoBox:
                physicsui = get_physicsui_instance()
                if physicsui:
                    self._infoBox = physicsui.create_text_info_box(self._currentText, screen_coords=[0, -200])
            else:
                self._infoBox.set_text(self._currentText)

            if (len(self._currentText) == 0) or (not curItem.demoInstance.title):
                self._infoBox.visible = False
            else:
                self._infoBox.visible = True

    def getDistance2D(self, vec1, vec2):
        a = Gf.Vec2d(vec1[0], vec1[1])
        b = Gf.Vec2d(vec2[0], vec2[1])
        return (a - b).GetLength()

    def update(self, stage, dt, viewport, physxIFace):
        cameraPrim = stage.GetPrimAtPath(vp_utils.get_active_viewport_camera_path())
        cameraTransform = core_utils.CameraTransformHelper(cameraPrim)
        self._cameraPos = cameraTransform.get_pos()
        self._cameraForward = cameraTransform.get_forward()
        self._crosshairBoxPos = self._cameraPos + self._cameraForward*800.0

        


        distance2 = self.getDistance2D(self._crosshairBoxPos, self._lastCrosshairBoxPos2)
        if distance2 > (self._demoSize * 0.25):
            self._lastCrosshairBoxPos2[0] = self._crosshairBoxPos[0]
            self._lastCrosshairBoxPos2[1] = self._crosshairBoxPos[1]
            self.sortDistance()

        distance = self.getDistance2D(self._crosshairBoxPos, self._lastCrosshairBoxPos)
        if distance > (self._demoSize):
            self._lastCrosshairBoxPos[0] = self._crosshairBoxPos[0]
            self._lastCrosshairBoxPos[1] = self._crosshairBoxPos[1]
            self.loadNearest()

        self.updateTriggers(stage, dt, viewport, physxIFace)
        self.updateCCT(stage)

        self.draw_info_text()


    def updateTargetLightPos(self):
        sortedItem = self._sortedArr[0]
        curItem = self._demoArray[sortedItem.indY][sortedItem.indX]
        if curItem.demoInstance:
            self._targetLightPos = Gf.Vec3f(curItem.demoInstance._room._originWorld) + curItem.demoInstance._room._targetLightPos

    def sortDistance(self):
        for sortedItem in self._sortedArr:
            sortedItem.distance = self.getDistance2D(self._crosshairBoxPos,sortedItem.rootPos)

        self._sortedArr.sort(key=lambda x: x.distance, reverse=False)

        if (self._sortedArr[0].index != self._lastSortedIndex):
            self._lastSortedIndex = self._sortedArr[0].index
            self._dirtyText = True

        self.updateTargetLightPos()

    def loadNearest(self):

        loadedCount = 0
        for i, sortedItem in enumerate(self._sortedArr):
            curItem = self._demoArray[sortedItem.indY][sortedItem.indX]

            if i < self._maxLoadedDemos:
                if not curItem.demoInstance:
                    if loadedCount < self._maxLoadingDemos or loadedCount < self._minLoadingDemos:
                        perc = str(int(loadedCount * 100.0 / self._totalRooms))
                        loadedCount += 1
                        self._dirtyText = True
                        print("["+perc+"%] loading: " + curItem.demoName + " (index: " + str(curItem.index) + ")")
                        # self.startProf()
                        curItem.demoInstance = self.loadDemo(curItem)
                        # self.stopProf()
            
            if curItem.demoInstance:
                imageable = UsdGeom.Imageable.Get(self._stage, curItem.demoInstance._room._defaultPrimPath)
                imageable.GetVisibilityAttr().Set("inherited" if (i < self._maxDisplayedDemos) else "invisible")
                curItem.isVisible = (i <= self._maxDisplayedDemos)

                # if i > self._maxLoadedDemos:
                #     self.unloadDemo(curItem)

                # print(i, curItem.isVisible)

        # print("done loading")

        self.updateTargetLightPos()




    ###### triggers ######

    def updateTriggers(self, stage, dt, viewport, physxIFace):
        is_playing = omni.timeline.get_timeline_interface().is_playing()
        if not is_playing:
            return
        
        box_color = 0xffffff00
        closest_color = 0xff00ff00
        active_color = 0xff0000ff
        interface = get_physx_simulation_interface()

        self._currentLightPos += (self._targetLightPos - self._currentLightPos) * 0.03
        self._sphereLight.GetPrim().GetAttribute("xformOp:translate").Set(self._currentLightPos)

        closestItem = self._sortedArr[0]
        for row in self._demoArray:
            for demoItem in row:
                demoInstance = demoItem.demoInstance
                if not demoInstance:
                    continue

                camDistance = self.getDistance2D(demoInstance._room._originWorld, self._crosshairBoxPos)
                lastActive = demoInstance.active

                isClosest = (closestItem.index == demoItem.index)
                demoInstance.active = isClosest
                cur_color = closest_color if isClosest else box_color
                if demoInstance.active:
                    cur_color = active_color

                # if demoItem.isVisible:
                #     get_debug_draw_interface().draw_box(
                #         demoInstance._room._originWorld,
                #         carb.Float4(0.0, 0.0, 0.0, 1.0),
                #         Gf.Vec3f(demoInstance.triggerBoxSize),
                #         cur_color,
                #         3.0
                #     )

                if demoInstance.active and self._doReset:
                    self._isSimulating = False
                    self.unloadDemo(demoItem)
                    demoItem.demoInstance = self.loadDemo(demoItem)

                if demoInstance.active and self._isSimulating:
                    deltaTime = 1.0/60.0
                    if getattr(demoInstance, "update", None):
                        demoInstance.update(self._stage, deltaTime, viewport, physxIFace)

                    demoInstance.runningSteps += 1.0
                    demo.animate_attributes(demoInstance, self._stage, demoInstance.runningSteps)
                    interface.simulate_scene(demoInstance.scenePathAsInt, deltaTime, 0.0)
                    interface.fetch_results_scene(demoInstance.scenePathAsInt)

                    if getattr(demoInstance, "scenePathsInt", None):
                        for scenePathInt in demoInstance.scenePathsInt:
                            interface.simulate_scene(scenePathInt, deltaTime, 0.0)
                            interface.fetch_results_scene(scenePathInt)

                if (lastActive != demoInstance.active):
                    if not demoInstance.active:
                        self._isSimulating = False
                        if getattr(demoInstance, "on_stop", None):
                            demoInstance.on_stop()

        self._doReset = False

    ###### character controller ######

    params = {
        "mouse_sensitivity": demo.IntParam(25, 0, 100, 1),
        "gamepad_sensitivity": demo.IntParam(25, 0, 100, 1),
    }

    def reset(self):
        self.shoot = False
        self.captured = False

    def initCharacterController(self, stage, mouse_sensitivity, gamepad_sensitivity):
        self.reset()
        self.hit_boxes = []

        if self._disableCCT:
            return

        self.cct_path = str(self._defaultPrimPath.AppendPath("capsuleActor"))
        self._cct_utils.spawn_capsule(stage, self.cct_path, Gf.Vec3f(300, -900, 55), 100, 50)
        imageable = UsdGeom.Imageable.Get(self._stage, self.cct_path)
        imageable.GetVisibilityAttr().Set("invisible")

        try: # even if import is successful, must check this if module is disabled at runtime
            camera_path = "/OmniverseKit_Persp"
            fpCam = UsdGeom.Camera.Get(stage, camera_path)
            if not fpCam:
                fpCam = UsdGeom.Camera.Define(stage, "/OmniverseKit_Persp")
            fpCam.CreateFocalLengthAttr().Set(6.0)
            self._cct = self._cct_utils.CharacterController(self.cct_path, camera_path, True, 0.01)
            self._cct.activate(stage)
        except Exception as e:
            self._cct_utils = None
            self._disableCCT = True
            return  

        usdPrim = stage.GetPrimAtPath(self.cct_path)
        rbAPI = UsdPhysics.RigidBodyAPI.Apply(usdPrim)
        colliderAPI = UsdPhysics.CollisionAPI.Apply(usdPrim)

        self.appwindow = omni.appwindow.get_default_app_window()
        self._mouse = self.appwindow.get_mouse()
        input_iface = carb.input.acquire_input_interface()
        self._mouse_sub = input_iface.subscribe_to_input_events(self._mouse_info_event_cb, EVENT_TYPE_ALL, self._mouse, 0)
        self._viewport_overlay_frame = None

        self._cct.setup_controls(2000, self._cct_utils.ControlFlag.DEFAULT)
        self._cct.control_state.jump_speed = 500
        self._cct.control_state.mouse_sensitivity = mouse_sensitivity
        self._cct.control_state.gamepad_sensitivity = gamepad_sensitivity * 10

        stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)

    def _cleanup_overlay(self):
        if self._viewport_overlay_frame:
            self._viewport_overlay_frame.visible = False
            self._viewport_overlay_frame = None
        self._viewport_window = None

    def refresh_crosshair(self, is_playing):
        if is_playing:
            if not self._viewport_overlay_frame:
                self._viewport_window = vp_utils.get_active_viewport_window()
                self._viewport_overlay_frame = self._viewport_window.get_frame("omni.physx.cct.crosshair_frame")
                self._viewport_overlay_frame.visible = True
                with self._viewport_overlay_frame:
                    with ui.Placer(offset_x=ui.Percent(50), offset_y=ui.Percent(50)):
                        self._crosshair = ui.Circle(width=10, height=10, radius=10, style={"background_color": 4283782485})
        elif self._viewport_overlay_frame:
            self._cleanup_overlay()

    def updateCCT(self, stage):
        if self._disableCCT:
            return

        is_playing = omni.timeline.get_timeline_interface().is_playing()
        self.refresh_crosshair(is_playing)

        if not is_playing:
            return
            
        interface = get_physx_simulation_interface()

        if self._firstTime: # this address a bug with first person camera movement, must simulate at least once before doing anything else here
            self._firstTime = False
            interface.simulate_scene(self._mainScenePathAsInt, 1.0/60.0, 0.0)
            interface.fetch_results_scene(self._mainScenePathAsInt)

        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)

        camera_prim = stage.GetPrimAtPath(vp_utils.get_active_viewport_camera_path())
        camera_th = core_utils.CameraTransformHelper(camera_prim)
        cameraPos = camera_th.get_pos()
        cameraForward = camera_th.get_forward()

        origin = carb.Float3(cameraPos[0], cameraPos[1], cameraPos[2])
        rayDir = carb.Float3(cameraForward[0], cameraForward[1], cameraForward[2])
        
        hitInfo = get_physx_scene_query_interface().raycast_closest(origin, rayDir, 10000.0)
        if hitInfo["hit"]:
            usdGeom = UsdGeom.Mesh.Get(stage, hitInfo["rigidBody"])
            usdPrim = usdGeom.GetPrim()
            if usdPrim.HasAPI(UsdPhysics.RigidBodyAPI):
                self.hit_boxes.append(usdGeom)
                if self.shoot:
                    impulseSize = 200.0 / (metersPerUnit * kilogramsPerUnit)
                    impulse = carb.Float3(
                        cameraForward[0] * impulseSize,
                        cameraForward[1] * impulseSize,
                        cameraForward[2] * impulseSize
                    )                    
                    rbo_encoded = PhysicsSchemaTools.sdfPathToInt(hitInfo["rigidBody"])
                    interface.apply_force_at_pos(self._stage_id, rbo_encoded, impulse, hitInfo["position"], "Impulse")
        self.shoot = False

        interface.simulate_scene(self._mainScenePathAsInt, 1.0/60.0, 0.0)
        interface.fetch_results_scene(self._mainScenePathAsInt)

        

    def set_colors(self, color, reset_array):
        for usdGeom in self.hit_boxes:
            usdGeom.GetDisplayColorAttr().Set(color)

        if reset_array:
            self.hit_boxes = []
        return

    def _mouse_info_event_cb(self, event_in, *args, **kwargs):
        if self.captured:
            event = event_in.event
            if event.type == MouseEventType.LEFT_BUTTON_DOWN:
                self.shoot = True
        return not self.captured

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            self.captured = True
        elif e.type == int(omni.timeline.TimelineEventType.STOP):
            self.reset()
        elif e.type == int(omni.timeline.TimelineEventType.PAUSE):
            self.captured = False

    def __del__(self):
        if not self._wasShutdown:
            self.on_shutdown()

    def on_shutdown(self):
        self._wasShutdown = True

        for row in self._demoArray:
            for demoItem in row:
                if getattr(demoItem.demoInstance, "on_shutdown", None):
                    demoItem.demoInstance.on_shutdown()

        super().on_shutdown()
        close_keyboard(self._sub_keyboard)
        self._sub_keyboard = None
        self._timeline_subscription = None
        settings = carb.settings.get_settings()
        settings.set("/app/player/playSimulations", True)
        # self._stage_event_sub = None
        if self._infoBox:
            self._infoBox.destroy()
            self._infoBox = None

        if self._disableCCT:
            return

        carb.input.acquire_input_interface().unsubscribe_to_input_events(self._mouse_sub)
        self._mouse_sub = None
        self._cleanup_overlay()
        self._cct.disable()
        self._cct.shutdown()
        self._cct = None