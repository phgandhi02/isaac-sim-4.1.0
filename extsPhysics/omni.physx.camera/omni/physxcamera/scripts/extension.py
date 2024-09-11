import omni.ext
import omni.physx
import omni.physxvehicle
from omni.physxcamera.bindings import _physxCamera
import omni.physxdemos as demo
from omni.physxui import PhysicsMenu
from .propertyWidgetManager import PropertyWidgetManager
from omni.physx.scripts.utils import safe_import_tests

RUNNING_WTESTS = safe_import_tests("omni.physxcamera.scripts", ["tests"])
DEMO_SCENES = "omni.physxcamera.scripts.samples"


class PhysxCameraExtension(omni.ext.IExt):
    def __init__(self):
        super().__init__()

    def on_startup(self):
        self._physxCameraInterface = _physxCamera.acquire_physx_camera_interface()        
        self._physxCameraInterface = omni.physxcamera.get_physx_camera_interface()
        self._cameraPropertyWidgetManager = PropertyWidgetManager(self._physxCameraInterface)
        self._cameraPropertyWidgetManager.set_up()
        self._physxVehicleInterface = omni.physxvehicle.get_physx_vehicle_interface()
        self._physxInterface = omni.physx.get_physx_interface()
        self._physxSimInterface = omni.physx.get_physx_simulation_interface()

        if RUNNING_WTESTS:
            omni.physxcamera.scripts.tests.setPhysxCameraInterface(self._physxCameraInterface)
            omni.physxcamera.scripts.tests.setPhysxVehicleInterface(self._physxVehicleInterface)
            omni.physxcamera.scripts.tests.setPhysxInterface(self._physxInterface)
            omni.physxcamera.scripts.tests.setPhysxSimInterface(self._physxSimInterface)

        demo.register(DEMO_SCENES)

    def on_shutdown(self):
        if self._cameraPropertyWidgetManager is not None:
            self._cameraPropertyWidgetManager.tear_down()
            self._cameraPropertyWidgetManager = None

        if self._physxCameraInterface is not None:
            if RUNNING_WTESTS:
                omni.physx.camera.scripts.tests.clearPhysxSimInterface()
            _physxCamera.release_physx_camera_interface(self._physxCameraInterface)
            _physxCamera.release_physx_camera_interface_scripting(self._physxCameraInterface) # OM-60917
            self._physxCameraInterface = None

        if RUNNING_WTESTS:
            omni.physxcamera.scripts.tests.clearPhysxInterface()
            omni.physxcamera.scripts.tests.clearPhysxVehicleInterface()
            omni.physxcamera.scripts.tests.clearPhysxCameraInterface()

        self._physxInterface = None
        self._physxVehicleInterface = None
        self._physxCameraInterface = None

        demo.unregister(DEMO_SCENES)
