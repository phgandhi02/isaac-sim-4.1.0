import omni.ext
import omni.physxdemos as demo
from omni.physx.scripts.utils import safe_import_tests

DEMO_SCENES = "omni.blockworld.samples"

safe_import_tests("omni.blockworld.tests", ["tests"])

class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        demo.register(DEMO_SCENES)

    def on_shutdown(self):
        demo.unregister(DEMO_SCENES)
