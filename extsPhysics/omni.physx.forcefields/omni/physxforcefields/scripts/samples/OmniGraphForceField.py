import omni.physxdemos as demo
from .BasicSetup import _open_test_usd_file


class OmniGraphForceFieldDemo(demo.Base):
    title = "OmniGraph Demo"
    category = demo.Categories.FORCE_FIELDS
    short_description = "Use forces to Control physics objects with Shift-C."
    description = "A demo of how to use force fields and OmniGraph to respond to keyboard inputs and interact with " \
        "rigid bodies in the scene. Press Shift-C to grab objects directly in front of the camera. Release Shift-C " \
        "to shoot the objects collected away from the camera."

    def create(self, stage):
        _open_test_usd_file(stage, "force_field_omni_graph")