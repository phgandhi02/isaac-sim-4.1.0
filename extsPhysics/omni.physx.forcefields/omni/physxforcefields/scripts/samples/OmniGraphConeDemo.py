import omni.physxdemos as demo
from .BasicSetup import _open_test_usd_file


class OmniGraphForceFieldDemo(demo.Base):
    title = "Cone Demo"
    category = demo.Categories.FORCE_FIELDS
    short_description = "Demo of how the various Conical Force Field properties work."
    description = "After starting the simulation, select the cone to translate and rotate it and observe " \
        "the directionality of the conical force field. Tune the cone's radius to widen the force field. " \
        "Select the ActionGraph/force_field_conical prim to adjust the Falloff properties and see how that " \
        "affects the cubes as they get further away from the central axis."

    def create(self, stage):
        _open_test_usd_file(stage, "force_field_cone_ogn_demo")