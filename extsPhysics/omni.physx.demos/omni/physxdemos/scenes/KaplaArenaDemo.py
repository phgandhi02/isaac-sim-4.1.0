from .KaplaBaseDemo import KaplaDemo
import omni.physxdemos as demo
import carb


class KaplaArenaDemo(demo.Base, KaplaDemo):
    title = "Kapla arena"
    category = demo.Categories.RIGID_BODIES
    short_description = "Large demo scene showing massive rigid body point instancer usage"
    description = "Large demo scene showing massive rigid body point instancer usage. Press play (space) to run the simulation."

    def create(self, stage):
        kaplaDemo = KaplaDemo()
        kaplaDemo.create_kapla_arena(stage, self)
        room = demo.get_demo_room(self, stage, zoom=0.1, enableCollisionAudio=False)
        # increase picking force for easier destruction
        carb.settings.get_settings().set("/physics/pickingForce",10)
