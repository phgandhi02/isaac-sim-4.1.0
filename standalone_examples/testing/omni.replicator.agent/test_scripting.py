import asyncio

import isaacsim
from omni.isaac.kit import SimulationApp

CONFIG = {"renderer": "RayTracedLighting", "headless": True, "width": 1920, "height": 1080}

if __name__ == "__main__":
    app = SimulationApp(launch_config=CONFIG)

    from omni.isaac.core.utils.extensions import enable_extension

    app.update()

    enable_extension("omni.kit.scripting")

    import omni.usd
    from omni.kit.scripting import ApplyScriptingAPICommand
    from pxr import OmniScriptingSchema, Sdf

    async def work():

        # Create new prim and attach python scripting api.
        await omni.usd.get_context().new_stage_async("tmp")
        stage = omni.usd.get_context().get_stage()
        stage.DefinePrim("/test")
        ApplyScriptingAPICommand(paths=["/test"]).do()

        # Test
        prim = stage.GetPrimAtPath("/test")
        assert prim.HasAPI(OmniScriptingSchema.OmniScriptingAPI)

    asyncio.run(work())
