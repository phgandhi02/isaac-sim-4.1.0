from omni.kit.test import AsyncTestCase
from omni.physx import get_physx_interface, get_physxunittests_interface, utils
import omni.physx.bindings._physx as pxb
import omni.kit.stage_templates
import carb.settings


class PhysxMiniBase(AsyncTestCase):
    async def test_get_iface(self):
        iface = get_physx_interface()
        print(iface)
        iface = None

    async def test_stage_open(self):
        await omni.kit.stage_templates.new_stage_async()
        stage = omni.usd.get_context().get_stage()
        print(stage)

    async def test_stage_open_force_cuda_context(self):
        test_iface = get_physxunittests_interface()
        settings = carb.settings.get_settings()
        prev = settings.get(pxb.SETTING_USE_ACTIVE_CUDA_CONTEXT)
        try:
            settings.set(pxb.SETTING_USE_ACTIVE_CUDA_CONTEXT, True)

            # this will print physxsdk error with cuda lib present
            # and should not print any error without cuda lib
            if test_iface.is_cuda_lib_present():
                with utils.ExpectMessage(self, "Failed to create Cuda Context Manager."):
                    await omni.kit.stage_templates.new_stage_async()
                    stage = omni.usd.get_context().get_stage()
            else:
                await omni.kit.stage_templates.new_stage_async()
                stage = omni.usd.get_context().get_stage()

            print(stage)
        finally:
            settings.set(pxb.SETTING_USE_ACTIVE_CUDA_CONTEXT, prev)
