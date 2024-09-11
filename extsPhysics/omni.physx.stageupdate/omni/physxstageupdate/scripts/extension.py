import omni.ext
from .. import get_physx_stage_update_node_interface
from omni.physxstageupdate.bindings._physxStageUpdateNode import release_physx_stage_update_node_interface, release_physx_stage_update_node_scripting
from .live_sync import StageUpdateLiveSync
from .fsd_check import StageUpdateFSDCheck


class OmniPhysXStageUpdateExtension(omni.ext.IExt):
    def on_startup(self):        
        self._physx_stage_update_node_interface = get_physx_stage_update_node_interface()
        self._live_sync = StageUpdateLiveSync()
        self._live_sync.startup(self._physx_stage_update_node_interface)
        self._fsd_check = StageUpdateFSDCheck()
        self._fsd_check.startup(self._physx_stage_update_node_interface)

    def on_shutdown(self):
        self._live_sync.shutdown()
        self._live_sync = None
        self._fsd_check.shutdown()
        self._fsd_check = None
        release_physx_stage_update_node_interface(self._physx_stage_update_node_interface)
        release_physx_stage_update_node_scripting(self._physx_stage_update_node_interface) # OM-60917
        self._physx_stage_update_node_interface = None
