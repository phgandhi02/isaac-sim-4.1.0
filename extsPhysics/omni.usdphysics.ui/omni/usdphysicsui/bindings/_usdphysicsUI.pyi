"""pybind11 carb.usdphysicsuiprivate bindings"""
from __future__ import annotations
import omni.usdphysicsui.bindings._usdphysicsUI
import typing
import carb._carb

__all__ = [
    "IUsdPhysicsUI",
    "IUsdPhysicsUIPrivate",
    "SETTING_DISPLAY_JOINTS",
    "SETTING_DISPLAY_JOINTS_DEFAULT",
    "acquire_usdphysics_ui_interface",
    "acquire_usdphysics_ui_private_interface",
    "release_usdphysics_ui_interface",
    "release_usdphysics_ui_interface_scripting",
    "release_usdphysics_ui_private_interface",
    "release_usdphysics_ui_private_interface_scripting"
]


class IUsdPhysicsUI():
    def block_usd_notice_handler(self, arg0: bool) -> None: ...
    def is_usd_notice_handler_enabled(self) -> bool: ...
    pass
class IUsdPhysicsUIPrivate():
    def private_draw_immediate_mode_viewport_overlays(self, arg0: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)], arg1: typing.Annotated[typing.List[float], pybind11_stubgen.typing_ext.FixedSize(16)], arg2: float, arg3: float, arg4: float, arg5: float, arg6: float, arg7: typing.Callable[[bool], None]) -> None: ...
    def private_update_gizmo_transform(self, arg0: str, arg1: carb._carb.Float3, arg2: carb._carb.Float4) -> bool: ...
    pass
def acquire_usdphysics_ui_interface(plugin_name: str = None, library_path: str = None) -> IUsdPhysicsUI:
    pass
def acquire_usdphysics_ui_private_interface(plugin_name: str = None, library_path: str = None) -> IUsdPhysicsUIPrivate:
    pass
def release_usdphysics_ui_interface(arg0: IUsdPhysicsUI) -> None:
    pass
def release_usdphysics_ui_interface_scripting(arg0: IUsdPhysicsUI) -> None:
    pass
def release_usdphysics_ui_private_interface(arg0: IUsdPhysicsUIPrivate) -> None:
    pass
def release_usdphysics_ui_private_interface_scripting(arg0: IUsdPhysicsUIPrivate) -> None:
    pass
SETTING_DISPLAY_JOINTS = '/persistent/physics/visualizationDisplayJoints'
SETTING_DISPLAY_JOINTS_DEFAULT = '/defaults/persistent/physics/visualizationDisplayJoints'
