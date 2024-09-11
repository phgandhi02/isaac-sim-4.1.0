import carb
import omni.usd
import omni.kit.commands
from omni.usdphysicsui import get_usdphysicsuiprivate_interface
from pxr import Gf

# This class can be removed once the C++ manipulator gizmo will be removed
class USDPhysicsUIUpdateGizmoTransformCommand(omni.kit.commands.Command):
    def __init__(
        self,
        joint_path: str,
        new_transform: Gf.Matrix4d,
        orig_transform: Gf.Matrix4d,
    ):
        self._initial_pass = True
        self._joint_path = joint_path
        self._new_transform = new_transform
        self._orig_transform = orig_transform

    def _send_data(self, transform):
        translate = transform.ExtractTranslation()
        rotation = transform.ExtractRotation()
        pos = carb.Float3(translate[0], translate[1], translate[2])
        rot = carb.Float4(rotation.axis[0], rotation.axis[1], rotation.axis[2], rotation.angle)
        return get_usdphysicsuiprivate_interface().private_update_gizmo_transform(str(self._joint_path), pos, rot)

    def do(self):
        if not self._initial_pass:
            return self._send_data(self._new_transform)
        self._initial_pass = False
        return True

    def undo(self):
        return self._send_data(self._orig_transform)

omni.kit.commands.register_all_commands_in_module(__name__)
