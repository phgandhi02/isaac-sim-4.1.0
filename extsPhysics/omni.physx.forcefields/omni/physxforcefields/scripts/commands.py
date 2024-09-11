import omni.kit.commands
import math
import copy

from omni.usd.commands.usd_commands import DeletePrimsCommand

from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf, UsdPhysics, PhysxSchema
from omni.physxcommands import PhysicsCommand
from omni.physx.scripts.pythonUtils import autoassign
from omni.physxforcefields.bindings import _physxForceFields


#
# Some general info:
# - closing a stage is supposed to clear the undo stack
# - storing references to prims in commands will not work in most cases since
#   after the command, the referenced prim might get deleted. An undo will 
#   recreate the object but the prim will not be the same. Thus, only path
#   strings seem safe to store.
#


def getRelationshipAtPath(path):
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage:
        return stage.GetObjectAtPath(path)
    else:
        return None


def getPrimAtPath(path):
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage:
        return stage.GetPrimAtPath(path)
    else:
        return None


class PhysXForceFieldsSetRelationshipCommand(PhysicsCommand):
    @autoassign
    def __init__(self, relationshipPath, targetPath):
        pass

    def do(self):
        relationship = getRelationshipAtPath(self._relationshipPath)
        if relationship:
            targets = relationship.GetTargets()
            if (len(targets) > 0):
                self._oldTargetPath = targets[0].pathString
            else:
                self._oldTargetPath = None

            relationship.SetTargets([self._targetPath])

    def undo(self):
        relationship = getRelationshipAtPath(self._relationshipPath)
        if (relationship and (self._oldTargetPath is not None)):
            relationship.SetTargets([self._oldTargetPath])


class CreateForceFieldCommand(PhysicsCommand):

    @autoassign
    def __init__(self, forcefield_type, path):
        self._forceFieldIndex = -1
        pass

    def do(self):
        self._physxForceFieldsInterface = _physxForceFields.acquire_physx_force_fields_interface()
        self._forceFieldIndex = self._physxForceFieldsInterface.add_forcefield(self._forcefield_type)

        return self._forceFieldIndex

    def undo(self):
        if self._forceFieldIndex >= 0:
            self._physxForceFieldsInterface.remove_forcefield(self._forceFieldIndex)


class AddPrimsToForceFieldCommand(PhysicsCommand):

    @autoassign
    def __init__(self, forcefield_index, paths):
        self._forceFieldIndex = -1
        pass

    def do(self):
        self._physxForceFieldsInterface = _physxForceFields.acquire_physx_force_fields_interface()

        added = False

        if len(self._paths) > 0:
            added = True

            for path in self._paths:
                added = added and self._physxForceFieldsInterface.add_prim(self._force_field_index, path)

        return added

    def undo(self):
        for path in self._paths:
            self._physxForceFieldsInterface.remove_prim(self._force_field_index, path)


class RemovePrimsFromForceFieldCommand(PhysicsCommand):

    @autoassign
    def __init__(self, forcefield_index, paths):
        self._forceFieldIndex = -1
        pass

    def do(self):
        self._physxForceFieldsInterface = _physxForceFields.acquire_physx_force_fields_interface()

        for path in self._paths:
            self._physxForceFieldsInterface.add_prim(self._force_field_index, path)

    def undo(self):
        for path in self._paths:
            self._physxForceFieldsInterface.add_prim(self._force_field_index, path)
   

omni.kit.commands.register_all_commands_in_module(__name__)
