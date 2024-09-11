import omni

from .mesh_merger import MeshMerger


class MergeMeshesCommand(omni.kit.commands.Command):
    """Merges selected meshes and all children of given prims into a single mesh.
    Options:
    Clear Parent Transofrm - Sets the mesh origin at world origin, otherwise origin is the same as the first element
    Deactivate source assets - Sets source prims to Inactive after performing the merge operation
    Combine Materials - Redirects all assets materials to a given folder, and every geomsubset that shares a same
    material name uses the same material, each geom subset uses the original material from the source assets

    typical usage:
    .. code-block:: python

    result, prim = omni.kit.commands.execute(
        "MergeMeshesCommand",
        source=["/World/Cube","World/Cone", "World/ManyToruses"],
        clear_transform=False,
        deactivate_source=True,
        combine_materials=True,
        materials_destination="/World/Looks"
    )

    """

    def __init__(
        self,
        source: str,
        clear_transform: bool = False,
        deactivate_source: bool = False,
        combine_materials: bool = False,
        materials_destination: str = "/World/Looks",
    ):
        self._stage = omni.usd.get_context().get_stage()
        self.mesh_merger = MeshMerger(self._stage)
        self.mesh_merger.clear_parent_xform = clear_transform
        self.mesh_merger.deactivate_source = deactivate_source
        self.mesh_merger.combine_materials = combine_materials
        self.mesh_merger.materials_destination = materials_destination
        self.mesh_merger.update_selection(selection=source, stage=self._stage)

        self.mesh_merger.output_mesh = "/Merged/" + str(self._stage.GetPrimAtPath(source[0]).GetName())

    def do(self):
        self.mesh_merger.merge_meshes()

        return self.mesh_merger.output_mesh

    def undo(self):
        self.mesh_merger.reactivate_sources()
        self._stage.RemovePrim(self.mesh_merger.output_mesh)
        self.mesh_merger.remove_created_materials()
