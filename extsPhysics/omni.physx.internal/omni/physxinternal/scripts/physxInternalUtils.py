from functools import partial
import carb
from pxr import Usd, UsdGeom, Sdf, Gf, PhysxSchema
import omni.usd
import typing
from enum import Enum


class TetrahedralMeshType(str, Enum):
    CONFORMING = "conforming"
    VOXEL = "voxel"


def create_tetrahedral_mesh_path(
    stage, source_mesh_path: Sdf.Path, tetrahedral_mesh_type: TetrahedralMeshType
) -> Sdf.Path:
    name = source_mesh_path.name
    conforming_str = "_" + TetrahedralMeshType.CONFORMING
    voxel_str = "_" + TetrahedralMeshType.VOXEL
    new_str = "_" + tetrahedral_mesh_type

    if name.find(conforming_str) > -1:
        name = name.replace(conforming_str, new_str)
    elif name.find(voxel_str) > -1:
        name = name.replace(voxel_str, new_str)
    else:
        name = name + new_str

    new_path = source_mesh_path.ReplaceName(name)
    return Sdf.Path(omni.usd.get_stage_next_free_path(stage, str(new_path), False))
