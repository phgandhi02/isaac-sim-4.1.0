# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import List, Union

import carb
import numpy as np
import omni.usd
import torch
from omni.physx import get_physx_replicator_interface, get_physx_simulation_interface
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdUtils, Vt


class Cloner:

    """This class provides a set of simple APIs to make duplication of objects simple.
    Objects can be cloned using this class to create copies of the same object,
    placed at user-specified locations in the scene.

    Note that the cloning process is performed in a for-loop, so performance should
    be expected to follow linear scaling with an increase of clones.
    """

    def __init__(self, stage: Usd.Stage = None):
        """
        Args:
            stage (Usd.Stage): Usd stage where source prim and clones are added to.
        """
        self._base_env_path = None
        self._root_path = None
        self._stage = stage
        if stage is None:
            self._stage = omni.usd.get_context().get_stage()

    def define_base_env(self, base_env_path: str):
        """Creates a USD Scope at base_env_path. This is designed to be the parent that holds all clones.

        Args:
            base_env_path (str): Path to create the USD Scope at.
        """

        UsdGeom.Scope.Define(self._stage, base_env_path)
        self._base_env_path = base_env_path

    def generate_paths(self, root_path: str, num_paths: int):

        """Generates a list of paths under the root path specified.

        Args:
            root_path (str): Base path where new paths will be created under.
            num_paths (int): Number of paths to generate.

        Returns:
            paths (List[str]): A list of paths
        """

        self._root_path = root_path + "_"
        return [f"{root_path}_{i}" for i in range(num_paths)]

    def replicate_physics(self, source_prim_path: str, prim_paths: list, base_env_path: str, root_path: str):
        """Replicates physics properties directly in omni.physics to avoid performance bottlenecks when parsing physics.

        Args:
            source_prim_path (str): Path of source object.
            prim_paths (List[str]): List of destination paths.
            base_env_path (str): Path to namespace for all environments.
            root_path (str): Prefix path for each environment.
        Raises:
            Exception: Raises exception if base_env_path is None or root_path is None.

        """
        if base_env_path is None and self._base_env_path is None:
            raise ValueError("base_env_path needs to be specified!")
        if root_path is None and self._root_path is None:
            raise ValueError("root_path needs to be specified!")

        # resolve number of replications being made
        replicate_first = source_prim_path not in prim_paths
        # resolve inputs
        clone_base_path = self._root_path if root_path is None else root_path
        clone_root = self._base_env_path if base_env_path is None else base_env_path
        num_replications = len(prim_paths) if replicate_first else len(prim_paths) - 1

        def replicationAttachFn(stageId):
            exclude_paths = [clone_root]
            return exclude_paths

        def replicationAttachEndFn(stageId):
            get_physx_replicator_interface().replicate(stageId, source_prim_path, num_replications)

        def hierarchyRenameFn(replicatePath, index):
            if replicate_first:
                stringPath = clone_base_path + str(index)
            else:
                stringPath = clone_base_path + str(index + 1)
            return stringPath

        stageId = UsdUtils.StageCache.Get().Insert(self._stage).ToLongInt()

        get_physx_replicator_interface().register_replicator(
            stageId, replicationAttachFn, replicationAttachEndFn, hierarchyRenameFn
        )

    def clone(
        self,
        source_prim_path: str,
        prim_paths: List[str],
        positions: Union[np.ndarray, torch.Tensor] = None,
        orientations: Union[np.ndarray, torch.Tensor] = None,
        replicate_physics: bool = False,
        base_env_path: str = None,
        root_path: str = None,
        copy_from_source: bool = False,
    ):

        """Clones a source prim at user-specified destination paths.
            Clones will be placed at user-specified positions and orientations.

        Args:
            source_prim_path (str): Path of source object.
            prim_paths (List[str]): List of destination paths.
            positions (Union[np.ndarray, torch.Tensor]): An array containing target positions of clones. Dimension must equal length of prim_paths.
                                    Defaults to None. Clones will be placed at (0, 0, 0) if not specified.
            orientations (Union[np.ndarray, torch.Tensor]): An array containing target orientations of clones. Dimension must equal length of prim_paths.
                                    Defaults to None. Clones will have identity orientation (1, 0, 0, 0) if not specified.
            replicate_physics (bool): Uses omni.physics replication. This will replicate physics properties directly for paths beginning with root_path and skip physics parsing for anything under the base_env_path.
            base_env_path (str): Path to namespace for all environments. Required if replicate_physics=True and define_base_env() not called.
            root_path (str): Prefix path for each environment. Required if replicate_physics=True and generate_paths() not called.
            copy_from_source: (bool): Setting this to False will inherit all clones from the source prim; any changes made to the source prim will be reflected in the clones.
                         Setting this to True will make copies of the source prim when creating new clones; changes to the source prim will not be reflected in clones. Defaults to False. Note that setting this to True will take longer to execute.
        Raises:
            Exception: Raises exception if source prim path is not valid.

        """
        # check if inputs are valid
        if positions is not None:
            if len(positions) != len(prim_paths):
                raise ValueError("Dimension mismatch between positions and prim_paths!")
            # convert to numpy array
            if isinstance(positions, torch.Tensor):
                positions = positions.detach().cpu().numpy()
            elif not isinstance(positions, np.ndarray):
                positions = np.asarray(positions)
            # convert to pxr gf
            positions = Vt.Vec3fArray.FromNumpy(positions)
        if orientations is not None:
            if len(orientations) != len(prim_paths):
                raise ValueError("Dimension mismatch between orientations and prim_paths!")
            # convert to numpy array
            if isinstance(orientations, torch.Tensor):
                orientations = orientations.detach().cpu().numpy()
            elif not isinstance(orientations, np.ndarray):
                orientations = np.asarray(orientations)
            # convert to pxr gf -- wxyz to xyzw
            orientations = np.roll(orientations, -1, -1)
            orientations = Vt.QuatdArray.FromNumpy(orientations)

        # make sure source prim has valid xform properties
        source_prim = self._stage.GetPrimAtPath(source_prim_path)
        if not source_prim:
            raise Exception("Source prim does not exist")
        properties = source_prim.GetPropertyNames()
        xformable = UsdGeom.Xformable(source_prim)
        # get current position and orientation
        T_p_w = xformable.ComputeParentToWorldTransform(Usd.TimeCode.Default())
        T_l_w = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        T_l_p = Gf.Transform()
        T_l_p.SetMatrix(Gf.Matrix4d(np.matmul(T_l_w, np.linalg.inv(T_p_w)).tolist()))
        current_translation = T_l_p.GetTranslation()
        current_orientation = T_l_p.GetRotation().GetQuat()
        # get current scale
        current_scale = Gf.Vec3d(1, 1, 1)
        if "xformOp:scale" in properties:
            current_scale = Gf.Vec3d(source_prim.GetAttribute("xformOp:scale").Get())

        # remove all xform ops except for translate, orient, and scale
        properties_to_remove = [
            "xformOp:rotateX",
            "xformOp:rotateXZY",
            "xformOp:rotateY",
            "xformOp:rotateYXZ",
            "xformOp:rotateYZX",
            "xformOp:rotateZ",
            "xformOp:rotateZYX",
            "xformOp:rotateZXY",
            "xformOp:rotateXYZ",
            "xformOp:transform",
            "xformOp:scale",
        ]
        xformable.ClearXformOpOrder()
        for prop_name in properties:
            if prop_name in properties_to_remove:
                source_prim.RemoveProperty(prop_name)

        properties = source_prim.GetPropertyNames()
        # add xform ops if they don't exist
        if "xformOp:translate" not in properties:
            xform_op_translate = xformable.AddXformOp(
                UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.PrecisionDouble, ""
            )
        else:
            xform_op_translate = UsdGeom.XformOp(source_prim.GetAttribute("xformOp:translate"))
        xform_op_translate.Set(current_translation)

        if "xformOp:orient" not in properties:
            xform_op_rot = xformable.AddXformOp(UsdGeom.XformOp.TypeOrient, UsdGeom.XformOp.PrecisionDouble, "")
        else:
            xform_op_rot = UsdGeom.XformOp(source_prim.GetAttribute("xformOp:orient"))
        xform_op_rot.Set(current_orientation)

        if "xformOp:scale" not in properties:
            xform_op_scale = xformable.AddXformOp(UsdGeom.XformOp.TypeScale, UsdGeom.XformOp.PrecisionDouble, "")
        else:
            xform_op_scale = UsdGeom.XformOp(source_prim.GetAttribute("xformOp:scale"))
        xform_op_scale.Set(current_scale)
        # set xform op order
        xformable.SetXformOpOrder([xform_op_translate, xform_op_rot, xform_op_scale])

        # set source actor transform
        if source_prim_path in prim_paths:
            idx = prim_paths.index(source_prim_path)
            prim = UsdGeom.Xform(self._stage.GetPrimAtPath(source_prim_path))

            if positions is not None:
                translation = positions[idx]
            else:
                translation = current_translation

            if orientations is not None:
                orientation = orientations[idx]
            else:
                orientation = current_orientation

            # overwrite translation and orientation to values specified
            prim.GetPrim().GetAttribute("xformOp:translate").Set(translation)
            prim.GetPrim().GetAttribute("xformOp:orient").Set(orientation)

        has_clones = False
        with Sdf.ChangeBlock():
            for i, prim_path in enumerate(prim_paths):
                if prim_path != source_prim_path:
                    has_clones = True

                    env_spec = Sdf.CreatePrimInLayer(self._stage.GetRootLayer(), prim_path)
                    stack = UsdGeom.Xform(self._stage.GetPrimAtPath(source_prim_path)).GetPrim().GetPrimStack()

                    if copy_from_source:
                        Sdf.CopySpec(env_spec.layer, Sdf.Path(source_prim_path), env_spec.layer, Sdf.Path(prim_path))
                    else:
                        env_spec.inheritPathList.Prepend(source_prim_path)

                    if positions is not None:
                        translation = positions[i]  # use specified translation
                    else:
                        translation = current_translation  # use the same translation as source

                    if orientations is not None:
                        orientation = orientations[i]  # use specified orientation
                    else:
                        orientation = current_orientation  # use the same orientation as source

                    translate_spec = env_spec.GetAttributeAtPath(prim_path + ".xformOp:translate")
                    if translate_spec is None:
                        translate_spec = Sdf.AttributeSpec(env_spec, "xformOp:translate", Sdf.ValueTypeNames.Double3)
                    translate_spec.default = translation

                    orient_spec = env_spec.GetAttributeAtPath(prim_path + ".xformOp:orient")
                    if orient_spec is None:
                        orient_spec = Sdf.AttributeSpec(env_spec, "xformOp:orient", Sdf.ValueTypeNames.Quatd)
                    orient_spec.default = orientation

                    scale_spec = env_spec.GetAttributeAtPath(prim_path + ".xformOp:scale")
                    if scale_spec is None:
                        scale_spec = Sdf.AttributeSpec(env_spec, "xformOp:scale", Sdf.ValueTypeNames.Double3)
                    scale_spec.default = current_scale

                    op_order_spec = env_spec.GetAttributeAtPath(prim_path + ".xformOpOrder")
                    if op_order_spec is None:
                        op_order_spec = Sdf.AttributeSpec(
                            env_spec, UsdGeom.Tokens.xformOpOrder, Sdf.ValueTypeNames.TokenArray
                        )
                    op_order_spec.default = Vt.TokenArray(["xformOp:translate", "xformOp:orient", "xformOp:scale"])

        if replicate_physics and has_clones:
            self.replicate_physics(source_prim_path, prim_paths, base_env_path, root_path)
        else:
            get_physx_replicator_interface().unregister_replicator(
                UsdUtils.StageCache.Get().Insert(self._stage).ToLongInt()
            )

    def filter_collisions(
        self, physicsscene_path: str, collision_root_path: str, prim_paths: List[str], global_paths: List[str] = []
    ):
        """Filters collisions between clones. Clones will not collide with each other, but can collide with objects specified in global_paths.

        Args:
            physicsscene_path (str): Path to PhysicsScene object in stage.
            collision_root_path (str): Path to place collision groups under.
            prim_paths (List[str]): Paths of objects to filter out collision.
            global_paths (List[str]): Paths of objects to generate collision (e.g. ground plane).

        """

        physx_scene = PhysxSchema.PhysxSceneAPI(self._stage.GetPrimAtPath(physicsscene_path))

        # We invert the collision group filters for more efficient collision filtering across environments
        physx_scene.CreateInvertCollisionGroupFilterAttr().Set(True)

        collision_scope = UsdGeom.Scope.Define(self._stage, collision_root_path)

        with Sdf.ChangeBlock():
            if len(global_paths) > 0:
                global_collision_group_path = collision_root_path + "/global_group"
                # add collision group prim
                global_collision_group = Sdf.PrimSpec(
                    self._stage.GetRootLayer().GetPrimAtPath(collision_root_path),
                    "global_group",
                    Sdf.SpecifierDef,
                    "PhysicsCollisionGroup",
                )
                # prepend collision API schema
                global_collision_group.SetInfo(
                    Usd.Tokens.apiSchemas, Sdf.TokenListOp.Create({"CollectionAPI:colliders"})
                )

                # expansion rule
                expansion_rule = Sdf.AttributeSpec(
                    global_collision_group,
                    "collection:colliders:expansionRule",
                    Sdf.ValueTypeNames.Token,
                    Sdf.VariabilityUniform,
                )
                expansion_rule.default = "expandPrims"

                # includes rel
                global_includes_rel = Sdf.RelationshipSpec(
                    global_collision_group, "collection:colliders:includes", False
                )
                for global_path in global_paths:
                    global_includes_rel.targetPathList.Append(global_path)

                # filteredGroups rel
                global_filtered_groups = Sdf.RelationshipSpec(global_collision_group, "physics:filteredGroups", False)
                # We are using inverted collision group filtering, which means objects by default don't collide across
                # groups. We need to add this group as a filtered group, so that objects within this group collide with
                # each other.
                global_filtered_groups.targetPathList.Append(global_collision_group_path)

            # set collision groups and filters
            for i, prim_path in enumerate(prim_paths):
                collision_group_path = collision_root_path + f"/group{i}"
                # add collision group prim
                collision_group = Sdf.PrimSpec(
                    self._stage.GetRootLayer().GetPrimAtPath(collision_root_path),
                    f"group{i}",
                    Sdf.SpecifierDef,
                    "PhysicsCollisionGroup",
                )
                # prepend collision API schema
                collision_group.SetInfo(Usd.Tokens.apiSchemas, Sdf.TokenListOp.Create({"CollectionAPI:colliders"}))

                # expansion rule
                expansion_rule = Sdf.AttributeSpec(
                    collision_group,
                    "collection:colliders:expansionRule",
                    Sdf.ValueTypeNames.Token,
                    Sdf.VariabilityUniform,
                )
                expansion_rule.default = "expandPrims"

                # includes rel
                includes_rel = Sdf.RelationshipSpec(collision_group, "collection:colliders:includes", False)
                includes_rel.targetPathList.Append(prim_path)

                # filteredGroups rel
                filtered_groups = Sdf.RelationshipSpec(collision_group, "physics:filteredGroups", False)
                # We are using inverted collision group filtering, which means objects by default don't collide across
                # groups. We need to add this group as a filtered group, so that objects within this group collide with
                # each other.
                filtered_groups.targetPathList.Append(collision_group_path)
                if len(global_paths) > 0:
                    filtered_groups.targetPathList.Append(global_collision_group_path)
                    global_filtered_groups.targetPathList.Append(collision_group_path)
