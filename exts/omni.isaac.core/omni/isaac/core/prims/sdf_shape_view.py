# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import List, Optional, Union

import carb
import numpy as np
import omni.kit.app
import torch
import warp as wp
from omni.isaac.core.prims.geometry_prim_view import GeometryPrimView
from omni.isaac.core.simulation_context.simulation_context import SimulationContext
from omni.isaac.core.utils.prims import find_matching_prim_paths, get_prim_at_path
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics


class SdfShapeView(GeometryPrimView):
    """High level functions to deal with geometry prims that provide their Signed Distance Field (SDF).

    This object wraps all matching mesh geometry prims found at the regex provided at the prim_paths_expr.

    Args:

        prim_paths_expr (str): prim paths regex to encapsulate all prims that match it.
                                example: "/World/Env[1-5]/Microwave" will match /World/Env1/Microwave,
                                /World/Env2/Microwave..etc.
                                (a non regex prim path can also be used to encapsulate one XForm).
        num_query_points (int): number of points queried by this view object
        prepare_sdf_schemas (bool, optional): apply PhysxSDFMeshCollisionAPI to prims in prim_paths_expr. Defaults to True.
        name (str, optional): shortname to be used as a key by Scene class.
                                Note: needs to be unique if the object is added to the Scene.
                                Defaults to "sdf_shape_view".
        positions (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional):
                                                        default positions in the world frame of the prim.
                                                        shape is (N, 3).
                                                        Defaults to None, which means left unchanged.
        translations (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional):
                                                        default translations in the local frame of the prims
                                                        (with respect to its parent prims). shape is (N, 3).
                                                        Defaults to None, which means left unchanged.
        orientations (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional):
                                                        default quaternion orientations in the world/ local frame of the prim
                                                        (depends if translation or position is specified).
                                                        quaternion is scalar-first (w, x, y, z). shape is (N, 4).
                                                        Defaults to None, which means left unchanged.
        scales (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): local scales to be applied to
                                                        the prim's dimensions. shape is (N, 3).
                                                        Defaults to None, which means left unchanged.
        visibilities (Optional[Union[np.ndarray, torch.Tensor, wp.array], optional): set to false for an invisible prim in
                                                                            the stage while rendering. shape is (N,).
                                                                            Defaults to None.
        reset_xform_properties (bool, optional): True if the prims don't have the right set of xform properties
                                                (i.e: translate, orient and scale) ONLY and in that order.
                                                Set this parameter to False if the object were cloned using using
                                                the cloner api in omni.isaac.cloner. Defaults to True.
        collisions (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): Set to True if the geometry already have/
                                                    should have a collider (i.e not only a visual geometry). shape is (N,).
                                                    Defaults to None.
        track_contact_forces (bool, Optional) : if enabled, the view will track the net contact forces on each geometry prim
                                                in the view. Note that the collision flag should be set to True to report
                                                contact forces. Defaults to False.
        prepare_contact_sensors (bool, Optional): applies contact reporter API to the prim if it already does not have one.
                                                    Defaults to False.
        disable_stablization (bool, optional): disables the contact stabilization parameter in the physics context.
                                                Defaults to True.
        contact_filter_prim_paths_expr (Optional[List[str]], Optional): a list of filter expressions which allows for tracking
                                                                        contact forces between the geometry prim and this subset
                                                                        through get_contact_force_matrix().

    """

    def __init__(
        self,
        prim_paths_expr: str,
        num_query_points: int,
        prepare_sdf_schemas: bool = True,
        name: str = "sdf_shape_view",
        positions: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        translations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        scales: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        visibilities: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        reset_xform_properties: bool = True,
        collisions: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        track_contact_forces: bool = False,
        prepare_contact_sensors: bool = False,
        disable_stablization: bool = True,
        contact_filter_prim_paths_expr: Optional[List[str]] = [],
    ) -> None:
        GeometryPrimView.__init__(
            self,
            prim_paths_expr=prim_paths_expr,
            name=name,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            visibilities=visibilities,
            reset_xform_properties=reset_xform_properties,
            collisions=collisions,
            track_contact_forces=track_contact_forces,
            prepare_contact_sensors=prepare_contact_sensors,
            disable_stablization=disable_stablization,
            contact_filter_prim_paths_expr=contact_filter_prim_paths_expr,
        )

        self._num_query_points = num_query_points
        self._physics_view = None

        if prepare_sdf_schemas:
            self._prim_paths = find_matching_prim_paths(prim_paths_expr)
            for path in self._prim_paths:
                prim = get_prim_at_path(path)
                if not prim.IsA(UsdGeom.Mesh):
                    carb.log_error(f"prim at path'{path}' is not a UsdGeom.Mesh and cannot provide sdf information!")
                else:
                    self._apply_sdf_schema(get_prim_at_path(path))
                    self._prims.append(prim)

        self._sdf_collision_apis = [None] * self._count

        return

    @property
    def num_query_points(self) -> int:
        """
        Returns:
            int: number of points queried by this view object.
        """
        return self._num_query_points

    def _apply_sdf_schema(self, prim_at_path):
        """apply appropriate sdf schemas to prims."""

        if not prim_at_path.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim_at_path)
        if not prim_at_path.HasAPI(UsdPhysics.MeshCollisionAPI):
            meshcollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(prim_at_path)
        meshcollisionAPI.CreateApproximationAttr().Set("sdf")

        if not prim_at_path.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
            PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim_at_path)

    def is_physics_handle_valid(self) -> bool:
        """
        Returns:
            bool: True if the physics handle of the view is valid (i.e physics is initialized for the view). Otherwise False.
        """
        return self._physics_view is not None

    def _invalidate_physics_handle_callback(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physics_view = None
        return

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates a sdf shape view in physX.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.
        """
        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
        carb.log_info("initializing view for {}".format(self._name))
        self._physics_sim_view = physics_sim_view
        self._physics_view = physics_sim_view.create_sdf_shape_view(
            self._regex_prim_paths[0].replace(".*", "*"), self._num_query_points
        )
        if not carb.settings.get_settings().get_as_bool("/physics/suppressReadback"):
            carb.log_error("Using SDFShapeView requires the gpu pipeline or (a World initialized with a cuda device)")
        carb.log_info("SDF Shape View Device: {}".format(self._device))
        self._num_shapes = self._physics_view.count
        self._num_query_points = self._physics_view.max_num_points
        return

    def get_sdf_and_gradients(
        self,
        points: Union[np.ndarray, torch.Tensor],
        indices: Optional[Union[np.ndarray, torch.Tensor]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor]:
        """Get the SDF values and gradients of the query points

        Args:
            points ([Union[np.ndarray, torch.Tensor]]): points (represented in the local frames of meshes) to be queried for sdf and gradients.
                                                                          shape is (self.num_shapes, self.num_query_points, 3).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: SDF values and gradients of points for prims with shape (self.num_shapes, self.num_query_points, 4).
            The first component is the SDF value while the last three represent the gradient
        """
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self._num_shapes, self._device)
            points = self._backend_utils.move_data(points, self._device)
            sdf_and_gradients = self._physics_view.get_sdf_and_gradients(points)
            if not clone:
                return sdf_and_gradients[indices]
            else:
                return self._backend_utils.clone_tensor(sdf_and_gradients[indices], device=self._device)
        else:
            carb.log_warn("Physics Simulation View is not created yet to use the SdfShapeView")
            return None

    def get_sdf_margins(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets sdf margin values.

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: margins of the sdf collision apis for prims in the view. shape is (M,).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = np.zeros(indices.shape[0], dtype="float32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            values[write_idx] = self._sdf_collision_apis[i].GetSdfMarginAttr().Get()
            write_idx += 1
        values = self._backend_utils.convert(values, dtype="float32", device=self._device, indexed=True)

        return values

    def get_sdf_narrow_band_thickness(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets sdf collision narrow band thickness values.

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: narrow band thickness of the sdf collision apis for prims in the view. shape is (M,).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = np.zeros(indices.shape[0], dtype="float32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            values[write_idx] = self._sdf_collision_apis[i].GetSdfNarrowBandThicknessAttr().Get()
            write_idx += 1
        values = self._backend_utils.convert(values, dtype="float32", device=self._device, indexed=True)

        return values

    def get_sdf_subgrid_resolution(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets sdf collision subgrid resolution values.

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: subgrid resolutions of the sdf collision apis for prims in the view. shape is (M,).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = np.zeros(indices.shape[0], dtype="int32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            values[write_idx] = self._sdf_collision_apis[i].GetSdfSubgridResolutionAttr().Get()
            write_idx += 1
        values = self._backend_utils.convert(values, dtype="int32", device=self._device, indexed=True)

        return values

    def get_sdf_resolution(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets sdf collision resolution values.

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: resolutions of the sdf collision apis for prims in the view. shape is (M,).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = np.zeros(indices.shape[0], dtype="float32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            values[write_idx] = self._sdf_collision_apis[i].GetSdfResolutionAttr().Get()
            write_idx += 1
        values = self._backend_utils.convert(values, dtype="float32", device=self._device, indexed=True)

        return values

    def set_sdf_margins(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """Sets signed distance field margins for prims in the view.

        Args:
            values (Union[np.ndarray, torch.Tensor]): sdf margins to be set. shape (M,).
            indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        read_idx = 0
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            self._sdf_collision_apis[i].GetSdfMarginAttr().Set(values[read_idx].tolist())
            read_idx += 1

    def set_sdf_narrow_band_thickness(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """Sets signed distance field narrow band thicknesses for prims in the view.

        Args:
            values (Union[np.ndarray, torch.Tensor]): sdf margins to be set. shape (M,).
            indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        read_idx = 0
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            self._sdf_collision_apis[i].GetSdfNarrowBandThicknessAttr().Set(values[read_idx].tolist())
            read_idx += 1

    def set_sdf_subgrid_resolution(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """Sets signed distance field subgrid resolutions for prims in the view.

        Args:
            values (Union[np.ndarray, torch.Tensor]): sdf margins to be set. shape (M,).
            indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        read_idx = 0
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            self._sdf_collision_apis[i].GetSdfSubgridResolutionAttr().Set(values[read_idx].tolist())
            read_idx += 1

    def set_sdf_resolution(
        self, values: Union[np.ndarray, torch.Tensor], indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None
    ) -> None:
        """Sets signed distance field subgrid resolutions for prims in the view.

        Args:
            values (Union[np.ndarray, torch.Tensor]): sdf margins to be set. shape (M,).
            indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        read_idx = 0
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            self._sdf_collision_apis[i].GetSdfResolutionAttr().Set(values[read_idx].tolist())
            read_idx += 1
