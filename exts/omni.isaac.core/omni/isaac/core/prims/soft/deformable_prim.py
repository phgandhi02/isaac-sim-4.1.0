# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import List, Optional, Sequence, Union

import carb
import numpy as np

# omniverse
import omni
import omni.isaac.core.utils.deformable_mesh_utils as deformableMeshUtils
import torch
from omni.isaac.core.materials.deformable_material import DeformableMaterial
from omni.isaac.core.prims._impl.single_prim_wrapper import _SinglePrimWrapper
from omni.isaac.core.prims.soft.deformable_prim_view import DeformablePrimView

# isaac.core.soft
from omni.isaac.core.prims.xform_prim import XFormPrim

# isaac-core
from omni.isaac.core.simulation_context.simulation_context import SimulationContext
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.utils.types import DynamicState
from omni.physx.scripts import deformableUtils, physicsUtils
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade


class DeformablePrim(_SinglePrimWrapper):
    """Deformable primitive object provide functionalities to create and control deformable objects"""

    def __init__(
        self,
        prim_path: str,
        deformable_material: Optional[DeformableMaterial] = None,
        name: Optional[str] = "deformable",
        position: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        vertex_velocity_damping: Optional[float] = None,
        sleep_damping: Optional[float] = None,
        sleep_threshold: Optional[float] = None,
        settling_threshold: Optional[float] = None,
        self_collision: Optional[bool] = True,
        self_collision_filter_distance: Optional[float] = None,
        solver_position_iteration_count: Optional[int] = None,
        kinematic_enabled: Optional[bool] = False,
        simulation_rest_points: Optional[Sequence[float]] = None,
        simulation_indices: Optional[Sequence[int]] = None,
        simulation_hexahedral_resolution: Optional[int] = 10,
        collision_rest_points: Optional[Sequence[float]] = None,
        collision_indices: Optional[Sequence[int]] = None,
        collision_simplification: Optional[bool] = True,
        collision_simplification_remeshing: Optional[bool] = True,
        collision_simplification_remeshing_resolution: Optional[int] = 0,
        collision_simplification_target_triangle_count: Optional[int] = 0,
        collision_simplification_force_conforming: Optional[bool] = False,
        embedding: Optional[Sequence[int]] = None,
    ) -> None:
        """Creates a deformable body at prim_path given the deformable parameters.
        Note that although this class provide methods for retrieving the rest points and element indices of the underlying mesh, using the constructor of the class is the only way to set the rest points and element indices of the underlying mesh. This is to ensure the compatibility of the relevant input parameters and to avoid corrupting the mesh.
        Note also that this class does not provide methods to change USD attributes related to meshing, because once those are used for constructing the mesh, changing the parameters at runtime does not have any effect. Using the constructor of the class is the only way to set desired values for such parameters.
        TODO: indicated the range and dimensions of the input parameters
        Args:
            prim_path (str): The absolute path that the prim is supposed to be registered in.
            name (str, optional): Name given to the prim, this can be different than the prim path. Defaults to None.
            position (Sequence[float], optional): The position of the center of the deformable.
            orientation (Sequence[float], optional): The initial orientation of the deformable, assuming deformable is flat.
            scale (Sequence[float], optional): The scale of the deformable.
            visible (bool, optional): True if the deformable is supposed to be visible, False otherwise.
            vertex_velocity_damping (float, optional): Velocity damping parameter controlling how much after every time step the nodal velocity is reduced
            sleep_damping (float, optional): Damping value that damps the motion of bodies that move slow enough to be candidates for sleeping (see sleep_threshold)
            sleep_threshold (float, optional): Threshold that defines the maximal magnitude of the linear motion a soft body can move in one second such that it can go to sleep in the next frame
            settling_threshold (float, optional): Threshold that defines the maximal magnitude of the linear motion a fem body can move in one second before it becomes a candidate for sleeping
            self_collision (bool, optional): Enables the self collision for the deformable body based on the rest position distances.
            self_collision_filter_distance (float, optional): Penetration value that needs to get exceeded before contacts for self collision are generated. Will only have an effect if self collisions are enabled based on the rest position distances.
            solver_position_iteration_count (int, optional): Number of the solver's positional iteration counts
            kinematic_enabled (bool, optional): Enables kinematic body.
            simulation_rest_points (Sequence[float], optional): List of vertices of the simulation tetrahedral mesh at rest. If a simulation mesh is provided, the collision mesh needs to be provided too. If no simulation mesh is provided it will be computed implicitly based on simulation_hexahedral_resolution
            simulation_indices (Sequence[int], optional): List of indices of the simulation tetrahedral mesh. It is mandatory to provide this list if simulation_rest_points is specified as well.
            simulation_hexahedral_resolution (int, optional): The parameter controlling the resolution of the soft body simulation mesh
            collision_rest_points (Sequence[float], optional): List of vertices of the collision tetrahedral mesh at rest. If a simulation mesh is provided, the collision mesh needs to be provided too.
            collision_indices (Sequence[int], optional): List of indices of the simulation tetrahedral mesh. It is mandatory to provide this list if collision_rest_points is specified as well.
            collision_simplification (bool, optional): Flag indicating if simplification should be applied to the mesh before creating a soft body out of it. This is ignored if simulation mesh has been provided.
            collision_simplification_remeshing (bool, optional): Flag indicating if the simplification should be based on remeshing. This is ignored if collision_simplification is False.
            collision_simplification_remeshing_resolution (int, optional): The resolution used for remeshing. A value of 0 indicates that a heuristic is used to determine the resolution. Ignored if collision_simplification_remeshing is False.
            collision_simplification_target_triangle_count (int, optional): The target triangle count used for the simplification. A value of 0 indicates that a heuristic based on the simulation_hexahedral_resolution is to determine the target count. This is ignored if collision_simplification equals False.
            collision_simplification_force_conforming (bool, optional): Flag indicating that the tretrahedralizer used to generate the collision mesh should produce tetrahedra that conform to the triangle mesh. If False the implementation chooses the tretrahedralizer used.
            embedding (Sequence[int], optional):embedding information mapping collision points to the containing simulation tetrahedra.
        """
        self._stage = get_current_stage()
        self._prim = self._stage.GetPrimAtPath(prim_path)
        self._mesh = UsdGeom.Mesh.Get(self._stage, prim_path)
        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._device = SimulationContext.instance().device
            self._backend_utils = SimulationContext.instance().backend_utils
        else:
            import omni.isaac.core.utils.numpy as np_utils

            self._backend = "numpy"
            self._device = None
            self._backend_utils = np_utils

        # configure as deformable
        deformableUtils.add_physx_deformable_body(
            self._stage,
            prim_path=prim_path,
            vertex_velocity_damping=vertex_velocity_damping,
            sleep_damping=sleep_damping,
            sleep_threshold=sleep_threshold,
            settling_threshold=settling_threshold,
            self_collision=self_collision,
            self_collision_filter_distance=self_collision_filter_distance,
            solver_position_iteration_count=solver_position_iteration_count,
            kinematic_enabled=kinematic_enabled,
            simulation_rest_points=simulation_rest_points,
            simulation_indices=simulation_indices,
            simulation_hexahedral_resolution=simulation_hexahedral_resolution,
            collision_rest_points=collision_rest_points,
            collision_indices=collision_indices,
            collision_simplification=collision_simplification,
            collision_simplification_remeshing=collision_simplification_remeshing,
            collision_simplification_remeshing_resolution=collision_simplification_remeshing_resolution,
            collision_simplification_target_triangle_count=collision_simplification_target_triangle_count,
            collision_simplification_force_conforming=collision_simplification_force_conforming,
            embedding=embedding,
        )

        # prepare inputs for view construction
        if vertex_velocity_damping is not None:
            vertex_velocity_damping = self._backend_utils.create_tensor_from_list(
                [vertex_velocity_damping], dtype="float32", device=self._device
            )
        if sleep_damping is not None:
            sleep_damping = self._backend_utils.create_tensor_from_list(
                [sleep_damping], dtype="float32", device=self._device
            )
        if sleep_threshold is not None:
            sleep_threshold = self._backend_utils.create_tensor_from_list(
                [sleep_threshold], dtype="float32", device=self._device
            )
        if settling_threshold is not None:
            settling_threshold = self._backend_utils.create_tensor_from_list(
                [settling_threshold], dtype="float32", device=self._device
            )
        if self_collision is not None:
            self_collision = self._backend_utils.create_tensor_from_list(
                [self_collision], dtype="bool", device=self._device
            )
        if self_collision_filter_distance is not None:
            self_collision_filter_distance = self._backend_utils.create_tensor_from_list(
                [self_collision_filter_distance], dtype="float32", device=self._device
            )
        if solver_position_iteration_count is not None:
            solver_position_iteration_count = self._backend_utils.create_tensor_from_list(
                [solver_position_iteration_count], dtype="int32", device=self._device
            )

        if position is not None:
            position = self._backend_utils.convert(position, self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        if scale is not None:
            scale = self._backend_utils.convert(scale, self._device)
            scale = self._backend_utils.expand_dims(scale, 0)
        if visible is not None:
            visible = self._backend_utils.create_tensor_from_list([visible], dtype="bool", device=self._device)

        self._deformable_prim_view = DeformablePrimView(
            prim_paths_expr=prim_path,
            name=name,
            positions=position,
            orientations=orientation,
            scales=scale,
            visibilities=visible,
            vertex_velocity_dampings=vertex_velocity_damping,
            sleep_dampings=sleep_damping,
            sleep_thresholds=sleep_threshold,
            settling_thresholds=settling_threshold,
            self_collisions=self_collision,
            self_collision_filter_distances=self_collision_filter_distance,
            solver_position_iteration_counts=solver_position_iteration_count,
        )

        # add deformable material
        if deformable_material is not None:
            self.apply_deformable_material(deformable_material)

        _SinglePrimWrapper.__init__(self, view=self._deformable_prim_view)

    """
    Properties.
    """

    @property
    def mesh(self) -> UsdGeom.Mesh:
        """
        Returns:
            Usd.Prim: USD Prim object that this object tracks.
        """
        return self._mesh

    """
    Operations- State.
    """

    def get_current_dynamic_state(self) -> DynamicState:
        """Return the DynamicState that contains the position and orientation of the underlying xform prim

        Returns:
            DynamicState:
                position (np.ndarray, optional):
                            position in the world frame of the prim. shape is (3, ).
                            Defaults to None, which means left unchanged.
                orientation (np.ndarray, optional):
                            quaternion orientation in the world frame of the prim.
                            quaternion is scalar-first (w, x, y, z). shape is (4, ).
                            Defaults to None, which means left unchanged.
        """
        position, orientation = self.get_world_pose()
        return DynamicState(position=position, orientation=orientation)

    def _get_points_pose(self):
        """Return the position of the points of the deformable prim with respect to the center of the deformable prim

        Returns:
            Union[np.ndarray, torch.Tensor]: position of the points that the deformable is composed of.
        """
        points = self._prim.GetAttribute("points").Get()
        if points is None:
            raise Exception(f"The prim {self.name} does not have points attribute.")
        return self._backend_utils.create_tensor_from_list(
            self._prim.GetAttribute("points").Get(), dtype="float32", device=self._device
        )

    def apply_deformable_material(self, deformable_materials: DeformableMaterial) -> None:
        self._deformable_prim_view.apply_deformable_materials(deformable_materials)

    def get_applied_deformable_material(self) -> DeformableMaterial:
        return self._deformable_prim_view.get_applied_deformable_materials()[0]

    """
    Operations- Setters.
    """

    def set_vertex_velocity_damping(self, vertex_velocity_damping: float) -> None:
        """
        Args:
            vertex_velocity_damping(float): deformable body vertex velocity damping parameter.
        """
        self._deformable_prim_view.set_vertex_velocity_dampings(
            self._backend_utils.create_tensor_from_list([vertex_velocity_damping], dtype="float32")
        )

    def set_sleep_damping(self, sleep_damping: float) -> None:
        """
        Args:
            sleep_damping(float): deformable body sleep damping parameter.
        """
        self._deformable_prim_view.set_sleep_dampings(
            self._backend_utils.create_tensor_from_list([sleep_damping], dtype="float32")
        )

    def set_sleep_threshold(self, sleep_threshold: float) -> None:
        """
        Args:
            sleep_threshold(float): deformable body sleep threshold parameter.
        """
        self._deformable_prim_view.set_sleep_thresholds(
            self._backend_utils.create_tensor_from_list([sleep_threshold], dtype="float32")
        )

    def set_settling_threshold(self, settling_threshold: float) -> None:
        """
        Args:
            settling_threshold(float): deformable body settling threshold parameter.
        """
        self._deformable_prim_view.set_settling_thresholds(
            self._backend_utils.create_tensor_from_list([settling_threshold], dtype="float32")
        )

    def set_self_collision_filter_distance(self, self_collision_filter_distance: float) -> None:
        """
        Args:
            self_collision_filter_distance(float): deformable body self collision filter distance parameter.
        """
        self._deformable_prim_view.set_self_collision_filter_distances(
            self._backend_utils.create_tensor_from_list([self_collision_filter_distance], dtype="float32")
        )

    def set_self_collision(self, self_collision: bool) -> None:
        """
        Args:
            self_collision(bool): deformable body self collision parameter.
        """
        self._deformable_prim_view.set_self_collisions(
            self._backend_utils.create_tensor_from_list([self_collision], dtype="bool")
        )

    def set_solver_position_iteration_count(self, iterations: int) -> None:
        """
        Args:
            iterations(float): solver position iteration count
        """
        self._deformable_prim_view.set_solver_position_iteration_counts(
            self._backend_utils.create_tensor_from_list([iterations], dtype="int32")
        )

    """
    Operations- Getters.
    """

    def get_simulation_mesh_rest_points(self) -> Union[np.ndarray, torch.Tensor]:
        """
        Gets the simulation mesh rest positions

        Returns:
            Union[np.ndarray, torch.Tensor]: simulation mesh vertices rest positions
        """
        return self._deformable_prim_view.get_simulation_mesh_rest_points()[0]

    def get_simulation_mesh_indices(self) -> Union[np.ndarray, torch.Tensor]:
        """
        Gets the simulation mesh element indices

        Returns:
            Union[np.ndarray, torch.Tensor]: simulation mesh element indices
        """
        return self._deformable_prim_view.get_simulation_mesh_indices()[0]

    # def get_collision_mesh_rest_points(self) -> Union[np.ndarray, torch.Tensor]:
    #     """
    #     Gets the collision mesh rest positions

    #     Returns:
    #         Union[np.ndarray, torch.Tensor]: collision mesh vertices rest positions
    #     """
    #     return self._deformable_prim_view.get_collision_mesh_rest_points()[0]

    def get_collision_mesh_indices(self) -> Union[np.ndarray, torch.Tensor]:
        """
        Gets the collision mesh element indices

        Returns:
            Union[np.ndarray, torch.Tensor]: collision mesh element indices
        """
        return self._deformable_prim_view.get_collision_mesh_indices()[0]

    def get_solver_position_iteration_count(self) -> int:
        """
        Gets the solver's positional iteration count

        Returns:
            int: positional iteration count
        """
        return self._deformable_prim_view.get_solver_position_iteration_counts()[0]

    def get_vertex_velocity_damping(self) -> float:
        """
        Gets the deformable body vertex velocity damping parameter

        Returns:
            float: vertex velocity damping
        """
        return self._deformable_prim_view.get_vertex_velocity_dampings()[0]

    def get_sleep_damping(self) -> float:
        """
        Gets the deformable body sleep damping parameter

        Returns:
            float: sleep damping
        """
        return self._deformable_prim_view.get_sleep_dampings()[0]

    def get_sleep_threshold(self) -> float:
        """
        Gets the deformable body sleep threshold

        Returns:
            float: sleep threshold
        """
        return self._deformable_prim_view.get_sleep_thresholds()[0]

    def get_settling_threshold(self) -> float:
        """
        Gets the deformable body settling threshold

        Returns:
            float: settling threshold
        """
        return self._deformable_prim_view.get_settling_thresholds()[0]

    def get_self_collision_filter_distance(self) -> float:
        """
        Gets the deformable body self collision filter distance

        Returns:
            float: self collision filter distance
        """
        return self._deformable_prim_view.get_self_collision_filter_distances()[0]

    def get_self_collision(self) -> bool:
        """
        Gets the deformable body self collision

        Returns:
            float: self collision
        """
        return self._deformable_prim_view.get_self_collisions()[0]
