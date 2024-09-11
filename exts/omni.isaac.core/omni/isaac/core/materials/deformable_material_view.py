# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from typing import Optional, Tuple, Union

# omniverse
import carb
import numpy as np

# isaac-core
import omni.kit.app
import torch
from omni.isaac.core.simulation_context.simulation_context import SimulationContext
from omni.isaac.core.utils.prims import find_matching_prim_paths, get_prim_at_path, is_prim_path_valid
from pxr import PhysxSchema


class DeformableMaterialView:
    """The view class to deal with deformableMaterial prims.
    Provides high level functions to deal with deformable material (1 or more deformable materials)
    as well as its attributes/ properties. This object wraps all matching materials found at the regex provided at the prim_paths_expr.
    This object wraps all matching materials Prims found at the regex provided at the prim_paths_expr.
    """

    def __init__(
        self,
        prim_paths_expr: str,
        name: str = "deformable_material_view",
        dynamic_frictions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        youngs_moduli: Optional[Union[np.ndarray, torch.Tensor]] = None,
        poissons_ratios: Optional[Union[np.ndarray, torch.Tensor]] = None,
        elasticity_dampings: Optional[Union[np.ndarray, torch.Tensor]] = None,
        damping_scales: Optional[Union[np.ndarray, torch.Tensor]] = None,
    ):
        """
        Args:
            prim_paths_expr(str): Prim paths regex to encapsulate all prims that match it.
            name(str): Shortname to be used as a key by Scene class.
            dynamic_frictions (Union[np.ndarray, torch.Tensor], optional): The dynamic friction coefficient tensor, shape is (N, ).
            youngs_moduli (Union[np.ndarray, torch.Tensor], optional): The Young's modulus coefficient tensor, shape is (N, ).
            poissons_ratios (Union[np.ndarray, torch.Tensor], optional): The Possion ratio coefficient tensor, shape is (N, ).
            elasticity_dampings (Union[np.ndarray, torch.Tensor], optional): Material damping parameter tensor, shape is (N, ).
            damping_scales (Union[np.ndarray, torch.Tensor], optional): The damping scale coefficient tensor, shape is (N, ).
        """
        self._name = name
        self._physics_view = None
        self._prim_paths = find_matching_prim_paths(prim_paths_expr)
        if len(self._prim_paths) == 0:
            raise Exception(
                "Prim path expression {} is invalid, a prim matching the expression needs to created before wrapping it as view".format(
                    prim_paths_expr
                )
            )
        self._count = len(self._prim_paths)
        self._prims = []
        self._regex_prim_paths = prim_paths_expr
        for prim_path in self._prim_paths:
            self._prims.append(get_prim_at_path(prim_path))

        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._backend_utils = SimulationContext.instance().backend_utils
        else:
            import omni.isaac.core.utils.numpy as np_utils

            self._backend = "numpy"
            self._backend_utils = np_utils

        # NOTE: deformableSystemView is only supported on the host
        self._device = "cpu"
        self._material_apis = [None] * self._count

        # set properties
        if dynamic_frictions is not None:
            self.set_dynamic_frictions(dynamic_frictions)
        if youngs_moduli is not None:
            self.set_youngs_moduli(youngs_moduli)
        if poissons_ratios is not None:
            self.set_poissons_ratios(poissons_ratios)
        if elasticity_dampings is not None:
            self.set_elasticity_dampings(elasticity_dampings)
        if damping_scales is not None:
            self.set_damping_scales(damping_scales)

    """
    Properties.
    """

    @property
    def count(self) -> int:
        """
        Returns:
            int: number of rigid shapes for the prims in the view.
        """
        return self._count

    @property
    def name(self) -> str:
        """
        Returns:
            str: name given to the view when instantiating it.
        """
        return self._name

    def _apply_material_api(self, index):
        if self._material_apis[index] is None:
            if self._prims[index].HasAPI(PhysxSchema.PhysxDeformableBodyMaterialAPI):
                material_api = PhysxSchema.PhysxDeformableBodyMaterialAPI(self._prims[index])
            else:
                material_api = PhysxSchema.PhysxDeformableBodyMaterialAPI.Apply(self._prims[index])
            self._material_apis[index] = material_api

    def is_physics_handle_valid(self) -> bool:
        """
        Returns:
            bool: True if the physics handle of the view is valid (i.e physics is initialized for the view). Otherwise False.
        """
        return self._physics_view is not None

    def is_valid(self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None) -> bool:
        """
        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            bool: True if all prim paths specified in the view correspond to a valid prim in stage. False otherwise.
        """

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = True
        for index in indices:
            result = result and is_prim_path_valid(self._prim_paths[index.tolist()])
        return result

    def post_reset(self) -> None:
        """Resets the deformables to their initial states."""
        # TODO:
        return

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates a rigid body view in physX.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.
        """
        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
        carb.log_info("initializing view for {}".format(self._name))
        self._physics_sim_view = physics_sim_view
        self._physics_view = self._physics_sim_view.create_soft_body_material_view(
            self._regex_prim_paths.replace(".*", "*")
        )
        self._count = self._physics_view.count
        carb.log_info("Deformable material View Device: {}".format(self._device))
        return

    def _invalidate_physics_handle_callback(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physics_view = None
        return

    def set_dynamic_frictions(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the dynamic friction for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material dynamic friction tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_dynamic_frictions(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_dynamic_friction(current_values, indices)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxDeformableBodyMaterial:dynamicFriction" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateDynamicFrictionAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetDynamicFrictionAttr().Set(values[idx_count].tolist())

                idx_count += 1

    def get_dynamic_frictions(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the dynamic friction of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
           Union[np.ndarray, torch.Tensor]: dynamic friction tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            current_values = self._physics_view.get_dynamic_friction()
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxDeformableBodyMaterial:dynamicFriction" not in self._prims[i.tolist()].GetPropertyNames():
                    result[write_idx] = self._material_apis[i.tolist()].CreateDynamicFrictionAttr().Get()
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetDynamicFrictionAttr().Get()
                write_idx += 1
            return result

    def set_elasticity_dampings(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the elasticity_dampings for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material damping tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_elasticity_dampings(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_damping(current_values, indices)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxDeformableBodyMaterial:elasticityDamping" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateElasticityDampingAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetElasticityDampingAttr().Set(values[idx_count].tolist())

                idx_count += 1

    def get_elasticity_dampings(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the elasticity dampings of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: elasticity dampings tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            current_values = self._physics_view.get_damping()
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxDeformableBodyMaterial:elasticityDamping" not in self._prims[i.tolist()].GetPropertyNames():
                    result[write_idx] = self._material_apis[i.tolist()].CreateElasticityDampingAttr().Get()
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetElasticityDampingAttr().Get()
                write_idx += 1
            return result

    def set_damping_scales(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the damping scale for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material damping scale tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_damping_scales(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_damping_scale(current_values, indices)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxDeformableBodyMaterial:dampingScale" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateDampingScaleAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetDampingScaleAttr().Set(values[idx_count].tolist())
                idx_count += 1

    def get_damping_scales(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the damping scale of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: damping scale tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            current_values = self._physics_view.get_damping_scale()
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxDeformableBodyMaterial:dampingScale" not in self._prims[i.tolist()].GetPropertyNames():
                    result[write_idx] = self._material_apis[i.tolist()].CreateDampingScaleAttr().Get()
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetDampingScaleAttr().Get()
                write_idx += 1
            return result

    def set_poissons_ratios(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the poissons ratios for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material poissons ratio tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_poissons_ratios(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_poissons_ratio(current_values, indices)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxDeformableBodyMaterial:poissonsRatio" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreatePoissonsRatioAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetPoissonsRatioAttr().Set(values[idx_count].tolist())
                idx_count += 1

    def get_poissons_ratios(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the poissons ratios of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: poissons ratio tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            current_values = self._physics_view.get_poissons_ratio()
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxDeformableBodyMaterial:poissonsRatio" not in self._prims[i.tolist()].GetPropertyNames():
                    result[write_idx] = self._material_apis[i.tolist()].CreatePoissonsRatioAttr().Get()
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetPoissonsRatioAttr().Get()
                write_idx += 1
            return result

    def set_youngs_moduli(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the youngs moduli for the material prims indicated by the indices.

        Args:
            values (Optional[Union[np.ndarray, torch.Tensor]], optional): material drag tensor with the shape (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_youngs_moduli(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_youngs_modulus(current_values, indices)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxDeformableBodyMaterial:youngsModulus" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateYoungsModulusAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetYoungsModulusAttr().Set(values[idx_count].tolist())
                idx_count += 1

    def get_youngs_moduli(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the Youngs moduli of materials indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which material prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: Youngs moduli tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            current_values = self._physics_view.get_youngs_modulus()
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxDeformableBodyMaterial:youngsModulus" not in self._prims[i.tolist()].GetPropertyNames():
                    result[write_idx] = self._material_apis[i.tolist()].CreateYoungsModulusAttr().Get()
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetYoungsModulusAttr().Get()
                write_idx += 1
            return result
