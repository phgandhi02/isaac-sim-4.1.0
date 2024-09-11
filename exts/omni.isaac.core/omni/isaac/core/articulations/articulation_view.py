# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import gc
from collections import OrderedDict
from typing import List, Optional, Tuple, Union

import carb
import numpy as np
import omni.kit.app
import omni.physx
import torch
import warp as wp
from omni.isaac.core import SimulationContext
from omni.isaac.core.prims.xform_prim_view import XFormPrimView
from omni.isaac.core.utils.prims import (
    get_articulation_root_api_prim_path,
    get_prim_at_path,
    get_prim_parent,
    get_prim_property,
    set_prim_property,
)
from omni.isaac.core.utils.types import ArticulationActions, JointsState, XFormPrimViewState
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics


class ArticulationView(XFormPrimView):
    """High level wrapper to deal with prims (one or many) that have the Root Articulation API applied
    and their attributes/properties

    This class wraps all matching articulations found at the regex provided at the ``prim_paths_expr`` argument

    .. note::

        Each prim will have ``xformOp:orient``, ``xformOp:translate`` and ``xformOp:scale`` only post-init,
        unless it is a non-root articulation link.

    .. warning::

        The articulation view object must be initialized in order to be able to operate on it.
        See the ``initialize`` method for more details.

    Args:
        prim_paths_expr (Union[str, List[str]]): prim paths regex to encapsulate all prims that match it.
                                example: "/World/Env[1-5]/Franka" will match /World/Env1/Franka,
                                /World/Env2/Franka..etc.
                                (a non regex prim path can also be used to encapsulate one rigid prim).
        name (str, optional): shortname to be used as a key by Scene class.
                                Note: needs to be unique if the object is added to the Scene.
                                Defaults to "articulation_prim_view".
        positions (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): default positions in the world frame of the prims.
                                                                        shape is (N, 3). Defaults to None, which means left unchanged.
        translations (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional):
                                                        default translations in the local frame of the prims
                                                        (with respect to its parent prims). shape is (N, 3).
                                                        Defaults to None, which means left unchanged.
        orientations (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional):
                                                        default quaternion orientations in the world/ local frame of the prims
                                                        (depends if translation or position is specified).
                                                        quaternion is scalar-first (w, x, y, z). shape is (N, 4).
        scales (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): local scales to be applied to
                                                        the prim's dimensions in the view. shape is (N, 3).
                                                        Defaults to None, which means left unchanged.
        visibilities (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): set to false for an invisible prim in
                                                                            the stage while rendering. shape is (N,).
                                                                            Defaults to None.
        reset_xform_properties (bool, optional): True if the prims don't have the right set of xform properties
                                                (i.e: translate, orient and scale) ONLY and in that order.
                                                Set this parameter to False if the object were cloned using using
                                                the cloner api in omni.isaac.cloner. Defaults to True.

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.stage as stage_utils
        >>> from omni.isaac.cloner import GridCloner
        >>> from omni.isaac.core.articulations import ArticulationView
        >>> from pxr import UsdGeom
        >>>
        >>> usd_path = "/home/<user>/Documents/Assets/Robots/Franka/franka_alt_fingers.usd"
        >>> env_zero_path = "/World/envs/env_0"
        >>> num_envs = 5
        >>>
        >>> # load the Franka Panda robot USD file
        >>> stage_utils.add_reference_to_stage(usd_path, prim_path=f"{env_zero_path}/panda")  # /World/envs/env_0/panda
        >>>
        >>> # clone the environment (num_envs)
        >>> cloner = GridCloner(spacing=1.5)
        >>> cloner.define_base_env(env_zero_path)
        >>> UsdGeom.Xform.Define(stage_utils.get_current_stage(), env_zero_path)
        >>> cloner.clone(source_prim_path=env_zero_path, prim_paths=cloner.generate_paths("/World/envs/env", num_envs))
        >>>
        >>> # wrap all articulations
        >>> prims = ArticulationView(prim_paths_expr="/World/envs/env.*/panda", name="franka_panda_view")
        >>> prims
        <omni.isaac.core.articulations.articulation_view.ArticulationView object at 0x7ff174054b20>
    """

    def __init__(
        self,
        prim_paths_expr: Union[str, List[str]],
        name: str = "articulation_prim_view",
        positions: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        translations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        scales: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        visibilities: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        reset_xform_properties: bool = True,
    ) -> None:
        self._physics_view = None
        if isinstance(prim_paths_expr, list):
            prim_paths_expr = [
                get_articulation_root_api_prim_path(prim_paths_expression) for prim_paths_expression in prim_paths_expr
            ]
        else:
            prim_paths_expr = get_articulation_root_api_prim_path(prim_paths_expr)
        XFormPrimView.__init__(
            self,
            prim_paths_expr=prim_paths_expr,
            name=name,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            visibilities=visibilities,
            reset_xform_properties=reset_xform_properties,
        )
        self._is_initialized = False
        self._num_dof = None
        self._dof_paths = None
        self._default_joints_state = None
        self._dofs_infos = OrderedDict()
        self._dof_names = None
        self._body_names = None
        self._body_indices = None
        self._dof_indices = None
        self._dof_types = None
        self._metadata = None
        self._paused_motion = False
        self._paused_position_targets = None
        self._paused_velocity_targets = None
        self._paused_dof_velocities = None

    def __del__(self):
        del self._physics_view
        self._invalidate_physics_handle_event = None

    @property
    def num_dof(self) -> int:
        """Number of DOF of the articulations

        Returns:
            int: maximum number of DOFs for the articulations in the view

        Example:

        .. code-block:: python

            >>> prims.num_dof
            9
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._num_dof

    @property
    def num_bodies(self) -> int:
        """Number of rigid bodies (links) of the articulations

        Returns:
            int: maximum number of rigid bodies for the articulations in the view

        Example:

        .. code-block:: python

            >>> prims.num_bodies
            12
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._num_bodies

    @property
    def num_shapes(self) -> int:
        """Number of rigid shapes of the articulations

        Returns:
            int: maximum number of rigid shapes for the articulations in the view

        Example:

        .. code-block:: python

            >>> prims.num_shapes
            17
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._num_shapes

    @property
    def num_joints(self) -> int:
        """Number of joints of the articulations

        Returns:
            int: number of joints of the articulations in the view

        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._num_joints

    @property
    def num_fixed_tendons(self) -> int:
        """Number of fixed tendons of the articulations

        Returns:
            int: maximum number of fixed tendons for the articulations in the view

        Example:

        .. code-block:: python

            >>> prims.num_fixed_tendons
            0
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._num_fixed_tendons

    @property
    def body_names(self) -> List[str]:
        """List of prim names for each rigid body (link) of the articulations

        Returns:
            List[str]: ordered names of bodies that corresponds to links for the articulations in the view

        Example:

        .. code-block:: python

            >>> prims.body_names
            ['panda_link0', 'panda_link1', 'panda_link2', 'panda_link3', 'panda_link4', 'panda_link5',
             'panda_link6', 'panda_link7', 'panda_link8', 'panda_hand', 'panda_leftfinger', 'panda_rightfinger']
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._body_names

    @property
    def dof_names(self) -> List[str]:
        """List of prim names for each DOF of the articulations

        Returns:
            List[str]: ordered names of joints that corresponds to degrees of freedom for the articulations in the view

        Example:

        .. code-block:: python

            >>> prims.dof_names
            ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
             'panda_joint6', 'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2']
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._dof_names

    @property
    def joint_names(self) -> List[str]:
        """List of prim names for each joint of the articulations

        Returns:
            List[str]: ordered names of joints that corresponds to degrees of freedom for the articulations in the view

        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._joint_names

    @property
    def initialized(self) -> bool:
        """Check if articulation view is initialized

        Returns:
            bool: True if the view object was initialized (after the first call of .initialize()). False otherwise.

        Example:

        .. code-block:: python

            >>> # given an initialized articulation view
            >>> prims.initialized
            True
        """
        return self._is_initialized

    def is_physics_handle_valid(self) -> bool:
        """Check if articulation view's physics handler is initialized

        .. warning::

            If the physics handler is not valid many of the methods that requires PhysX will return None.

        Returns:
            bool: False if .initialize() needs to be called again for the physics handle to be valid. Otherwise True

        Example:

        .. code-block:: python

            >>> prims.is_physics_handle_valid()
            True
        """
        return self._physics_view is not None

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and set other properties using the PhysX tensor API

        .. note::

            If the articulation view has been added to the world scene (e.g., ``world.scene.add(prims)``),
            it will be automatically initialized when the world is reset (e.g., ``world.reset()``).

        .. warning::

            This method needs to be called after each hard reset (e.g., Stop + Play on the timeline)
            before interacting with any other class method.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.

        Example:

        .. code-block:: python

            >>> prims.initialize()
        """
        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
        carb.log_info("initializing view for {}".format(self._name))
        self._physics_view = physics_sim_view.create_articulation_view(
            [regular_expression.replace(".*", "*") for regular_expression in self._regex_prim_paths]
        )
        assert self._physics_view.is_homogeneous
        self._physics_sim_view = physics_sim_view
        if not self._is_initialized:
            self._metadata = self._physics_view.shared_metatype
            self._num_dof = self._physics_view.max_dofs
            self._num_bodies = self._physics_view.max_links
            self._num_shapes = self._physics_view.max_shapes
            self._num_fixed_tendons = self._physics_view.max_fixed_tendons
            self._body_names = self._metadata.link_names
            self._body_indices = dict(zip(self._body_names, range(len(self._body_names))))
            self._dof_names = self._metadata.dof_names
            self._dof_indices = self._metadata.dof_indices
            self._dof_types = self._metadata.dof_types
            self._dof_paths = self._physics_view.dof_paths
            self._joint_indices = self._metadata.joint_indices
            self._joint_names = self._metadata.joint_names
            self._joint_types = self._metadata.joint_types
            self._num_joints = self._metadata.joint_count
            self._link_indices = self._metadata.link_indices
            self._prim_paths = self._physics_view.prim_paths
            carb.log_info("Articulation Prim View Device: {}".format(self._device))
            self._is_initialized = True
            self._default_kps, self._default_kds = self.get_gains(clone=True)
            default_actions = self.get_applied_actions(clone=True)
            # TODO: implement effort part
            default_positions, default_orientations = self.get_world_poses()
            if self._backend == "warp":
                self._default_state = XFormPrimViewState(
                    positions=default_positions.data, orientations=default_orientations.data
                )
            else:
                self._default_state = XFormPrimViewState(positions=default_positions, orientations=default_orientations)
            if self._default_joints_state is None:
                self._default_joints_state = JointsState(positions=None, velocities=None, efforts=None)
            if self._default_joints_state.positions is None:
                self._default_joints_state.positions = default_actions.joint_positions
            if self._default_joints_state.velocities is None:
                self._default_joints_state.velocities = default_actions.joint_velocities
            if self._default_joints_state.efforts is None:
                self._default_joints_state.efforts = self._backend_utils.create_zeros_tensor(
                    shape=[self.count, self.num_dof], dtype="float32", device=self._device
                )
        timeline = omni.timeline.get_timeline_interface()
        self._invalidate_physics_handle_event = timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._invalidate_physics_handle_callback
        )

    def _invalidate_physics_handle_callback(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physics_view = None
            self._invalidate_physics_handle_event = None
        return

    def get_body_index(self, body_name: str) -> int:
        """Get a ridig body (link) index in the articulation view given its name

        Args:
            body_name (str): name of the ridig body to query

        Returns:
            int: index of the rigid body in the articulation buffers

        Example:

        .. code-block:: python

            >>> # get the index of the left finger: panda_leftfinger
            >>> prims.get_body_index("panda_leftfinger")
            10
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._body_indices[body_name]

    def get_dof_index(self, dof_name: str) -> int:
        """Get a DOF index in the joint buffers given its name

        Args:
            dof_name (str): name of the joint that corresponds to the degree of freedom to query

        Returns:
            int: index of the degree of freedom in the joint buffers

        Example:

        .. code-block:: python

            >>> # get the index of the left finger joint: panda_finger_joint1
            >>> prims.get_dof_index("panda_finger_joint1")
            7
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._dof_indices[dof_name]

    def get_dof_types(self, dof_names: List[str] = None) -> List[str]:
        """Get the DOF types given the DOF names

        Args:
            dof_names (List[str], optional): names of the joints that corresponds to the degrees of freedom to query. Defaults to None.

        Returns:
            List[str]: types of the joints that corresponds to the degrees of freedom. Types can be invalid, translation or rotation.

        Example:

        .. code-block:: python

            >>> # get all DOF types
            >>> prims.get_dof_types()
            [<DofType.Rotation: 0>, <DofType.Rotation: 0>, <DofType.Rotation: 0>,
             <DofType.Rotation: 0>, <DofType.Rotation: 0>, <DofType.Rotation: 0>,
             <DofType.Rotation: 0>, <DofType.Translation: 1>, <DofType.Translation: 1>]
            >>>
            >>> # get only the finger DOF types: panda_finger_joint1 and panda_finger_joint2
            >>> prims.get_dof_types(dof_names=["panda_finger_joint1", "panda_finger_joint2"])
            [<DofType.Translation: 1>, <DofType.Translation: 1>]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if dof_names is None:
            return self._dof_types
        else:
            return [self._dof_types[self.get_dof_index(dof_name)] for dof_name in dof_names]

    def get_dof_limits(self) -> Union[np.ndarray, torch.Tensor]:
        """Get the articulations DOFs limits (lower and upper)

        Returns:
            Union[np.ndarray, torch.Tensor, wp.array]: degrees of freedom position limits.
            Shape is (N, num_dof, 2). For the last dimension, index 0 corresponds to lower limits and index 1 corresponds to upper limits

        Example:

        .. code-block:: python

            >>> # get DOF limits. Returned shape is (5, 9, 2) for the example: 5 envs, 9 DOFs
            >>> prims.get_dof_limits()
            [[[-2.8973  2.8973]
             [-1.7628  1.7628]
             [-2.8973  2.8973]
             [-3.0718 -0.0698]
             [-2.8973  2.8973]
             [-0.0175  3.7525]
             [-2.8973  2.8973]
             [ 0.      0.04  ]
             [ 0.      0.04  ]]
            ...
            [[-2.8973  2.8973]
             [-1.7628  1.7628]
             [-2.8973  2.8973]
             [-3.0718 -0.0698]
             [-2.8973  2.8973]
             [-0.0175  3.7525]
             [-2.8973  2.8973]
             [ 0.      0.04  ]
             [ 0.      0.04  ]]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._physics_view.get_dof_limits()

    def get_drive_types(self) -> Union[np.ndarray, torch.Tensor]:
        """Get the articulations DOFs limits (lower and upper)

        Returns:
            Union[np.ndarray, torch.Tensor, wp.array]: degrees of freedom position limits.
            Shape is (N, num_dof). For the last dimension, index 0 corresponds to lower limits and index 1 corresponds to upper limits

        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._physics_view.get_drive_types()

    def get_joint_index(self, joint_name: str) -> int:
        """Get a joint index in the joint buffers given its name

        Args:
            joint_name (str): name of the joint that corresponds to the index of the joint in the articulation

        Returns:
            int: index of the joint in the joint buffers

        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._joint_indices[joint_name]

    def get_link_index(self, link_name: str) -> int:
        """Get a link index in the link buffers given its name

        Args:
            link_name (str): name of the link that corresponds to the index of the link in the articulation

        Returns:
            int: index of the link in the link buffers

        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._link_indices[link_name]

    def set_friction_coefficients(
        self,
        values: Union[np.ndarray, torch.Tensor],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the friction coefficients for articulation joints in the view

        Search for *"Joint Friction Coefficient"* in |physx_docs| for more details.

        Args:
            values (Union[np.ndarray, torch.Tensor, wp.array]): friction coefficients for articulation joints in the view. shape (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        Example:

        .. code-block:: python

            >>> # set all joint friction coefficients to 0.05 for all envs
            >>> prims.set_friction_coefficients(np.full((num_envs, prims.num_dof), 0.05))
            >>>
            >>> # set only the finger joint (panda_finger_joint1 (7) and panda_finger_joint2 (8)) friction coefficients
            >>> # for the first, middle and last of the 5 envs to 0.05
            >>> prims.set_friction_coefficients(np.full((3, 2), 0.05), indices=np.array([0,2,4]), joint_indices=np.array([7,8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, "cpu")
            new_values = self._physics_view.get_dof_friction_coefficients()
            values = self._backend_utils.move_data(values, device="cpu")
            new_values = self._backend_utils.assign(
                values,
                new_values,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            self._physics_view.set_dof_friction_coefficients(new_values, indices)
        else:
            indices = self._backend_utils.to_list(
                self._backend_utils.resolve_indices(indices, self.count, self._device)
            )
            dof_types = self._backend_utils.to_list(self.get_dof_types())
            joint_indices = self._backend_utils.to_list(
                self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            )
            values = self._backend_utils.to_list(values)
            articulation_read_idx = 0
            for i in indices:
                dof_read_idx = 0
                for dof_index in joint_indices:
                    drive_type = (
                        "angular" if dof_types[dof_index] == omni.physics.tensors.DofType.Rotation else "linear"
                    )
                    prim = PhysxSchema.PhysxJointAPI(get_prim_at_path(self._dof_paths[i][dof_index]))
                    if not prim.GetJointFrictionAttr():
                        prim.CreateJointFrictionAttr().Set(values[articulation_read_idx][dof_read_idx])
                    else:
                        prim.GetJointFrictionAttr().Set(values[articulation_read_idx][dof_read_idx])
                    dof_read_idx += 1
                articulation_read_idx += 1
        return

    def get_friction_coefficients(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.array]:
        """Get the friction coefficients for the articulation joints in the view

        Search for *"Joint Friction Coefficient"* in |physx_docs| for more details.

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (Optional[bool]): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: joint friction coefficients for articulations in the view. shape (M, K).

        Example:

        .. code-block:: python

            >>> # get joint friction coefficients. Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> prims.get_friction_coefficients()
            [[0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]]
            >>>
            >>> # get only the finger joint (panda_finger_joint1 (7) and panda_finger_joint2 (8)) friction coefficients
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_friction_coefficients(indices=np.array([0,2,4]), joint_indices=np.array([7,8]))
            [[0. 0.]
             [0. 0.]
             [0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            values = self._backend_utils.move_data(self._physics_view.get_dof_friction_coefficients(), self._device)
            if clone:
                values = self._backend_utils.clone_tensor(values, device=self._device)
            result = values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            return result
        else:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            values = np.zeros(shape=(indices.shape[0], joint_indices.shape[0]), dtype="float32")
            articulation_write_idx = 0
            indices = self._backend_utils.to_list(indices)
            joint_indices = self._backend_utils.to_list(joint_indices)
            for i in indices:
                dof_write_idx = 0
                for dof_index in joint_indices:
                    prim = PhysxSchema.PhysxJointAPI(get_prim_at_path(self._dof_paths[i][dof_index]))
                    if prim.GetJointFrictionAttr().Get():
                        values[articulation_write_idx][dof_write_idx] = prim.GetJointFrictionAttr().Get()
                    dof_write_idx += 1
                articulation_write_idx += 1
            values = self._backend_utils.convert(values, dtype="float32", device=self._device, indexed=True)
            return values

    def set_armatures(
        self,
        values: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set armatures for articulation joints in the view

        Search for *"Joint Armature"* in |physx_docs| for more details.

        Args:
            values (Union[np.ndarray, torch.Tensor, wp.array]): armatures for articulation joints in the view. shape (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        Example:

        .. code-block:: python

            >>> # set all joint armatures to 0.05 for all envs
            >>> prims.set_armatures(np.full((num_envs, prims.num_dof), 0.05))
            >>>
            >>> # set only the finger joint (panda_finger_joint1 (7) and panda_finger_joint2 (8)) armatures
            >>> # for the first, middle and last of the 5 envs to 0.05
            >>> prims.set_armatures(np.full((3, 2), 0.05), indices=np.array([0,2,4]), joint_indices=np.array([7,8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, "cpu")
            new_values = self._physics_view.get_dof_armatures()
            values = self._backend_utils.move_data(values, device="cpu")
            new_values = self._backend_utils.assign(
                values,
                new_values,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            self._physics_view.set_dof_armatures(new_values, indices)
        else:
            indices = self._backend_utils.to_list(
                self._backend_utils.resolve_indices(indices, self.count, self._device)
            )
            joint_indices = self._backend_utils.to_list(
                self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            )
            values = self._backend_utils.to_list(values)
            articulation_read_idx = 0
            for i in indices:
                dof_read_idx = 0
                for dof_index in joint_indices:
                    prim = PhysxSchema.PhysxJointAPI(get_prim_at_path(self._dof_paths[i][dof_index]))
                    if not prim.GetArmatureAttr():
                        prim.CreateArmatureAttr().Set(values[articulation_read_idx][dof_read_idx])
                    else:
                        prim.GetArmatureAttr().Set(values[articulation_read_idx][dof_read_idx])
                    dof_read_idx += 1
                articulation_read_idx += 1
        return

    def get_armatures(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get armatures for articulation joints in the view

        Search for *"Joint Armature"* in |physx_docs| for more details.

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (Optional[bool]): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: joint armatures for articulations in the view. shape (M, K).

        Example:

        .. code-block:: python

            >>> # get joint armatures. Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> prims.get_armatures()
            [[0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]]
            >>>
            >>> # get only the finger joint (panda_finger_joint1 (7) and panda_finger_joint2 (8)) armatures
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_armatures(indices=np.array([0,2,4]), joint_indices=np.array([7,8]))
            [[0. 0.]
             [0. 0.]
             [0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            values = self._backend_utils.move_data(self._physics_view.get_dof_armatures(), device=self._device)
            if clone:
                values = self._backend_utils.clone_tensor(values, device=self._device)
            result = values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            return result
        else:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            values = np.zeros(shape=(indices.shape[0], joint_indices.shape[0]), dtype="float32")
            indices = self._backend_utils.to_list(indices)
            joint_indices = self._backend_utils.to_list(joint_indices)
            articulation_write_idx = 0
            for i in indices:
                dof_write_idx = 0
                for dof_index in joint_indices:
                    prim = PhysxSchema.PhysxJointAPI(get_prim_at_path(self._dof_paths[i][dof_index]))
                    if prim.GetArmatureAttr().Get():
                        values[articulation_write_idx, dof_write_idx] = prim.GetArmatureAttr().Get()
                    dof_write_idx += 1
                articulation_write_idx += 1
            values = self._backend_utils.convert(values, dtype="float32", device=self._device, indexed=True)
            return values

    def get_articulation_body_count(self) -> int:
        """Get the number of rigid bodies (links) of the articulations

        Returns:
            int: maximum number of rigid bodies (links) in the articulation

        Example:

        .. code-block:: python

            >>> prims.get_articulation_body_count()
            12
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._metadata.link_count

    def set_joint_position_targets(
        self,
        positions: Optional[Union[np.ndarray, torch.Tensor, wp.array]],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the joint position targets for the implicit Proportional-Derivative (PD) controllers

        .. note::

            This is an independent method for controlling joints. To apply multiple targets (position, velocity,
            and/or effort) in the same call, consider using the ``apply_action`` method

        Args:
            positions (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): joint position targets for the implicit PD controller.
                                                                    shape is (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        .. hint::

            High stiffness makes the joints snap faster and harder to the desired target,
            and higher damping smoothes but also slows down the joint's movement to target

            * For position control, set relatively high stiffness and low damping (to reduce vibrations)

        Example:

        .. code-block:: python

            >>> # apply the target positions (to move all the robot joints) to the indicated values.
            >>> # Since there are 5 envs, the joint positions are repeated 5 times
            >>> positions = np.tile(np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04]), (num_envs, 1))
            >>> prims.set_joint_position_targets(positions)
            >>>
            >>> # close the robot fingers: panda_finger_joint1 (7) and panda_finger_joint2 (8) to 0.0
            >>> # for the first, middle and last of the 5 envs
            >>> positions = np.tile(np.array([0.0, 0.0]), (3, 1))
            >>> prims.set_joint_position_targets(positions, indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            action = self._physics_view.get_dof_position_targets()
            action = self._backend_utils.assign(
                self._backend_utils.move_data(positions, device=self._device),
                action,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            self._physics_view.set_dof_position_targets(action, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_joint_position_targets")

    def set_joint_positions(
        self,
        positions: Optional[Union[np.ndarray, torch.Tensor, wp.array]],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the joint positions of articulations in the view

        .. warning::

            This method will immediately set (teleport) the affected joints to the indicated value.
            Use the ``set_joint_position_targets`` or the ``apply_action`` methods to control the articulation joints.

        Args:
            positions (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): joint positions of articulations in the view to be set to in the next frame.
                                                                    shape is (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        .. hint::

            This method belongs to the methods used to set the articulation kinematic states:

            ``set_velocities`` (``set_linear_velocities``, ``set_angular_velocities``),
            ``set_joint_positions``, ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> # set all the articulation joints.
            >>> # Since there are 5 envs, the joint positions are repeated 5 times
            >>> positions = np.tile(np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04]), (num_envs, 1))
            >>> prims.set_joint_positions(positions)
            >>>
            >>> # set only the fingers in closed position: panda_finger_joint1 (7) and panda_finger_joint2 (8) to 0.0
            >>> # for the first, middle and last of the 5 envs
            >>> positions = np.tile(np.array([0.0, 0.0]), (3, 1))
            >>> prims.set_joint_positions(positions, indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            new_dof_pos = self._physics_view.get_dof_positions()
            new_dof_pos = self._backend_utils.assign(
                self._backend_utils.move_data(positions, device=self._device),
                new_dof_pos,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            self._physics_view.set_dof_positions(new_dof_pos, indices)
            self._physics_view.set_dof_position_targets(new_dof_pos, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_joint_positions")

    def set_joint_velocity_targets(
        self,
        velocities: Optional[Union[np.ndarray, torch.Tensor, wp.array]],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the joint velocity targets for the implicit Proportional-Derivative (PD) controllers

        .. note::

            This is an independent method for controlling joints. To apply multiple targets (position, velocity,
            and/or effort) in the same call, consider using the ``apply_action`` method

        Args:
            velocities (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): joint velocity targets for the implicit PD controller.
                                                                    shape is (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        .. hint::

            High stiffness makes the joints snap faster and harder to the desired target,
            and higher damping smoothes but also slows down the joint's movement to target

            * For velocity control, stiffness must be set to zero with a non-zero damping

        Example:

        .. code-block:: python

            >>> # apply the target velocities for all the articulation joints to the indicated values.
            >>> # Since there are 5 envs, the joint velocities are repeated 5 times
            >>> velocities = np.tile(np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]), (num_envs, 1))
            >>> prims.set_joint_velocity_targets(velocities)
            >>>
            >>> # apply the fingers target velocities: panda_finger_joint1 (7) and panda_finger_joint2 (8) to -1.0
            >>> # for the first, middle and last of the 5 envs
            >>> velocities = np.tile(np.array([-0.1, -0.1]), (3, 1))
            >>> prims.set_joint_velocity_targets(velocities, indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            action = self._physics_view.get_dof_velocity_targets()
            action = self._backend_utils.assign(
                self._backend_utils.move_data(velocities, device=self._device),
                action,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            self._physics_view.set_dof_velocity_targets(action, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_joint_velocity_targets")

    def set_joint_velocities(
        self,
        velocities: Optional[Union[np.ndarray, torch.Tensor, wp.array]],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the joint velocities of articulations in the view

        .. warning::

            This method will immediately set the affected joints to the indicated value.
            Use the ``set_joint_velocity_targets`` or the ``apply_action`` methods to control the articulation joints.

        Args:
            velocities (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): joint velocities of articulations in the view to be set to in the next frame.
                                                                    shape is (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        .. hint::

            This method belongs to the methods used to set the articulation kinematic states:

            ``set_velocities`` (``set_linear_velocities``, ``set_angular_velocities``),
            ``set_joint_positions``, ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> # set the velocities for all the articulation joints to the indicated values.
            >>> # Since there are 5 envs, the joint velocities are repeated 5 times
            >>> velocities = np.tile(np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]), (num_envs, 1))
            >>> prims.set_joint_velocities(velocities)
            >>>
            >>> # set the fingers velocities: panda_finger_joint1 (7) and panda_finger_joint2 (8) to -0.1
            >>> # for the first, middle and last of the 5 envs
            >>> velocities = np.tile(np.array([-0.1, -0.1]), (3, 1))
            >>> prims.set_joint_velocities(velocities, indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            new_dof_vel = self._physics_view.get_dof_velocities()
            new_dof_vel = self._backend_utils.assign(
                self._backend_utils.move_data(velocities, device=self._device),
                new_dof_vel,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            self._physics_view.set_dof_velocities(new_dof_vel, indices)
            self._physics_view.set_dof_velocity_targets(new_dof_vel, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_joint_velocities")
        return

    def set_joint_efforts(
        self,
        efforts: Optional[Union[np.ndarray, torch.Tensor, wp.array]],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the joint efforts of articulations in the view

        .. note::

            This method can be used for effort control. For this purpose, there must be no joint drive
            or the stiffness and damping must be set to zero.

        Args:
            efforts (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): efforts of articulations in the view to be set to in the next frame.
                                                                    shape is (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        .. hint::

            This method belongs to the methods used to set the articulation kinematic states:

            ``set_velocities`` (``set_linear_velocities``, ``set_angular_velocities``),
            ``set_joint_positions``, ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> # set the efforts for all the articulation joints to the indicated values.
            >>> # Since there are 5 envs, the joint efforts are repeated 5 times
            >>> efforts = np.tile(np.array([10, 20, 30, 40, 50, 60, 70, 80, 90]), (num_envs, 1))
            >>> prims.set_joint_efforts(efforts)
            >>>
            >>> # set the fingers efforts: panda_finger_joint1 (7) and panda_finger_joint2 (8) to 10
            >>> # for the first, middle and last of the 5 envs
            >>> efforts = np.tile(np.array([10, 10]), (3, 1))
            >>> prims.set_joint_efforts(efforts, indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return

        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            # TODO: missing get_dof efforts/ forces?
            new_dof_efforts = self._backend_utils.create_zeros_tensor(
                shape=[self.count, self.num_dof], dtype="float32", device=self._device
            )
            new_dof_efforts = self._backend_utils.assign(
                self._backend_utils.move_data(efforts, device=self._device),
                new_dof_efforts,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            # TODO: double check this/ is this setting a force or applying a force?
            self._physics_view.set_dof_actuation_forces(new_dof_efforts, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_joint_efforts")
        return

    def get_applied_joint_efforts(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the joint efforts of articulations in the view

        This method will return the efforts set by the ``set_joint_efforts`` method

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: joint efforts of articulations in the view. Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get all applied joint efforts. Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> prims.get_applied_joint_efforts()
            [[0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]]
            >>>
            >>> # get finger applied efforts: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_applied_joint_efforts(indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
            [[0. 0.]
             [0. 0.]
             [0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            current_joint_forces = self._physics_view.get_dof_actuation_forces()
            if clone:
                current_joint_forces = self._backend_utils.clone_tensor(current_joint_forces, device=self._device)
            result = current_joint_forces[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_applied_joint_efforts")
            return None

    def get_measured_joint_efforts(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Returns the efforts computed/measured by the physics solver of the joint forces in the DOF motion direction

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: computed joint efforts of articulations in the view. shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get all measured joint efforts. Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> prims.get_measured_joint_efforts()
            [[ 4.8250298e-05 -6.9073005e+00  5.3364405e-05  1.9157070e+01 -5.8759182e-05
               1.1863427e+00 -5.6388220e-05  5.1680300e-03 -5.1910817e-03]
             [ 4.8250298e-05 -6.9073005e+00  5.3364405e-05  1.9157070e+01 -5.8759182e-05
               1.1863427e+00 -5.6388220e-05  5.1680300e-03 -5.1910817e-03]
             [ 4.8254540e-05 -6.9072919e+00  5.3344327e-05  1.9157072e+01 -5.8761045e-05
               1.1863427e+00 -5.6405144e-05  5.1680212e-03 -5.1910840e-03]
             [ 4.8254540e-05 -6.9072919e+00  5.3344327e-05  1.9157072e+01 -5.8761045e-05
               1.1863427e+00 -5.6405144e-05  5.1680212e-03 -5.1910840e-03]
             [ 4.8250298e-05 -6.9073005e+00  5.3364405e-05  1.9157070e+01 -5.8759182e-05
               1.1863427e+00 -5.6388220e-05  5.1680300e-03  -5.1910817e-03]]
            >>>
            >>> # get finger measured joint efforts: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_measured_joint_efforts(indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
            [[ 0.00516803 -0.00519108]
             [ 0.00516802 -0.00519108]
             [ 0.00516803 -0.00519108]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            current_joint_forces = self._physics_view.get_dof_projected_joint_forces()
            if clone:
                current_joint_forces = self._backend_utils.clone_tensor(current_joint_forces, device=self._device)
            result = current_joint_forces[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_measured_joint_efforts")
            return None

    def get_measured_joint_forces(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor]:
        """Get the measured joint reaction forces and torques (link incoming joint forces and torques) to external loads

        .. note::

            Since the *name->index* map for joints has not been exposed yet,
            it is possible to access the joint names and their indices through the articulation metadata.

            .. code-block:: python

                prims._metadata.joint_names  # list of names
                prims._metadata.joint_indices  # dict of name: index

            To retrieve a specific row for the link incoming joint force/torque use ``joint_index + 1``

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor]], optional): link indices to specify which link's incoming joints to query. Shape (K,).
                                                                                    Where K <= num of links/bodies.
                                                                                    Defaults to None (i.e: all dofs).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor]: joint forces and torques of articulations in the view.
            Shape is (M, num_joint + 1, 6). Column index 0 is the incoming joint of the base link.
            For the last dimension the first 3 values are for forces and the last 3 for torques

        Example:

        .. code-block:: python

            >>> # get all measured joint forces and torques. Returned shape is (5, 12, 6) for the example:
            >>> # 5 envs, 9 DOFs (but 12 joints including the fixed and root joints)
            >>> prims.get_measured_joint_forces()
            [[[ 0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00]
              [ 1.49950760e+02  3.52353277e-06  5.62586996e-04  4.82502983e-05 -6.90729856e+00  2.69259126e-05]
              [-2.60467059e-05 -1.06778236e+02 -6.83844986e+01 -6.90730047e+00 -5.27759657e-05 -1.24897576e-06]
              [ 8.71209946e+01 -4.46646191e-05 -5.57951622e+01  5.33644052e-05 -2.45385647e+01  1.38957939e-05]
              [ 5.18576926e-05 -4.81099091e+01  6.07092705e+01  1.91570702e+01 -5.81023924e-05  1.46875891e-06]
              [-3.16910419e+01  2.31799815e-04  3.99901695e+01 -5.87591821e-05 -1.18634319e+00  2.24427877e-05]
              [-1.07621672e-04  1.53405371e+01 -1.54584875e+01  1.18634272e+00  6.09036942e-05 -1.60679410e-05]
              [-7.54189777e+00 -5.08146524e+00 -5.65130091e+00 -5.63882204e-05  3.88599992e-01 -3.49432468e-01]
              [ 4.74214745e+00 -3.19458222e+00  3.55281782e+00  5.58562024e-05  8.47946014e-03  7.64050474e-03]
              [ 4.07607269e+00  2.16406956e-01 -4.05131817e+00 -5.95658377e-04  1.14070829e-02  2.13965313e-06]
              [ 5.16803004e-03 -9.77545828e-02 -9.70939621e-02 -8.41282599e-12 -1.29066744e-12 -1.93477560e-11]
              [-5.19108167e-03  9.75882635e-02 -9.71064270e-02  8.41282859e-12  1.29066018e-12 -1.93477543e-11]]
             ...
             [[ 0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00  0.00000000e+00]
              [ 1.49950760e+02  3.52353277e-06  5.62586996e-04  4.82502983e-05 -6.90729856e+00  2.69259126e-05]
              [-2.60467059e-05 -1.06778236e+02 -6.83844986e+01 -6.90730047e+00 -5.27759657e-05 -1.24897576e-06]
              [ 8.71209946e+01 -4.46646191e-05 -5.57951622e+01  5.33644052e-05 -2.45385647e+01  1.38957939e-05]
              [ 5.18576926e-05 -4.81099091e+01  6.07092705e+01  1.91570702e+01 -5.81023924e-05  1.46875891e-06]
              [-3.16910419e+01  2.31799815e-04  3.99901695e+01 -5.87591821e-05 -1.18634319e+00  2.24427877e-05]
              [-1.07621672e-04  1.53405371e+01 -1.54584875e+01  1.18634272e+00  6.09036942e-05 -1.60679410e-05]
              [-7.54189777e+00 -5.08146524e+00 -5.65130091e+00 -5.63882204e-05  3.88599992e-01 -3.49432468e-01]
              [ 4.74214745e+00 -3.19458222e+00  3.55281782e+00  5.58562024e-05  8.47946014e-03  7.64050474e-03]
              [ 4.07607269e+00  2.16406956e-01 -4.05131817e+00 -5.95658377e-04  1.14070829e-02  2.13965313e-06]
              [ 5.16803004e-03 -9.77545828e-02 -9.70939621e-02 -8.41282599e-12 -1.29066744e-12 -1.93477560e-11]
              [-5.19108167e-03  9.75882635e-02 -9.71064270e-02  8.41282859e-12  1.29066018e-12 -1.93477543e-11]]]
            >>>
            >>> # get measured joint forces and torques for the fingers for the first, middle and last of the 5 envs.
            >>> # Returned shape is (3, 2, 6)
            >>> metadata = prims._metadata
            >>> joint_indices = 1 + np.array([
            >>>     metadata.joint_indices["panda_finger_joint1"],
            >>>     metadata.joint_indices["panda_finger_joint2"],
            >>> ])
            >>> joint_indices
            [10 11]
            >>> prims.get_measured_joint_forces(indices=np.array([0, 2, 4]), joint_indices=joint_indices)
            [[[ 5.1680300e-03 -9.7754583e-02 -9.7093962e-02 -8.4128260e-12 -1.2906674e-12 -1.9347756e-11]
              [-5.1910817e-03  9.7588263e-02 -9.7106427e-02  8.4128286e-12  1.2906602e-12 -1.9347754e-11]]
             [[ 5.1680212e-03 -9.7754560e-02 -9.7093947e-02 -8.4141834e-12 -1.2907383e-12 -1.9348209e-11]
              [-5.1910840e-03  9.7588278e-02 -9.7106412e-02  8.4141869e-12  1.2907335e-12 -1.9348207e-11]]
             [[ 5.1680300e-03 -9.7754583e-02 -9.7093962e-02 -8.4128260e-12 -1.2906674e-12 -1.9347756e-11]
              [-5.1910817e-03  9.7588263e-02 -9.7106427e-02  8.4128286e-12  1.2906602e-12 -1.9347754e-11]]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_bodies, self._device)
            current_joint_forces = self._physics_view.get_link_incoming_joint_force()
            if clone:
                current_joint_forces = self._backend_utils.clone_tensor(current_joint_forces, device=self._device)
            result = current_joint_forces[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_measured_joint_forces")
            return None

    def get_joint_positions(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the joint positions of articulations in the view

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: joint positions of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get all joint positions. Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> prims.get_joint_positions()
            [[ 1.1999921e-02 -5.6962633e-01  1.3219320e-08 -2.8105433e+00  6.8276213e-06
               3.0301569e+00  7.3234755e-01  3.9912373e-02  3.9999999e-02]
             [ 1.1999921e-02 -5.6962633e-01  1.3219320e-08 -2.8105433e+00  6.8276213e-06
               3.0301569e+00  7.3234755e-01  3.9912373e-02  3.9999999e-02]
             [ 1.1999921e-02 -5.6962633e-01  1.3220056e-08 -2.8105433e+00  6.8276104e-06
               3.0301569e+00  7.3234755e-01  3.9912373e-02  3.9999999e-02]
             [ 1.1999921e-02 -5.6962633e-01  1.3220056e-08 -2.8105433e+00  6.8276104e-06
               3.0301569e+00  7.3234755e-01  3.9912373e-02  3.9999999e-02]
             [ 1.1999921e-02 -5.6962633e-01  1.3219320e-08 -2.8105433e+00  6.8276213e-06
               3.0301569e+00  7.3234755e-01  3.9912373e-02  3.9999999e-02]]
            >>>
            >>> # get finger joint positions: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_joint_positions(indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
            [[0.03991237 0.04      ]
             [0.03991237 0.04      ]
             [0.03991237 0.04      ]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            current_joint_positions = self._physics_view.get_dof_positions()
            if clone:
                current_joint_positions = self._backend_utils.clone_tensor(current_joint_positions, device=self._device)
            result = current_joint_positions[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_joint_positions")
            return None

    def get_joint_velocities(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the joint velocities of articulations in the view

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: joint velocities of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get all joint velocities. Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> prims.get_joint_velocities()
            [[ 1.9010375e-06 -7.6763844e-03 -2.1396865e-07  1.1063669e-02 -4.6333633e-05
               3.4824573e-02  8.8469200e-02  5.4033857e-04  1.0287426e-05]
             [ 1.9010375e-06 -7.6763844e-03 -2.1396865e-07  1.1063669e-02 -4.6333633e-05
               3.4824573e-02  8.8469200e-02  5.4033857e-04  1.0287426e-05]
             [ 1.9010074e-06 -7.6763779e-03 -2.1403629e-07  1.1063648e-02 -4.6333400e-05
               3.4824558e-02  8.8469170e-02  5.4033566e-04  1.0287110e-05]
             [ 1.9010074e-06 -7.6763779e-03 -2.1403629e-07  1.1063648e-02 -4.6333400e-05
               3.4824558e-02  8.8469170e-02  5.4033566e-04  1.0287110e-05]
             [ 1.9010375e-06 -7.6763844e-03 -2.1396865e-07  1.1063669e-02 -4.6333633e-05
               3.4824573e-02  8.8469200e-02  5.4033857e-04  1.0287426e-05]]
            >>>
            >>> # get finger joint velocities: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_joint_velocities(indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
            [[5.4033857e-04 1.0287426e-05]
             [5.4033566e-04 1.0287110e-05]
             [5.4033857e-04 1.0287426e-05]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            current_joint_velocities = self._physics_view.get_dof_velocities()
            if clone:
                current_joint_velocities = self._backend_utils.clone_tensor(
                    current_joint_velocities, device=self._device
                )
            result = current_joint_velocities[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_joint_velocities")
            return None

    def apply_action(
        self,
        control_actions: ArticulationActions,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Apply joint positions (targets), velocities (targets) and/or efforts to control an articulation

        .. note::

            This method can be used instead of the separate ``set_joint_position_targets``,
            ``set_joint_velocity_targets`` and ``set_joint_efforts``

        Args:
            control_actions (ArticulationActions): actions to be applied for next physics step.
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        .. hint::

            High stiffness makes the joints snap faster and harder to the desired target,
            and higher damping smoothes but also slows down the joint's movement to target

            * For position control, set relatively high stiffness and low damping (to reduce vibrations)
            * For velocity control, stiffness must be set to zero with a non-zero damping
            * For effort control, stiffness and damping must be set to zero

        Example:

        .. code-block:: python

            >>> from omni.isaac.core.utils.types import ArticulationActions
            >>>
            >>> # move all the articulation joints to the indicated position.
            >>> # Since there are 5 envs, the joint positions are repeated 5 times
            >>> positions = np.tile(np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04]), (num_envs, 1))
            >>> action = ArticulationActions(joint_positions=positions)
            >>> prims.apply_action(action)
            >>>
            >>> # close the robot fingers: panda_finger_joint1 (7) and panda_finger_joint2 (8) to 0.0
            >>> # for the first, middle and last of the 5 envs
            >>> positions = np.tile(np.array([0.0, 0.0]), (3, 1))
            >>> action = ArticulationActions(joint_positions=positions, joint_indices=np.array([7, 8]))
            >>> prims.apply_action(action, indices=np.array([0, 2, 4]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(
                control_actions.joint_indices, self.num_dof, self._device
            )

            if control_actions.joint_positions is not None:
                # TODO: optimize this operation
                action = self._physics_view.get_dof_position_targets()
                action = self._backend_utils.assign(
                    self._backend_utils.move_data(control_actions.joint_positions, device=self._device),
                    action,
                    [
                        self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices,
                        joint_indices,
                    ],
                )
                self._physics_view.set_dof_position_targets(action, indices)
            if control_actions.joint_velocities is not None:
                # TODO: optimize this operation
                action = self._physics_view.get_dof_velocity_targets()
                action = self._backend_utils.assign(
                    self._backend_utils.move_data(control_actions.joint_velocities, device=self._device),
                    action,
                    [
                        self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices,
                        joint_indices,
                    ],
                )
                self._physics_view.set_dof_velocity_targets(action, indices)
            if control_actions.joint_efforts is not None:
                # TODO: optimize this operation
                # action = self._backend_utils.clone_tensor(self._physics_view.get_dof_actuation_forces(), device=self._device)
                action = self._backend_utils.create_zeros_tensor(
                    (self.count, self.num_dof), dtype="float32", device=self._device
                )
                action = self._backend_utils.assign(
                    self._backend_utils.move_data(control_actions.joint_efforts, device=self._device),
                    action,
                    [
                        self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices,
                        joint_indices,
                    ],
                )
                self._physics_view.set_dof_actuation_forces(action, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use apply_action")
        return

    def get_applied_actions(self, clone: bool = True) -> ArticulationActions:
        """Get the last applied actions

        Args:
            clone (bool, optional): True to return clones of the internal buffers. Otherwise False. Defaults to True.

        Returns:
            ArticulationActions: current applied actions (i.e: current position targets and velocity targets)

        Example:

        .. code-block:: python

            >>> # last applied action: joint_positions -> [0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04].
            >>> # Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> actions = prims.get_applied_actions()
            >>> actions
            <omni.isaac.core.utils.types.ArticulationActions object at 0x7f28af31d870>
            >>> actions.joint_positions
            [[ 0.   -1.    0.   -2.2   0.    2.4   0.8   0.04  0.04]
             [ 0.   -1.    0.   -2.2   0.    2.4   0.8   0.04  0.04]
             [ 0.   -1.    0.   -2.2   0.    2.4   0.8   0.04  0.04]
             [ 0.   -1.    0.   -2.2   0.    2.4   0.8   0.04  0.04]
             [ 0.   -1.    0.   -2.2   0.    2.4   0.8   0.04  0.04]]
            >>> actions.joint_velocities
            [[0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]]
            >>> actions.joint_efforts
            [[0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            joint_positions = self._physics_view.get_dof_position_targets()
            if clone:
                joint_positions = self._backend_utils.clone_tensor(joint_positions, device=self._device)
            joint_velocities = self._physics_view.get_dof_velocity_targets()
            if clone:
                joint_velocities = self._backend_utils.clone_tensor(joint_velocities, device=self._device)
            joint_efforts = self._physics_view.get_dof_actuation_forces()
            if clone:
                joint_efforts = self._backend_utils.clone_tensor(joint_efforts, device=self._device)
            # TODO: implement the effort part
            return ArticulationActions(
                joint_positions=joint_positions,
                joint_velocities=joint_velocities,
                joint_efforts=joint_efforts,
                joint_indices=None,
            )
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_applied_actions")
            return None

    def set_world_poses(
        self,
        positions: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
        usd: bool = True,
    ) -> None:
        """Set poses of prims in the view with respect to the world's frame.

        .. warning::

            This method will change (teleport) the prim poses immediately to the indicated value

        Args:
            positions (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): positions in the world frame of the prim. shape is (M, 3).
                                                                             Defaults to None, which means left unchanged.
            orientations (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): quaternion orientations in the world frame of the prims.
                                                                                quaternion is scalar-first (w, x, y, z). shape is (M, 4).
                                                                                Defaults to None, which means left unchanged.
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            usd (bool, optional): True to query from usd. Otherwise False to query from Fabric data. Defaults to True.

        .. hint::

            This method belongs to the methods used to set the prim state

        Example:

        .. code-block:: python

            >>> # reposition all articulations in row (x-axis)
            >>> positions = np.zeros((num_envs, 3))
            >>> positions[:,0] = np.arange(num_envs)
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (num_envs, 1))
            >>> prims.set_world_poses(positions, orientations)
            >>>
            >>> # reposition only the articulations for the first, middle and last of the 5 envs in column (y-axis)
            >>> positions = np.zeros((3, 3))
            >>> positions[:,1] = np.arange(3)
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (3, 1))
            >>> prims.set_world_poses(positions, orientations, indices=np.array([0, 2, 4]))
        """
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_positions, current_orientations = self.get_world_poses(clone=False)
            if not hasattr(self, "_pose_buf"):
                self._pose_buf = self._backend_utils.create_zeros_tensor(
                    shape=[self.count, 7], dtype="float32", device=self._device
                )
            if positions is not None:
                positions = self._backend_utils.move_data(positions, self._device)
            if orientations is not None:
                orientations = self._backend_utils.move_data(orientations, self._device)
            pose = self._backend_utils.assign_pose(
                current_positions, current_orientations, positions, orientations, indices, self._device, self._pose_buf
            )
            self._physics_view.set_root_transforms(pose, indices)
            return
        else:
            XFormPrimView.set_world_poses(
                self, positions=positions, orientations=orientations, indices=indices, usd=usd
            )
        return

    def get_world_poses(
        self,
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
        clone: bool = True,
        usd: bool = True,
    ) -> Union[
        Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor], Tuple[wp.indexedarray, wp.indexedarray]
    ]:
        """Get the poses of the prims in the view with respect to the world's frame.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.
            usd (bool, optional): True to query from usd. Otherwise False to query from Fabric data. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor], Tuple[wp.indexedarray, wp.indexedarray]]:
            first index is positions in the world frame of the prims. shape is (M, 3). Second index is quaternion orientations
            in the world frame of the prims. Quaternion is scalar-first (w, x, y, z). shape is (M, 4).

        Example:

        .. code-block:: python

            >>> # get all articulation poses with respect to the world's frame.
            >>> # Returned shape is position (5, 3) and orientation (5, 4) for the example: 5 envs
            >>> positions, orientations = prims.get_world_poses()
            >>> positions
            [[ 1.5000000e+00 -7.5000000e-01 -2.8610229e-08]
             [ 1.5000000e+00  7.5000000e-01 -2.8610229e-08]
             [-4.5299529e-08 -7.5000000e-01 -2.8610229e-08]
             [-4.5299529e-08  7.5000000e-01 -2.8610229e-08]
             [-1.5000000e+00 -7.5000000e-01 -2.8610229e-08]]
            >>> orientations
            [[1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]]
            >>>
            >>> # get only the articulation poses with respect to the world's frame for the first, middle and last of the 5 envs.
            >>> # Returned shape is position (3, 3) and orientation (3, 4) for the example: 3 envs selected
            >>> positions, orientations = prims.get_world_poses(indices=np.array([0, 2, 4]))
            >>> positions
            [[ 1.5000000e+00 -7.5000000e-01 -2.8610229e-08]
             [-4.5299529e-08 -7.5000000e-01 -2.8610229e-08]
             [-1.5000000e+00 -7.5000000e-01 -2.8610229e-08]]
            >>> orientations
            [[1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]]
        """
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            pose = self._physics_view.get_root_transforms()
            if clone:
                pose = self._backend_utils.clone_tensor(pose, device=self._device)
            pos = pose[indices, 0:3]
            rot = self._backend_utils.xyzw2wxyz(pose[indices, 3:7])
            return pos, rot
        else:
            pos, rot = XFormPrimView.get_world_poses(self, indices=indices, usd=usd)
            ret_pos = self._backend_utils.convert(pos, dtype="float32", device=self._device, indexed=True)
            ret_rot = self._backend_utils.convert(rot, dtype="float32", device=self._device, indexed=True)
            return ret_pos, ret_rot

    def get_local_poses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None
    ) -> Union[
        Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor], Tuple[wp.indexedarray, wp.indexedarray]
    ]:
        """Get prim poses in the view with respect to the local frame (the prim's parent frame).

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor], Tuple[wp.indexedarray, wp.indexedarray]]:
            first index is positions in the local frame of the prims. shape is (M, 3). Second index is quaternion orientations
            in the local frame of the prims. Quaternion is scalar-first (w, x, y, z). shape is (M, 4).

        Example:

        .. code-block:: python

            >>> # get all articulation poses with respect to the local frame.
            >>> # Returned shape is position (5, 3) and orientation (5, 4) for the example: 5 envs
            >>> positions, orientations = prims.get_local_poses()
            >>> positions
            [[ 0.0000000e+00  0.0000000e+00 -2.8610229e-08]
             [ 0.0000000e+00  0.0000000e+00 -2.8610229e-08]
             [-4.5299529e-08  0.0000000e+00 -2.8610229e-08]
             [-4.5299529e-08  0.0000000e+00 -2.8610229e-08]
             [ 0.0000000e+00  0.0000000e+00 -2.8610229e-08]]
            >>> orientations
            [[1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]]
            >>>
            >>> # get only the articulation poses with respect to the local frame for the first, middle and last of the 5 envs.
            >>> # Returned shape is position (3, 3) and orientation (3, 4) for the example: 3 envs selected
            >>> positions, orientations = prims.get_local_poses(indices=np.array([0, 2, 4]))
            >>> positions
            [[ 0.0000000e+00  0.0000000e+00 -2.8610229e-08]
             [-4.5299529e-08  0.0000000e+00 -2.8610229e-08]
             [ 0.0000000e+00  0.0000000e+00 -2.8610229e-08]]
            >>> orientations
            [[1. 0. 0. 0.]
             [1. 0. 0. 0.]
             [1. 0. 0. 0.]]
        """
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            world_positions, world_orientations = self.get_world_poses(indices=indices)
            parent_transforms = np.zeros(shape=(indices.shape[0], 4, 4), dtype="float32")
            write_idx = 0
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                parent_transforms[write_idx] = np.array(
                    UsdGeom.Xformable(get_prim_parent(self._prims[i])).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    ),
                    dtype="float32",
                )
                write_idx += 1
            parent_transforms = self._backend_utils.convert(parent_transforms, dtype="float32", device=self._device)
            res = self._backend_utils.get_local_from_world(
                parent_transforms, world_positions, world_orientations, self._device
            )
            return res
        else:
            return XFormPrimView.get_local_poses(self, indices=indices)

    def set_local_poses(
        self,
        translations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set prim poses in the view with respect to the local frame (the prim's parent frame).

        .. warning::

            This method will change (teleport) the prim poses immediately to the indicated value

        Args:
            translations (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional):
                                                          translations in the local frame of the prims
                                                          (with respect to its parent prim). shape is (M, 3).
                                                          Defaults to None, which means left unchanged.
            orientations (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional):
                                                          quaternion orientations in the local frame of the prims.
                                                          quaternion is scalar-first (w, x, y, z). shape is (M, 4).
                                                          Defaults to None, which means left unchanged.
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        .. hint::

            This method belongs to the methods used to set the prim state

        Example:

        .. code-block:: python

            >>> # reposition all articulations
            >>> positions = np.zeros((num_envs, 3))
            >>> positions[:,0] = np.arange(num_envs)
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (num_envs, 1))
            >>> prims.set_local_poses(positions, orientations)
            >>>
            >>> # reposition only the articulations for the first, middle and last of the 5 envs
            >>> positions = np.zeros((3, 3))
            >>> positions[:,1] = np.arange(3)
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (3, 1))
            >>> prims.set_local_poses(positions, orientations, indices=np.array([0, 2, 4]))
        """
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            if translations is None or orientations is None:
                current_translations, current_orientations = ArticulationView.get_local_poses(self)
                if translations is None:
                    translations = current_translations
                if orientations is None:
                    orientations = current_orientations
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            parent_transforms = np.zeros(shape=(indices.shape[0], 4, 4), dtype="float32")
            write_idx = 0
            indices = self._backend_utils.to_list(indices)
            for i in indices:
                parent_transforms[write_idx] = np.array(
                    UsdGeom.Xformable(get_prim_parent(self._prims[i])).ComputeLocalToWorldTransform(
                        Usd.TimeCode.Default()
                    ),
                    dtype="float32",
                )
                write_idx += 1
            parent_transforms = self._backend_utils.convert(parent_transforms, dtype="float32", device=self._device)
            calculated_positions, calculated_orientations = self._backend_utils.get_world_from_local(
                parent_transforms, translations, orientations, self._device
            )
            ArticulationView.set_world_poses(
                self, positions=calculated_positions, orientations=calculated_orientations, indices=indices
            )
        else:
            XFormPrimView.set_local_poses(self, translations=translations, orientations=orientations, indices=indices)
        return

    def set_velocities(
        self,
        velocities: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the linear and angular velocities of the prims in the view at once.

        The method does this through the PhysX API only. It has to be called after initialization

        .. warning::

            This method will immediately set the articulation state

        Args:
            velocities (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): linear and angular velocities respectively to set the rigid prims to. shape is (M, 6).
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        .. hint::

            This method belongs to the methods used to set the articulation kinematic state:

            ``set_velocities`` (``set_linear_velocities``, ``set_angular_velocities``),
            ``set_joint_positions``, ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> # set each articulation linear velocity to (1., 1., 1.) and angular velocity to (.1, .1, .1)
            >>> velocities = np.ones((num_envs, 6))
            >>> velocities[:,3:] = 0.1
            >>> prims.set_velocities(velocities)
            >>>
            >>> # set only the articulation velocities for the first, middle and last of the 5 envs
            >>> velocities = np.ones((3, 6))
            >>> velocities[:,3:] = 0.1
            >>> prims.set_velocities(velocities, indices=np.array([0, 2, 4]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            root_vel = self._physics_view.get_root_velocities()
            root_vel = self._backend_utils.assign(
                self._backend_utils.move_data(velocities, self._device), root_vel, indices
            )
            self._physics_view.set_root_velocities(root_vel, indices)
        else:
            self.set_linear_velocities(velocities[:, 0:3], indices=indices)
            self.set_angular_velocities(velocities[:, 3:6], indices=indices)

    def get_velocities(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the linear and angular velocities of prims in the view.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: linear and angular velocities of the prims in the view concatenated. shape is (M, 6).
            For the last dimension the first 3 values are for linear velocities and the last 3 for angular velocities

        Example:

        .. code-block:: python

            >>> # get all articulation velocities. Returned shape is (5, 6) for the example: 5 envs, linear (3) and angular (3)
            >>> prims.get_velocities()
            [[0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]]
            >>>
            >>> # get only the articulation velocities for the first, middle and last of the 5 envs.
            >>> # Returned shape is (3, 6) for the example: 3 envs selected, linear (3) and angular (3)
            >>> prims.get_velocities(indices=np.array([0, 2, 4]))
            [[0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            velocities = self._physics_view.get_root_velocities()
            if clone:
                velocities = self._backend_utils.clone_tensor(velocities, device=self._device)
            return velocities[indices]
        else:
            linear_velocities = self.get_linear_velocities(indices, clone)
            angular_velocities = self.get_angular_velocities(indices, clone)
            return self._backend_utils.tensor_cat([linear_velocities, angular_velocities], dim=-1, device=self._device)

    def set_linear_velocities(
        self,
        velocities: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the linear velocities of the prims in the view

        The method does this through the PhysX API only. It has to be called after initialization.
        Note: This method is not supported for the gpu pipeline. ``set_velocities`` method should be used instead.

        .. warning::

            This method will immediately set the articulation state

        Args:
            velocities (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): linear velocities to set the rigid prims to. shape is (M, 3).
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        .. hint::

            This method belongs to the methods used to set the articulation kinematic state:

            ``set_velocities`` (``set_linear_velocities``, ``set_angular_velocities``),
            ``set_joint_positions``, ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> # set each articulation linear velocity to (1.0, 1.0, 1.0)
            >>> velocities = np.ones((num_envs, 3))
            >>> prims.set_linear_velocities(velocities)
            >>>
            >>> # set only the articulation linear velocities for the first, middle and last of the 5 envs
            >>> velocities = np.ones((3, 3))
            >>> prims.set_linear_velocities(velocities, indices=np.array([0, 2, 4]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if self._device is not None and "cuda" in self._device:
            carb.log_warn(
                "set_linear_velocities function is not supported for the gpu pipeline, use set_velocities instead."
            )
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            root_velocities = self._physics_view.get_root_velocities()
            if self._backend == "warp":
                root_velocities = self._backend_utils.assign(
                    self._backend_utils.move_data(velocities, device=self._device),
                    root_velocities,
                    [indices, wp.array([0, 1, 2], device=self._device, dtype=wp.int32)],
                )
            else:
                root_velocities[indices, 0:3] = self._backend_utils.move_data(velocities, device=self._device)
            self._physics_view.set_root_velocities(root_velocities, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_linear_velocities")

    def get_linear_velocities(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None, clone=True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the linear velocities of prims in the view.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: linear velocities of the prims in the view. shape is (M, 3).

        Example:

        .. code-block:: python

            >>> # get all articulation linear velocities. Returned shape is (5, 3) for the example: 5 envs, linear (3)
            >>> prims.get_linear_velocities()
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
            >>>
            >>> # get only the articulation linear velocities for the first, middle and last of the 5 envs.
            >>> # Returned shape is (3, 3) for the example: 3 envs selected, linear (3)
            >>> prims.get_linear_velocities(indices=np.array([0, 2, 4]))
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            linear_velocities = self._physics_view.get_root_velocities()
            if clone:
                linear_velocities = self._backend_utils.clone_tensor(linear_velocities, device=self._device)
            return linear_velocities[indices, 0:3]
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_linear_velocities")
            return None

    def set_angular_velocities(
        self,
        velocities: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the angular velocities of the prims in the view

        The method does this through the physx API only. It has to be called after initialization.
        Note: This method is not supported for the gpu pipeline. ``set_velocities`` method should be used instead.

        .. warning::

            This method will immediately set the articulation state

        Args:
            velocities (Optional[Union[np.ndarray, torch.Tensor, wp.array]]): angular velocities to set the rigid prims to. shape is (M, 3).
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        .. hint::

            This method belongs to the methods used to set the articulation kinematic state:

            ``set_velocities`` (``set_linear_velocities``, ``set_angular_velocities``),
            ``set_joint_positions``, ``set_joint_velocities``, ``set_joint_efforts``

        Example:

        .. code-block:: python

            >>> # set each articulation linear velocity to (0.1, 0.1, 0.1)
            >>> velocities = np.full((num_envs, 3), fill_value=0.1)
            >>> prims.set_angular_velocities(velocities)
            >>>
            >>> # set only the articulation linear velocities for the first, middle and last of the 5 envs
            >>> velocities = np.full((3, 3), fill_value=0.1)
            >>> prims.set_angular_velocities(velocities, indices=np.array([0, 2, 4]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
        if self._device is not None and "cuda" in self._device:
            carb.log_warn(
                "set_angular_velocities function is not supported for the gpu pipeline, use set_velocities instead."
            )
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            root_velocities = self._physics_view.get_root_velocities()
            if self._backend == "warp":
                root_velocities = self._backend_utils.assign(
                    self._backend_utils.move_data(velocities, device=self._device),
                    root_velocities,
                    [indices, wp.array([3, 4, 5], device=self._device, dtype=wp.int32)],
                )
            else:
                root_velocities[indices, 3:6] = self._backend_utils.move_data(velocities, device=self._device)
            self._physics_view.set_root_velocities(root_velocities, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_angular_velocities")
        return

    def get_angular_velocities(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor, wp.array]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the angular velocities of prims in the view.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                    to query. Shape (M,).
                                                                                    Where M <= size of the encapsulated prims in the view.
                                                                                    Defaults to None (i.e: all prims in the view)
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: angular velocities of the prims in the view. shape is (M, 3).

        Example:

        .. code-block:: python

            >>> # get all articulation angular velocities. Returned shape is (5, 3) for the example: 5 envs, angular (3)
            >>> prims.get_angular_velocities()
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
            >>>
            >>> # get only the articulation angular velocities for the first, middle and last of the 5 envs
            >>> # Returned shape is (5, 3) for the example: 3 envs selected, angular (3)
            >>> prims.get_angular_velocities(indices=np.array([0, 2, 4]))
            [[0. 0. 0.]
             [0. 0. 0.]
             [0. 0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            angular_velocities = self._physics_view.get_root_velocities()
            if clone:
                angular_velocities = self._backend_utils.clone_tensor(angular_velocities, device=self._device)
            return angular_velocities[indices, 3:6]
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_angular_velocities")
            return None

    def set_joints_default_state(
        self,
        positions: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        velocities: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        efforts: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the joints default state (joint positions, velocities and efforts) to be applied after each reset.

        .. note::

            The default states will be set during post-reset (e.g., calling ``.post_reset()`` or ``world.reset()`` methods)

        Args:
            positions (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): default joint positions.
                                                                             shape is (N, num of dofs). Defaults to None.
            velocities (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): default joint velocities.
                                                                             shape is (N, num of dofs). Defaults to None.
            efforts (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): default joint efforts.
                                                                             shape is (N, num of dofs). Defaults to None.

        Example:

        .. code-block:: python

            >>> # configure default joint states for all articulations
            >>> positions = np.tile(np.array([0.0, -1.0, 0.0, -2.2, 0.0, 2.4, 0.8, 0.04, 0.04]), (num_envs, 1))
            >>> prims.set_joints_default_state(
            ...     positions=positions,
            ...     velocities=np.zeros((num_envs, prims.num_dof)),
            ...     efforts=np.zeros((num_envs, prims.num_dof))
            ... )
            >>>
            >>> # set default states during post-reset
            >>> prims.post_reset()
        """
        if self._default_joints_state is None:
            self._default_joints_state = JointsState(positions=None, velocities=None, efforts=None)
        if positions is not None:
            self._default_joints_state.positions = positions
        if velocities is not None:
            self._default_joints_state.velocities = velocities
        if efforts is not None:
            self._default_joints_state.efforts = efforts
        return

    def get_joints_default_state(self) -> JointsState:
        """Get the default joint states defined with the ``set_joints_default_state`` method

        Returns:
            JointsState: an object that contains the default joint states

        Example:

        .. code-block:: python

            >>> # returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> states = prims.get_joints_default_state()
            >>> states
            <omni.isaac.core.utils.types.JointsState object at 0x7fc2c174fd90>
            >>> states.positions
            [[ 0.   -1.    0.   -2.2   0.    2.4   0.8   0.04  0.04]
             [ 0.   -1.    0.   -2.2   0.    2.4   0.8   0.04  0.04]
             [ 0.   -1.    0.   -2.2   0.    2.4   0.8   0.04  0.04]
             [ 0.   -1.    0.   -2.2   0.    2.4   0.8   0.04  0.04]
             [ 0.   -1.    0.   -2.2   0.    2.4   0.8   0.04  0.04]]
            >>> states.velocities
            [[0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]]
            >>> states.efforts
            [[0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]
             [0. 0. 0. 0. 0. 0. 0. 0. 0.]]
        """
        return self._default_joints_state

    def get_joints_state(self) -> JointsState:
        """Get the current joint states (positions and velocities)

        Returns:
            JointsState: an object that contains the current joint positions and velocities

        Example:

        .. code-block:: python

            >>> # returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> states = prims.get_joints_state()
            >>> states
            <omni.isaac.core.utils.types.JointsState object at 0x7fc1a23a82e0>
            >>> states.positions
            [[ 1.1999921e-02 -5.6962633e-01  1.3219320e-08 -2.8105433e+00  6.8276213e-06
               3.0301569e+00  7.3234755e-01  3.9912373e-02  3.9999999e-02]
             [ 1.1999921e-02 -5.6962633e-01  1.3219320e-08 -2.8105433e+00  6.8276213e-06
               3.0301569e+00  7.3234755e-01  3.9912373e-02  3.9999999e-02]
             [ 1.1999921e-02 -5.6962633e-01  1.3220056e-08 -2.8105433e+00  6.8276104e-06
               3.0301569e+00  7.3234755e-01  3.9912373e-02  3.9999999e-02]
             [ 1.1999921e-02 -5.6962633e-01  1.3220056e-08 -2.8105433e+00  6.8276104e-06
               3.0301569e+00  7.3234755e-01  3.9912373e-02  3.9999999e-02]
             [ 1.1999921e-02 -5.6962633e-01  1.3219320e-08 -2.8105433e+00  6.8276213e-06
               3.0301569e+00  7.3234755e-01  3.9912373e-02  3.9999999e-02]]
            >>> states.velocities
            [[ 1.9010375e-06 -7.6763844e-03 -2.1396865e-07  1.1063669e-02 -4.6333633e-05
               3.4824573e-02  8.8469200e-02  5.4033857e-04  1.0287426e-05]
             [ 1.9010375e-06 -7.6763844e-03 -2.1396865e-07  1.1063669e-02 -4.6333633e-05
               3.4824573e-02  8.8469200e-02  5.4033857e-04  1.0287426e-05]
             [ 1.9010074e-06 -7.6763779e-03 -2.1403629e-07  1.1063648e-02 -4.6333400e-05
               3.4824558e-02  8.8469170e-02  5.4033566e-04  1.0287110e-05]
             [ 1.9010074e-06 -7.6763779e-03 -2.1403629e-07  1.1063648e-02 -4.6333400e-05
               3.4824558e-02  8.8469170e-02  5.4033566e-04  1.0287110e-05]
             [ 1.9010375e-06 -7.6763844e-03 -2.1396865e-07  1.1063669e-02 -4.6333633e-05
               3.4824573e-02  8.8469200e-02  5.4033857e-04  1.0287426e-05]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
        # TODO: implement effort part
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            return JointsState(
                positions=self.get_joint_positions(), velocities=self.get_joint_velocities(), efforts=None
            )
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_joints_state")
            return None

    def post_reset(self) -> None:
        """Reset the articulations to their default states

        .. note::

            For the articulations, in addition to configuring the root prim's default positions and spatial orientations
            (defined via the ``set_default_state`` method), the joint's positions, velocities, and efforts
            (defined via the ``set_joints_default_state`` method) and the joint stiffnesses and dampings
            (defined via the ``set_gains`` method) are imposed

        Example:

        .. code-block:: python

            >>> prims.post_reset()
        """
        XFormPrimView.post_reset(self)
        ArticulationView.set_joint_positions(self, self._default_joints_state.positions)
        ArticulationView.set_joint_velocities(self, self._default_joints_state.velocities)
        ArticulationView.set_joint_efforts(self, self._default_joints_state.efforts)
        ArticulationView.set_gains(self, kps=self._default_kps, kds=self._default_kds)

    def get_effort_modes(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> List[str]:
        """Get effort modes for articulations in the view

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        Returns:
            List: Returns a List of size (M, K) indicating the effort modes: ``acceleration`` or ``force``

        Example:

        .. code-block:: python

            >>> # get the effort mode for all joints
            >>> prims.get_effort_modes()
            [['acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration'],
             ['acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration'],
             ['acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration'],
             ['acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration'],
             ['acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration', 'acceleration']]
            >>>
            >>> # get only the finger joints effort modes for the first, middle and last of the 5 envs
            >>> prims.get_effort_modes(indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
            [['acceleration', 'acceleration'], ['acceleration', 'acceleration'], ['acceleration', 'acceleration']]
        """
        if not self._is_initialized:
            carb.log_warn("Physics Simulation View was never created in order to use get_effort_modes")
            return None
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        dof_types = self.get_dof_types()
        joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
        result = [[None for i in range(joint_indices.shape[0])] for j in range(indices.shape[0])]
        articulation_write_idx = 0
        indices = self._backend_utils.to_list(indices)
        joint_indices = self._backend_utils.to_list(joint_indices)
        for i in indices:
            dof_write_idx = 0
            for dof_index in joint_indices:
                drive_type = "angular" if dof_types[dof_index] == omni.physics.tensors.DofType.Rotation else "linear"
                prim = get_prim_at_path(self._dof_paths[i][dof_index])
                if prim.HasAPI(UsdPhysics.DriveAPI):
                    drive = UsdPhysics.DriveAPI(prim, drive_type)
                else:
                    drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
                result[articulation_write_idx][dof_write_idx] = drive.GetTypeAttr().Get()
                dof_write_idx += 1
            articulation_write_idx += 1
        return result

    def set_effort_modes(
        self,
        mode: str,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor]] = None,
    ) -> None:
        """Set effort modes for articulations in the view

        Args:
            mode (str): effort mode to be applied to prims in the view: ``acceleration`` or ``force``.
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        Example:

        .. code-block:: python

            >>> # set the effort mode for all joints to 'force'
            >>> prims.set_effort_modes("force")
            >>>
            >>> # set only the finger joints effort mode to 'force' for the first, middle and last of the 5 envs
            >>> prims.set_effort_modes("force", indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if mode not in ["force", "acceleration"]:
            raise Exception("Effort Mode specified {} is not recognized".format(mode))
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        dof_types = self.get_dof_types()
        joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
        indices = self._backend_utils.to_list(indices)
        joint_indices = self._backend_utils.to_list(joint_indices)
        for i in indices:
            for dof_index in joint_indices:
                drive_type = "angular" if dof_types[dof_index] == omni.physics.tensors.DofType.Rotation else "linear"
                prim = get_prim_at_path(self._dof_paths[i][dof_index])
                if prim.HasAPI(UsdPhysics.DriveAPI):
                    drive = UsdPhysics.DriveAPI(prim, drive_type)
                else:
                    drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
                if not drive.GetTypeAttr():
                    drive.CreateTypeAttr().Set(mode)
                else:
                    drive.GetTypeAttr().Set(mode)
        return

    def set_max_efforts(
        self,
        values: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set maximum efforts for articulation in the view

        Args:
            values (Union[np.ndarray, torch.Tensor, wp.array]): maximum efforts for articulations in the view. shape (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        Example:

        .. code-block:: python

            >>> # set the max efforts for all the articulation joints to the indicated values.
            >>> # Since there are 5 envs, the joint efforts are repeated 5 times
            >>> max_efforts = np.tile(np.array([10000, 9000, 8000, 7000, 6000, 5000, 4000, 1000, 1000]), (num_envs, 1))
            >>> prims.set_max_efforts(max_efforts)
            >>>
            >>> # set the fingers max efforts: panda_finger_joint1 (7) and panda_finger_joint2 (8) to 1000
            >>> # for the first, middle and last of the 5 envs
            >>> max_efforts = np.tile(np.array([1000, 1000]), (3, 1))
            >>> prims.set_max_efforts(max_efforts, indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, "cpu")
            new_values = self._physics_view.get_dof_max_forces()
            new_values = self._backend_utils.assign(
                self._backend_utils.move_data(values, device="cpu"),
                new_values,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            self._physics_view.set_dof_max_forces(new_values, indices)
        else:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            dof_types = self.get_dof_types()
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            articulation_read_idx = 0
            indices = self._backend_utils.to_list(indices)
            joint_indices = self._backend_utils.to_list(joint_indices)
            values = self._backend_utils.to_list(values)
            for i in indices:
                dof_read_idx = 0
                for dof_index in joint_indices:
                    drive_type = (
                        "angular" if dof_types[dof_index] == omni.physics.tensors.DofType.Rotation else "linear"
                    )
                    prim = get_prim_at_path(self._dof_paths[i][dof_index])
                    if prim.HasAPI(UsdPhysics.DriveAPI):
                        drive = UsdPhysics.DriveAPI(prim, drive_type)
                    else:
                        drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
                    if not drive.GetMaxForceAttr():
                        drive.CreateMaxForceAttr().Set(values[articulation_read_idx][dof_read_idx])
                    else:
                        drive.GetMaxForceAttr().Set(values[articulation_read_idx][dof_read_idx])
                    dof_read_idx += 1
                articulation_read_idx += 1
        return

    def get_max_efforts(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the maximum efforts for articulation in the view

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (Optional[bool]): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: maximum efforts for articulations in the view. shape (M, K).

        Example:

        .. code-block:: python

            >>> # get all joint maximum efforts. Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> prims.get_max_efforts()
            [[5220. 5220. 5220. 5220.  720.  720.  720.  720.  720.]
             [5220. 5220. 5220. 5220.  720.  720.  720.  720.  720.]
             [5220. 5220. 5220. 5220.  720.  720.  720.  720.  720.]
             [5220. 5220. 5220. 5220.  720.  720.  720.  720.  720.]
             [5220. 5220. 5220. 5220.  720.  720.  720.  720.  720.]]
            >>>
            >>> # get finger joint maximum efforts: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_max_efforts(indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
            [[720. 720.]
             [720. 720.]
             [720. 720.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, "cpu")
            max_efforts = self._physics_view.get_dof_max_forces()
            if clone:
                max_efforts = self._backend_utils.clone_tensor(max_efforts, device="cpu")
            result = self._backend_utils.move_data(
                max_efforts[
                    self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
                ],
                device=self._device,
            )
            return result
        else:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            dof_types = self.get_dof_types()
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            max_efforts = np.zeros(shape=(indices.shape[0], joint_indices.shape[0]), dtype="float32")
            indices = self._backend_utils.to_list(indices)
            joint_indices = self._backend_utils.to_list(joint_indices)
            articulation_write_idx = 0
            for i in indices:
                dof_write_idx = 0
                for dof_index in joint_indices:
                    drive_type = (
                        "angular" if dof_types[dof_index] == omni.physics.tensors.DofType.Rotation else "linear"
                    )
                    prim = get_prim_at_path(self._dof_paths[i][dof_index])
                    if prim.HasAPI(UsdPhysics.DriveAPI):
                        drive = UsdPhysics.DriveAPI(prim, drive_type)
                    else:
                        drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
                    max_efforts[articulation_write_idx][dof_write_idx] = drive.GetMaxForceAttr().Get()
                    dof_write_idx += 1
                articulation_write_idx += 1
            max_efforts = self._backend_utils.convert(max_efforts, dtype="float32", device=self._device, indexed=True)
            return max_efforts

    def set_max_joint_velocities(
        self,
        values: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set maximum velocities for articulation in the view

        Args:
            values (Union[np.ndarray, torch.Tensor, wp.array]): maximum velocities for articulations in the view. shape (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, "cpu")
            new_values = self._physics_view.get_dof_max_velocities()
            new_values = self._backend_utils.assign(
                self._backend_utils.move_data(values, device="cpu"),
                new_values,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            self._physics_view.set_dof_max_velocities(new_values, indices)
        else:
            return None

    def get_joint_max_velocities(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the maximum joint velocities for articulation dofs in the view

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (Optional[bool]): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: maximum joint velocities for articulations dofs in the view. shape (M, K).
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, "cpu")
            max_velocities = self._physics_view.get_dof_max_velocities()
            if clone:
                max_velocities = self._backend_utils.clone_tensor(max_velocities, device="cpu")
            result = self._backend_utils.move_data(
                max_velocities[
                    self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
                ],
                device=self._device,
            )
            return result
        else:
            return None

    def set_gains(
        self,
        kps: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        kds: Optional[Union[np.ndarray, torch.Tensor, wp.array]] = None,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        save_to_usd: bool = False,
    ) -> None:
        """Set the implicit Proportional-Derivative (PD) controller's Kps (stiffnesses) and Kds (dampings) of articulations in the view

        Args:
            kps (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): stiffness of the drives. shape is (M, K). Defaults to None.
            kds (Optional[Union[np.ndarray, torch.Tensor, wp.array]], optional): damping of the drives. shape is (M, K).. Defaults to None.
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            save_to_usd (bool, optional): True to save the gains in the usd. otherwise False.

        Example:

        .. code-block:: python

            >>> # set the gains (stiffnesses and dampings) for all the articulation joints to the indicated values.
            >>> # Since there are 5 envs, the gains are repeated 5 times
            >>> stiffnesses = np.tile(np.array([100000, 100000, 100000, 100000, 80000, 80000, 80000, 50000, 50000]), (num_envs, 1))
            >>> dampings = np.tile(np.array([8000, 8000, 8000, 8000, 5000, 5000, 5000, 2000, 2000]), (num_envs, 1))
            >>> prims.set_gains(kps=stiffnesses, kds=dampings)
            >>>
            >>> # set the fingers gains (stiffnesses and dampings): panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> # to 50000 and 2000 respectively for the first, middle and last of the 5 envs
            >>> stiffnesses = np.tile(np.array([50000, 50000]), (3, 1))
            >>> dampings = np.tile(np.array([2000, 2000]), (3, 1))
            >>> prims.set_gains(kps=stiffnesses, kds=dampings, indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        if (
            not omni.timeline.get_timeline_interface().is_stopped()
            and self._physics_view is not None
            and not save_to_usd
        ):
            indices = self._backend_utils.resolve_indices(indices, self.count, device="cpu")
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, device="cpu")
            if kps is None:
                kps = self._physics_view.get_dof_stiffnesses()[
                    self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
                ]
            else:
                kps = self._backend_utils.move_data(kps, device="cpu")
            if kds is None:
                kds = self._physics_view.get_dof_dampings()[
                    self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
                ]
            else:
                kds = self._backend_utils.move_data(kds, device="cpu")
            stiffnesses = self._physics_view.get_dof_stiffnesses()
            stiffnesses = self._backend_utils.assign(
                kps,
                stiffnesses,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            dampings = self._physics_view.get_dof_dampings()
            dampings = self._backend_utils.assign(
                kds,
                dampings,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices],
            )
            self._physics_view.set_dof_stiffnesses(stiffnesses, indices)
            self._physics_view.set_dof_dampings(dampings, indices)
        else:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            dof_types = self.get_dof_types()
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            articulation_read_idx = 0
            indices = self._backend_utils.to_list(indices)
            joint_indices = self._backend_utils.to_list(joint_indices)
            if kps is not None:
                kps = self._backend_utils.to_list(kps)
            if kds is not None:
                kds = self._backend_utils.to_list(kds)
            for i in indices:
                dof_read_idx = 0
                for dof_index in joint_indices:
                    drive_type = (
                        "angular" if dof_types[dof_index] == omni.physics.tensors.DofType.Rotation else "linear"
                    )
                    prim = get_prim_at_path(self._dof_paths[i][dof_index])
                    if prim.HasAPI(UsdPhysics.DriveAPI):
                        drive = UsdPhysics.DriveAPI(prim, drive_type)
                    else:
                        drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
                    if kps is not None:
                        if not drive.GetStiffnessAttr():
                            if kps[articulation_read_idx][dof_read_idx] == 0 or drive_type == "linear":
                                drive.CreateStiffnessAttr(kps[articulation_read_idx][dof_read_idx])
                            else:
                                drive.CreateStiffnessAttr(
                                    1.0
                                    / omni.isaac.core.utils.numpy.rad2deg(
                                        float(1.0 / kps[articulation_read_idx][dof_read_idx])
                                    )
                                )
                        else:
                            if kps[articulation_read_idx][dof_read_idx] == 0 or drive_type == "linear":
                                drive.GetStiffnessAttr().Set(kps[articulation_read_idx][dof_read_idx])
                            else:
                                drive.GetStiffnessAttr().Set(
                                    1.0
                                    / omni.isaac.core.utils.numpy.rad2deg(
                                        float(1.0 / kps[articulation_read_idx][dof_read_idx])
                                    )
                                )
                    if kds is not None:
                        if not drive.GetDampingAttr():
                            if kds[articulation_read_idx][dof_read_idx] == 0 or drive_type == "linear":
                                drive.CreateDampingAttr(kds[articulation_read_idx][dof_read_idx])
                            else:
                                drive.CreateDampingAttr(
                                    1.0
                                    / omni.isaac.core.utils.numpy.rad2deg(
                                        float(1.0 / kds[articulation_read_idx][dof_read_idx])
                                    )
                                )
                        else:
                            if kds[articulation_read_idx][dof_read_idx] == 0 or drive_type == "linear":
                                drive.GetDampingAttr().Set(kds[articulation_read_idx][dof_read_idx])
                            else:
                                drive.GetDampingAttr().Set(
                                    1.0
                                    / omni.isaac.core.utils.numpy.rad2deg(
                                        float(1.0 / kds[articulation_read_idx][dof_read_idx])
                                    )
                                )
                    dof_read_idx += 1
                articulation_read_idx += 1
        self._default_kps, self._default_kds = self.get_gains(clone=True)
        return

    def get_gains(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Tuple[Union[np.ndarray, torch.Tensor], Union[np.ndarray, torch.Tensor], Union[wp.indexedarray, wp.index]]:
        """Get the implicit Proportional-Derivative (PD) controller's Kps (stiffnesses) and Kds (dampings) of articulations in the view

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (bool, optional): True to return clones of the internal buffers. Otherwise False. Defaults to True.

        Returns:
            Tuple[Union[np.ndarray, torch.Tensor], Union[np.ndarray, torch.Tensor], Union[wp.indexedarray, wp.index]]:
            stiffness and damping of articulations in the view respectively. shapes are (M, K).

        Example:

        .. code-block:: python

            >>> # get all joint stiffness and damping. Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> stiffnesses, dampings = prims.get_gains()
            >>> stiffnesses
            [[60000. 60000. 60000. 60000. 25000. 15000.  5000.  6000.  6000.]
             [60000. 60000. 60000. 60000. 25000. 15000.  5000.  6000.  6000.]
             [60000. 60000. 60000. 60000. 25000. 15000.  5000.  6000.  6000.]
             [60000. 60000. 60000. 60000. 25000. 15000.  5000.  6000.  6000.]
             [60000. 60000. 60000. 60000. 25000. 15000.  5000.  6000.  6000.]]
            >>> dampings
            [[3000. 3000. 3000. 3000. 3000. 3000. 3000. 1000. 1000.]
             [3000. 3000. 3000. 3000. 3000. 3000. 3000. 1000. 1000.]
             [3000. 3000. 3000. 3000. 3000. 3000. 3000. 1000. 1000.]
             [3000. 3000. 3000. 3000. 3000. 3000. 3000. 1000. 1000.]
             [3000. 3000. 3000. 3000. 3000. 3000. 3000. 1000. 1000.]]
            >>>
            >>> # get finger joints stiffness and damping: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> stiffnesses, dampings = prims.get_gains(indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
            >>> stiffnesses
            [[6000. 6000.]
             [6000. 6000.]
             [6000. 6000.]]
            >>> dampings
            [[1000. 1000.]
             [1000. 1000.]
             [1000. 1000.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, device=self._device)
            kps = self._physics_view.get_dof_stiffnesses()
            kds = self._physics_view.get_dof_dampings()
            kps = self._backend_utils.move_data(kps, device=self._device)
            kds = self._backend_utils.move_data(kds, device=self._device)
            if clone:
                kps = self._backend_utils.clone_tensor(kps, device=self._device)
                kds = self._backend_utils.clone_tensor(kds, device=self._device)
            result_kps = kps[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            result_kds = kds[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            return result_kps, result_kds
        else:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            dof_types = self.get_dof_types()
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            kps = np.zeros(shape=[indices.shape[0], joint_indices.shape[0]], dtype="float32")
            kds = np.zeros(shape=[indices.shape[0], joint_indices.shape[0]], dtype="float32")
            indices = self._backend_utils.to_list(indices)
            joint_indices = self._backend_utils.to_list(joint_indices)
            articulation_write_idx = 0
            for i in indices:
                dof_write_idx = 0
                for dof_index in joint_indices:
                    drive_type = (
                        "angular" if dof_types[dof_index] == omni.physics.tensors.DofType.Rotation else "linear"
                    )
                    prim = get_prim_at_path(self._dof_paths[i][dof_index])
                    if prim.HasAPI(UsdPhysics.DriveAPI):
                        drive = UsdPhysics.DriveAPI(prim, drive_type)
                    else:
                        drive = UsdPhysics.DriveAPI.Apply(prim, drive_type)
                    if drive.GetStiffnessAttr().Get() == 0.0 or drive_type == "linear":
                        kps[articulation_write_idx][dof_write_idx] = drive.GetStiffnessAttr().Get()
                    else:
                        kps[articulation_write_idx][dof_write_idx] = 1.0 / omni.isaac.core.utils.numpy.deg2rad(
                            float(1.0 / drive.GetStiffnessAttr().Get())
                        )
                    if drive.GetDampingAttr().Get() == 0.0 or drive_type == "linear":
                        kds[articulation_write_idx][dof_write_idx] = drive.GetDampingAttr().Get()
                    else:
                        kds[articulation_write_idx][dof_write_idx] = 1.0 / omni.isaac.core.utils.numpy.deg2rad(
                            float(1.0 / drive.GetDampingAttr().Get())
                        )
                    dof_write_idx += 1
                articulation_write_idx += 1
            result_kps = self._backend_utils.convert(kps, dtype="float32", device=self._device, indexed=True)
            result_kds = self._backend_utils.convert(kds, dtype="float32", device=self._device, indexed=True)
            return result_kps, result_kds

    def switch_control_mode(
        self,
        mode: str,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Switch control mode between ``"position"``, ``"velocity"``, or ``"effort"`` for all joints

        This method will set the implicit Proportional-Derivative (PD) controller's Kps (stiffnesses) and Kds (dampings),
        defined via the ``set_gains``  method, of the selected articulations and joints according to the following rule:

        .. list-table::
            :header-rows: 1

            * - Control mode
              - Stiffnesses
              - Dampings
            * - ``"position"``
              - Kps
              - Kds
            * - ``"velocity"``
              - 0
              - Kds
            * - ``"effort"``
              - 0
              - 0

        Args:
            mode (str): control mode to switch the articulations specified to. It can be ``"position"``, ``"velocity"``, or ``"effort"``
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).

        Example:

        .. code-block:: python

            >>> # set 'velocity' as control mode for all joints
            >>> prims.switch_control_mode("velocity")
            >>>
            >>> # set 'effort' as control mode only for the fingers: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> # for the first, middle and last of the 5 envs
            >>> prims.switch_control_mode("effort", indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
        if mode == "velocity":
            self.set_gains(
                kps=self._backend_utils.create_zeros_tensor(
                    shape=[indices.shape[0], joint_indices.shape[0]], dtype="float32", device=self._device
                ),
                kds=self._default_kds[indices][:, joint_indices]
                if self._backend != "warp"
                else self._default_kps.data[indices, joint_indices],
                indices=indices,
                joint_indices=joint_indices,
            )
        elif mode == "position":
            self.set_gains(
                kps=self._default_kps[indices][:, joint_indices]
                if self._backend != "warp"
                else self._default_kps.data[indices, joint_indices],
                kds=self._default_kds[indices][:, joint_indices]
                if self._backend != "warp"
                else self._default_kds.data[indices, joint_indices],
                indices=indices,
                joint_indices=joint_indices,
            )
        elif mode == "effort":
            self.set_gains(
                kps=self._backend_utils.create_zeros_tensor(
                    shape=[indices.shape[0], joint_indices.shape[0]], dtype="float32", device=self._device
                ),
                kds=self._backend_utils.create_zeros_tensor(
                    shape=[indices.shape[0], joint_indices.shape[0]], dtype="float32", device=self._device
                ),
                indices=indices,
                joint_indices=joint_indices,
            )
        return

    def switch_dof_control_mode(
        self, mode: str, dof_index: int, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None
    ) -> None:
        """Switch control mode between ``"position"``, ``"velocity"``, or ``"effort"`` for the specified DOF

        This method will set the implicit Proportional-Derivative (PD) controller's Kps (stiffnesses) and Kds (dampings),
        defined via the ``set_gains``  method, of the selected DOF according to the following rule:

        .. list-table::
            :header-rows: 1

            * - Control mode
              - Stiffnesses
              - Dampings
            * - ``"position"``
              - Kps
              - Kds
            * - ``"velocity"``
              - 0
              - Kds
            * - ``"effort"``
              - 0
              - 0

        Args:
            mode (str): control mode to switch the DOF specified to. It can be ``"position"``, ``"velocity"`` or ``"effort"``
            dof_index (int): dof index to switch the control mode of.
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set 'velocity' as control mode for the panda_joint1 (0) joint for all envs
            >>> prims.switch_dof_control_mode("velocity", dof_index=0)
            >>>
            >>> # set 'effort' as control mode for the panda_joint1 (0) for the first, middle and last of the 5 envs
            >>> prims.switch_dof_control_mode("effort", dof_index=0, indices=np.array([0, 2, 4]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if mode == "velocity":
            self.set_gains(
                kps=self._backend_utils.create_zeros_tensor(
                    shape=[indices.shape[0], 1], dtype="float32", device=self._device
                ),
                kds=self._backend_utils.expand_dims(self._default_kds[indices, dof_index], 1)
                if self._backend != "warp"
                else self._default_kds.data[
                    indices, wp.array([dof_index], dtype=wp.int32, device=self._default_kds.device)
                ],
                indices=indices,
                joint_indices=[dof_index],
            )
        elif mode == "position":
            self.set_gains(
                kps=self._backend_utils.expand_dims(self._default_kps[indices, dof_index], 1)
                if self._backend != "warp"
                else self._default_kps.data[
                    indices, wp.array(dof_index, dtype=wp.int32, device=self._default_kds.device)
                ],
                kds=self._backend_utils.expand_dims(self._default_kds[indices, dof_index], 1)
                if self._backend != "warp"
                else self._default_kds.data[
                    indices, wp.array([dof_index], dtype=wp.int32, device=self._default_kds.device)
                ],
                indices=indices,
                joint_indices=[dof_index],
            )
        elif mode == "effort":
            self.set_gains(
                kps=self._backend_utils.create_zeros_tensor(
                    shape=[indices.shape[0], 1], dtype="float32", device=self._device
                ),
                kds=self._backend_utils.create_zeros_tensor(
                    shape=[indices.shape[0], 1], dtype="float32", device=self._device
                ),
                indices=indices,
                joint_indices=[dof_index],
            )
        return

    def set_solver_position_iteration_counts(
        self,
        counts: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the solver (position) iteration count for the articulations

        The solver iteration count determines how accurately contacts, drives, and limits are resolved.
        Search for *Solver Iteration Count* in |physx_docs| for more details.

        .. warning::

            Setting a higher number of iterations may improve the fidelity of the simulation, although it may affect its performance.

        Args:
            counts (Union[np.ndarray, torch.Tensor, wp.array]): number of iterations for the solver. Shape (M,).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set the position iteration count for all envs
            >>> prims.set_solver_position_iteration_counts(np.full((num_envs,), 64))
            >>>
            >>> # set only the position iteration count for the first, middle and last of the 5 envs
            >>> prims.set_solver_position_iteration_counts(np.full((3,), 64), indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        counts = self._backend_utils.to_list(counts)
        for i in indices:
            set_prim_property(self.prim_paths[i], "physxArticulation:solverPositionIterationCount", counts[read_idx])
            read_idx += 1
        return

    def get_solver_position_iteration_counts(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the solver (position) iteration count for the articulations

        The solver iteration count determines how accurately contacts, drives, and limits are resolved.
        Search for *Solver Iteration Count* in |physx_docs| for more details.

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: position iteration count. Shape (M,).

        Example:

        .. code-block:: python

            >>> # get all position iteration count. Returned shape is (5,) for the example: 5 envs
            >>> prims.get_solver_position_iteration_counts()
            [32 32 32 32 32]
            >>>
            >>> # get the position iteration count for the first, middle and last of the 5 envs. Returned shape is (3,)
            >>> prims.get_solver_position_iteration_counts(indices=np.array([0, 2, 4]))
            [32 32 32]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = np.zeros(shape=indices.shape[0], dtype="int32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            result[write_idx] = get_prim_property(self.prim_paths[i], "physxArticulation:solverPositionIterationCount")
            write_idx += 1
        result = self._backend_utils.convert(result, device=self._device, dtype="int32", indexed=True)
        return result

    def set_solver_velocity_iteration_counts(
        self,
        counts: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the solver (velocity) iteration count for the articulations

        The solver iteration count determines how accurately contacts, drives, and limits are resolved.
        Search for *Solver Iteration Count* in |physx_docs| for more details.

        .. warning::

            Setting a higher number of iterations may improve the fidelity of the simulation, although it may affect its performance.

        Args:
            counts (Union[np.ndarray, torch.Tensor, wp.array]): number of iterations for the solver. Shape (M,).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set the velocity iteration count for all envs
            >>> prims.set_solver_velocity_iteration_counts(np.full((num_envs,), 64))
            >>>
            >>> # set only the velocity iteration count for the first, middle and last of the 5 envs
            >>> prims.set_solver_velocity_iteration_counts(np.full((3,), 64), indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        counts = self._backend_utils.to_list(counts)
        for i in indices:
            set_prim_property(self.prim_paths[i], "physxArticulation:solverVelocityIterationCount", counts[read_idx])
            read_idx += 1
        return

    def get_solver_velocity_iteration_counts(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the solver (velocity) iteration count for the articulations

        The solver iteration count determines how accurately contacts, drives, and limits are resolved.
        Search for *Solver Iteration Count* in |physx_docs| for more details.

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: velocity iteration count. Shape (M,).

        Example:

        .. code-block:: python

            >>> # get all velocity iteration count. Returned shape is (5,) for the example: 5 envs
            >>> prims.get_solver_velocity_iteration_counts()
            [32 32 32 32 32]
            >>>
            >>> # get the velocity iteration count for the first, middle and last of the 5 envs. Returned shape is (3,)
            >>> prims.get_solver_velocity_iteration_counts(indices=np.array([0, 2, 4]))
            [32 32 32]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = np.zeros(shape=indices.shape[0], dtype="int32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            result[write_idx] = get_prim_property(self.prim_paths[i], "physxArticulation:solverVelocityIterationCount")
            write_idx += 1
        result = self._backend_utils.convert(result, device=self._device, dtype="int32", indexed=True)
        return result

    def set_stabilization_thresholds(
        self,
        thresholds: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the mass-normalized kinetic energy below which the articulation may participate in stabilization

        Search for *Stabilization Threshold* in |physx_docs| for more details

        Args:
            thresholds (Union[np.ndarray, torch.Tensor, wp.array]): stabilization thresholds to be applied. Shape (M,).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set the stabilization threshold for all envs
            >>> prims.set_stabilization_thresholds(np.full((num_envs,), 0.005))
            >>>
            >>> # set only the stabilization threshold for the first, middle and last of the 5 envs
            >>> prims.set_stabilization_thresholds(np.full((3,), 0.0051), indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        thresholds = self._backend_utils.to_list(thresholds)
        for i in indices:
            set_prim_property(self.prim_paths[i], "physxArticulation:stabilizationThreshold", thresholds[read_idx])
            read_idx += 1
        return

    def get_stabilization_thresholds(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the mass-normalized kinetic energy below which the articulations may participate in stabilization

        Search for *Stabilization Threshold* in |physx_docs| for more details

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: stabilization threshold. Shape (M,).

        Example:

        .. code-block:: python

            >>> # get all stabilization thresholds. Returned shape is (5,) for the example: 5 envs
            >>> prims.get_solver_velocity_iteration_counts()
            [0.001 0.001 0.001 0.001 0.001]
            >>>
            >>> # get the stabilization thresholds for the first, middle and last of the 5 envs. Returned shape is (3,)
            >>> prims.get_solver_velocity_iteration_counts(indices=np.array([0, 2, 4]))
            [0.001 0.001 0.001]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = np.zeros(shape=indices.shape[0], dtype="float32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            result[write_idx] = get_prim_property(self.prim_paths[i], "physxArticulation:stabilizationThreshold")
            write_idx += 1
        result = self._backend_utils.convert(result, dtype="float32", device=self._device, indexed=True)
        return result

    def set_enabled_self_collisions(
        self,
        flags: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the enable self collisions flag (``physxArticulation:enabledSelfCollisions``)

        Args:
            flags (Union[np.ndarray, torch.Tensor, wp.array]): true to enable self collision. otherwise false. shape (M,)
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # enable the self collisions flag for all envs
            >>> prims.set_enabled_self_collisions(np.full((num_envs,), True))
            >>>
            >>> # enable the self collisions flag only for the first, middle and last of the 5 envs
            >>> prims.set_enabled_self_collisions(np.full((3,), True), indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        flags = self._backend_utils.to_list(flags)
        for i in indices:
            set_prim_property(self.prim_paths[i], "physxArticulation:enabledSelfCollisions", flags[read_idx])
            read_idx += 1
        return

    def get_enabled_self_collisions(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the enable self collisions flag (``physxArticulation:enabledSelfCollisions``) for all articulations

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: self collisions flags (boolean interpreted as int). shape (M,)

        Example:

        .. code-block:: python

            >>> # get all self collisions flags. Returned shape is (5,) for the example: 5 envs
            >>> prims.get_enabled_self_collisions()
            [0 0 0 0 0]
            >>>
            >>> # get the self collisions flags for the first, middle and last of the 5 envs. Returned shape is (3,)
            >>> prims.get_enabled_self_collisions(indices=np.array([0, 2, 4]))
            [0 0 0]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = np.zeros(shape=indices.shape[0], dtype="bool")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            result[write_idx] = get_prim_property(self.prim_paths[i], "physxArticulation:enabledSelfCollisions")
            write_idx += 1
        result = self._backend_utils.convert(result, device=self._device, dtype="uint8", indexed=True)
        return result

    def set_sleep_thresholds(
        self,
        thresholds: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set the threshold for articulations to enter a sleep state

        Search for *Articulations and Sleeping* in |physx_docs| for more details

        Args:
            thresholds (Union[np.ndarray, torch.Tensor, wp.array]): sleep thresholds to be applied. shape (M,).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set the sleep threshold for all envs
            >>> prims.set_sleep_thresholds(np.full((num_envs,), 0.01))
            >>>
            >>> # set only the sleep threshold for the first, middle and last of the 5 envs
            >>> prims.set_sleep_thresholds(np.full((3,), 0.01), indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        thresholds = self._backend_utils.to_list(thresholds)
        for i in indices:
            set_prim_property(self.prim_paths[i], "physxArticulation:sleepThreshold", thresholds[read_idx])
            read_idx += 1
        return

    def get_sleep_thresholds(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the threshold for articulations to enter a sleep state

        Search for *Articulations and Sleeping* in |physx_docs| for more details

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: sleep thresholds. shape (M,).

        Example:

        .. code-block:: python

            >>> # get all sleep thresholds. Returned shape is (5,) for the example: 5 envs
            >>> prims.get_sleep_thresholds()
            [0.005 0.005 0.005 0.005 0.005]
            >>>
            >>> # get the sleep thresholds for the first, middle and last of the 5 envs. Returned shape is (3,)
            >>> prims.get_sleep_thresholds(indices=np.array([0, 2, 4]))
            [0.005 0.005 0.005]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = np.zeros(shape=indices.shape[0], dtype="float32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            result[write_idx] = get_prim_property(self.prim_paths[i], "physxArticulation:sleepThreshold")
            write_idx += 1
        result = self._backend_utils.convert(result, dtype="float32", device=self._device, indexed=True)
        return result

    def get_jacobian_shape(self) -> Union[np.ndarray, torch.Tensor, wp.array]:
        """Get the Jacobian matrix shape of a single articulation

        The Jacobian matrix maps the joint space velocities of a DOF to it's cartesian and angular velocities

        The shape of the Jacobian depends on the number of links (rigid bodies), DOFs,
        and whether the articulation base is fixed (e.g., robotic manipulators) or not (e.g,. mobile robots).

        * Fixed articulation base: ``(num_bodies - 1, 6, num_dof)``
        * Non-fixed articulation base: ``(num_bodies, 6, num_dof + 6)``

        Each body has 6 values in the Jacobian representing its linear and angular motion along the
        three coordinate axes. The extra 6 DOFs in the last dimension, for non-fixed base cases,
        correspond to the linear and angular degrees of freedom of the free root link

        Returns:
            Union[np.ndarray, torch.Tensor, wp.array]: shape of jacobian for a single articulation.

        Example:

        .. code-block:: python

            >>> # for the Franka Panda (a robotic manipulator with fixed base):
            >>> # - num_bodies: 12
            >>> # - num_dof: 9
            >>> prims.get_jacobian_shape()
            (11, 6, 9)
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        shape = self._physics_view.jacobian_shape
        return (shape[0] // 6, 6, shape[1])

    def get_mass_matrix_shape(self) -> Union[np.ndarray, torch.Tensor, wp.array]:
        """Get the mass matrix shape of a single articulation

        The mass matrix contains the generalized mass of the robot depending on the current configuration

        The shape of the max matrix depends on the number of DOFs: ``(num_dof, num_dof)``

        Returns:
            Union[np.ndarray, torch.Tensor, wp.array]: shape of mass matrix for a single articulation.

        Example:

        .. code-block:: python

            >>> # for the Franka Panda:
            >>> # - num_dof: 9
            >>> prims.get_jacobian_shape()
            (9, 9)
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        return self._physics_view.mass_matrix_shape

    def get_jacobians(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the Jacobian matrices of articulations in the view

        .. note::

            The first dimension corresponds to the amount of wrapped articulations while the last 3 dimensions are the
            Jacobian matrix shape. Refer to the ``get_jacobian_shape`` method for details about the Jacobian matrix shape

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: Jacobian matrices of articulations in the view.
            Shape is (M, jacobian_shape).

        Example:

        .. code-block:: python

            >>> # get the Jacobian matrices. Returned shape is (5, 11, 6, 9) for the example: 5 envs, 12 links, 9 DOFs
            >>> prims.get_jacobians()
            [[[[ 4.2254178e-09  0.0000000e+00  0.0000000e+00 ...  0.0000000e+00  0.0000000e+00  0.0000000e+00]
               [ 1.2093576e-08  0.0000000e+00  0.0000000e+00 ...  0.0000000e+00  0.0000000e+00  0.0000000e+00]
               [-6.0873992e-16  0.0000000e+00  0.0000000e+00 ...  0.0000000e+00  0.0000000e+00  0.0000000e+00]
               [ 1.4458647e-07  0.0000000e+00  0.0000000e+00 ...  0.0000000e+00  0.0000000e+00  0.0000000e+00]
               [-1.8178657e-10  0.0000000e+00  0.0000000e+00 ...  0.0000000e+00  0.0000000e+00  0.0000000e+00]
               [ 9.9999976e-01  0.0000000e+00  0.0000000e+00 ...  0.0000000e+00  0.0000000e+00  0.0000000e+00]]
              ...
              [[-4.5089945e-02  8.1210062e-02 -3.8495898e-02 ...  2.8108317e-02  0.0000000e+00 -4.9317405e-02]
               [ 4.2863289e-01  9.7436900e-04  4.0475106e-01 ...  2.4577195e-03  0.0000000e+00  9.9807423e-01]
               [ 6.5973169e-09 -4.2914307e-01 -2.1542320e-02 ...  2.8352857e-02  0.0000000e+00 -3.7625343e-02]
               [ 1.4458647e-07 -1.1999309e-02 -5.3927803e-01 ...  7.0976764e-01  0.0000000e+00  0.0000000e+00]
               [-1.8178657e-10  9.9992776e-01 -6.4710006e-03 ...  8.5178167e-03  0.0000000e+00  0.0000000e+00]
               [ 9.9999976e-01 -3.8743019e-07  8.4210289e-01 ... -7.0438433e-01  0.0000000e+00  0.0000000e+00]]]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._physics_view.get_jacobians()
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[indices]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_jacobians")
            return None

    def get_mass_matrices(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the mass matrices of articulations in the view

        .. note::

            The first dimension corresponds to the amount of wrapped articulations while the last 2 dimensions are the
            mass matrix shape. Refer to the ``get_mass_matrix_shape`` method for details about the mass matrix shape

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: mass matrices of articulations in the view.
            Shape is (M, mass_matrix_shape).

        Example:

        .. code-block:: python

            >>> # get the mass matrices. Returned shape is (5, 9, 9) for the example: 5 envs, 9 DOFs
            >>> prims.get_mass_matrices()
            [[[ 5.0900602e-01  1.1794259e-06  4.2570841e-01 -1.6387942e-06 -3.1573933e-02
               -1.9736715e-06 -3.1358242e-04 -6.0441834e-03  6.0441834e-03]
              [ 1.1794259e-06  1.0598221e+00  7.4729815e-07 -4.2621672e-01  2.3612277e-08
               -4.9647894e-02 -2.9080724e-07 -1.8432185e-04  1.8432130e-04]
              ...
              [-6.0441834e-03 -1.8432185e-04 -5.7159867e-03  4.0070520e-04  9.6930371e-04
                1.2324301e-04  2.5264668e-10  1.4055224e-02  0.0000000e+00]
              [ 6.0441834e-03  1.8432130e-04  5.7159867e-03 -4.0070404e-04 -9.6930366e-04
               -1.2324269e-04 -3.6906206e-10  0.0000000e+00  1.4055224e-02]]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._physics_view.get_mass_matrices()
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[indices]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_mass_matrices")
            return None

    def get_coriolis_and_centrifugal_forces(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the Coriolis and centrifugal forces (joint DOF forces required to counteract Coriolis and
        centrifugal forces for the given articulation state) of articulations in the view

        Search for *Coriolis and Centrifugal Forces* in |physx_docs| for more details

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: Coriolis and centrifugal forces of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get all coriolis and centrifugal forces. Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> prims.get_coriolis_and_centrifugal_forces()
            [[ 1.6842524e-06 -1.8269569e-04  5.2162073e-07 -9.7677548e-05  3.0365106e-07
               6.7375149e-06  6.1105780e-08 -4.6237556e-06 -4.1627968e-06]
             [ 1.6842524e-06 -1.8269569e-04  5.2162073e-07 -9.7677548e-05  3.0365106e-07
               6.7375149e-06  6.1105780e-08 -4.6237556e-06 -4.1627968e-06]
             [ 1.6842561e-06 -1.8269687e-04  5.2162375e-07 -9.7677454e-05  3.0365084e-07
               6.7375931e-06  6.1106007e-08 -4.6237533e-06 -4.1627954e-06]
             [ 1.6842561e-06 -1.8269687e-04  5.2162375e-07 -9.7677454e-05  3.0365084e-07
               6.7375931e-06  6.1106007e-08 -4.6237533e-06 -4.1627954e-06]
             [ 1.6842524e-06 -1.8269569e-04  5.2162073e-07 -9.7677548e-05  3.0365106e-07
               6.7375149e-06  6.1105780e-08 -4.6237556e-06 -4.1627968e-06]]
            >>>
            >>> # get finger joint coriolis and centrifugal forces: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_coriolis_and_centrifugal_forces(indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
            [[-4.6237556e-06 -4.1627968e-06]
             [-4.6237533e-06 -4.1627954e-06]
             [-4.6237556e-06 -4.1627968e-06]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            current_values = self._physics_view.get_coriolis_and_centrifugal_forces()
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            return result
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use get_coriolis_and_centrifugal_forces"
            )
            return None

    def get_generalized_gravity_forces(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        joint_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the generalized gravity forces (joint DOF forces required to counteract gravitational
        forces for the given articulation pose) of articulations in the view

        Search for *Generalized Gravity Force* in |physx_docs| for more details

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            joint_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): joint indices to specify which joints
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of dofs.
                                                                                 Defaults to None (i.e: all dofs).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: generalized gravity forces of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>>

            >>> # get all generalized gravity forces. Returned shape is (5, 9) for the example: 5 envs, 9 DOFs
            >>> prims.get_generalized_gravity_forces()
            [[ 1.32438602e-08 -6.90832138e+00 -1.08629465e-05  1.91585541e+01  5.13810664e-06
               1.18674076e+00  8.01788883e-06  5.18786255e-03 -5.18784765e-03]
             [ 1.32438602e-08 -6.90832138e+00 -1.08629465e-05  1.91585541e+01  5.13810664e-06
               1.18674076e+00  8.01788883e-06  5.18786255e-03 -5.18784765e-03]
             [ 1.32438585e-08 -6.90830994e+00 -1.08778477e-05  1.91585541e+01  5.14090061e-06
               1.18674052e+00  8.02161412e-06  5.18786255e-03 -5.18784765e-03]
             [ 1.32438585e-08 -6.90830994e+00 -1.08778477e-05  1.91585541e+01  5.14090061e-06
               1.18674052e+00  8.02161412e-06  5.18786255e-03 -5.18784765e-03]
             [ 1.32438602e-08 -6.90832138e+00 -1.08629465e-05  1.91585541e+01  5.13810664e-06
               1.18674076e+00  8.01788883e-06  5.18786255e-03 -5.18784765e-03]]
            >>>
            >>> # get finger joint generalized gravity forces: panda_finger_joint1 (7) and panda_finger_joint2 (8)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_generalized_gravity_forces(indices=np.array([0, 2, 4]), joint_indices=np.array([7, 8]))
            [[ 0.00518786 -0.00518785]
             [ 0.00518786 -0.00518785]
             [ 0.00518786 -0.00518785]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            joint_indices = self._backend_utils.resolve_indices(joint_indices, self.num_dof, self._device)
            current_values = self._physics_view.get_generalized_gravity_forces()
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, joint_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_generalized_gravity_forces")
            return None

    def get_body_masses(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        body_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get rigid body masses of articulations in the view

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            body_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): body indices to specify which bodies
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of bodies.
                                                                                 Defaults to None (i.e: all bodies).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: rigid body masses of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get all body masses. Returned shape is (5, 12) for the example: 5 envs, 12 rigid bodies
            >>> prims.get_body_masses()
            [[2.8142028  2.3599997  2.3795187  2.6498823  2.6948018  2.981282
              1.1285807  0.40529126 0.1  0.5583305  0.01405522 0.01405522]
             [2.8142028  2.3599997  2.3795187  2.6498823  2.6948018  2.981282
              1.1285807  0.40529126 0.1  0.5583305  0.01405522 0.01405522]
             [2.8142028  2.3599997  2.3795187  2.6498823  2.6948018  2.981282
              1.1285807  0.40529126 0.1  0.5583305  0.01405522 0.01405522]
             [2.8142028  2.3599997  2.3795187  2.6498823  2.6948018  2.981282
              1.1285807  0.40529126 0.1  0.5583305  0.01405522 0.01405522]
             [2.8142028  2.3599997  2.3795187  2.6498823  2.6948018  2.981282
              1.1285807  0.40529126 0.1  0.5583305  0.01405522 0.01405522]]
            >>>
            >>> # get finger body masses: panda_leftfinger (10) and panda_rightfinger (11)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_body_masses(indices=np.array([0, 2, 4]), body_indices=np.array([10, 11]))
            [[0.01405522 0.01405522]
             [0.01405522 0.01405522]
             [0.01405522 0.01405522]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            body_indices = self._backend_utils.resolve_indices(body_indices, self.num_bodies, self._device)
            current_values = self._backend_utils.move_data(self._physics_view.get_masses(), self._device)
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, body_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_body_masses")
            return None

    def get_body_inv_masses(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        body_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get rigid body inverse masses of articulations in the view

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            body_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): body indices to specify which bodies
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of bodies.
                                                                                 Defaults to None (i.e: all bodies).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: rigid body inverse masses of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get all body inverse masses. Returned shape is (5, 12) for the example: 5 envs, 12 rigid bodies
            >>> prims.get_body_inv_masses()
            [[ 0.35534042  0.42372888  0.42025304  0.37737525  0.3710848  0.33542618  0.8860687
               2.4673615  10. 1.7910539  71.14793  71.14793]
             [ 0.35534042  0.42372888  0.42025304  0.37737525  0.3710848  0.33542618  0.8860687
               2.4673615  10. 1.7910539  71.14793  71.14793]
             [ 0.35534042  0.42372888  0.42025304  0.37737525  0.3710848  0.33542618  0.8860687
               2.4673615  10. 1.7910539  71.14793  71.14793]
             [ 0.35534042  0.42372888  0.42025304  0.37737525  0.3710848  0.33542618  0.8860687
               2.4673615  10. 1.7910539  71.14793  71.14793]
             [ 0.35534042  0.42372888  0.42025304  0.37737525  0.3710848  0.33542618  0.8860687
               2.4673615  10. 1.7910539  71.14793  71.14793]]
            >>>
            >>> # get finger body inverse masses: panda_leftfinger (10) and panda_rightfinger (11)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2)
            >>> prims.get_body_inv_masses(indices=np.array([0, 2, 4]), body_indices=np.array([10, 11]))
            [[71.14793 71.14793]
             [71.14793 71.14793]
             [71.14793 71.14793]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            body_indices = self._backend_utils.resolve_indices(body_indices, self._num_bodies, self._device)
            current_values = self._backend_utils.move_data(self._physics_view.get_inv_masses(), self._device)
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, body_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_body_inv_masses")
            return None

    def get_body_coms(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        body_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get rigid body center of mass (COM) of articulations in the view.

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            body_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): body indices to specify which bodies
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of bodies.
                                                                                 Defaults to None (i.e: all bodies).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: rigid body center of mass positions and orientations
            of articulations in the view. Position shape is (M, K, 3), orientation shape is (M, k, 4).

        Example:

        .. code-block:: python

            >>> # get all body center of mass. Returned shape is (5, 12, 3) for positions and (5, 12, 4) for orientations
            >>> # for the example: 5 envs, 12 rigid bodies
            >>> positions, orientations = prims.get_body_coms()
            >>> positions
            [[[0. 0. 0.]
              [0. 0. 0.]
              ...
              [0. 0. 0.]
              [0. 0. 0.]]]
            >>> orientations
            [[[1. 0. 0. 0.]
              [1. 0. 0. 0.]
              ...
              [1. 0. 0. 0.]
              [1. 0. 0. 0.]]]
            >>>
            >>> # get finger body center of mass: panda_leftfinger (10) and panda_rightfinger (11) for the first,
            >>> # middle and last of the 5 envs. Returned shape is (3, 2, 3) for positions and (3, 2, 4) for orientations
            >>> positions, orientations = prims.get_body_coms(indices=np.array([0, 2, 4]), body_indices=np.array([10, 11]))
            >>> positions
            [[[0. 0. 0.]
              [0. 0. 0.]]
             [[0. 0. 0.]
              [0. 0. 0.]]
             [[0. 0. 0.]
              [0. 0. 0.]]]
            >>> orientations
            [[[1. 0. 0. 0.]
              [1. 0. 0. 0.]]
             [[1. 0. 0. 0.]
              [1. 0. 0. 0.]]
             [[1. 0. 0. 0.]
              [1. 0. 0. 0.]]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            body_indices = self._backend_utils.resolve_indices(body_indices, self._num_bodies, self._device)
            current_values = self._backend_utils.move_data(
                self._physics_view.get_coms().reshape((self.count, self.num_bodies, 7)), self._device
            )
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            positions = current_values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, body_indices, 0:3
            ]
            orientations = self._backend_utils.xyzw2wxyz(
                current_values[
                    self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices,
                    body_indices,
                    3:7,
                ]
            )
            return positions, orientations
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_body_coms")
            return None

    def get_body_inertias(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        body_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get rigid body inertias of articulations in the view

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            body_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): body indices to specify which bodies
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of bodies.
                                                                                 Defaults to None (i.e: all bodies).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: rigid body inertias of articulations in the view. Shape is (M, K, 9).

        Example:

        .. code-block:: python

            >>> # get all body inertias. Returned shape is (5, 12, 9) for the example: 5 envs, 12 rigid bodies
            >>> prims.get_body_inertias()
            [[[1.2988697e-06  0.0  0.0  0.0  1.6535528e-06  0.0  0.0  0.0  2.0331163e-06]
              [1.8686389e-06  0.0  0.0  0.0  1.4378986e-06  0.0  0.0  0.0  9.0681192e-07]
              ...
              [4.2041304e-10  0.0  0.0  0.0  3.9026365e-10  0.0  0.0  0.0  1.3347495e-10]
              [4.2041304e-10  0.0  0.0  0.0  3.9026365e-10  0.0  0.0  0.0  1.3347495e-10]]]
            >>>
            >>> # get finger body inertias: panda_leftfinger (10) and panda_rightfinger (11)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2, 9)
            >>> prims.get_body_inertias(indices=np.array([0, 2, 4]), body_indices=np.array([10, 11]))
            [[[4.2041304e-10  0.0  0.0  0.0  3.9026365e-10  0.0  0.0  0.0  1.3347495e-10]
              [4.2041304e-10  0.0  0.0  0.0  3.9026365e-10  0.0  0.0  0.0  1.3347495e-10]]
             ...
             [[4.2041304e-10  0.0  0.0  0.0  3.9026365e-10  0.0  0.0  0.0  1.3347495e-10]
              [4.2041304e-10  0.0  0.0  0.0  3.9026365e-10  0.0  0.0  0.0  1.3347495e-10]]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            body_indices = self._backend_utils.resolve_indices(body_indices, self.num_bodies, self._device)
            current_values = self._backend_utils.move_data(
                self._physics_view.get_inertias().reshape((self.count, self.num_bodies, 9)), self._device
            )
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, body_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_body_inertias")
            return None

    def get_body_inv_inertias(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        body_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get rigid body inverse inertias of articulations in the view

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            body_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): body indices to specify which bodies
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of bodies.
                                                                                 Defaults to None (i.e: all bodies).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: rigid body inverse inertias of articulations in the view.
            Shape is (M, K, 9).

        Example:

        .. code-block:: python

            >>> # get all body inverse inertias. Returned shape is (5, 12, 9) for the example: 5 envs, 12 rigid bodies
            >>> prims.get_body_inv_inertias()
            [[[7.6990012e+05  0.0  0.0  0.0  6.0475844e+05  0.0  0.0  0.0  4.9185578e+05]
              [5.3514888e+05  0.0  0.0  0.0  6.9545931e+05  0.0  0.0  0.0  1.1027645e+06]
              ...
              [2.3786132e+09  0.0  0.0  0.0  2.5623703e+09  0.0  0.0  0.0  7.4920422e+09]
              [2.3786132e+09  0.0  0.0  0.0  2.5623703e+09  0.0  0.0  0.0  7.4920422e+09]]]
            >>>
            >>> # get finger body inverse inertias: panda_leftfinger (10) and panda_rightfinger (11)
            >>> # for the first, middle and last of the 5 envs. Returned shape is (3, 2, 9)
            >>> prims.get_body_inv_inertias(indices=np.array([0, 2, 4]), body_indices=np.array([10, 11]))
            [[[2.3786132e+09  0.0  0.0  0.0  2.5623703e+09  0.0  0.0  0.0  7.4920422e+09]
              [2.3786132e+09  0.0  0.0  0.0  2.5623703e+09  0.0  0.0  0.0  7.4920422e+09]]
             ...
             [[2.3786132e+09  0.0  0.0  0.0  2.5623703e+09  0.0  0.0  0.0  7.4920422e+09]
              [2.3786132e+09  0.0  0.0  0.0  2.5623703e+09  0.0  0.0  0.0  7.4920422e+09]]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            body_indices = self._backend_utils.resolve_indices(body_indices, self._num_bodies, self._device)
            current_values = self._backend_utils.move_data(
                self._physics_view.get_inv_inertias().reshape((self.count, self.num_bodies, 9)), self._device
            )
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, body_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_body_inv_inertias")
            return None

    def get_body_disable_gravity(
        self,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        body_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        clone: bool = True,
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get whether the rigid bodies of articulations in the view have gravity disabled or not

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            body_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): body indices to specify which bodies
                                                                                 to query. Shape (K,).
                                                                                 Where K <= num of bodies.
                                                                                 Defaults to None (i.e: all bodies).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: rigid body gravity activation of articulations in the view.
            Shape is (M, K).

        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            body_indices = self._backend_utils.resolve_indices(body_indices, self._num_bodies, self._device)
            current_values = self._backend_utils.move_data(
                self._physics_view.get_disable_gravities().reshape((self.count, self.num_bodies)), self._device
            )
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[
                self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, body_indices
            ]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_body_disable_gravity")
            return None

    def set_body_masses(
        self,
        values: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        body_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set body masses for articulation bodies in the view

        Args:
            values (Union[np.ndarray, torch.Tensor, wp.array]): body masses for articulations in the view. shape (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            body_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): body indices to specify which bodies
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of bodies.
                                                                                 Defaults to None (i.e: all bodies).

        Example:

        .. code-block:: python

            >>> # set the masses for all the articulation rigid bodies to the indicated values.
            >>> # Since there are 5 envs, the masses are repeated 5 times
            >>> masses = np.tile(np.array([1.2, 1.1, 1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.2]), (num_envs, 1))
            >>> prims.set_body_masses(masses)
            >>>
            >>> # set the fingers masses: panda_leftfinger (10) and panda_rightfinger (11) to 0.2
            >>> # for the first, middle and last of the 5 envs
            >>> masses = np.tile(np.array([0.2, 0.2]), (3, 1))
            >>> prims.set_body_masses(masses, indices=np.array([0, 2, 4]), body_indices=np.array([10, 11]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return

        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            body_indices = self._backend_utils.resolve_indices(body_indices, self.num_bodies, self._device)
            data = self._backend_utils.clone_tensor(self._physics_view.get_masses(), device="cpu")
            data = self._backend_utils.assign(
                self._backend_utils.move_data(values, device="cpu"),
                data,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, body_indices],
            )
            self._physics_view.set_masses(data, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_body_masses")

    def set_body_inertias(
        self,
        values: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        body_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set body inertias for articulation bodies in the view.

        Args:
            values (Union[np.ndarray, torch.Tensor, wp.array]): body inertias for articulations in the view. shape (M, K, 9).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            body_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): body indices to specify which bodies
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of bodies.
                                                                                 Defaults to None (i.e: all bodies).

        Example:

        .. code-block:: python

            >>> # set the inertias for all the articulation rigid bodies to the indicated values.
            >>> # Since there are 5 envs, the inertias are repeated 5 times
            >>> inertias = np.tile(np.array([0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]), (num_envs, prims.num_bodies, 1))
            >>> prims.set_body_inertias(inertias)
            >>>
            >>> # set the fingers inertias: panda_leftfinger (10) and panda_rightfinger (11) to 0.2
            >>> # for the first, middle and last of the 5 envs
            >>> inertias = np.tile(np.array([0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]), (3, 2, 1))
            >>> prims.set_body_inertias(inertias, indices=np.array([0, 2, 4]), body_indices=np.array([10, 11]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return

        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            body_indices = self._backend_utils.resolve_indices(body_indices, self.num_bodies, self._device)
            data = self._backend_utils.clone_tensor(self._physics_view.get_inertias(), device="cpu")
            data = self._backend_utils.assign(
                self._backend_utils.move_data(values, device="cpu"),
                data,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, body_indices],
            )
            self._physics_view.set_inertias(data, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_body_inertias")

    def set_body_coms(
        self,
        positions: Union[np.ndarray, torch.Tensor, wp.array] = None,
        orientations: Union[np.ndarray, torch.Tensor, wp.array] = None,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        body_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set body center of mass (COM) positions and orientations for articulation bodies in the view.

        Args:
            positions (Union[np.ndarray, torch.Tensor, wp.array]): body center of mass positions for articulations in the view. shape (M, K, 3).
            orientations (Union[np.ndarray, torch.Tensor, wp.array]): body center of mass orientations for articulations in the view. shape (M, K, 4).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            body_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): body indices to specify which bodies
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of bodies.
                                                                                 Defaults to None (i.e: all bodies).

        Example:

        .. code-block:: python

            >>> # set the center of mass for all the articulation rigid bodies to the indicated values.
            >>> # Since there are 5 envs, the inertias are repeated 5 times
            >>> positions = np.tile(np.array([0.01, 0.02, 0.03]), (num_envs, prims.num_bodies, 1))
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (num_envs, prims.num_bodies, 1))
            >>> prims.set_body_coms(positions, orientations)
            >>>
            >>> # set the fingers center of mass: panda_leftfinger (10) and panda_rightfinger (11) to 0.2
            >>> # for the first, middle and last of the 5 envs
            >>> positions = np.tile(np.array([0.01, 0.02, 0.03]), (3, 2, 1))
            >>> orientations = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (3, 2, 1))
            >>> prims.set_body_coms(positions, orientations, indices=np.array([0, 2, 4]), body_indices=np.array([10, 11]))
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return

        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            body_indices = self._backend_utils.resolve_indices(body_indices, self.num_bodies, self._device)
            coms = self._physics_view.get_coms().reshape((self.count, self.num_bodies, 7))
            if positions is not None:
                if self._backend == "warp":
                    coms = self._backend_utils.assign(
                        self._backend_utils.move_data(positions, device="cpu"),
                        coms,
                        [indices, body_indices, wp.array([0, 1, 2], dtype=wp.int32, device="cpu")],
                    )
                else:
                    coms[
                        self._backend_utils.expand_dims(indices, 1), body_indices, 0:3
                    ] = self._backend_utils.move_data(positions, device="cpu")
            if orientations is not None:
                if self._backend == "warp":
                    coms = self._backend_utils.assign(
                        self._backend_utils.move_data(self._backend_utils.wxyz2xyzw(orientations), device="cpu"),
                        coms,
                        [indices, body_indices, wp.array([3, 4, 5, 6], dtype=wp.int32, device="cpu")],
                    )
                else:
                    coms[
                        self._backend_utils.expand_dims(indices, 1), body_indices, 3:7
                    ] = self._backend_utils.move_data(orientations[:, :, [1, 2, 3, 0]], device="cpu")
            self._physics_view.set_coms(coms, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_body_coms")

    def set_body_disable_gravity(
        self,
        values: Union[np.ndarray, torch.Tensor, wp.array],
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
        body_indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set body gravity activation articulation bodies in the view.

        Args:
            values (Union[np.ndarray, torch.Tensor, wp.array]): body gravity activation for articulations in the view. shape (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            body_indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): body indices to specify which bodies
                                                                                 to manipulate. Shape (K,).
                                                                                 Where K <= num of bodies.
                                                                                 Defaults to None (i.e: all bodies).
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return

        indices = self._backend_utils.resolve_indices(indices, self.count, "cpu")
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            body_indices = self._backend_utils.resolve_indices(body_indices, self.num_bodies, self._device)
            data = self._backend_utils.clone_tensor(self._physics_view.get_disable_gravities(), device="cpu")
            data = self._backend_utils.assign(
                self._backend_utils.move_data(values, device="cpu"),
                data,
                [self._backend_utils.expand_dims(indices, 1) if self._backend != "warp" else indices, body_indices],
            )
            self._physics_view.set_disable_gravities(data, indices)
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_body_disable_gravity")

    def get_fixed_tendon_stiffnesses(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the stiffness of fixed tendons for articulations in the view

        Search for *Fixed Tendon* in |physx_docs| for more details

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: fixed tendon stiffnesses of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get the fixed tendon stiffnesses
            >>> # for the ShadowHand articulation that has 4 fixed tendons (prims.num_fixed_tendons)
            >>> prims.get_fixed_tendon_stiffnesses()
            [[0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._physics_view.get_fixed_tendon_stiffnesses()
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[indices]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_fixed_tendon_stiffnesses")
            return None

    def get_fixed_tendon_dampings(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the dampings of fixed tendons for articulations in the view

        Search for *Fixed Tendon* in |physx_docs| for more details

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: fixed tendon dampings of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get the fixed tendon dampings
            >>> # for the ShadowHand articulation that has 4 fixed tendons (prims.num_fixed_tendons)
            >>> prims.get_fixed_tendon_dampings()
            [[0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._physics_view.get_fixed_tendon_dampings()
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[indices]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_fixed_tendon_dampings")
            return None

    def get_fixed_tendon_limit_stiffnesses(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the limit stiffness of fixed tendons for articulations in the view

        Search for *Fixed Tendon* in |physx_docs| for more details

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: fixed tendon stiffnesses of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get the fixed tendon limit stiffnesses
            >>> # for the ShadowHand articulation that has 4 fixed tendons (prims.num_fixed_tendons)
            >>> prims.get_fixed_tendon_limit_stiffnesses()
            [[0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._physics_view.get_fixed_tendon_limit_stiffnesses()
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[indices]
            return result
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use get_fixed_tendon_limit_stiffnesses"
            )
            return None

    def get_fixed_tendon_limits(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the limits of fixed tendons for articulations in the view

        Search for *Fixed Tendon* in |physx_docs| for more details

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: fixed tendon stiffnesses of articulations in the view.
            Shape is (M, K, 2).

        Example:

        .. code-block:: python

            >>> # get the fixed tendon limits
            >>> # for the ShadowHand articulation that has 4 fixed tendons (prims.num_fixed_tendons)
            >>> prims.get_fixed_tendon_limits()
            [[[-0.001  0.001] [-0.001  0.001] [-0.001  0.001] [-0.001  0.001]]
             [[-0.001  0.001] [-0.001  0.001] [-0.001  0.001] [-0.001  0.001]]
             [[-0.001  0.001] [-0.001  0.001] [-0.001  0.001] [-0.001  0.001]]
             [[-0.001  0.001] [-0.001  0.001] [-0.001  0.001] [-0.001  0.001]]
             [[-0.001  0.001] [-0.001  0.001] [-0.001  0.001] [-0.001  0.001]]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._physics_view.get_fixed_tendon_limits().reshape(
                (self.count, self.num_fixed_tendons, 2)
            )
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[indices]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_fixed_tendon_limits")
            return None

    def get_fixed_tendon_rest_lengths(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the rest length of fixed tendons for articulations in the view

        Search for *Fixed Tendon* in |physx_docs| for more details

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: fixed tendon stiffnesses of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get the fixed tendon rest lengths
            >>> # for the ShadowHand articulation that has 4 fixed tendons (prims.num_fixed_tendons)
            >>> prims.get_fixed_tendon_rest_lengths()
            [[0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._physics_view.get_fixed_tendon_rest_lengths()
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[indices]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_fixed_tendon_rest_lengths")
            return None

    def get_fixed_tendon_offsets(
        self, indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor, wp.indexedarray]:
        """Get the offsets of fixed tendons for articulations in the view

        Search for *Fixed Tendon* in |physx_docs| for more details

        Args:
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[np.ndarray, torch.Tensor, wp.indexedarray]: fixed tendon stiffnesses of articulations in the view.
            Shape is (M, K).

        Example:

        .. code-block:: python

            >>> # get the fixed tendon offsets
            >>> # for the ShadowHand articulation that has 4 fixed tendons (prims.num_fixed_tendons)
            >>> prims.get_fixed_tendon_offsets()
            [[0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]
             [0. 0. 0. 0.]]
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
            current_values = self._physics_view.get_fixed_tendon_offsets()
            if clone:
                current_values = self._backend_utils.clone_tensor(current_values, device=self._device)
            result = current_values[indices]
            return result
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_fixed_tendon_offsets")
            return None

    def set_fixed_tendon_properties(
        self,
        stiffnesses: Union[np.ndarray, torch.Tensor, wp.array] = None,
        dampings: Union[np.ndarray, torch.Tensor, wp.array] = None,
        limit_stiffnesses: Union[np.ndarray, torch.Tensor, wp.array] = None,
        limits: Union[np.ndarray, torch.Tensor, wp.array] = None,
        rest_lengths: Union[np.ndarray, torch.Tensor, wp.array] = None,
        offsets: Union[np.ndarray, torch.Tensor, wp.array] = None,
        indices: Optional[Union[np.ndarray, List, torch.Tensor, wp.array]] = None,
    ) -> None:
        """Set fixed tendon properties for articulations in the view

        Search for *Fixed Tendon* in |physx_docs| for more details

        Args:
            stiffnesses (Union[np.ndarray, torch.Tensor, wp.array]): fixed tendon stiffnesses for articulations in the view. shape (M, K).
            dampings (Union[np.ndarray, torch.Tensor, wp.array]): fixed tendon dampings for articulations in the view. shape (M, K).
            limit_stiffnesses (Union[np.ndarray, torch.Tensor, wp.array]): fixed tendon limit stiffnesses for articulations in the view. shape (M, K).
            limits (Union[np.ndarray, torch.Tensor, wp.array]): fixed tendon limits for articulations in the view. shape (M, K, 2).
            rest_lengths (Union[np.ndarray, torch.Tensor, wp.array]): fixed tendon rest lengths for articulations in the view. shape (M, K).
            offsets (Union[np.ndarray, torch.Tensor, wp.array]): fixed tendon offsets for articulations in the view. shape (M, K).
            indices (Optional[Union[np.ndarray, List, torch.Tensor, wp.array]], optional): indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).

        Example:

        .. code-block:: python

            >>> # set the limit stiffnesses and dampings
            >>> # for the ShadowHand articulation that has 4 fixed tendons (prims.num_fixed_tendons)
            >>> limit_stiffnesses = np.full((num_envs, prims.num_fixed_tendons), fill_value=10.0)
            >>> dampings = np.full((num_envs, prims.num_fixed_tendons), fill_value=0.1)
            >>> prims.set_fixed_tendon_properties(dampings=dampings, limit_stiffnesses=limit_stiffnesses)
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return

        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            current_stiffnesses = self._physics_view.get_fixed_tendon_stiffnesses()
            current_dampings = self._physics_view.get_fixed_tendon_dampings()
            current_limit_stiffnesses = self._physics_view.get_fixed_tendon_limit_stiffnesses()
            current_limits = self._physics_view.get_fixed_tendon_limits().reshape(
                (self.count, self.num_fixed_tendons, 2)
            )
            current_rest_lengths = self._physics_view.get_fixed_tendon_rest_lengths()
            current_offsets = self._physics_view.get_fixed_tendon_offsets()
            if stiffnesses is not None:
                current_stiffnesses = self._backend_utils.assign(
                    self._backend_utils.move_data(stiffnesses, device=self._device), current_stiffnesses, indices
                )
            if dampings is not None:
                current_dampings = self._backend_utils.assign(
                    self._backend_utils.move_data(dampings, device=self._device), current_dampings, indices
                )
            if limit_stiffnesses is not None:
                current_limit_stiffnesses = self._backend_utils.assign(
                    self._backend_utils.move_data(limit_stiffnesses, device=self._device),
                    current_limit_stiffnesses,
                    indices,
                )
            if limits is not None:
                current_limits = self._backend_utils.assign(
                    self._backend_utils.move_data(limits, device=self._device), current_limits, indices
                )
            if rest_lengths is not None:
                current_rest_lengths = self._backend_utils.assign(
                    self._backend_utils.move_data(rest_lengths, device=self._device), current_rest_lengths, indices
                )
            if offsets is not None:
                current_offsets = self._backend_utils.assign(
                    self._backend_utils.move_data(offsets, device=self._device), current_offsets, indices
                )
            self._physics_view.set_fixed_tendon_properties(
                current_stiffnesses,
                current_dampings,
                current_limit_stiffnesses,
                current_limits,
                current_rest_lengths,
                current_offsets,
                indices,
            )
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use set_fixed_tendon_properties")

    def pause_motion(self) -> None:
        """
        Pauses the motion of all articulations wrapped under the ArticulationView.
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(None, self.count, self._device)
            self._paused_position_targets = self._physics_view.get_dof_position_targets()
            self._paused_velocity_targets = self._physics_view.get_dof_velocity_targets()
            self._paused_dof_velocities = self._physics_view.get_dof_velocities()
            self._physics_view.set_dof_velocities(
                self._backend_utils.create_zeros_tensor(
                    shape=[self.count, self.num_dof], dtype="float32", device=self._device
                ),
                indices,
            )
            self._physics_view.set_dof_position_targets(self._physics_view.get_dof_positions(), indices)
            self._physics_view.set_dof_velocity_targets(
                self._backend_utils.create_zeros_tensor(
                    shape=[self.count, self.num_dof], dtype="float32", device=self._device
                ),
                indices,
            )
            self._paused_motion = True
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use pause_motion")
            return None

    def resume_motion(self):
        """
        Resumes the motion of all articulations wrapped under the ArticulationView using the position and velocity dof targets
        cached when pause_motion was called.
        """
        if not self._is_initialized:
            carb.log_warn("ArticulationView needs to be initialized.")
            return None
        if not self._paused_motion:
            carb.log_warn("ArticulationView needs to be paused in order to use resume_motion.")
            return None

        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(None, self.count, self._device)
            self._physics_view.set_dof_velocities(self._paused_dof_velocities, indices)
            self._physics_view.set_dof_position_targets(self._paused_position_targets, indices)
            self._physics_view.set_dof_velocity_targets(self._paused_velocity_targets, indices)
            self._paused_motion = False
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use resume_motion")
            return None
