# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Sequence, Tuple

import numpy as np
from omni.isaac.core.materials.visual_material import VisualMaterial
from omni.isaac.core.utils.types import XFormPrimState
from pxr import Usd


class _SinglePrimWrapper(object):
    def __init__(self, view) -> None:
        self._prim_view = view
        return

    def initialize(self, physics_sim_view=None) -> None:
        """Create a physics simulation view if not passed and using PhysX tensor API

        .. note::

            If the prim has been added to the world scene (e.g., ``world.scene.add(prim)``),
            it will be automatically initialized when the world is reset (e.g., ``world.reset()``).

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.

        Example:

        .. code-block:: python

            >>> prim.initialize()
        """
        self._prim_view.initialize(physics_sim_view=physics_sim_view)
        return

    @property
    def prim_path(self) -> str:
        """
        Returns:
            str: prim path in the stage
        """
        return self._prim_view.prim_paths[0]

    @property
    def name(self) -> Optional[str]:
        """
        Returns:
            str: name given to the prim when instantiating it. Otherwise None.
        """
        return self._prim_view.name

    @property
    def prim(self) -> Usd.Prim:
        """
        Returns:
            Usd.Prim: USD Prim object that this object holds.
        """
        return self._prim_view.prims[0]

    @property
    def non_root_articulation_link(self) -> bool:
        """Used to query if the prim is a non root articulation link

        Returns:
            bool: True if the prim itself is a non root link

        Example:

        .. code-block:: python

            >>> # for a wrapped articulation (where the root prim has the Physics Articulation Root property applied)
            >>> prim.non_root_articulation_link
            False
        """
        return self._prim_view._non_root_link

    def set_visibility(self, visible: bool) -> None:
        """Set the visibility of the prim in stage

        Args:
            visible (bool): flag to set the visibility of the usd prim in stage.

        Example:

        .. code-block:: python

            >>> # make prim not visible in the stage
            >>> prim.set_visibility(visible=False)
        """
        self._prim_view.set_visibilities(
            self._backend_utils.create_tensor_from_list([visible], dtype="bool", device=self._device)
        )
        return

    def get_visibility(self) -> bool:
        """
        Returns:
            bool: true if the prim is visible in stage. false otherwise.

        Example:

        .. code-block:: python

            >>> # get the visible state of an visible prim on the stage
            >>> prim.get_visibility()
            True
        """
        return self._prim_view.get_visibilities()[0]

    def post_reset(self) -> None:
        """Reset the prim to its default state (position and orientation).

        .. note::

            For an articulation, in addition to configuring the root prim's default position and spatial orientation
            (defined via the ``set_default_state`` method), the joint's positions, velocities, and efforts
            (defined via the ``set_joints_default_state`` method) are imposed

        Example:

        .. code-block:: python

            >>> prim.post_reset()
        """
        self._prim_view.post_reset()
        return

    def get_default_state(self) -> XFormPrimState:
        """Get the default prim states (spatial position and orientation).

        Returns:
            XFormPrimState: an object that contains the default state of the prim (position and orientation)

        Example:

        .. code-block:: python

            >>> state = prim.get_default_state()
            >>> state
            <omni.isaac.core.utils.types.XFormPrimState object at 0x7f33addda650>
            >>>
            >>> state.position
            [-4.5299529e-08 -1.8347054e-09 -2.8610229e-08]
            >>> state.orientation
            [1. 0. 0. 0.]
        """
        view_default_state = self._prim_view.get_default_state()
        default_state = self._view_state_conversion(view_default_state)
        return default_state

    def set_default_state(
        self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None
    ) -> None:
        """Set the default state of the prim (position and orientation), that will be used after each reset.

        Args:
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                       Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world frame of the prim.
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.

        Example:

        .. code-block:: python

            >>> # configure default state
            >>> prim.set_default_state(position=np.array([1.0, 0.5, 0.0]), orientation=np.array([1, 0, 0, 0]))
            >>>
            >>> # set default states during post-reset
            >>> prim.post_reset()
        """
        if position is not None:
            position = self._backend_utils.convert(position, device=self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, device=self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        self._prim_view.set_default_state(positions=position, orientations=orientation)
        return

    def apply_visual_material(self, visual_material: VisualMaterial, weaker_than_descendants: bool = False) -> None:
        """Apply visual material to the held prim and optionally its descendants.

        Args:
            visual_material (VisualMaterial): visual material to be applied to the held prim. Currently supports
                                              PreviewSurface, OmniPBR and OmniGlass.
            weaker_than_descendants (bool, optional): True if the material shouldn't override the descendants
                                                      materials, otherwise False. Defaults to False.

        Example:

        .. code-block:: python

            >>> from omni.isaac.core.materials import OmniGlass
            >>>
            >>> # create a dark-red glass visual material
            >>> material = OmniGlass(
            ...     prim_path="/World/material/glass",  # path to the material prim to create
            ...     ior=1.25,
            ...     depth=0.001,
            ...     thin_walled=False,
            ...     color=np.array([0.5, 0.0, 0.0])
            ... )
            >>> prim.apply_visual_material(material)
        """
        self._prim_view.apply_visual_materials(
            visual_materials=[visual_material], weaker_than_descendants=[weaker_than_descendants]
        )
        return

    def get_applied_visual_material(self) -> VisualMaterial:
        """Return the current applied visual material in case it was applied using apply_visual_material
        or it's one of the following materials that was already applied before: PreviewSurface, OmniPBR and OmniGlass.

        Returns:
            VisualMaterial: the current applied visual material if its type is currently supported.

        Example:

        .. code-block:: python

            >>> # given a visual material applied
            >>> prim.get_applied_visual_material()
            <omni.isaac.core.materials.omni_glass.OmniGlass object at 0x7f36263106a0>
        """
        return self._prim_view.get_applied_visual_materials()[0]

    def is_visual_material_applied(self) -> bool:
        """Check if there is a visual material applied

        Returns:
            bool: True if there is a visual material applied. False otherwise.

        Example:

        .. code-block:: python

            >>> # given a visual material applied
            >>> prim.is_visual_material_applied()
            True
        """
        return self._prim_view.is_visual_material_applied()[0]

    def set_world_pose(
        self, position: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None
    ) -> None:
        """Ses prim's pose with respect to the world's frame

        .. warning::

            This method will change (teleport) the prim pose immediately to the indicated value

        Args:
            position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                       Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the world frame of the prim.
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.

        .. hint::

            This method belongs to the methods used to set the prim state

        Example:

        .. code-block:: python

            >>> prim.set_world_pose(position=np.array([1.0, 0.5, 0.0]), orientation=np.array([1., 0., 0., 0.]))
        """
        if position is not None:
            position = self._backend_utils.convert(position, device=self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, device=self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        self._prim_view.set_world_poses(positions=position, orientations=orientation)
        return

    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get prim's pose with respect to the world's frame

        Returns:
            Tuple[np.ndarray, np.ndarray]: first index is the position in the world frame (with shape (3, )).
            Second index is quaternion orientation (with shape (4, )) in the world frame

        Example:

        .. code-block:: python

            >>> # if the prim is in position (1.0, 0.5, 0.0) with respect to the world frame
            >>> position, orientation = prim.get_world_pose()
            >>> position
            [1.  0.5 0. ]
            >>> orientation
            [1. 0. 0. 0.]
        """
        positions, orientations = self._prim_view.get_world_poses()
        if self._backend == "warp":
            return positions.numpy()[0], orientations.numpy()[0]
        else:
            return positions[0], orientations[0]

    def get_local_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get prim's pose with respect to the local frame (the prim's parent frame)

        Returns:
            Tuple[np.ndarray, np.ndarray]: first index is the position in the local frame (with shape (3, )).
            Second index is quaternion orientation (with shape (4, )) in the local frame

        Example:

        .. code-block:: python

            >>> # if the prim is in position (1.0, 0.5, 0.0) with respect to the world frame
            >>> position, orientation = prim.get_local_pose()
            >>> position
            [0. 0. 0.]
            >>> orientation
            [0. 0. 0.]
        """
        translations, orientations = self._prim_view.get_local_poses()
        if self._backend == "warp":
            return translations.numpy()[0], orientations.numpy()[0]
        else:
            return translations[0], orientations[0]

    def set_local_pose(
        self, translation: Optional[Sequence[float]] = None, orientation: Optional[Sequence[float]] = None
    ) -> None:
        """Set prim's pose with respect to the local frame (the prim's parent frame).

        .. warning::

            This method will change (teleport) the prim pose immediately to the indicated value

        Args:
            translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                          (with respect to its parent prim). shape is (3, ).
                                                          Defaults to None, which means left unchanged.
            orientation (Optional[Sequence[float]], optional): quaternion orientation in the local frame of the prim.
                                                          quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                          Defaults to None, which means left unchanged.
        .. hint::

            This method belongs to the methods used to set the prim state

        Example:

        .. code-block:: python

            >>> prim.set_local_pose(translation=np.array([1.0, 0.5, 0.0]), orientation=np.array([1., 0., 0., 0.]))
        """
        if translation is not None:
            translation = self._backend_utils.convert(translation, device=self._device)
            translation = self._backend_utils.expand_dims(translation, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, device=self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        self._prim_view.set_local_poses(translations=translation, orientations=orientation)
        return

    def get_world_scale(self) -> np.ndarray:
        """Get prim's scale with respect to the world's frame

        Returns:
            np.ndarray: scale applied to the prim's dimensions in the world frame. shape is (3, ).

        Example:

        .. code-block:: python

            >>> prim.get_world_scale()
            [1. 1. 1.]
        """
        return self._prim_view.get_world_scales()[0]

    def set_local_scale(self, scale: Optional[Sequence[float]]) -> None:
        """Set prim's scale with respect to the local frame (the prim's parent frame).

        Args:
            scale (Optional[Sequence[float]]): scale to be applied to the prim's dimensions. shape is (3, ).
                                          Defaults to None, which means left unchanged.

        Example:

        .. code-block:: python

            >>> # scale prim 10 times smaller
            >>> prim.set_local_scale(np.array([0.1, 0.1, 0.1]))
        """
        scale = self._backend_utils.convert(scale, device=self._device)
        scale = self._backend_utils.expand_dims(scale, 0)
        self._prim_view.set_local_scales(scales=scale)
        return

    def get_local_scale(self) -> np.ndarray:
        """Get prim's scale with respect to the local frame (the parent's frame)

        Returns:
            np.ndarray: scale applied to the prim's dimensions in the local frame. shape is (3, ).

        Example:

        .. code-block:: python

            >>> prim.get_local_scale()
            [1. 1. 1.]
        """
        if self._backend == "warp":
            return self._prim_view.get_local_scales().numpy()[0]
        else:
            return self._prim_view.get_local_scales()[0]

    def is_valid(self) -> bool:
        """Check if the prim path has a valid USD Prim at it

        Returns:
            bool: True is the current prim path corresponds to a valid prim in stage. False otherwise.

        Example:

        .. code-block:: python

            >>> # given an existing and valid prim
            >>> prims.is_valid()
            True
        """
        return self._prim_view.is_valid()

    def _view_state_conversion(self, view_state):
        # TODO: a temp function
        position = None
        orientation = None
        if view_state.positions is not None:
            position = view_state.positions[0]
        if view_state.orientations is not None:
            orientation = view_state.orientations[0]
        return XFormPrimState(position=position, orientation=orientation)
