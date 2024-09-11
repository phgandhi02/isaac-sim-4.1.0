# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Sequence

from omni.isaac.core.prims.xform_prim import XFormPrim


class BaseSensor(XFormPrim):
    """Provides a common properties and methods to deal with prims as a sensor

    .. note::

        This class, which inherits from ``XFormPrim``, does not currently add any new property/method to it.
        Its definition is oriented to future implementations.

    Args:
        prim_path (str): prim path of the Prim to encapsulate or create.
        name (str, optional): shortname to be used as a key by Scene class.
                                Note: needs to be unique if the object is added to the Scene.
                                Defaults to "base_sensor".
        position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                    Defaults to None, which means left unchanged.
        translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                        (with respect to its parent prim). shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                        (depends if translation or position is specified).
                                                        quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                        Defaults to None, which means left unchanged.
        scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                Defaults to None, which means left unchanged.
        visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.

    Raises:
        Exception: if translation and position defined at the same time
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "base_sensor",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
    ) -> None:
        XFormPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
        )
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
        XFormPrim.initialize(self, physics_sim_view=physics_sim_view)
        return

    def post_reset(self) -> None:
        # XFormPrim.post_reset(self)
        return
