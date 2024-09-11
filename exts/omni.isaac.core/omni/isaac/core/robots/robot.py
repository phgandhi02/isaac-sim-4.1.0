# Copyright (c) 2021-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
from typing import Optional, Sequence

from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.controllers.articulation_controller import ArticulationController


class Robot(Articulation):
    """Implementation (on ``Articulation`` class) to deal with an articulation prim as a robot

    .. warning::

        The robot (articulation) object must be initialized in order to be able to operate on it.
        See the ``initialize`` method for more details.

    Args:
        prim_path (str): prim path of the Prim to encapsulate or create.
        name (str, optional): shortname to be used as a key by Scene class.
                              Note: needs to be unique if the object is added to the Scene. Defaults to "robot".
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
        articulation_controller (Optional[ArticulationController], optional): a custom ArticulationController which
                                                                              inherits from it. Defaults to creating the
                                                                              basic ArticulationController.

    Example:

    .. code-block:: python

        >>> import omni.isaac.core.utils.stage as stage_utils
        >>> from omni.isaac.core.robots import Robot
        >>>
        >>> usd_path = "/home/<user>/Documents/Assets/Robots/Franka/franka_alt_fingers.usd"
        >>> prim_path = "/World/envs/env_0/panda"
        >>>
        >>> # load the Franka Panda robot USD file
        >>> stage_utils.add_reference_to_stage(usd_path, prim_path)
        >>>
        >>> # wrap the prim as a robot (articulation)
        >>> prim = Robot(prim_path=prim_path, name="franka_panda")
        >>> print(prim)
        <omni.isaac.core.robots.robot.Robot object at 0x7fdd4875a1d0>
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "robot",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: bool = True,
        articulation_controller: Optional[ArticulationController] = None,
    ) -> None:
        Articulation.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            articulation_controller=articulation_controller,
        )
        self._sensors = list()
        return

    def post_reset(self) -> None:
        """Reset the robot to its default state

        .. note::

            For a robot, in addition to configuring the root prim's default position and spatial orientation
            (defined via the ``set_default_state`` method), the joint's positions, velocities, and efforts
            (defined via the ``set_joints_default_state`` method) are imposed

        Example:

        .. code-block:: python

            >>> prim.post_reset()
        """
        Articulation.post_reset(self)
        return
