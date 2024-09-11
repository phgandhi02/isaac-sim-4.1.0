# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

""" Tools for exponentially smoothing motion commands as they come in.

These tools are important for real-world execution. They ensure that discontinuities from discretely
changing motion commands are never directly sent to the underlying motion policies. They're smoothed
first. This allows motion policies whose evolution is smooth w.r.t. state to be smooth even given
discontinuities in commands.
"""

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np
import omni.isaac.cortex.math_util as math_util


class TargetAdapter(ABC):
    """Abstract interface to a target.

    Different use cases might have different target data structures. The SmoothedCommand object
    expects the target to have the API characterized here.

    Note that the target does not need to explicitly derive from this interface. It just needs to
    have this API.
    """

    @abstractmethod
    def get_position(self) -> np.array:
        """Retrieve the position target.

        Returns: The postion target in robot base coordinates.
        """
        raise NotImplementedError()

    @abstractmethod
    def has_rotation(self) -> bool:
        """Reports whether a the target has a rotational component.

        Returns: True if the target has a rotation component, False otherwise.
        """
        raise NotImplementedError()

    @abstractmethod
    def get_rotation_matrix(self) -> np.array:
        """Retrieve the rotational target as a rotation matrix.

        If has_rotation() returns true, this method should return the target rotation matrix in
        robot base coordinates. Otherwise, the behavior is undefined.

        Returns: The rotation matrix.
        """
        raise NotImplementedError()


class SmoothedCommand:
    """Tools for smoothing a stream of commands.

    The API includes:
    - reset(): Clear the current smoothed target data.
    - update(): Updating the data given a new target.

    A command consists of a position target, an optional rotation matrix target, and a posture
    config. The smoothed command is stored in members x (position), R (rotation matrix), q (posture
    config), directly accessible. On the first update of any given component, the component is set
    directly to the value provided. On subsequent updates the current value is averaged with the new
    value, creating an exponentially weighted average of values received. If a particular component
    is never received (e.g. the posture config, or the rotation matrix) the corresponding member is
    never initialized and remains None.

    Rotation recursive averaging is done by averaging the matrices themselves then projecting using
    math_util.proj_R(), which converts the (invalid) rotation matrix to a quaternion, normalizes,
    then converts back to a matrix.

    If use_distance_based_smoothing_regulation is set to True (default) the degree of smoothing
    diminishes to a minimum value of 0.5 as the system approaches the target. This feature is
    optimized for discrete jumps in targets. When a large jump is detected, the smoothing increases
    immediately to the interpolation_alpha provided on initialization, but then decreases to the
    minimum value as it nears the target. Note that the distance between rotation matrices factors
    into the distance to target.

    Args:
        interpolation_alpha: The value the interpolator is set to on initialization or when a
            discrete jump is detected. The value should be between 0 and 1. Larger values mean more
            smoothing. A value of 0 does no smoothing, a value of 1 keeps only the first value.
        use_distance_based_smoothing_regulation: If True, reduces the alpha as a function of
            distance down to a minimum value of min_alpha.
        min_alpha: If use_distance_based_smoothing_regulation is True, this is the min_alpha
            interpolated toward as the system nears the target.
    """

    def __init__(
        self,
        interpolation_alpha: Optional[float] = 0.95,
        use_distance_based_smoothing_regulation: Optional[float] = True,
        min_alpha: Optional[float] = 0.5,
    ):
        self.x = None
        self.R = None
        self.q = None
        self.init_interpolation_alpha = interpolation_alpha
        self.use_distance_based_smoothing_regulation = use_distance_based_smoothing_regulation
        self.min_alpha = min_alpha
        self.reset()

    def reset(self) -> None:
        """Reset the smoother back to its initial state."""
        self.x = None
        self.R = None
        self.q = None

        self.interpolation_alpha = self.init_interpolation_alpha

    def update(self, target: TargetAdapter, posture_config: np.ndarray, eff_x: np.ndarray, eff_R: np.ndarray) -> None:
        """Update the smoothed target given the current command (target, posture_config) and the
        current end-effector frame (eff_{x,R}).

        Args:
            target: A target object implementing the TargetAdapter API. (It need not have a rotational
                target.)
            posture_config: The posture configuration for this command. None is valid.
            eff_x: The position component of the current end-effector frame.
            eff_R: The rotational component of the current end-effector frame.
        """
        x_curr = target.get_position()
        R_curr = None
        if target.has_rotation():
            R_curr = target.get_rotation_matrix()
        q_curr = None
        if posture_config is not None:
            q_curr = np.array(posture_config)

        if self.x is None:
            self.x = eff_x
        if self.R is None:
            self.R = eff_R
        if self.q is None:
            self.q = q_curr

        # Clear the R if there's no rotation command. But don't do the same for the posture config.
        # Always keep around the previous posture config.
        if R_curr is None:
            self.R = None

        if self.use_distance_based_smoothing_regulation and self.interpolation_alpha > self.min_alpha:
            d = np.linalg.norm([eff_x - x_curr])
            if self.R is not None:
                d2 = np.linalg.norm([eff_R - self.R]) * 1.0
                d = max(d, d2)
            std_dev = 0.05
            scalar = 1.0 - np.exp(-0.5 * (d / std_dev) ** 2)
            a = scalar * self.interpolation_alpha + (1.0 - scalar) * self.min_alpha
        else:
            a = self.interpolation_alpha

        self.x = a * self.x + (1.0 - a) * x_curr
        if self.R is not None and R_curr is not None:
            self.R = math_util.proj_R(a * self.R + (1.0 - a) * R_curr)
        if self.q is not None and q_curr is not None:
            self.q = a * self.q + (1.0 - a) * q_curr
