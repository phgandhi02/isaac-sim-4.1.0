import carb
import numpy as np

# Import packages.
from omni.isaac.core.controllers.base_controller import BaseController
from omni.isaac.core.utils.types import ArticulationAction


class AckermannController(BaseController):
    """
    This controller uses a bicycle model for Ackermann drive. The controller computes the left turning angle, right turning angle, and wheel rotation velocity of a robot's angular wheels. The controller can be used to find the appropriate joint values of a wheeled robot when it is being steered at a specific speed. The conversions are

        .. math::

            \theta_R = \\arctan[frac{1}{R + t}(b)] \n
            \theta_L = \\arctan[frac{1}{R - t}(b)] \n

            \pi_wRV = \\frac{1}{r}(s)

    where :math:`\theta` is the desired angle, `\pi` is the desired wheel rotation velocity, :math:`R` is the turning radius, :math:`b` is the distance between rear and front axles, :math:`t` is the wheel turning distance, :math:`r` is the wheel turning radius, and :math:`s` is the speed of the robot.


    Args:
        name (str): [description]
        wheel_base (float): Distance between front and rear axles in m
        track_width (float): Distance between left and right rear wheels of the robot in m
        turning_wheel_radius (float): Radius of the front wheels of the robot in m
        acceleration (float): Desired forward acceleration for the robot in m/s^2
        max_wheel_velocity (float): Maximum angular velocity of the robot wheel in rad/s
        use_acceleration (bool): Default set to false to use speed as input instead; use acceleration as an input when set to True
        invert_steering_angle (bool): Flips the sign of the steering angle; set to true for rear wheel steering
        max_wheel_rotation_angle (float): limits the maximum linear speed that will be produced by the controller. Defaults to 6.28 rad.
    """

    def __init__(
        self,
        name: str,
        wheel_base: float,
        track_width: float,
        turning_wheel_radius: float,
        max_wheel_velocity: float = 1.0e20,
        use_acceleration: bool = False,
        invert_steering_angle: bool = False,
        max_wheel_rotation_angle: float = 6.28,
    ) -> None:
        super().__init__(name)
        self.wheel_base = wheel_base
        self.track_width = track_width
        self.turning_wheel_radius = turning_wheel_radius
        self.max_wheel_velocity = max_wheel_velocity
        self.use_acceleration = use_acceleration
        self.invert_steering_angle = invert_steering_angle
        self.max_wheel_rotation_angle = max_wheel_rotation_angle
        self.wheel_rotation_velocity = 0.0

    def forward(self, command: np.ndarray) -> ArticulationAction:
        """Calculate right and left wheel angles given wheel rotation and velocity. If use_acceleration flag is enabled, desired velocity command (index 1) will be ignored.

        Args:
            command (np.ndarray): steering angle (rad), linear velocity [speed] (m/s), current linear velocity [current speed y] (m/s), delta time (s), acceleration (m/s^2)

        Returns:
            ArticulationAction: the articulation action to be applied to the robot.
        """
        if isinstance(command, list):
            command = np.array(command)

        if self.use_acceleration and command.shape[0] < 5:
            raise Exception("command should be length 5 for acceleration command")
        elif command.shape[0] < 2:
            raise Exception("command should be length 2 for velocity command")

        # avoid division by zero
        if self.turning_wheel_radius is None or self.turning_wheel_radius <= 0:
            carb.log_warn("turning wheel radius is invalid or is less than or equal to 0, skipping current step")
            return ArticulationAction()

        self.max_linear_velocity = self.max_wheel_velocity * self.turning_wheel_radius

        # limit linear velocity and steering angle
        command[0] = np.clip(command[0], -self.max_wheel_rotation_angle, self.max_wheel_rotation_angle)
        command[1] = np.clip(command[1], -self.max_linear_velocity, self.max_linear_velocity)

        # If input steering angle is less than 0.9 degrees ~ 0.0157 rad
        if np.fabs(command[0]) < 0.0157:
            self.left_wheel_angle = self.right_wheel_angle = 0.0
        else:
            R = ((-1.0 if self.invert_steering_angle else 1.0) * self.wheel_base) / np.tan(command[0])

            # Equations were simplied from the ones shown in https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html
            # compute the wheel angles by taking into account their offset from the center of the turning axle (where the bicycle model is centered), then computing the angles of each wheel relative to the turning point of the robot
            # Assuming front wheel drive
            self.left_wheel_angle = np.arctan(self.wheel_base / (R - 0.5 * self.track_width))
            self.right_wheel_angle = np.arctan(self.wheel_base / (R + 0.5 * self.track_width))

        # clamp wheel angles to max wheel rotation
        self.left_wheel_angle = np.clip(
            self.left_wheel_angle, -self.max_wheel_rotation_angle, self.max_wheel_rotation_angle
        )
        self.right_wheel_angle = np.clip(
            self.right_wheel_angle, -self.max_wheel_rotation_angle, self.max_wheel_rotation_angle
        )

        if self.use_acceleration:
            if command[3] == 0.0:
                carb.log_warn("Delta time for the simulation step is 0. Acceleration input will be ignored.")
                return ArticulationAction()

            # compute wheel rotation velocity --> (current linear velocity + acceleration * dt) / turning (driven) wheel radius
            self.wheel_rotation_velocity = (command[2] + (command[4] * command[3])) / self.turning_wheel_radius
        else:
            self.wheel_rotation_velocity = command[1] / self.turning_wheel_radius
        # clamp wheel rotation velocity to max wheel velocity
        self.wheel_rotation_velocity = np.clip(
            self.wheel_rotation_velocity, -self.max_wheel_velocity, self.max_wheel_velocity
        )

        # output wheel rotation angular velocity and wheel angles
        return ArticulationAction(
            joint_velocities=(self.wheel_rotation_velocity, self.wheel_rotation_velocity, None, None),
            joint_positions=(None, None, self.left_wheel_angle, self.right_wheel_angle),
        )
