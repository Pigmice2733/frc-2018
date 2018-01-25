import math

from wpilib import drive
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from ctre.wpi_talonsrx import WPI_TalonSRX

from motioncontrol.path import Path
from motioncontrol.execution import PathTracker
from motioncontrol.utils import (RobotCharacteristics, RobotState, Completed,
                                 Point)


class Drivetrain:
    robot_drive = drive.DifferentialDrive
    rotation = 0
    forward = 0
    curvature = 0
    robot_characteristics = RobotCharacteristics(
        acceleration_time=0.5, deceleration_time=3,
        max_speed=1, wheel_base=0.7)
    robot_state = RobotState()
    wheel_distances = (0.0, 0.0)

    left_drive_motor = WPI_TalonSRX
    right_drive_motor = WPI_TalonSRX
    navx = AHRS

    def forward_at(self, speed):
        self.forward = speed

    def turn_at(self, speed, squaredInputs=False):
        self.rotation = speed
        if squaredInputs:
            self.rotation = math.copysign(speed**2, speed)

    def curve_at(self, curvature):
        self.curvature = curvature

    def set_path(self, path: Path):
        self.path_tracker = PathTracker(
            path, self.robot_characteristics, 0.2, 0.05, 0.4,
            self.get_odometry, self.forward_at, self.curve_at)

    def follow_path(self) -> Completed:
        return self.path_tracker.update()

    def get_odometry(self):
        return self.robot_state

    def _update_odometry(self):
        new_wheel_distances = (
            self.left_drive_motor.getQuadraturePosition() / 1000,
            self.right_drive_motor.getQuadraturePosition() / 1000)
        delta_left = new_wheel_distances[0] - self.wheel_distances[0]
        delta_right = new_wheel_distances[1] - self.wheel_distances[1]
        distance = (delta_left + delta_right) / 2

        theta = math.radians(self.navx.getAngle()) + math.pi / 2

        old_position = self.robot_state.position

        delta_x = round(distance * math.cos(theta), 3)
        delta_y = round(distance * math.sin(theta), 3)

        current_velocity = (self.right_drive_motor.getQuadratureVelocity() -
                            self.left_drive_motor.getQuadratureVelocity()) / 2

        new_position = Point(old_position.x + delta_x,
                             old_position.y + delta_y)

        self.wheel_distances = new_wheel_distances
        current_velocity = (self.left_drive_motor.getQuadratureVelocity(
        ) + self.right_drive_motor.getQuadratureVelocity()) / 2
        self.robot_state = RobotState(
            velocity=current_velocity, position=new_position, angle=theta)

    def _scale_speeds(self, vl: float, vr: float) -> (float, float):
        """Scales left and right motor speeds to a max of 1.0 if either is
        greater
        """

        if abs(vl) >= abs(vr) and abs(vl) > 1.0:
            return math.copysign(1.0, vl), math.copysign(vr / vl, vr)
        if abs(vr) >= abs(vl) and abs(vr) > 1.0:
            return math.copysign(vl / vr, vl), math.copysign(1.0, vr)
        return vl, vr

    def execute(self):
        self._update_odometry()

        if self.curvature is not None:
            if math.fabs(self.curvature) > 1e-6:
                radius = 1.0 / self.curvature
                left_radius = (
                    radius - self.robot_characteristics.wheel_base / 2)
                right_radius = (
                    radius + self.robot_characteristics.wheel_base / 2)

                v_left = left_radius / radius
                v_right = right_radius / radius

                v_left, v_right = self._scale_speeds(
                    v_left, v_right)
                self.robot_drive.tankDrive(
                    v_left, v_right, squaredInputs=False)
            else:
                self.robot_drive.tankDrive(self.forward, self.forward)
        else:
            self.robot_drive.arcadeDrive(
                -self.forward, self.rotation, squaredInputs=False)

        self.rotation = 0
        self.forward = 0
        self.curvature = None
