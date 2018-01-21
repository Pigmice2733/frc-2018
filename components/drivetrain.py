import math

from wpilib import drive
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from ctre.wpi_talonsrx import WPI_TalonSRX

from motioncontrol.path import Path
from motioncontrol.execution import PathTracker
from motioncontrol.utils import (
    RobotCharacteristics, RobotState, Completed, Point)


class Drivetrain:
    robot_drive = drive.DifferentialDrive
    rotation = 0
    forward = 0
    robot_characteristics = RobotCharacteristics(
        acceleration_time=2, deceleration_time=3,
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
            self.rotation = speed**2 if speed >= 0 else -(speed**2)

    def curve_at(self, curvature):
        self.curvature = curvature

    def set_path(self, path: Path):
        self.path_tracker = PathTracker(
            path, self.robot_characteristics, 0.02, 0.125, 0.4,
            self.get_odometry, self.forward_at, self.turn_at)

    def follow_path(self) -> Completed:
        return self.path_tracker.update()

    def get_odometry(self):
        return self.robot_state

    def _update_odometry(self):
        new_wheel_distances = (
            self.left_drive_motor.getQuadraturePosition(),
            self.right_drive_motor.getQuadraturePosition())
        delta_left = new_wheel_distances[0] - self.wheel_distances[0]
        delta_right = new_wheel_distances[1] - self.wheel_distances[1]
        distance = (delta_left + delta_right) / 2

        theta = self.navx.getAngle()

        old_position = self.robot_state.position

        delta_x = distance * math.cos(theta)
        delta_y = distance * math.sin(theta)

        new_position = Point(
            old_position.x + delta_x, old_position.y + delta_y)

        self.wheel_distances = new_wheel_distances
        self.robot_state = RobotState(position=new_position, angle=theta)

    def _scale_speeds(self, vl: float, vr: float) -> (float, float):
        """Scales left and right motor speeds to a max of 1.0 if either is
        greater
        """

        if math.abs(vl) >= math.abs(vr) and math.abs(vl) > 1.0:
            return math.copysign(1.0, vl), math.copysign(vr / vl, vr)
        if math.abs(vr) >= math.abs(vl) and math.abs(vr) > 1.0:
            return math.copysign(vl / vr, vl), math.copysign(1.0, vr)
        return vl, vr

    def execute(self):
        self._update_odometry()

        if self.curvature is not None:
            vl = (self.forward *
                  (1 - (self.robot_characteristics.wheel_base / 2) *
                   self.curvature))
            vr = (self.forward *
                  (1 + (self.robot_characteristics.wheel_base / 2) *
                   self.curvature))
            # Ensure speeds are within +1.0/-1.0
            vl, vr = self._scale_speeds(vl, vr)
            self.robot_drive.tankDrive(vl, vr)
        else:
            self.robot_drive.arcadeDrive(self.forward, self.rotation)

        self.rotation = 0
        self.forward = 0
        self.curvature = None
