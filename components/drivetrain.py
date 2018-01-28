import math

from ctre.wpi_talonsrx import WPI_TalonSRX
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from wpilib import drive

from motioncontrol.execution import PathTracker
from motioncontrol.path import Path
from motioncontrol.utils import (Completed, RobotCharacteristics, RobotState,
                                 tank_drive_odometry,
                                 tank_drive_wheel_velocities)
from utils import NetworkTablesSender


class Drivetrain:
    robot_drive = drive.DifferentialDrive
    rotation = 0
    forward = 0
    curvature = 0
    robot_characteristics = RobotCharacteristics(
        acceleration_time=0.5, deceleration_time=3,
        max_speed=1, wheel_base=0.7)
    wheel_distances = (0.0, 0.0)

    left_drive_motor = WPI_TalonSRX
    right_drive_motor = WPI_TalonSRX
    navx = AHRS

    robot_state = RobotState()

    path_tracking_sender = NetworkTablesSender

    def forward_at(self, speed):
        self.forward = speed

    def turn_at(self, speed, squaredInputs=False):
        self.rotation = speed
        if squaredInputs:
            self.rotation = math.copysign(speed**2, speed)

    def curve_at(self, curvature):
        self.curvature = curvature

    def set_path(self, path: Path):
        self.robot_state = path.initial_state

        self.path_tracker = PathTracker(
            path,
            self.robot_characteristics,
            0.2,
            0.1,
            self.get_odometry,
            self.forward_at,
            self.curve_at,
            lambda value: self.path_tracking_sender.send(value, "path_state"))
        self.path_tracking_sender.send(self.robot_state, "robot_state")
        self.path_tracking_sender.send(path.points, "path")

    def follow_path(self) -> Completed:
        return self.path_tracker.update()

    def get_odometry(self) -> RobotState:
        return self.robot_state

    def _update_odometry(self):
        current_wheel_distances = (
            self.left_drive_motor.getQuadraturePosition() / (1024 * 4),
            -self.right_drive_motor.getQuadraturePosition() / (1024 * 4))

        current_velocity = ((self.left_drive_motor.getQuadratureVelocity() -
                             self.right_drive_motor.getQuadratureVelocity()) /
                            (2 * (1024 * 4)))

        self.robot_state = tank_drive_odometry(
            current_wheel_distances,
            self.wheel_distances,
            self.robot_state.position,
            math.radians(-self.navx.getAngle()) + math.pi / 2,
            current_velocity)

        self.wheel_distances = current_wheel_distances
        self.path_tracking_sender.send(self.robot_state, "robot_state")

    def _scale_speeds(self, vl: float, vr: float) -> (float, float):
        """Scales left and right motor speeds to a max of ±1.0 if either is
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
            v_left, v_right = tank_drive_wheel_velocities(
                self.robot_characteristics.wheel_base,
                self.forward,
                self.curvature)
            v_left, v_right = self._scale_speeds(
                v_left, v_right)
            self.robot_drive.tankDrive(
                v_left, v_right, squaredInputs=False)
        else:
            self.robot_drive.arcadeDrive(
                self.forward, self.rotation, squaredInputs=False)

        self.rotation = 0
        self.forward = 0
        self.curvature = None
