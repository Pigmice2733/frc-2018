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
        acceleration_time=0.8,
        deceleration_time=1.25,
        max_speed=2.5,
        wheel_base=0.6096,
        curvature_scaling=14,
        encoder_ticks=1024 * 4,
        revolutions_to_distance=6 * math.pi * 0.02540,
        speed_scaling=3.7)
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
        self.navx.setAngleAdjustment(math.degrees(-self.robot_state.rotation))
        self.wheel_distances = (0, 0)

        self.path_tracker = PathTracker(
            path,
            self.robot_characteristics,
            0.1,
            0.2,
            self.get_odometry,
            lambda speed: self.forward_at(
                speed / self.robot_characteristics.speed_scaling),
            lambda curvature: self.curve_at(
                curvature * self.robot_characteristics.curvature_scaling),
            lambda value: self.path_tracking_sender.send(value, "path_state"))

        self.path_tracking_sender.send(self.robot_state, "robot_state")
        self.path_tracking_sender.send(path.points, "path")

    def follow_path(self) -> Completed:
        return self.path_tracker.update()

    def get_odometry(self) -> RobotState:
        return self.robot_state

    def _update_odometry(self):
        encoder_scaling = (self.robot_characteristics.encoder_ticks /
                           self.robot_characteristics.revolutions_to_distance)

        current_wheel_distances = (
            -self.left_drive_motor.getQuadraturePosition() / encoder_scaling,
            self.right_drive_motor.getQuadraturePosition() / encoder_scaling)

        enc_velocity = (self.left_drive_motor.getQuadratureVelocity() -
                        self.right_drive_motor.getQuadratureVelocity()) / 2
        velocity = 10 * enc_velocity / encoder_scaling

        self.robot_state = tank_drive_odometry(
            current_wheel_distances,
            self.wheel_distances,
            self.robot_state.position,
            math.radians(-self.navx.getAngle()),
            velocity)

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
