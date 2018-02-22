import math

from ctre.wpi_talonsrx import WPI_TalonSRX
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from wpilib import drive

from motioncontrol.execution import PathTracker
from motioncontrol.path import Path
from motioncontrol.utils import (Completed, RobotCharacteristics, RobotState, tank_drive_odometry,
                                 tank_drive_wheel_velocities)
from utils import NetworkTablesSender


class Drivetrain:
    robot_drive = drive.DifferentialDrive
    rotation = 0
    forward = 0
    curvature = None
    robot_characteristics = RobotCharacteristics(
        acceleration_time=0.7,
        deceleration_time=2.15,
        max_speed=1.85,
        wheel_base=0.6096,
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
        self._set_orientation(self.robot_state.rotation)

        self.left_drive_motor.setQuadraturePosition(0, 0)
        self.right_drive_motor.setQuadraturePosition(0, 0)
        self.wheel_distances = (0, 0)

        self.path_tracker = PathTracker(
            path, self.robot_characteristics, 0.12, 0.2, self.get_odometry,
            lambda speed: self.forward_at(speed / self.robot_characteristics.speed_scaling),
            self.curve_at, lambda value: self.path_tracking_sender.send(value, "path_state"))

        self.path_tracking_sender.send(self.robot_state, "robot_state")
        path_points = []
        for segment in path.segments:
            path_points.append(segment.start)
        path_points.append(path.segments[-1].end)
        self.path_tracking_sender.send(path_points, "path")

    def follow_path(self) -> Completed:
        return self.path_tracker.update()

    def get_odometry(self) -> RobotState:
        return self.robot_state

    def _get_orientation(self) -> float:
        return math.radians(-self.navx.getAngle())

    def _set_orientation(self, orientation) -> float:
        self.navx.setAngleAdjustment(0)
        self.navx.setAngleAdjustment(-self.navx.getAngle() - math.degrees(orientation))

    def _update_odometry(self):
        encoder_scaling = (self.robot_characteristics.encoder_ticks /
                           self.robot_characteristics.revolutions_to_distance)

        current_wheel_distances = (-self.left_drive_motor.getQuadraturePosition() / encoder_scaling,
                                   self.right_drive_motor.getQuadraturePosition() / encoder_scaling)

        enc_velocity = (self.right_drive_motor.getQuadratureVelocity() -
                        self.left_drive_motor.getQuadratureVelocity()) / 2

        velocity = 10 * enc_velocity / encoder_scaling

        self.robot_state = tank_drive_odometry(current_wheel_distances, self.wheel_distances,
                                               self._get_orientation(), self.robot_state.rotation,
                                               self.robot_state.position, velocity)

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
            if self.curvature > 1e-4:
                radius = abs(1 / self.curvature)
                if radius < 1.2 and (self.forward / self.robot_characteristics.speed_scaling) > 0.6:
                    self.forward *= (radius / 1.2)
            v_left, v_right = tank_drive_wheel_velocities(self.robot_characteristics.wheel_base,
                                                          self.forward, self.curvature)
            v_left, v_right = self._scale_speeds(v_left, v_right)
            self.robot_drive.tankDrive(v_left, v_right, squaredInputs=False)
        else:
            pitch = self.navx.getPitch()
            if abs(pitch) > 20:
                self.rotation = 0
                self.forward = pitch / 360 # needs tuning
            self.robot_drive.arcadeDrive(self.forward, self.rotation, squaredInputs=False)

        self.rotation = 0
        self.forward = 0
        self.curvature = None
