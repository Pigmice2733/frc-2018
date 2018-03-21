import math

from ctre.wpi_talonsrx import WPI_TalonSRX
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from wpilib import Compressor, drive

from motioncontrol.execution import PathTracker, PositionProfileExecutor
from motioncontrol.path import Path

from motioncontrol.motionprofiling import PositionProfile
from motioncontrol.pid import PIDCoefficients, PIDParameters, PIDType
from motioncontrol.utils import (Completed, RobotCharacteristics, RobotState, interpolate,
                                 tank_drive_odometry, tank_drive_wheel_velocities)
from utils import NTStreamer
from wpilib.timer import Timer


class Drivetrain:
    robot_drive = drive.DifferentialDrive
    compressor = Compressor
    rotation = 0
    forward = 0
    left = 0
    right = 0
    curvature = None

    rotation_setpoint = None
    profile_executor = None

    robot_characteristics = RobotCharacteristics(
        acceleration_time=0.4,
        deceleration_time=3,
        max_speed=3.7,
        wheel_base=0.6096,
        encoder_ticks=1024 * 4,
        revolutions_to_distance=6 * math.pi * 0.02540,
        speed_scaling=3.7)
    wheel_distances = (0.0, 0.0)
    previous_motor_voltages = (0.0, 0.0)

    left_drive_motor = WPI_TalonSRX
    right_drive_motor = WPI_TalonSRX
    navx = AHRS

    robot_state = RobotState()

    def setup(self):
        self.odometry_streamer = NTStreamer(self.robot_state, "drivetrain", round_digits=2)
        self.path_streamer = NTStreamer([], "path", table="path_tracking")

    def forward_at(self, speed):
        self.left = speed
        self.right = speed
        self.forward = speed

    def curve_at(self, curvature):
        self.curvature = curvature

    def tank(self, left, right):
        self.left = left
        self.right = right

    def rotate(self, degrees=0):
        radians = math.radians(degrees)

        if radians == self.rotation_setpoint:
            return self._update_executor()

        self.rotation_setpoint = radians

        orientation = self.get_orientation() % (2 * math.pi)
        self._set_orientation(orientation)

        motion_profile = PositionProfile(
            self.robot_characteristics, target_distance=(radians - orientation))

        coefs = PIDCoefficients(p=0.38, i=0.02, d=0)
        params = PIDParameters(coefs, input_max=2 * math.pi, input_min=0, continuous=False)
        self.profile_executor = PositionProfileExecutor(
            params, motion_profile, Timer.getFPGATimestamp, self.get_orientation,
            lambda output: self.tank(-output, output), 0.04)

        return Completed(done=False)

    def reset_motion_profile(self):
        self.profile_executor = None
        self.rotation_setpoint = None

    def set_path(self, max_speed: float, end_threshold: float, path: Path):
        robot_characteristics = RobotCharacteristics(
            acceleration_time=self.robot_characteristics.acceleration_time,
            deceleration_time=self.robot_characteristics.deceleration_time,
            max_speed=max_speed,
            wheel_base=self.robot_characteristics.wheel_base,
            encoder_ticks=self.robot_characteristics.encoder_ticks,
            revolutions_to_distance=self.robot_characteristics.revolutions_to_distance,
            speed_scaling=self.robot_characteristics.speed_scaling)

        velocity_pid_coefs = PIDCoefficients(p=0.4, i=0.05, d=0.0, f=1.0)
        velocity_pid_params = PIDParameters(
            velocity_pid_coefs, output_max=4.0, output_min=-4.0, pid_type=PIDType.Rate)

        self.path_tracker = PathTracker(
            path, robot_characteristics, velocity_pid_params, Timer.getFPGATimestamp, 0.1,
            end_threshold, self.get_odometry,
            lambda speed: self.forward_at(speed / self.robot_characteristics.speed_scaling),
            self.curve_at, None)

        path_points = []
        for segment in path.segments:
            path_points.append(segment.start)
        path_points.append(path.segments[-1].end)
        self.path_streamer.send(path_points)

    def follow_path(self) -> (Completed, float):
        return self.path_tracker.update()

    def set_odometry(self, odometry: RobotState):
        self.robot_state = odometry
        self._set_orientation(odometry.rotation)

        self.left_drive_motor.setQuadraturePosition(0, 0)
        self.right_drive_motor.setQuadraturePosition(0, 0)
        self.wheel_distances = (0, 0)

    def get_odometry(self) -> RobotState:
        return self.robot_state

    def get_orientation(self) -> float:
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
                                               self.get_orientation(), self.robot_state.rotation,
                                               self.robot_state.position, velocity)

        self.wheel_distances = current_wheel_distances

    def _scale_speeds(self, vl: float, vr: float) -> (float, float):
        """Scales left and right motor speeds to a max of ±1.0 if either is
        greater
        """
        if abs(vl) >= abs(vr) and abs(vl) > 1.0:
            return math.copysign(1.0, vl), math.copysign(vr / vl, vr)
        if abs(vr) >= abs(vl) and abs(vr) > 1.0:
            return math.copysign(vl / vr, vl), math.copysign(1.0, vr)
        return vl, vr

    def _update_executor(self):
        completed = self.profile_executor.update()
        if completed.done:
            self.reset_motion_profile()
        return completed

    def execute(self):
        self._update_odometry()

        self.odometry_streamer.send(self.robot_state)

        if self.curvature is not None:
            if self.curvature > 1e-4:
                radius = abs(1 / self.curvature)
                scale = interpolate(0.4, 1, 0, 1.25, radius)
                scale = min(scale, 1.0)
                self.forward *= scale
            v_left, v_right = tank_drive_wheel_velocities(self.robot_characteristics.wheel_base,
                                                          self.forward, self.curvature)
            self.left, self.right = self._scale_speeds(v_left, v_right)
        else:
            self.left = math.copysign(math.pow(self.left, 2), self.left)
            self.right = math.copysign(math.pow(self.right, 2), self.right)

        self.robot_drive.tankDrive(self.left, self.right, squaredInputs=False)

        if self.left > 0.05 or self.right > 0.05:
            self.compressor.stop()
        else:
            self.compressor.start()

        self.rotation = 0
        self.forward = 0
        self.curvature = None
        self.left = 0
        self.right = 0
