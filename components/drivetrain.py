import wpilib

from motioncontrol.path import Path
from motioncontrol.execution import PathTracker
from motioncontrol.utilities import RobotCharacteristics, RobotState, Completed


class Drivetrain:
    robot_drive = wpilib.RobotDrive
    rotation = 0
    forward = 0
    robot_characteristics = RobotCharacteristics(
        acceleration_time=2, deceleration_time=3, max_speed=1)

    def forward_at(self, speed):
        self.forward = speed

    def turn_at(self, speed, squaredInputs=False):
        self.rotation = speed
        if squaredInputs:
            self.rotation = speed**2 if speed >= 0 else -(speed**2)

    def set_path(self, path: Path):
        self.path_tracker = PathTracker(
            path, self.robot_characteristics, 0.02, 0.125, 0.4,
            self.get_odometry, self.forward_at, self.turn_at)

    def follow_path(self) -> Completed:
        return self.path_tracker.update()

    def get_odometry(self):
        return RobotState(velocity=1)

    def execute(self):
        self.robot_drive.arcadeDrive(self.forward, self.rotation)

        self.rotation = 0
        self.forward = 0
