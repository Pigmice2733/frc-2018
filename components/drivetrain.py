import wpilib


class Drivetrain:
    robot_drive = wpilib.RobotDrive
    rotation = 0
    forward = 0

    def forward_at(self, speed):
        self.forward = speed

    def turn_at(self, speed):
        self.rotation = speed

    def execute(self):
        self.robot_drive.arcadeDrive(self.forward, self.rotation)

        self.rotation = 0
        self.forward = 0
