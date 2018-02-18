from ctre.wpi_talonsrx import WPI_TalonSRX


class ScaleArm:
    scale_arm_motor = WPI_TalonSRX
    speed = 0

    def up(self):
        self.speed = -0.1

    def down(self):
        self.speed = 0.2

    def set_speed(self, speed):
        self.speed = speed - 0.1

    def execute(self):
        self.scale_arm_motor.set(self.speed)
        self.speed = 0
