from ctre.wpi_talonsrx import WPI_TalonSRX


class Climber:
    climber_motor = WPI_TalonSRX
    speed = 0

    def set_speed(self, speed):
        self.speed = speed

    def execute(self):
        return
        self.climber_motor.set(self.speed)
        self.speed = 0
