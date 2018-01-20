from ctre.wpi_talonsrx import WPI_TalonSRX


class Climber:
    climber_motor = WPI_TalonSRX
    climbing = False

    def climb(self):
        self.climbing = True

    def execute(self):
        if self.climbing:
            self.climber_motor.set(0.2)
        else:
            self.climber_motor.set(0)
        self.climbing = False
