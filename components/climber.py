from ctre.talonsrx import TalonSRX


class Climber:
    climber_motor = TalonSRX
    climbing = False

    def climb(self):
        self.climbing = True

    def execute(self):
        if self.climbing:
            self.climber_motor.set(0.2)
        else:
            self.climber_motor.set(0)
        self.climbing = False
