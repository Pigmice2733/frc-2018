from enum import Enum

from ctre.wpi_talonsrx import WPI_TalonSRX


class State(Enum):
    REST = 1
    UP = 2
    DOWN = 3


class ScaleArm:
    scale_arm_motor = WPI_TalonSRX
    state = State.REST

    def up(self):
        self.state = State.UP

    def down(self):
        self.state = State.DOWN

    def execute(self):
        if self.state == State.UP:
            self.scale_arm_motor.set(0.2)
        elif self.state == State.DOWN:
            self.scale_arm_motor.set(-0.2)
        else:
            self.scale_arm_motor.set(0)
        self.state = State.REST
