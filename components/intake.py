import enum

from ctre import CANTalon


class Action(enum.Enum):
    Intake = 1
    Stop = 0
    Outtake = -1


def mirror(l, r, val):
    l.set(val)
    r.set(-val)


class Intake:
    l_intake_motor = CANTalon
    r_intake_motor = CANTalon

    state = Action.Stop

    def intake(self):
        self.state = Action.Intake

    def outtake(self):
        self.state = Action.Outtake

    def stop(self):
        self.state = Action.Stop

    def execute(self):
        if self.state == Action.Intake:
            mirror(self.l_intake_motor, self.r_intake_motor, 1.0)
        elif self.state == Action.Outtake:
            mirror(self.l_intake_motor, self.r_intake_motor, -1.0)
        else:
            mirror(self.l_intake_motor, self.r_intake_motor, 0.0)

        self.state = Action.Stop
