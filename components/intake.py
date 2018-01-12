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

    cur_action = Action.Stop

    def intake(self):
        self.cur_action = Action.Intake

    def outtake(self):
        self.cur_action = Action.Outtake

    def stop(self):
        self.cur_action = Action.Stop

    def execute(self):
        if self.cur_action == Action.Intake:
            mirror(self.l_intake_motor, self.r_intake_motor, 1.0)
        elif self.cur_action == Action.Outtake:
            mirror(self.l_intake_motor, self.r_intake_motor, -1.0)
        elif self.cur_action == Action.Stop:
            mirror(self.l_intake_motor, self.r_intake_motor, 0.0)
