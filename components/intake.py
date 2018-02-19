from ctre.wpi_victorspx import WPI_VictorSPX


class Intake:
    l_intake_motor = WPI_VictorSPX
    r_intake_motor = WPI_VictorSPX

    left = 0
    right = 0

    def set(self, left, right):
        self.left = left
        self.right = right

    def intake(self):
        self.left = 0.6
        self.right = -0.6

    def outtake(self):
        self.left = -0.5
        self.right = 0.5

    def hold(self):
        self.left = 0.1
        self.right = -0.25

    def execute(self):
        self.l_intake_motor.set(self.left)
        self.r_intake_motor.set(self.right)

        self.left = 0
        self.right = 0
