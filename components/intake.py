from enum import Enum

from ctre.wpi_victorspx import WPI_VictorSPX
from wpilib import DoubleSolenoid


class ArmState(Enum):
    opened = 0
    closed = 1
    neutral = 2


class Intake:
    l_intake_motor = WPI_VictorSPX
    r_intake_motor = WPI_VictorSPX

    solenoid = DoubleSolenoid

    left = 0
    right = 0
    arm_state = ArmState.closed

    def open_arm(self):
        self.arm_state = ArmState.opened

    def close_arm(self):
        self.arm_state = ArmState.closed

    def neutral_arm(self):
        self.arm_state = ArmState.neutral

    def intake(self):
        self.left = -0.4
        self.right = 0.4

    def outtake(self):
        self.left = 0.45
        self.right = -0.45

    def hold(self):
        self.left = -0.175
        self.right = 0.175

    def strong_hold(self):
        self.left = -0.25
        self.right = 0.25

    def execute(self):
        self.l_intake_motor.set(self.left)
        self.r_intake_motor.set(self.right)

        if self.arm_state == ArmState.opened:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
        elif self.arm_state == ArmState.closed:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)

        self.left = 0
        self.right = 0
