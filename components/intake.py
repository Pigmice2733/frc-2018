from enum import Enum

from ctre.wpi_victorspx import WPI_VictorSPX
from wpilib import DoubleSolenoid

from utils import NTStreamer


class ArmState(Enum):
    opened = "opened"
    closed = "closed"
    neutral = "neutral"


class WheelSpeed:
    stopped = 0.0
    hold = -0.175
    strong_hold = -0.25
    intake = -0.4
    outake = 0.6


class Intake:
    l_intake_motor = WPI_VictorSPX
    r_intake_motor = WPI_VictorSPX

    solenoid = DoubleSolenoid

    wheel_speed = WheelSpeed.stopped
    arm_state = ArmState.closed

    def setup(self):
        self.arm_state_streamer = NTStreamer(self.arm_state, "intake/arm_state")
        self.wheel_speed_streamer = NTStreamer(self.wheel_speed, "intake/wheel_speed")

    def toggle_arm(self):
        if self.arm_state == ArmState.opened or self.arm_state == ArmState.neutral:
            self.arm_state = ArmState.closed
        else:
            self.arm_state = ArmState.opened

    def open_arm(self):
        self.arm_state = ArmState.opened

    def close_arm(self):
        self.arm_state = ArmState.closed

    def neutral_arm(self):
        self.arm_state = ArmState.neutral

    def intake(self):
        self.wheel_speed = WheelSpeed.intake

    def outtake(self):
        self.wheel_speed = WheelSpeed.outake

    def hold(self):
        self.wheel_speed = WheelSpeed.hold

    def set_speed(self, speed):
        self.wheel_speed = speed

    def strong_hold(self):
        self.wheel_speed = WheelSpeed.strong_hold
        self.arm_state = ArmState.closed

    def execute(self):
        print(self.wheel_speed)
        self.wheel_speed_streamer.send(self.wheel_speed)
        self.arm_state_streamer.send(self.arm_state)

        self.l_intake_motor.set(self.wheel_speed)
        self.r_intake_motor.set(-self.wheel_speed)

        if self.arm_state == ArmState.opened:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
        elif self.arm_state == ArmState.closed:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)

        self.wheel_speed = WheelSpeed.stopped
