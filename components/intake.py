from enum import Enum

from ctre.wpi_victorspx import WPI_VictorSPX
from wpilib import DoubleSolenoid, Timer

from utils import NTStreamer

class Oscillator:
    def __init__(self, period: float):
        self.period = period
        self.low = True
        self.time = Timer.getFPGATimestamp()

    def __call__(self):
        if (Timer.getFPGATimestamp() - self.time) > (self.period / 2):
            self.low = not self.low
            self.time = Timer.getFPGATimestamp()
        return self.low

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
    oscillating = False
    oscillator = Oscillator(0.3)

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
        self.oscillating = True

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
        self.wheel_speed_streamer.send(self.wheel_speed)
        self.arm_state_streamer.send(self.arm_state)

        if self.oscillating:
            offset = self.oscillator()
            self.l_intake_motor.set(self.wheel_speed + (0 if offset else -0.1))
            self.r_intake_motor.set(-self.wheel_speed + (-0.1 if offset else 0))
        else:
            self.l_intake_motor.set(self.wheel_speed)
            self.r_intake_motor.set(-self.wheel_speed)

        if self.arm_state == ArmState.opened:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
        elif self.arm_state == ArmState.closed:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)

        self.wheel_speed = WheelSpeed.stopped
        self.oscillating = False
