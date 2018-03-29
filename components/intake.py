from enum import Enum

from ctre.wpi_victorspx import WPI_VictorSPX
from ctre.wpi_talonsrx import WPI_TalonSRX
from wpilib import DoubleSolenoid, Timer, AnalogInput
from utils import NTStreamer
from motioncontrol.pid import PIDController, PIDCoefficients, PIDParameters


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


class WristPosition:
    down = 2
    up = 1300


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

    ir = AnalogInput

    wrist_motor = WPI_TalonSRX

    wheel_speed = WheelSpeed.stopped
    arm_state = ArmState.closed
    oscillating = False
    oscillator = Oscillator(0.3)
    wrist_position = WristPosition.down
    wrist_pid = PIDController(
        PIDParameters(PIDCoefficients(p=.0002, i=.0000001, d=0)),
        Timer.getFPGATimestamp)

    ir_stack = []

    def setup(self):
        self.arm_state_streamer = NTStreamer(self.arm_state,
                                             "intake/arm_state")
        self.wheel_speed_streamer = NTStreamer(self.wheel_speed,
                                               "intake/wheel_speed")

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

    def slide_average(self):
        if len(self.ir_stack) > 25:
            self.ir_stack.pop(0)
        self.ir_stack.append(self.ir.getValue())

    def get_raw_ir_value(self):
        return self.ir.getValue()

    def get_ir_value(self):
        if len(self.ir_stack) == 0:
            return self.ir.getValue()
        return sum(self.ir_stack) / len(self.ir_stack)

    def cube_is_in_range(self):
        return self.get_ir_value() > 490 or self.get_raw_ir_value() > 500

    def has_cube(self):
        return self.get_ir_value() > 500

    def reset_wrist(self):
        self.wrist_motor.setQuadraturePosition(0, 0)

    def wrist_down(self):
        self.wrist_position = WristPosition.down

    def wrist_up(self):
        self.wrist_position = WristPosition.up

    def execute(self):
        self.wheel_speed_streamer.send(self.wheel_speed)
        self.arm_state_streamer.send(self.arm_state)
        self.slide_average()

        print(self.get_ir_value())

        if self.oscillating:
            offset = 0.2 if self.oscillator() else -0.2
            self.l_intake_motor.set(-self.wheel_speed + offset)
            self.r_intake_motor.set(self.wheel_speed + offset)
        else:
            self.l_intake_motor.set(-self.wheel_speed)
            self.r_intake_motor.set(self.wheel_speed)

        if self.arm_state == ArmState.opened:
            self.solenoid.set(DoubleSolenoid.Value.kForward)
        elif self.arm_state == ArmState.closed:
            self.solenoid.set(DoubleSolenoid.Value.kReverse)

        current_wrist_position = self.wrist_motor.getQuadraturePosition()
        wrist_error = self.wrist_position - current_wrist_position

        output = self.wrist_pid.get_output(current_wrist_position,
                                           self.wrist_position)

        print('output:', output, 'current:', current_wrist_position, 'goal:',
              self.wrist_position)

        if wrist_error < 0:
            output *= 1

        self.wrist_motor.set(-output)

        self.wheel_speed = WheelSpeed.stopped
        self.oscillating = False
