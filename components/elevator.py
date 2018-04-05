import wpilib
from ctre.wpi_talonsrx import WPI_TalonSRX

from motioncontrol.pid import PIDCoefficients, PIDController, PIDParameters
from motioncontrol.utils import clamp, interpolate
from utils import NTStreamer


class Elevator:
    winch = WPI_TalonSRX
    limit_switch = wpilib.DigitalInput
    speed = 0
    target_position = 0
    holding_position = None
    using_position_control = False

    def setup(self):
        self.winch.setQuadraturePosition(0, 0)

        self.winch.configPeakOutputForward(1, 0)
        self.winch.configPeakOutputReverse(-1, 0)
        self.winch.configNominalOutputForward(0, 0)
        self.winch.configNominalOutputReverse(0, 0)

        self.winch.setInverted(True)

        position_pid_coefs = PIDCoefficients(p=0.75, i=0.0007, d=0)
        position_pid_parameters = PIDParameters(
            position_pid_coefs, output_max=1, output_min=0)
        self.position_pid = PIDController(position_pid_parameters,
                                          wpilib.Timer.getFPGATimestamp)

        self.position_streamer = NTStreamer(
            0.0, "elevator/position", round_digits=2)

    def move_setpoint(self, speed: float):
        self.target_position += speed

    def hold(self):
        if self.holding_position is None:
            self.holding_position = self.get_position()
        self.target_position = self.holding_position

    def set_position(self, position: float):
        """Set a target position for the elevator

        'position' is measured in revolutions of the elevator winch.
        """
        self.target_position = position
        self.holding_position = None

    def get_position(self) -> float:
        return self.winch.getQuadraturePosition() / -4096

    def reset_position(self):
        self.winch.setQuadraturePosition(0, 0)
        self.target_position = 0

    def execute(self):
        position = self.get_position()

        self.position_streamer.send(position)

        if not self.limit_switch.get():
            self.winch.setQuadraturePosition(0, 0)
            position = 0
            if self.target_position < 0:
                self.target_position = 0

        if self.target_position > 13.4:
            self.target_position = 13.4

        if self.target_position is not None:
            if not self.using_position_control:
                self.position_pid.reset()
            self.using_position_control = True
            self.speed = self.position_pid.get_output(position,
                                                      self.target_position)

        else:
            if position < 2.3 and self.speed < 0:
                scale = interpolate(0.05, 1, 0, 2.3, position)
                self.speed *= scale

        if self.speed < 0:
            self.speed *= 0.4
        else:
            self.speed *= 1.4

        if position > 200:
            self.speed += 0.12

        self.speed = clamp(self.speed, -1, 1)

        print('target', self.target_position, 'current', position, 'speed', self.speed)

        self.winch.set(self.speed)
        self.speed = 0
