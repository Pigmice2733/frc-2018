from ctre.wpi_talonsrx import WPI_TalonSRX

import wpilib
from motioncontrol.utils import interpolate
from motioncontrol.pid import PIDCoefficients, PIDParameters, PIDController


class Elevator:
    winch = WPI_TalonSRX
    limit_switch = wpilib.DigitalInput
    speed = 0
    target_position = None
    using_position_control = False

    def setup(self):
        self.winch.setQuadraturePosition(0, 0)
        self.winch.setInverted(True)

        pid_coefs = PIDCoefficients(0.04, 0.00001, 0.0)

        pid_parameters = PIDParameters(pid_coefs)

        self.pid = PIDController(pid_parameters, wpilib.Timer.getFPGATimestamp)

    def set_speed(self, speed: float):
        self.speed = speed

    def set_position(self, position: float):
        """Set a target position for the elevator

        'position' is measured in revolutions of the elevator winch.
        """
        self.target_position = position

    def get_position(self) -> float:
        return self.winch.getQuadraturePosition() / -4096

    def execute(self):
        position = self.winch.getQuadraturePosition() / -4096

        if not self.limit_switch.get():
            self.winch.setQuadraturePosition(0, 0)
            position = 0

        if self.target_position is not None:
            if not self.using_position_control:
                self.pid.reset()
            self.using_position_control = True
            self.speed = self.pid.get_output(position, self.target_position)
        else:
            if position < 2.3 and self.speed < 0:
                scale = interpolate(0.01, 1, 0, 2.3, position)
                self.speed *= scale

        if self.speed < 0:
            self.speed *= 0.2
        else:
            self.speed *= 1.4

        if position > 200:
            self.speed += 0.12

        self.winch.set(self.speed)
        self.speed = 0

        if self.target_position is None:
            self.using_position_control = False
            self.target_position = None
        else:
            self.target_position = None
