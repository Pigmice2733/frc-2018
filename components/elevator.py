from ctre.wpi_talonsrx import WPI_TalonSRX

from motioncontrol.utils import interpolate


class Elevator:
    winch = WPI_TalonSRX
    speed = 0

    def setup(self):
        self.winch.setQuadraturePosition(0, 0)

    def set_speed(self, speed):
        self.speed = speed

    def execute(self):
        position = self.winch.getQuadraturePosition()

        if position < 1:
            scale = interpolate(0.05, 1, -0.05, 1, position)
            self.speed *= scale
        elif position > 9.5:
            scale = interpolate(1, 0.05, 9.5, 10.5, position)
            self.speed *= scale

        self.winch.set(self.speed)
        self.speed = 0
