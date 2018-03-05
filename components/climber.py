from ctre.wpi_talonsrx import WPI_TalonSRX

from utils import NTStreamer


class Climber:
    climber_motor = WPI_TalonSRX
    speed = 0

    def setup(self):
        self.speed_streamer = NTStreamer(self.speed, "climber/speed")

    def set_speed(self, speed):
        self.speed = speed

    def execute(self):
        self.speed_streamer.send(self.speed)
        self.climber_motor.set(self.speed)
        self.speed = 0
