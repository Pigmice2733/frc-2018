import wpilib
from ctre.wpi_talonsrx import WPI_TalonSRX

from motioncontrol import pid


class Elevator:
    winch = WPI_TalonSRX
    winch_encoder = wpilib.AnalogPotentiometer
    goal_positions = [0, 50, 150]
    goal_index = goal_positions[0]

    def setup(self):
        parameters = pid.PIDParameters(coefs=pid.PIDCoefficients(p=1.0, i=0.3, d=0.1))
        self.pid = pid.PIDController(parameters, wpilib.timer.Timer.getFPGATimestamp)
        self.previous_encoder_value = self.winch_encoder.get() % self.winch_encoder.fullRange
        self.winch_rotations = 0

    def inc_goal(self):
        """Move position up to a higher goal position"""
        self.goal_index += 1
        self.goal_index = min(self.goal_index, len(self.goal_positions - 1))

    def dec_goal(self):
        """Move position down to a lower goal position"""
        self.goal_index -= 1
        self.goal_index = max(self.goal_index, len(self.goal_positions) - 1)

    def _update_encoder(self) -> float:
        """Update encoder rotations and return current position"""
        current_encoder_value = self.winch_encoder.get() % self.winch_encoder.fullRange
        delta = current_encoder_value - self.previous_encoder_value
        full_range = self.winch_encoder.fullRange

        if delta >= full_range / 2:
            self.winch_rotations += 1
        elif delta <= -full_range / 2:
            self.winch_rotations -= 1

        self.previous_encoder_value = current_encoder_value
        return self.winch_rotations + current_encoder_value / full_range

    def execute(self):
        current_position = self._update_encoder()
        target_position = self.goal_positions[self.goal_index]
        output = self.pid.get_output(current_position, target_position)
        self.winch.set(output)
