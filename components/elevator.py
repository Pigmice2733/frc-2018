import wpilib
from ctre.wpi_talonsrx import WPI_TalonSRX

from motioncontrol import pid


class Elevator:
    winch = WPI_TalonSRX
    goal_positions = [0, 50, 150]
    goal_index = goal_positions[0]

    def setup(self):
        self.winch.setQuadraturePosition(0)
        self.winch.configSelectedFeedbackSensor(WPI_TalonSRX.FeedbackDevice.QuadEncoder)

    def inc_goal(self):
        """Move position up to a higher goal position"""
        self.goal_index += 1
        self.goal_index = min(self.goal_index, len(self.goal_positions - 1))

    def dec_goal(self):
        """Move position down to a lower goal position"""
        self.goal_index -= 1
        self.goal_index = max(self.goal_index, len(self.goal_positions) - 1)

    def execute(self):
        target_position = self.goal_positions[self.goal_index]
        self.winch.set(WPI_TalonSRX.ControlMode.Position, target_position)
