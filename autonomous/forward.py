from magicbot.state_machine import AutonomousStateMachine, timed_state

from components.drivetrain import Drivetrain
from components.intake import Intake


class ForwardAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Just Forward'
    DEFAULT = False

    drivetrain = Drivetrain
    intake = Intake

    @timed_state(duration=3, first=True)
    def start(self, initial_call):
        self.drivetrain.forward_at(0.3)
