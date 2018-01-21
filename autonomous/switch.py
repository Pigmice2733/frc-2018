from magicbot.state_machine import AutonomousStateMachine, state
from components.drivetrain import Drivetrain
from motioncontrol.path import Path


class SwitchAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Switch'
    DEFAULT = True

    drivetrain = Drivetrain

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.drivetrain.set_path(
                Path(Path.forward(50), Path.rotate(90), Path.forward(1)))
        if self.drivetrain.follow_path().done:
            self.done()
