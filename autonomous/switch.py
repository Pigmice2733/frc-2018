from magicbot.state_machine import AutonomousStateMachine, state
from components.drivetrain import Drivetrain
from motioncontrol.path import Path
from motioncontrol.utils import RobotState, Point


class SwitchAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Switch'
    DEFAULT = True

    drivetrain = Drivetrain

    def __init__(self, *args, **kwargs):
        super().__init__()
        self.drivetrain.robot_state = RobotState(position=Point(5, 0))

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.drivetrain.set_path(
                Path(Path.forward(10), Path.rotate(90), Path.forward(1)))
        if self.drivetrain.follow_path().done:
            self.done()
