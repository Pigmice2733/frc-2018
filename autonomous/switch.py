import math

from magicbot.state_machine import AutonomousStateMachine, state
from components.drivetrain import Drivetrain
from motioncontrol.path import Path
from motioncontrol.utils import RobotState, Point


class SwitchAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Switch'
    DEFAULT = True

    drivetrain = Drivetrain

    def initialize_path(self):
        initial_robot_state = RobotState(
            position=Point(26.0, 0.0), rotation=math.pi / 2)

        waypoints = [Path.forward(10), Path.rotate(
            90), Path.forward(10), Path.rotate(-90), Path.forward(10)]
        path = Path(initial_robot_state, waypoints)

        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
