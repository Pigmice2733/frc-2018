import math

from magicbot.state_machine import AutonomousStateMachine, state

from components.drivetrain import Drivetrain
from motioncontrol.path import Path
from motioncontrol.utils import Point, RobotState


class SwitchAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Switch'
    DEFAULT = True

    drivetrain = Drivetrain

    def initialize_path(self):
        initial_robot_state = RobotState(
            position=Point(7.7796, 0.36), rotation=math.pi)

        # actions = [Path.rotate(-90),
        #            Path.forward(0.2),
        #            Path.rotate(90),

        #            Path.forward(1.625),
        #            Path.rotate(-30),
        #            Path.forward(0.469),
        #            Path.rotate(-30),
        #            Path.forward(0.813),
        #            Path.rotate(-30),

        #            Path.forward(1.562)]

        waypoints = [
            Point(7.7796, 0.56),
            Point(6.5315, 0.2),
            Point(6.3235, 0.4438),
            Point(6.1155, 0.6876),
            Point(6.1155, 2.7)
        ]

        path = Path(initial_robot_state, 1.8, 20, False, 3.0, waypoints=waypoints)

        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
