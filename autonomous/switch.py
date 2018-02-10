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
            position=Point(8.23 - 0.72 / 2, 0.84 / 2), rotation=math.pi / 2)

        # Robot width = 0.72m
        # Robot height = 0.84m
        # Side to middle of switch plate = 2.62m
        # Alliance station wall to edge of switch plate = 3.74m

        near_side_waypoints = [
            Point(8.23 - (2.62 / 4), 3.74 / 4),
            Point(8.23 - 2.62 + 0.1, 3.74 / 4),
            Point(8.23 - 2.62, 3.74 - 0.84 / 2)
        ]

        far_side_waypoints = [
            Point(),
            Point(),
            Point()
        ]

        path = Path(initial_robot_state, 1.8, 1, False, 3.0, waypoints=far_side_waypoints)
        path = Path(initial_robot_state, 1.8, 1, False, 3.0, waypoints=near_side_waypoints)

        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
