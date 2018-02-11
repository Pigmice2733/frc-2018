import math

from magicbot.state_machine import AutonomousStateMachine, state

from .path_selector import Selector
from components.drivetrain import Drivetrain
from motioncontrol.path import Path
from motioncontrol.utils import Point, RobotState


class SwitchAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Switch'
    DEFAULT = True

    drivetrain = Drivetrain

    right_near_side_waypoints = [
        Point(8.23 - (2.62 / 4), 3.74 / 4),
        Point(8.23 - 2.62 + 0.1, 3.74 / 4),
        Point(8.23 - 2.62, 3.74 - 0.84 / 2)
    ]

    right_far_side_waypoints = [
        Point(8.23 - 0.48, 5.20),
        Point(8.23 - 1.5, 6.05),
        Point(2.16, 6.00),
        Point(1.44, 6.00),
        Point(0.8, 5.36),
        Point(0.8, 4.45),
        Point(2.16 - 1.01 / 2 - 0.2, 4.45)
    ]

    left_position = RobotState(
        position=Point(0.72 / 2, 0.84 / 2), rotation=math.pi / 2
    )

    right_position = RobotState(
        position=Point(8.23 - 0.72 / 2, 0.84 / 2), rotation=math.pi / 2)

    def __init__(self):
        initial_states = [('left', self.left_position), ('right', self.right_position)]

        waypoints = {
            'left': Selector.mirror_waypoints(self.right_near_side_waypoints, 8.23),
            'right': self.right_near_side_waypoints
        }

        Selector.add_new_path(
            self.MODE_NAME, 'right', initial_states, waypoints)

    def initialize_path(self):
        initial_robot_state = RobotState(
            position=Point(8.23 - 0.76 - 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

        # Robot width = 0.89m
        # Robot height = 0.1.01m
        # Side to middle of switch plate = 2.62m
        # Portal x width = 0.76m
        # Alliance station wall to edge of switch plate = 3.74m
        # Alliance station wall to close edge of scale platform = 6.65m

        near_side_waypoints = [
            Point(8.23 - 0.762 - (2.62 / 3), 3.74 / 4.5),
            Point(8.23 - 2.62 + 0.05, 3.74 / 3.3),
            Point(8.23 - 2.62, 3.74 - (0.84 / 2))
        ]

        far_side_waypoints = [
            Point(8.23 - 0.48, 5.20),
            Point(8.23 - 1.5, 6.05),
            Point(2.16, 6.00),
            Point(1.44, 6.00),
            Point(0.8, 5.36),
            Point(0.8, 4.45),
            Point(2.16 - 1.01 / 2 - 0.2, 4.45)
        ]

        path = Path(initial_robot_state,
                    1.1,
                    0,
                    1.4,
                    1.2,
                    False,
                    waypoints=far_side_waypoints)
        # path = Path(initial_robot_state, math.pi / 2, 1.3, 1.5,
        #             False, 1.5, waypoints=near_side_waypoints)

        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
