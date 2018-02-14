import math

from magicbot.state_machine import AutonomousStateMachine, state

from .path_selector import Selector
from components.drivetrain import Drivetrain
from motioncontrol.path import Path, PathTuning
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

    left_position = RobotState(position=Point(0.76 + 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    right_position = RobotState(
        position=Point(8.23 - 0.76 - 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    def __init__(self):
        initial_states = [('left', self.left_position), ('right', self.right_position)]

        waypoints = {
            'left': Selector.mirror_waypoints(self.right_near_side_waypoints, 8.23),
            'right': self.right_near_side_waypoints
        }

        Selector.add_new_path(self.MODE_NAME, 'right', initial_states, waypoints)

    def initialize_path(self):
        right_starting_position = RobotState(
            position=Point(8.23 - 0.76 - 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

        center_starting_position = RobotState(
            position=Point(8.23 / 2, 1.01 / 2), rotation=math.pi / 2)

        center_path_tuning = PathTuning(
            lookahead=1.2, lookahead_reduction_factor=1.5, curvature_scaling=1.55)

        near_path_tuning = PathTuning(
            lookahead=1.12, lookahead_reduction_factor=1.1, curvature_scaling=1.28)

        far_path_tuning = PathTuning(
            lookahead=0.9, lookahead_reduction_factor=1.4, curvature_scaling=2.8)

        near_side_waypoints = [
            Point(8.23 - 0.762 - (2.62 / 3), 3.74 / 4.5),
            Point(8.23 - 2.62 + 0.25, 3.74 / 3.3),
            Point(8.23 - 2.62, 3.74 - (0.84 / 2))
        ]

        center_waypoints = [
            Point(8.23 - 2.62 - 0.3, 3.74 / 3.5),
            Point(8.23 - 2.62, 3.74 / 2),
            Point(8.23 - 2.62, 3.74 - (0.84 / 2))
        ]

        far_side_waypoints = [
            Point(8.23 - 0.48, 5.20),
            Point(8.23 - 1.5, 6.00),
            Point(2.16, 6.00),
            Point(1.44, 6.00),
            Point(0.55, 5.50),
            Point(0.55, 4.92),
            Point(2.16 - 1.01 / 2 - 0.3, 4.42),
            Point(2.16 - 1.01 / 2 - 0.2, 4.42)
        ]

        path = Path(center_path_tuning, center_starting_position, center_waypoints)

        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
