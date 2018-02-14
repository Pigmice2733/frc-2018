import math

from magicbot.state_machine import AutonomousStateMachine, state

from .path_selector import Selector
from components.drivetrain import Drivetrain
from motioncontrol.path import Path, PathTuning
from motioncontrol.utils import Point, RobotState


class SwitchAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Side Switch'
    DEFAULT = True

    drivetrain = Drivetrain

    right_near_side_waypoints = [
        Point(8.23 - 0.762 - (2.62 / 3), 3.74 / 4.5),
        Point(8.23 - 2.62 + 0.25, 3.74 / 3.3),
        Point(8.23 - 2.62, 3.74 - (0.84 / 2))
    ]

    right_far_side_waypoints = [
        Point(8.23 - 0.48, 5.20),
        Point(8.23 - 1.5, 6.00),
        Point(2.16, 6.00),
        Point(1.44, 6.00),
        Point(0.55, 5.50),
        Point(0.55, 4.92),
        Point(2.16 - 1.01 / 2 - 0.2, 4.42),
        Point(2.16 - 1.01 / 2 - 0.1, 4.42)
    ]

    left_position = RobotState(position=Point(0.76 + 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    right_position = RobotState(
        position=Point(8.23 - 0.76 - 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    near_path_tuning = PathTuning(
        lookahead=1.29, lookahead_reduction_factor=1.3, curvature_scaling=1.4)

    far_path_tuning = PathTuning(
        lookahead=1.33, lookahead_reduction_factor=1.3, curvature_scaling=1.4)

    def __init__(self):
        initial_states = [('left', self.left_position), ('right', self.right_position)]

        waypoints = {
            'left': Selector.mirror_waypoints(self.right_near_side_waypoints, 8.23),
            'right': self.right_near_side_waypoints
        }

        Selector.add_new_path(self.MODE_NAME, 'left', initial_states, waypoints)

    def initialize_path(self):
        if Selector.starting_position == 'left':
            left_waypoints = Selector.mirror_waypoints(self.right_far_side_waypoints, 8.23)
            path = Path(self.far_path_tuning, self.left_position, left_waypoints)
        else:
            path = Path(self.far_path_tuning, self.right_position, self.right_far_side_waypoints)

        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
