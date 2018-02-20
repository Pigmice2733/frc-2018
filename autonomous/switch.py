import math

from magicbot.state_machine import AutonomousStateMachine, state

from components.drivetrain import Drivetrain
from components.intake import Intake
from motioncontrol.path import Path, PathTuning
from motioncontrol.utils import Point, RobotState

from .path_selector import Selector


class SwitchAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Side Switch'
    DEFAULT = False

    drivetrain = Drivetrain
    intake = Intake

    right_near_side_waypoints = [
        Point(8.23 - 0.76 - (2.62 / 3), 3.74 / 2.9),
        Point(8.23 - 2.62 + 0.35, 3.74 / 2),
        Point(8.23 - 2.62 + 0.2, 3.74 / 1.4),
        Point(8.23 - 2.62 + 0.2, 3.74 - (0.84 / 2) - 0.05)
    ]

    right_far_side_waypoints = [
        Point(8.23 - 0.48, 5.20),
        Point(8.23 - 1.5, 6.00),
        Point(2.16, 6.00),
        Point(1.44, 6.00),
        Point(0.6, 5.50),
        Point(0.6, 4.92 - 0.2),
        Point(2.16 - 1.01 / 2 - 0.2, 4.42 - 0.28),
        Point(2.16 - 1.01 / 2 - 0.1, 4.42 - 0.28)
    ]

    left_position = RobotState(position=Point(0.76 + 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    right_position = RobotState(
        position=Point(8.23 - 0.76 - 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    near_path_tuning = PathTuning(
        lookahead=0.9, lookahead_reduction_factor=0.8, curvature_scaling=1.38)

    far_path_tuning = PathTuning(
        lookahead=1.7, lookahead_reduction_factor=1.4, curvature_scaling=2.35)

    def __init__(self):
        initial_states = [('left', self.left_position), ('right', self.right_position)]

        waypoints = {
            'left': Selector.mirror_waypoints(self.right_near_side_waypoints, 8.23),
            'right': self.right_near_side_waypoints
        }

        Selector.add_new_path(self.MODE_NAME, 'right', initial_states, waypoints)

    def initialize_path(self):
        try:
            switch_side = 'left' if Selector.game_message()[0] == 'L' else 'right'
        except IndexError:
            switch_side = None

        robot_side = Selector.starting_position()

        same_side = robot_side == switch_side

        tuning = self.near_path_tuning if same_side else self.far_path_tuning
        position = self.left_position if robot_side == 'left' else self.right_position
        waypoints = self.right_near_side_waypoints if same_side else self.right_far_side_waypoints
        max_speed = 1.4 if same_side else 2.0
        end_threshold = 0.35

        if robot_side == 'left':
            waypoints = Selector.mirror_waypoints(waypoints, 8.23)

        if robot_side is None or switch_side is None:
            waypoints = [position.position, position.position]

        path = Path(tuning, position, waypoints)

        self.drivetrain.set_path(max_speed, end_threshold, path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        self.intake.strong_hold()

        completion, remaining_distance = self.drivetrain.follow_path()

        if completion.done:
            self.done()
