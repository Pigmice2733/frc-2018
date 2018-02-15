import math

from magicbot.state_machine import AutonomousStateMachine, state

from .path_selector import Selector
from components.drivetrain import Drivetrain
from motioncontrol.path import Path, PathTuning
from motioncontrol.utils import Point, RobotState


class CenterAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Center Switch'
    DEFAULT = False

    drivetrain = Drivetrain

    center_starting_position = RobotState(position=Point(8.23 / 2, 1.01 / 2), rotation=math.pi / 2)

    center_path_tuning = PathTuning(
        lookahead=1.2, lookahead_reduction_factor=1.5, curvature_scaling=1.55)

    right_side_waypoints = [
        Point(8.23 - 2.62 - 0.3, 3.74 / 3.5),
        Point(8.23 - 2.62, 3.74 / 2),
        Point(8.23 - 2.62, 3.74 - (0.84 / 2))
    ]

    def __init__(self):
        initial_states = [('center', self.center_starting_position)]

        waypoints = {
            'center': self.right_side_waypoints,
        }

        Selector.add_new_path(self.MODE_NAME, 'center', initial_states, waypoints)

    def initialize_path(self):
        if Selector.game_message()[0] == 'R':
            waypoints = self.right_side_waypoints
        else:
            waypoints = Selector.mirror_waypoints(self.right_side_waypoints, 8.23)

        path = Path(self.center_path_tuning, self.center_starting_position, waypoints)
        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
