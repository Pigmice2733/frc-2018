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

    left_near_side_waypoints = [
        Point(2.62 / 4, 3.74 / 4),
        Point(2.62 + 0.1, 3.74 / 4),
        Point(2.62, 3.74 - 0.84 / 2)
    ]

    right_near_side_waypoints = [
        Point(8.23 - (2.62 / 4), 3.74 / 4),
        Point(8.23 - 2.62 + 0.1, 3.74 / 4),
        Point(8.23 - 2.62, 3.74 - 0.84 / 2)
    ]

    left_position = RobotState(
        position=Point(0.72 / 2, 0.84 / 2), rotation=math.pi / 2
    )

    right_position = RobotState(
        position=Point(8.23 - 0.72 / 2, 0.84 / 2), rotation=math.pi / 2)

    def __init__(self):
        initial_states = [('left', self.left_position), ('right', self.right_position)]

        waypoints = {
            'left': self.left_near_side_waypoints,
            'right': self.right_near_side_waypoints
        }

        Selector.add_new_path(
            self.MODE_NAME, 'right', initial_states, waypoints)

    def initialize_path(self):
        initial_robot_state = self.right_position

        # Robot width = 0.72m
        # Robot height = 0.84m
        # Side to middle of switch plate = 2.62m
        # Alliance station wall to edge of switch plate = 3.74m

        path = Path(initial_robot_state,
                    1.8,
                    end_angle=math.pi / 2,
                    end_stabilization_length=1.8,
                    lookahead_reduction_factor=1,
                    cte_dynamic_lookahead=False,
                    waypoints=self.right_near_side_waypoints)

        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
