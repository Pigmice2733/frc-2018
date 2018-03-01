import math

from magicbot.state_machine import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain
from components.intake import Intake
from components.elevator import Elevator
from motioncontrol.path import Path, PathTuning
from motioncontrol.utils import Point, RobotState

from .path_selector import Selector


class SwitchAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Side Switch'
    DEFAULT = True

    drivetrain = Drivetrain
    elevator = Elevator
    intake = Intake

    right_near_side_waypoints = [
        Point(8.23 - 0.76 - (2.62 / 3), 3.74 / 2.9),
        Point(8.23 - 2.62, 3.74 / 2),
        Point(8.23 - 2.62 - 0.2, 3.74 / 1.4),
        Point(8.23 - 2.62 - 0.2, 3.74 - (0.84 / 2) - 0.3)
    ]

    right_far_side_waypoints = [
        Point(8.23 - 0.88, 5.20),
        Point(8.23 - 1.9, 6.0),
        Point(2.16, 6.0),
        Point(1.44, 6.0),
        Point(0.8, 5.50),
        Point(0.6, 4.92 - 0.5),
        Point(2.16 - 1.01 / 2 - 0.35, 4.42 - 0.3),
        Point(2.16 - 1.01 / 2 - 0.25, 4.42 - 0.3)
    ]

    left_position = RobotState(position=Point(0.76 + 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    right_position = RobotState(
        position=Point(8.23 - 0.76 - 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    near_path_tuning = PathTuning(
        lookahead=0.9, lookahead_reduction_factor=0.8, curvature_scaling=1.38)

    far_path_tuning = PathTuning(
        lookahead=1.58, lookahead_reduction_factor=2.6, curvature_scaling=1.34)

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

        #robot_side = Selector.starting_position()

        robot_side = "right"

        self.same_side = robot_side == switch_side

        tuning = self.near_path_tuning if self.same_side else self.far_path_tuning
        position = self.left_position if robot_side == 'left' else self.right_position
        waypoints = self.right_near_side_waypoints if self.same_side else self.right_far_side_waypoints
        max_speed = 1.4 if self.same_side else 1.75
        end_threshold = 0.35

        if robot_side == 'left':
            waypoints = Selector.mirror_waypoints(waypoints, 8.23)

        if robot_side is None or switch_side is None:
            waypoints = [position.position, position.position]

        path = Path(tuning, position, waypoints)

        self.drivetrain.set_path(max_speed, end_threshold, path)

    @timed_state(duration=0.75, next_state='stop', first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        self.drivetrain.follow_path()
        self.intake.strong_hold()

    @timed_state(duration=0.15, next_state='drive')
    def stop(self):
        self.drivetrain.forward_at(0)
        self.intake.strong_hold()

    @state
    def drive(self):
        completion, remaining_distance = self.drivetrain.follow_path()

        self.intake.strong_hold()

        if remaining_distance < 1.2:
            self.elevator.set_position(3.8)

        if completion.done:
            self.next_state('raise_elevator')

    @state
    def raise_elevator(self):
        if self.elevator.get_position() > 3.25:
            self.next_state('outtake')
        else:
            self.elevator.set_position(3.35)
        self.intake.strong_hold()

    @timed_state(duration=0.8, next_state='reverse')
    def outtake(self):
        self.intake.open_arm()
        self.elevator.set_position(3.25)

    @timed_state(duration=1.4, next_state='lower')
    def reverse(self):
        self.elevator.set_position(2.8)
        if self.same_side:
            self.drivetrain.forward_at(-0.3)
        else:
            self.drivetrain.forward_at(-0.12)

    @timed_state(duration=1)
    def lower(self):
        self.elevator.set_position(0)
        self.drivetrain.forward_at(0)
