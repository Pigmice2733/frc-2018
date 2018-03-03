import math

import wpilib
from magicbot.state_machine import AutonomousStateMachine, state, timed_state
from networktables.networktable import NetworkTable

from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.intake import Intake
from motioncontrol.path import Path, PathTuning
from motioncontrol.utils import Point, RobotState


class ScaleAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Scale'
    DEFAULT = False

    drivetrain = Drivetrain
    elevator = Elevator
    intake = Intake

    path_selection_table = NetworkTable

    right_near_side_waypoints = [
        Point(8.23 - 1.2, 4),
        Point(8.23 - 1.25, 6.5),
        Point(8.23 - 1.8 - 1.01 / 2 + 0.6, 6.7),
    ]

    right_far_side_waypoints = [
        Point(8.23 - 1.1, 6),
        Point(8.23 - 2.62, 6.2),
        Point(2.16, 6.0),
        Point(1.44, 6.0),
        Point(0.8, 6.4),
        Point(0.6, 6.9),
        Point(1.8 - 1.01 / 2 - 0.3, 7.25),
        Point(1.8 - 1.01 / 2 - 0.2, 7.25)
    ]

    left_position = RobotState(position=Point(0.76 + 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    right_position = RobotState(
        position=Point(8.23 - 0.76 - 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    near_path_tuning = PathTuning(
        lookahead=1.25, lookahead_reduction_factor=0.8, curvature_scaling=1.38)

    far_path_tuning = PathTuning(
        lookahead=1.58, lookahead_reduction_factor=2.6, curvature_scaling=1.34)

    def __init__(self):
        self.initial_states = [('left', self.left_position), ('right', self.right_position)]

        self.waypoints = {
            'left': self.mirror_waypoints(self.right_near_side_waypoints, 8.23),
            'right': self.right_near_side_waypoints
        }

    def initialize_path(self):
        try:
            switch_side = 'left' if self.game_message()[1] == 'L' else 'right'
        except IndexError:
            switch_side = None

        robot_side = self.starting_position()

        self.same_side = robot_side == switch_side

        tuning = self.near_path_tuning if self.same_side else self.far_path_tuning
        position = self.left_position if robot_side == 'left' else self.right_position
        max_speed = 2 if self.same_side else 1.75
        end_threshold = 0.35

        if self.same_side:
            waypoints = self.right_near_side_waypoints
        else:
            waypoints = self.right_far_side_waypoints

        if robot_side == 'left':
            waypoints = self.mirror_waypoints(waypoints, 8.23)

        if robot_side is None or switch_side is None:
            waypoints = [position.position, position.position]

        path = Path(tuning, position, waypoints)

        self.drivetrain.set_path(max_speed, end_threshold, path)

    @timed_state(duration=1.25, next_state='stop', first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        self.drivetrain.follow_path()
        self.intake.strong_hold()

    @timed_state(duration=0.25, next_state='drive')
    def stop(self):
        self.drivetrain.forward_at(0)
        self.intake.strong_hold()

    @state
    def drive(self):
        completion, remaining_distance = self.drivetrain.follow_path()

        self.intake.strong_hold()

        if remaining_distance < 2.2:
            self.elevator.set_position(12)

        if completion.done:
            self.next_state('raise_elevator')

    @state
    def raise_elevator(self):
        if self.elevator.get_position() > 11.2:
            self.next_state('outtake')
        else:
            self.elevator.set_position(12)
        self.intake.strong_hold()

    @timed_state(duration=0.8, next_state='reverse')
    def outtake(self):
        self.intake.outtake()
        self.elevator.set_position(11.5)

    @timed_state(duration=1.4, next_state='lower')
    def reverse(self):
        self.elevator.set_position(11)
        self.drivetrain.forward_at(-0.175)

    @timed_state(duration=1)
    def lower(self):
        self.elevator.set_position(0)
        self.drivetrain.forward_at(0)

    def mirror_waypoints(self, waypoints, field_width: float):
        def mirrored_point(point: Point) -> Point:
            return Point(field_width - point.x, point.y)

        return [mirrored_point(point) for point in waypoints]

    def game_message(self) -> str:
        return wpilib.DriverStation.getInstance().getGameSpecificMessage()

    def starting_position(self) -> str:
        return self.path_selection_table.getString("starting_position", None)
