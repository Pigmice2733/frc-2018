import math

import wpilib
from magicbot.state_machine import AutonomousStateMachine, state, timed_state
from networktables import NetworkTables

from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.intake import Intake
from motioncontrol.path import Path, PathTuning
from motioncontrol.utils import Point, RobotState


class SwitchAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Side Switch'
    DEFAULT = True

    drivetrain = Drivetrain
    elevator = Elevator
    intake = Intake

    old_right_near_side_waypoints = [
        Point(8.23 - 0.76 - (2.62 / 3), 3.74 / 2.9),
        Point(8.23 - 2.62, 3.74 / 2),
        Point(8.23 - 2.62 - 0.2, 3.74 / 1.4),
        Point(8.23 - 2.62 - 0.2, 3.74 - (0.84 / 2) - 0.1)
    ]

    right_near_side_waypoints = [Point(8.23 - 1.15, 4.9), Point(8.23 - 2.16 - 1.01 / 2 - 0.2, 4.75)]

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
        self.set_starting_positions(["left", "right"])

    def initialize_path(self):
        try:
            switch_side = 'left' if self.game_message()[0] == 'L' else 'right'
        except IndexError:
            switch_side = None

        robot_side = self.starting_position()

        self.same_side = robot_side == switch_side

        tuning = self.near_path_tuning if self.same_side else self.far_path_tuning
        position = self.left_position if robot_side == 'left' else self.right_position
        max_speed = 1.4 if self.same_side else 1.75
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

    @timed_state(duration=1, next_state='stop', first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        self.drivetrain.follow_path()
        self.intake.strong_hold()

    @timed_state(duration=0.2, next_state='drive')
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
        self.drivetrain.forward_at(0.11)

    @timed_state(duration=0.8, next_state='reverse')
    def outtake(self):
        self.intake.outtake()
        self.elevator.set_position(3.25)

    @timed_state(duration=1.2, next_state='lower')
    def reverse(self):
        self.elevator.set_position(2.8)
        if self.same_side:
            self.drivetrain.forward_at(-0.24)
        else:
            self.drivetrain.forward_at(-0.12)

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

    def set_starting_positions(self, positions):
        table = NetworkTables.getTable("autonomous/" + self.MODE_NAME + "/starting_position")
        return table.putStringArray("options", positions)

    def starting_position(self) -> str:
        table = NetworkTables.getTable("autonomous/" + self.MODE_NAME + "/starting_position")
        return table.getString("selected", None)
