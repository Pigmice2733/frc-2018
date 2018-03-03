import math

import wpilib
from magicbot.state_machine import AutonomousStateMachine, state, timed_state
from networktables.networktable import NetworkTable

from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.intake import Intake
from motioncontrol.path import Path, PathTuning
from motioncontrol.utils import Point, RobotState


class CenterAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Center Switch'
    DEFAULT = False

    drivetrain = Drivetrain
    elevator = Elevator
    intake = Intake

    path_selection_table = NetworkTable

    center_starting_position = RobotState(
        position=Point(8.23 / 2 + 0.1, 1.01 / 2), rotation=math.pi / 2)

    center_path_tuning = PathTuning(
        lookahead=1.2, lookahead_reduction_factor=1.5, curvature_scaling=1.55)

    right_side_waypoints = [
        Point(8.23 - 2.62 - 0.35, 3.74 / 3.5),
        Point(8.23 - 2.62 - 0.15, 3.74 / 2),
        Point(8.23 - 2.62 - 0.15, 3.74 - (0.84 / 2) - 0.15)
    ]

    def initialize_path(self):
        try:
            if self.game_message()[0] == 'R':
                waypoints = self.right_side_waypoints
            else:
                waypoints = self.mirror_waypoints(self.right_side_waypoints, 8.23)
        except IndexError:
            waypoints = [
                self.center_starting_position.position, self.center_starting_position.position
            ]

        path = Path(self.center_path_tuning, self.center_starting_position, waypoints)
        max_speed = 2.4
        end_threshold = 0.25

        self.drivetrain.set_path(max_speed, end_threshold, path)

    @timed_state(duration=1.0, next_state='stop', first=True)
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

        if remaining_distance < 2:
            self.elevator.set_position(4)

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
        self.intake.outtake()
        self.elevator.set_position(3.25)

    @timed_state(duration=1.4, next_state='lower')
    def reverse(self):
        self.elevator.set_position(2.8)
        self.drivetrain.forward_at(-0.3)

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
