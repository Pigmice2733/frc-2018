import math

import wpilib
from magicbot.state_machine import AutonomousStateMachine, state, timed_state
from networktables.networktable import NetworkTable

from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.intake import Intake
from motioncontrol.path import Path, Waypoint
from motioncontrol.utils import Point, RobotState


class ScaleAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Scale'
    DEFAULT = False

    drivetrain = Drivetrain
    elevator = Elevator
    intake = Intake

    path_selection_table = NetworkTable

    right_near_side_waypoints = [
        Waypoint(Point(Point(8.23 - 0.76 - 0.89 / 2, 1.01 / 2)), 2.4, 0.8),
        Waypoint(Point(8.23 - 1.2, 4), 1.6, 1),
        Waypoint(Point(8.23 - 1.25, 6.5), 1.25, 1.4),
        Waypoint(Point(8.23 - 1.8 - 1.01 / 2 + 0.6, 6.7), 1, 1.5)
    ]

    right_far_side_waypoints = [
        Waypoint(Point(8.23 - 0.76 - 0.89 / 2, 1.01 / 2), 2, 1.2),
        Waypoint(Point(8.23 - 0.85, 5.4), 1.2, 1.6),
        Waypoint(Point(8.23 - 1.1, 6), 1.6, 1.6),
        Waypoint(Point(8.23 - 2.62, 5.9), 1.8, 1.2),
        Waypoint(Point(2.65, 5.9), 1, 1.7),
        Waypoint(Point(2.55, 6.3), 1, 1.7),
        Waypoint(Point(2.55, 7.57 - 1.01 / 2), 1, 1.7)
    ]

    def initialize_path(self):
        try:
            switch_side = 'left' if self.game_message()[1] == 'L' else 'right'
        except IndexError:
            switch_side = None

        robot_side = self.starting_position()

        if robot_side is None or switch_side is None:
            self.done()

        self.same_side = robot_side == switch_side

        end_angle = 3 * math.pi / 4 if self.same_side else math.pi / 2
        max_speed = 2 if self.same_side else 1.6
        end_threshold = 0.35 if self.same_side else 0.775

        if self.same_side:
            waypoints = self.right_near_side_waypoints
        else:
            waypoints = self.right_far_side_waypoints

        if robot_side == 'left':
            waypoints = self.mirror_waypoints(waypoints, 8.23)

        path = Path(waypoints, end_angle)
        self.drivetrain.set_path(max_speed, end_threshold, path)

        initial_position = RobotState(position=waypoints[0].position)
        self.drivetrain.set_odometry(initial_position)

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
            self.next_state('align')

    @state
    def align(self):
        self.elevator.set_position(12)
        self.intake.strong_hold()

        err = self.drivetrain.get_orientation() - math.pi / 2
        if abs(err) > 0.1:
            sign = -1 if err < 0 else 1
            self.drivetrain.tank(0.18 * sign, -0.18 * sign)
        else:
            self.next_state('raise_elevator')

    @state
    def raise_elevator(self):
        if self.elevator.get_position() > 11:
            self.next_state('outtake')
        else:
            self.elevator.set_position(12.5)
        self.intake.strong_hold()

    @timed_state(duration=1.25, next_state='reverse')
    def outtake(self):
        self.intake.outtake()
        self.elevator.set_position(12.5)

    @timed_state(duration=1.2, next_state='hold')
    def reverse(self):
        self.elevator.set_position(11.5)
        if self.same_side:
            self.drivetrain.forward_at(-0.3)
        else:
            self.drivetrain.forward_at(-0.175)

    @state
    def hold(self):
        self.elevator.set_position(11.5)

    @timed_state(duration=3)
    def straight_forward(self):
        self.drivetrain.forward_at(0.3)

    def mirror_waypoints(self, waypoints, field_width: float):
        def mirrored_point(waypoint: Waypoint) -> Waypoint:
            position = Point(field_width - waypoint.position.x, waypoint.position.y)
            return Waypoint(
                position=position,
                lookahead=waypoint.lookahead,
                curvature_scaling=waypoint.curvature_scaling)

        return [mirrored_point(waypoint) for waypoint in waypoints]

    def game_message(self) -> str:
        return wpilib.DriverStation.getInstance().getGameSpecificMessage()

    def starting_position(self) -> str:
        return self.path_selection_table.getString("starting_position", None)
