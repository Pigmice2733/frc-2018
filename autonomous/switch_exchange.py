import math

import wpilib
from magicbot.state_machine import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.intake import Intake
from motioncontrol.path import Path, Waypoint
from motioncontrol.utils import Point, RobotState


class SwitchExchangeAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Switch Exchange'
    DEFAULT = False

    drivetrain = Drivetrain
    elevator = Elevator
    intake = Intake

    centered_right_waypoints = [
        Waypoint(Point(8.23 / 2, 1.01 / 2), 1.25, 1.3),
        Waypoint(Point(8.23 - 2.62 - 0.15, 3.74 / 2), 1.2, 1.6),
        Waypoint(Point(8.23 - 2.62 - 0.15, 3.74 / 2), 1.2, 1.6),
        Waypoint(Point(8.23 - 2.62 - 0.15, 3.74 - (0.84 / 2) - 0.15), 1, 1.5)
    ]

    right_switch_to_center = [
        Waypoint(Point(8.23 - 2.62, 3.54 - (0.84 / 2)), 1, 1.5),
        Waypoint(Point(8.23 - 2.62, 3.74 / 2), 1.2, 1.6),
        Waypoint(Point(8.23 - 2.62, 3.74 / 2), 1.1, 1.6),
        Waypoint(Point(8.23 / 2, 0.5 + 1.01 / 2), 1.2, 1.8)
    ]

    center_to_pyramid = [
        Waypoint(Point(8.23 / 2, 0.5 + 1.01 / 2), 1.6, 1.2),
        Waypoint(Point(8.23 / 2, 2 - 1.01 / 2), 1, 1.3)
    ]

    pyramid_to_exchange = [
        Waypoint(Point(8.23 / 2, 2 - 1.01 / 2), 1.6, 1.3),
        Waypoint(Point(8.23 / 2 - 0.75, 2.4 + 1.01 / 2), 1, 1.5),
        Waypoint(Point(8.23 / 2 - 0.75, 1.01 / 2), 0.8, 1.4)
    ]

    initial_state = RobotState(position=Point(8.23 / 2, 1.01 / 2))

    def initialize_switch_path(self):
        try:
            self.switch_side = self.game_message()[0]
        except IndexError:
            self.done()

        if self.switch_side == 'R':
            waypoints = self.centered_right_waypoints
        elif self.switch_side == 'L':
            waypoints = self.mirror_waypoints(self.centered_right_waypoints, 8.23)
        else:
            self.done()

        waypoints[0] = self.shift_waypoint(waypoints[0], 0.1, 0)
        waypoints[1] = self.shift_waypoint(waypoints[1], 0.1, 0)

        path = Path(waypoints, math.pi / 2)
        max_speed = 2.4
        end_threshold = 0.25

        self.drivetrain.set_odometry(self.initial_state)
        self.drivetrain.set_path(max_speed, end_threshold, path)

    def initialize_switch_to_center_path(self):
        if self.switch_side == 'R':
            waypoints = self.right_switch_to_center
        else:
            waypoints = self.mirror_waypoints(self.right_switch_to_center, 8.23)

        path = Path(waypoints, math.pi / 2)
        max_speed = 2.5
        end_threshold = 0.3

        self.drivetrain.set_path(max_speed, end_threshold, path)

    @timed_state(duration=1.0, next_state='stop', first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_switch_path()

        self.drivetrain.forward_at(0.8)
        self.intake.strong_hold()

    @timed_state(duration=0.2, next_state='start_to_switch')
    def stop(self):
        self.drivetrain.forward_at(0)
        self.intake.strong_hold()

    @state
    def start_to_switch(self):
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
            self.elevator.set_position(3.5)
        self.intake.strong_hold()

    @timed_state(duration=0.8, next_state='switch_to_center')
    def outtake(self):
        self.intake.outtake()
        self.elevator.set_position(3.5)

    @state
    def switch_to_center(self, initial_call):
        if initial_call:
            self.initialize_switch_to_center_path()

        completion, remaining_distance = self.drivetrain.follow_path()

        self.intake.strong_hold()
        self.elevator.set_position(3)

        if completion.done:
            self.next_state('center_to_pyramid')

    @state
    def center_to_pyramid(self, initial_call):
        if initial_call:
            self.initialize_path(self.center_to_pyramid, math.pi / 2, 2, 0.15)

        completion, remaining_distance = self.drivetrain.follow_path()

        self.intake.intake()
        self.intake.open_arm()
        self.elevator.set_position(0)

        if completion.done:
            self.next_state('pyramid_intake')

    @timed_state(duration=1, next_state='pyramid_to_exchange')
    def pyramid_intake(self):
        self.elevator.set_position(0)
        self.intake.close_arm()
        self.intake.intake()

    @state
    def pyramid_to_exchange(self, initial_call):
        if initial_call:
            self.initialize_path(self.pyramid_to_exchange, math.pi / 2, 2, 0.15)

        completion, remaining_distance = self.drivetrain.follow_path()

        self.intake.strong_hold()
        self.elevator.set_position(0)

        if completion.done:
            self.next_state('flip')

    @state
    def flip(self):
        completed = self.drivetrain.rotate(-90)

        if completed.done:
            self.next_state('exchange_outtake')

    @timed_state(duration=0.8, next_state='reverse')
    def exchange_outtake(self):
        self.intake.outtake()

    @timed_state(duration=1.4)
    def reverse(self):
        self.drivetrain.forward_at(-0.3)

    def mirror_waypoints(self, waypoints, field_width: float):
        def mirrored_point(waypoint: Waypoint) -> Waypoint:
            position = Point(field_width - waypoint.position.x, waypoint.position.y)
            return Waypoint(
                position=position,
                lookahead=waypoint.lookahead,
                curvature_scaling=waypoint.curvature_scaling)

        return [mirrored_point(waypoint) for waypoint in waypoints]

    def shift_waypoint(self, waypoint: Waypoint, shift_x: float, shift_y: float) -> Waypoint:
        position = Point(waypoint.position.x + shift_x, waypoint.position.y + shift_y)
        return Waypoint(position, waypoint.lookahead, waypoint.curvature_scaling)

    def game_message(self) -> str:
        return wpilib.DriverStation.getInstance().getGameSpecificMessage()

    def initialize_path(self, waypoints, end_angle, max_speed, end_threshold):
        path = Path(waypoints, end_angle)
        self.drivetrain.set_path(max_speed, end_threshold, path)
