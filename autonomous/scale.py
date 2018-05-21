import math

import wpilib
from magicbot.state_machine import AutonomousStateMachine, state, timed_state
from networktables import NetworkTables

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

    right_near_side_waypoints = [
        Point(8.23 - 1.2, 4),
        Point(8.23 - 1.25, 6.55),
        Point(8.23 - 1.8 - 1.01 / 2 + 0.6, 6.75),
    ]

    right_far_side_waypoints = [
        Point(8.23 - 0.85, 5.4),
        Point(8.23 - 1.1, 6),
        Point(8.23 - 2.62, 6),
        Point(3.1, 6.2),
        Point(2, 6.4),
        Point(2, 7.3 - 1.01 / 2)
    ]

    left_position = RobotState(position=Point(0.76 + 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    right_position = RobotState(
        position=Point(8.23 - 0.76 - 0.89 / 2, 1.01 / 2), rotation=math.pi / 2)

    near_path_tuning = PathTuning(
        lookahead=1.25, lookahead_reduction_factor=0.8, curvature_scaling=1.38)

    far_path_tuning = PathTuning(
        lookahead=1.62, lookahead_reduction_factor=4, curvature_scaling=1.55)

    def __init__(self):
        self.set_starting_positions(["left", "right"])

    def initialize_path(self):
        try:
            switch_side = 'left' if self.game_message()[1] == 'L' else 'right'
        except IndexError:
            switch_side = None

        robot_side = self.starting_position()

        self.same_side = robot_side == switch_side

        tuning = self.near_path_tuning if self.same_side else self.far_path_tuning
        position = self.left_position if robot_side == 'left' else self.right_position
        max_speed = 2 if self.same_side else 1.6
        end_threshold = 0.35 if self.same_side else 0.775

        if self.same_side:
            waypoints = self.right_near_side_waypoints
        else:
            waypoints = self.right_far_side_waypoints

        if robot_side == 'left':
            waypoints = self.mirror_waypoints(waypoints, 8.23)

        if robot_side is None or switch_side is None:
            waypoints = [position.position, position.position]

        path = Path(tuning, position, waypoints)

        self.switch_side = switch_side

        self.drivetrain.set_path(max_speed, end_threshold, path)

    @state(first=True)
    def drive(self, initial_call):
        if initial_call:
            self.initialize_path()

        if not self.same_side:
            self.next_state('just_forward')
            return

        completion, remaining_distance = self.drivetrain.follow_path()

        self.intake.strong_hold()
        self.intake.close_arm()
        self.intake.wrist_down()

        if remaining_distance < 2.8:
            self.elevator.set_position(12)

        if completion.done:
            # self.next_state('align')
            self.next_state('raise_elevator')

    @timed_state(duration=3)
    def just_forward(self):
        self.drivetrain.forward_at(0.3)

    @state
    def align(self):
        self.elevator.set_position(12)
        self.intake.strong_hold()

        if self.same_side:
            if self.switch_side == "left":
                err = self.drivetrain.get_orientation() - 3 * math.pi / 8
            else:
                err = self.drivetrain.get_orientation() - 5 * math.pi / 8
        else:
            err = self.drivetrain.get_orientation() - math.pi / 2

        if abs(err) > 0.02:
            sign = -1 if err < 0 else 1
            self.drivetrain.tank(0.18 * sign, -0.18 * sign)
        else:
            self.next_state('raise_elevator')

    @state
    def raise_elevator(self):
        if self.elevator.get_position() > 11:
            # if self.same_side:
            #     self.next_state('forward_to_scale')
            # else:
            self.next_state('outtake')
        else:
            self.elevator.set_position(12.5)
        self.intake.strong_hold()

    @timed_state(duration=0.3, next_state='outtake')
    def forward_to_scale(self):
        self.drivetrain.forward_at(0.2)

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
        return table.getString("selected", 'left')
