import math

from magicbot.state_machine import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.intake import Intake
from motioncontrol.path import Path, PathTuning
from motioncontrol.utils import Point, RobotState


class ForwardAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Forward Switch'
    DEFAULT = False

    drivetrain = Drivetrain
    elevator = Elevator
    intake = Intake

    forward_waypoints = [Point(0, 2.9), Point(0, 3.55 - 1.01 / 2)]
    position = RobotState(position=Point(0, 1.01 / 2), rotation=math.pi / 2)
    path_tuning = PathTuning(lookahead=1.0, lookahead_reduction_factor=1, curvature_scaling=1.2)

    def initialize_path(self):
        path = Path(self.path_tuning, self.position, self.forward_waypoints)
        self.drivetrain.set_path(2.2, 0.2, path)

    @timed_state(duration=0.9, next_state='stop', first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        self.drivetrain.follow_path()

        self.intake.hold()

    @timed_state(duration=0.2, next_state='drive')
    def stop(self):
        self.drivetrain.forward_at(0)

        self.intake.hold()

    @state
    def drive(self):
        completion, remaining_distance = self.drivetrain.follow_path()

        self.intake.strong_hold()

        if remaining_distance < 1.6:
            self.elevator.set_position(4)

        if completion.done:
            self.next_state('raise_elevator')

    @state
    def raise_elevator(self):
        if self.elevator.get_position() > 3.8:
            self.next_state('outtake')
        else:
            self.elevator.set_position(4.2)
        self.drivetrain.forward_at(0.2)
        self.intake.strong_hold()

    @timed_state(duration=1, next_state='reverse')
    def outtake(self):
        self.intake.outtake()
        self.elevator.set_position(4.2)
        self.drivetrain.forward_at(0.1)

    @timed_state(duration=3)
    def reverse(self):
        self.elevator.set_position(4.2)
        self.drivetrain.forward_at(-0.4)
