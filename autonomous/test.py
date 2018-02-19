import math

from magicbot.state_machine import AutonomousStateMachine, state, timed_state

from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.intake import Intake
from motioncontrol.path import Path, PathTuning
from motioncontrol.utils import Point, RobotState


class TestAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Test'
    DEFAULT = True

    drivetrain = Drivetrain
    elevator = Elevator
    intake = Intake

    forward_waypoints = [Point(0, 2), Point(0, 2.25)]

    position = RobotState(position=Point(0, 0), rotation=math.pi / 2)

    path_tuning = PathTuning(lookahead=1.0, lookahead_reduction_factor=1, curvature_scaling=1.2)

    def initialize_path(self):
        path = Path(self.path_tuning, self.position, self.forward_waypoints)
        self.drivetrain.set_path(2.0, 0.2, path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        completion, remaining_distance = self.drivetrain.follow_path()

        self.intake.hold()

        if remaining_distance < 0.5:
            self.elevator.set_position(3)

        if completion.done:
            self.next_state('raise_elevator')

    @state
    def raise_elevator(self):
        if self.elevator.get_position() > 3:
            self.next_state('outtake')
        else:
            self.elevator.set_position(3.2)
            self.intake.hold()

    @timed_state(duration=2)
    def outtake(self):
        self.intake.outtake()
