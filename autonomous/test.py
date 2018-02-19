import math

from magicbot.state_machine import AutonomousStateMachine, state

from .path_selector import Selector
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

    forward_waypoints = [Point(2, 0), Point(2.25, 0)]

    position = RobotState(position=Point(0, 0), rotation=math.pi / 2)

    path_tuning = PathTuning(lookahead=1.0, lookahead_reduction_factor=1, curvature_scaling=1.2)

    def initialize_path(self):
        path = Path(self.path_tuning, self.position, self.forward_waypoints)
        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
