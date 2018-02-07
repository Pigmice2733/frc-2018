import math

from magicbot.state_machine import AutonomousStateMachine, state

from components.drivetrain import Drivetrain
from motioncontrol.path import Path
from motioncontrol.utils import Point, RobotState


class TestAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Test'
    DEFAULT = True

    drivetrain = Drivetrain

    def initialize_path(self):
        initial_robot_state = RobotState(
            position=Point(26.0, 0.0), rotation=math.pi / 2)

        waypoints = [Path.forward(5),
                     Path.rotate(90),
                     Path.forward(1),
                     Path.rotate(-90),
                     Path.forward(5)]
        path = Path(initial_robot_state, 3.0, 20, False, 2.0, waypoints)

        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
