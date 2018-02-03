import math

from magicbot.state_machine import AutonomousStateMachine, state, tunable

from components.drivetrain import Drivetrain
from motioncontrol.path import Path
from motioncontrol.utils import Point, RobotState


class TestAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Test'
    DEFAULT = True

    drivetrain = Drivetrain
    # mirror = tunable(False)

    def initialize_path(self):
        initial_robot_state = RobotState(
            position=Point(26.0, 0.0), rotation=math.pi / 2)

        waypoints = [Path.forward(6),
                     Path.rotate(90),
                     Path.forward(1.5),
                     Path.rotate(-90),
                     Path.forward(6)]

        path = Path(initial_robot_state, 2.5, waypoints, 20, self.mirror, False)

        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
