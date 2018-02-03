import math

from magicbot.state_machine import AutonomousStateMachine, state

from components.paths import Selector
from components.drivetrain import Drivetrain
from motioncontrol.path import Path
from motioncontrol.utils import Point, RobotState


class TestAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Test'
    DEFAULT = True

    drivetrain = Drivetrain
    path_selector = Selector

    main_path = [Path.forward(6),
                 Path.rotate(90),
                 Path.forward(1.5),
                 Path.rotate(-90),
                 Path.forward(6)]

    left_position = RobotState(position=Point(1.0, 0.0), rotation=math.pi / 2)
    right_position = RobotState(position=Point(26.0, 0.0), rotation=math.pi / 2)

    def __init__(self):
        initial_states = [('left', self.left_position), ('right', self.right_position)]
        self.path_selector.add_new_path(self.MODE_NAME, self.main_path, 'left', initial_states)

    def initialize_path(self):
        initial_robot_state = self.left_position
        path = Path(initial_robot_state, 2.5, self.main_path, 20, False, False)

        self.drivetrain.set_path(path)

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.initialize_path()

        if self.drivetrain.follow_path().done:
            self.done()
