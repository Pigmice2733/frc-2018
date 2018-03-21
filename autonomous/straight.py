from magicbot.state_machine import AutonomousStateMachine, state

from components.drivetrain import Drivetrain
from motioncontrol.utils import RobotState


class StraightAutonomous(AutonomousStateMachine):
    MODE_NAME = 'Straight Switch'
    DEFAULT = False

    drivetrain = Drivetrain

    @state(first=True)
    def start(self, initial_call):
        if initial_call:
            self.drivetrain.set_odometry(RobotState())

        completion = self.drivetrain.straight(20)
        if completion.done:
            self.done()
