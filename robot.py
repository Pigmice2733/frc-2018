#!/usr/bin/env python3

import wpilib
from ctre.wpi_talonsrx import WPI_TalonSRX
from ctre.wpi_victorspx import WPI_VictorSPX
from magicbot import MagicRobot
from networktables import NetworkTables
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from robotpy_ext.control.button_debouncer import ButtonDebouncer
from wpilib import drive

from autonomous.path_selector import Selector
from components.climber import Climber
from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.intake import Intake
from motioncontrol.utils import RobotState
from utils import NetworkTablesSender


class Robot(MagicRobot):

    drivetrain = Drivetrain
    climber = Climber
    elevator = Elevator
    intake = Intake

    def robotInit(self):
        super().robotInit()

        def selector_state_output(state: RobotState):
            self.drivetrain.robot_state = state
            self.drivetrain._set_orientation(state.rotation)

        Selector.set_up(self.autonomous_chooser_table, self.path_tracking_table,
                        self.path_selection_table, self.path_tracking_sender,
                        self.path_selection_sender, selector_state_output, self.isDisabled)

    def createObjects(self):
        self.left_drive_motor = WPI_TalonSRX(0)
        self.right_drive_motor = WPI_TalonSRX(2)

        WPI_TalonSRX(1).set(WPI_TalonSRX.ControlMode.Follower, self.left_drive_motor.getDeviceID())
        WPI_TalonSRX(3).set(WPI_TalonSRX.ControlMode.Follower, self.right_drive_motor.getDeviceID())

        self.robot_drive = drive.DifferentialDrive(self.left_drive_motor, self.right_drive_motor)

        self.r_intake_motor = WPI_VictorSPX(4)
        self.l_intake_motor = WPI_VictorSPX(5)
        self.intake_solenoid = wpilib.DoubleSolenoid(0, 1)

        self.climber_motor = WPI_TalonSRX(7)

        self.navx = AHRS.create_spi()

        self.right_drive_joystick = wpilib.Joystick(0)
        self.left_drive_joystick = wpilib.Joystick(1)
        self.operator_joystick = wpilib.Joystick(2)

        self.elevator_winch = WPI_TalonSRX(6)
        # Xbox 'A' button
        self.elevator_up = ButtonDebouncer(self.operator_joystick, 1)
        # Xbox 'Y' button
        self.elevator_down = ButtonDebouncer(self.operator_joystick, 4)
        self.elevator_limit_switch = wpilib.DigitalInput(0)

        self.climber_motor = WPI_TalonSRX(7)

        self.path_tracking_table = NetworkTables.getTable("path_tracking")
        self.path_tracking_sender = NetworkTablesSender(self.path_tracking_table)
        self.autonomous_chooser_table = NetworkTables.getTable("SmartDashboard/Autonomous Mode")
        self.path_selection_table = NetworkTables.getTable("path_selection")
        self.path_selection_sender = NetworkTablesSender(self.path_selection_table)

    def teleopPeriodic(self):
        self.drivetrain.tank(-self.right_drive_joystick.getRawAxis(1),
                             -self.left_drive_joystick.getRawAxis(1))

        if self.operator_joystick.getRawButton(1):
            self.intake.intake()
            self.intake.open_arm()
        else:
            self.intake.close_arm()
            self.intake.hold()

        if self.operator_joystick.getRawButton(2):
            self.intake.outtake()
        elif self.operator_joystick.getRawButton(3):
            self.intake.intake()

        elevator_speed = -self.operator_joystick.getY(0)
        if abs(elevator_speed) < 0.08:
            self.elevator.hold()
        else:
            self.elevator.set_speed(elevator_speed)

        self.climber.set_speed(self.operator_joystick.getRawAxis(5))

    def disabledPeriodic(self):
        self.drivetrain._update_odometry()


if __name__ == '__main__':
    wpilib.run(Robot)
