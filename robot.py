#!/usr/bin/env python3

import wpilib
from ctre.wpi_talonsrx import WPI_TalonSRX
from ctre.wpi_victorspx import WPI_VictorSPX
from magicbot import MagicRobot
from networktables import NetworkTables
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from robotpy_ext.control.button_debouncer import ButtonDebouncer

from components.climber import Climber
from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.intake import Intake


class Robot(MagicRobot):

    drivetrain = Drivetrain
    climber = Climber
    elevator = Elevator
    intake = Intake

    def createObjects(self):
        wpilib.CameraServer.launch()
        wpilib.LiveWindow.disableAllTelemetry()

        self.left_drive_motor = WPI_TalonSRX(0)
        self.right_drive_motor = WPI_TalonSRX(2)

        WPI_TalonSRX(1).set(WPI_TalonSRX.ControlMode.Follower, self.left_drive_motor.getDeviceID())
        WPI_TalonSRX(3).set(WPI_TalonSRX.ControlMode.Follower, self.right_drive_motor.getDeviceID())

        self.robot_drive = wpilib.drive.DifferentialDrive(self.left_drive_motor,
                                                          self.right_drive_motor)

        self.r_intake_motor = WPI_VictorSPX(4)
        self.l_intake_motor = WPI_VictorSPX(5)

        self.elevator_winch = WPI_TalonSRX(6)

        self.climber_motor = WPI_TalonSRX(7)
        self.wrist_motor = WPI_TalonSRX(8)

        self.intake_ir = wpilib.AnalogInput(0)

        self.intake_solenoid = wpilib.DoubleSolenoid(1, 3)

        self.right_drive_joystick = wpilib.Joystick(0)
        self.left_drive_joystick = wpilib.Joystick(1)
        self.operator_joystick = wpilib.Joystick(2)

        self.compressor = wpilib.Compressor()

        # Xbox 'A' button
        #self.elevator_up = ButtonDebouncer(self.operator_joystick, 1)
        # Xbox 'Y' button
        self.elevator_down = ButtonDebouncer(self.operator_joystick, 4)
        self.elevator_limit_switch = wpilib.DigitalInput(0)

        self.toggle_arm_button = ButtonDebouncer(self.operator_joystick, 1)

        self.climber_motor = WPI_TalonSRX(7)

        self.navx = AHRS.create_spi()

        self.path_tracking_table = NetworkTables.getTable("path_tracking")

    def teleopPeriodic(self):
        self.right = -self.right_drive_joystick.getRawAxis(1)
        self.left = -self.left_drive_joystick.getRawAxis(1)
        self.right = 0 if abs(self.right) < 0.1 else self.right
        self.left = 0 if abs(self.left) < 0.1 else self.left

        self.drivetrain.tank(self.right, self.left)

        if self.intake.cube_is_in_range():
            self.intake.close_arm()
        else:
            self.intake.open_arm()

        if self.operator_joystick.getRawAxis(3) > 0.1:
            self.intake.set_speed(self.operator_joystick.getRawAxis(3))
        elif self.operator_joystick.getRawButton(3):
            self.intake.intake()
        else:
            self.intake.hold()

        elevator_speed = -self.operator_joystick.getY(0)
        if abs(elevator_speed) < 0.08:
            self.elevator.hold()
        else:
            self.elevator.set_speed(elevator_speed)

        if self.operator_joystick.getRawButton(6) and self.operator_joystick.getRawButton(5):
            self.climber.set_speed(-self.operator_joystick.getRawAxis(5))

    def disabledPeriodic(self):
        self.drivetrain._update_odometry()
        self.elevator.reset_position()


if __name__ == '__main__':
    wpilib.run(Robot)
