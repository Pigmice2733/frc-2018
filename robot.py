#!/usr/bin/env python3

import wpilib
from wpilib import drive

from networktables import NetworkTables

from robotpy_ext.common_drivers.navx.ahrs import AHRS

from magicbot import MagicRobot

from ctre.wpi_talonsrx import WPI_TalonSRX

from utils import NetworkTablesSender

from components.drivetrain import Drivetrain
from components.climber import Climber
from components.intake import Intake
from components.scale_arm import ScaleArm


class Robot(MagicRobot):

    drivetrain = Drivetrain
    climber = Climber
    intake = Intake
    scale_arm = ScaleArm

    def createObjects(self):
        self.left_drive_motor = WPI_TalonSRX(0)
        self.right_drive_motor = WPI_TalonSRX(2)

        WPI_TalonSRX(1).set(WPI_TalonSRX.ControlMode.Follower,
                            self.left_drive_motor.getDeviceID())
        WPI_TalonSRX(3).set(WPI_TalonSRX.ControlMode.Follower,
                            self.right_drive_motor.getDeviceID())

        self.robot_drive = drive.DifferentialDrive(self.left_drive_motor,
                                                   self.right_drive_motor)

        self.l_intake_motor = WPI_TalonSRX(4)
        self.r_intake_motor = WPI_TalonSRX(5)
        self.scale_arm_motor = WPI_TalonSRX(6)
        self.climber_motor = WPI_TalonSRX(7)

        self.navx = AHRS.create_spi()

        self.drive_joystick = wpilib.Joystick(0)
        self.operator_joystick = wpilib.Joystick(1)

        path_tracking_table = NetworkTables.getTable("path_tracking")
        self.path_tracking_sender = NetworkTablesSender(
            path_tracking_table)

    def teleopPeriodic(self):
        self.drivetrain.turn_at(
            self.drive_joystick.getRawAxis(0), squaredInputs=True)
        self.drivetrain.forward_at(-self.drive_joystick.getRawAxis(1))
        if self.operator_joystick.getRawButton(10):
            self.climber.climb()

        if self.drive_joystick.getRawButton(1):
            self.intake.intake()

        if self.drive_joystick.getRawButton(2):
            self.intake.outtake()

        if self.drive_joystick.getRawButton(3):
            self.scale_arm.up()

        if self.drive_joystick.getRawButton(4):
            self.scale_arm.down()


if __name__ == '__main__':
    wpilib.run(Robot)
