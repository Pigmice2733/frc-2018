#!/usr/bin/env python3

import wpilib
from wpilib import drive

from robotpy_ext.common_drivers.navx.ahrs import AHRS

from magicbot import MagicRobot

from ctre.wpi_talonsrx import WPI_TalonSRX

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
        self.left_front_drive = WPI_TalonSRX(0)
        self.left_back_drive = WPI_TalonSRX(1)
        self.right_front_drive = WPI_TalonSRX(2)
        self.right_back_drive = WPI_TalonSRX(3)

        self.left_back_drive.set(WPI_TalonSRX.ControlMode.Follower,
                                 self.left_front_drive.getDeviceID())

        self.right_back_drive.set(WPI_TalonSRX.ControlMode.Follower,
                                  self.right_front_drive.getDeviceID())

        self.robot_drive = drive.DifferentialDrive(self.left_front_drive,
                                                   self.right_front_drive)

        self.l_intake_motor = WPI_TalonSRX(4)
        self.r_intake_motor = WPI_TalonSRX(5)
        self.scale_arm_motor = WPI_TalonSRX(6)
        self.climber_motor = WPI_TalonSRX(7)

        self.navx = AHRS.create_spi()

        self.drive_joystick = wpilib.Joystick(0)
        self.operator_joystick = wpilib.Joystick(1)

    def teleopPeriodic(self):
        self.drivetrain.turn_at(
            self.drive_joystick.getRawAxis(0), squaredInputs=True)
        self.drivetrain.forward_at(self.drive_joystick.getRawAxis(1))
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
