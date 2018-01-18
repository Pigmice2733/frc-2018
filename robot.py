#!/usr/bin/env python3

import wpilib
from wpilib import drive

from magicbot import MagicRobot

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
        self.robot_drive = drive.DifferentialDrive(
            wpilib.SpeedControllerGroup(
                wpilib.Spark(0),
                wpilib.Spark(1)),
            wpilib.SpeedControllerGroup(
                wpilib.Spark(2), wpilib.Spark(3)))

        self.l_intake_motor = wpilib.Spark(4)
        self.r_intake_motor = wpilib.Spark(5)
        self.scale_arm_motor = wpilib.Spark(6)
        self.climber_motor = wpilib.Spark(7)

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
