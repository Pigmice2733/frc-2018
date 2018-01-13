#!/usr/bin/env python3

import wpilib
from ctre.cantalon import CANTalon
from magicbot import MagicRobot

from components.drivetrain import Drivetrain
from components.intake import Intake
from components.scale_arm import ScaleArm


class Robot(MagicRobot):

    drivetrain = Drivetrain
    intake = Intake
    scalearm = ScaleArm

    def createObjects(self):
        self.robot_drive = wpilib.RobotDrive(
            0, 1, 2, 3, motorController=CANTalon)

        self.l_intake_motor = CANTalon(4)
        self.r_intake_motor = CANTalon(5)
        self.scale_arm_motor = CANTalon(6)

        self.drive_joystick = wpilib.Joystick(0)
        self.operator_joystick = wpilib.Joystick(1)

    def teleopPeriodic(self):
        self.drivetrain.turn_at(
            self.drive_joystick.getRawAxis(0), squaredInputs=True)
        self.drivetrain.forward_at(self.drive_joystick.getRawAxis(1))

        if self.drive_joystick.getRawButton(1):
            self.intake.intake()

        if self.drive_joystick.getRawButton(2):
            self.intake.outtake()

        if self.drive_joystick.getRawButton(3):
            self.scalearm.up()

        if self.drive_joystick.getRawButton(4):
            self.scalearm.down()


if __name__ == '__main__':
    wpilib.run(Robot)
