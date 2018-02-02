#!/usr/bin/env python3

import wpilib
from ctre.wpi_talonsrx import WPI_TalonSRX
from magicbot import MagicRobot
from networktables import NetworkTables
from robotpy_ext.common_drivers.navx.ahrs import AHRS
from wpilib import drive

from components.climber import Climber
from components.drivetrain import Drivetrain
from components.elevator import Elevator
from components.intake import Intake
from utils import NetworkTablesSender


class Robot(MagicRobot):

    drivetrain = Drivetrain
    climber = Climber
    elevator = Elevator
    intake = Intake

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

        self.elevator_winch = WPI_TalonSRX(6)
        self.elevator_winch_encoder = wpilib.AnalogPotentiometer(0)
        self.elevator_winch_encoder.offset = -self.elevator_winch_encoder.get()

        self.climber_motor = WPI_TalonSRX(7)

        self.navx = AHRS.create_spi()

        self.drive_joystick = wpilib.Joystick(0)
        self.operator_joystick = wpilib.Joystick(1)

        path_tracking_table = NetworkTables.getTable("path_tracking")
        self.path_tracking_sender = NetworkTablesSender(
            path_tracking_table)

    def autonomous(self):
        self.drivetrain.navx.reset()
        self.drivetrain.navx.setAngleAdjustment(0)
        self.left_drive_motor.setQuadraturePosition(0, 0)
        self.right_drive_motor.setQuadraturePosition(0, 0)
        super().autonomous()

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

    def disabledPeriodic(self):
        self.drivetrain._update_odometry()


if __name__ == '__main__':
    wpilib.run(Robot)
