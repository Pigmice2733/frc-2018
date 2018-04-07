#!/usr/bin/env python3

import wpilib
import math
from wpilib.buttons import JoystickButton

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


class RobotMode:
    climb = 3
    scale = 2
    switch = 1
    exchange = 0


class Robot(MagicRobot):

    drivetrain = Drivetrain
    climber = Climber
    elevator = Elevator
    intake = Intake
    mode = RobotMode.switch
    rumbling = False

    def createObjects(self):
        wpilib.CameraServer.launch()
        wpilib.LiveWindow.disableAllTelemetry()

        self.left_drive_motor = WPI_TalonSRX(0)
        WPI_TalonSRX(1).set(WPI_TalonSRX.ControlMode.Follower, self.left_drive_motor.getDeviceID())
        self.right_drive_motor = WPI_TalonSRX(2)
        WPI_TalonSRX(3).set(WPI_TalonSRX.ControlMode.Follower, self.right_drive_motor.getDeviceID())

        self.robot_drive = wpilib.drive.DifferentialDrive(
            self.left_drive_motor, self.right_drive_motor)

        self.r_intake_motor = WPI_VictorSPX(4)
        self.l_intake_motor = WPI_VictorSPX(5)

        self.elevator_winch = WPI_TalonSRX(6)

        self.climber_motor = WPI_TalonSRX(7)
        self.wrist_motor = WPI_TalonSRX(8)

        self.intake_ir = wpilib.AnalogInput(0)

        self.intake_solenoid = wpilib.DoubleSolenoid(2, 3)

        self.right_drive_joystick = wpilib.Joystick(0)
        self.left_drive_joystick = wpilib.Joystick(1)
        self.operator_joystick = wpilib.Joystick(2)

        self.compressor = wpilib.Compressor()

        self.elevator_limit_switch = wpilib.DigitalInput(0)

        self.climber_motor = WPI_TalonSRX(7)

        self.navx = AHRS.create_spi()

        self.path_tracking_table = NetworkTables.getTable("path_tracking")

        self.down_button = ButtonDebouncer(self.operator_joystick, 1)
        self.right_button = ButtonDebouncer(self.operator_joystick, 2)
        self.left_button = ButtonDebouncer(self.operator_joystick, 3)
        self.has_cube_button = ButtonDebouncer(self.operator_joystick, 5)
        self.up_button = ButtonDebouncer(self.operator_joystick, 4)
        self.left_bumper_button = JoystickButton(self.operator_joystick, 5)
        self.right_bumper_button = JoystickButton(self.operator_joystick, 6)

    def up_mode(self):
        self.mode += 1

    def down_mode(self):
        self.mode -= 1

    def autonomous(self):
        self.intake.reset_wrist()
        super().autonomous()

    def teleopInit(self):
        self.intake.reset_wrist_up()

    def teleopPeriodic(self):
        self.right = -self.right_drive_joystick.getRawAxis(1)
        self.left = -self.left_drive_joystick.getRawAxis(1)
        self.right = 0 if abs(self.right) < 0.1 else self.right
        self.left = 0 if abs(self.left) < 0.1 else self.left

        self.drivetrain.tank(math.copysign(self.right ** 2, self.right), math.copysign(self.left ** 2, self.left))

        # outtake
        if self.operator_joystick.getRawAxis(3) > 0.01:
            self.intake.set_speed(self.operator_joystick.getRawAxis(3) * 0.8 + 0.2)
        elif self.operator_joystick.getRawButton(3):
            self.intake.intake()
        else:
            self.intake.hold()

        elevator_speed = -self.operator_joystick.getY(0)

        if self.down_button.get():
            self.down_mode()
        elif self.up_button.get():
            self.up_mode()

        joystick_val = -self.operator_joystick.getRawAxis(1)
        if joystick_val > 0.2:
            joystick_val -= 0.2
        elif joystick_val < -0.2:
            joystick_val += 0.2
        else:
            joystick_val = 0.0
        self.elevator.move_setpoint(joystick_val / 50.0 * 20)
        if self.right_bumper_button.get():
            self.intake.wrist_down()
            self.intake.intake()
            self.intake.open_arm()
            if self.intake.has_cube():
                self.rumbling = True
        else:
            self.intake.close_arm()

        if self.left_bumper_button.get():
            self.climber.set_speed(-self.operator_joystick.getRawAxis(5))
        else:
            wrist_speed = self.operator_joystick.getRawAxis(5)
            wrist_speed = 0 if abs(wrist_speed) < 0.2 else wrist_speed
            self.intake.move_wrist_setpoint(wrist_speed)

        if self.has_cube_button.get():
            self.intake.toggle_has_cube()

        self.operator_joystick.setRumble(
            wpilib.Joystick.RumbleType.kRightRumble, 1 if self.rumbling else 0)
        self.operator_joystick.setRumble(
            wpilib.Joystick.RumbleType.kLeftRumble, 1 if self.rumbling else 0)

        self.rumbling = False

    def disabledPeriodic(self):
        self.drivetrain._update_odometry()
        self.elevator.reset_position()

    def testPeriodic(self):
        self.wrist_motor.set(self.operator_joystick.getRawAxis(5) * 0.6)
        self.elevator_winch.set(self.operator_joystick.getRawAxis(1))
        self.intake_solenoid.set(wpilib.DoubleSolenoid.Value.kReverse)
        self.left_drive_motor.set(0)
        self.right_drive_motor.set(0)

if __name__ == '__main__':
    wpilib.run(Robot)
