from ctre.wpi_talonsrx import WPI_TalonSRX


class Elevator:
    winch = WPI_TalonSRX
    goal_positions = [0, 2.55, 10.25]
    goal_index = goal_positions[0]

    def setup(self):
        self.winch.configSelectedFeedbackSensor(
            WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.winch.setSensorPhase(False)
        self.winch.setInverted(True)

        self.winch.configNominalOutputForward(0, 0)
        self.winch.configNominalOutputReverse(0, 0)
        self.winch.configPeakOutputForward(0.5, 0)
        self.winch.configPeakOutputReverse(-0.5, 0)

        self.winch.config_kP(0, 0.04, 0)
        self.winch.config_kI(0, 0.00005, 0)
        self.winch.config_kD(0, 0.0, 0)
        self.winch.config_kF(0, 0.0, 0)

        self.winch.selectProfileSlot(0, 0)

        self.winch.configAllowableClosedloopError(0, 100, 0)

        self.winch.setSelectedSensorPosition(0, 0, 0)

    def raise_goal(self):
        self.goal_index += 1
        self.goal_index = min(self.goal_index, len(self.goal_positions) - 1)

    def lower_goal(self):
        self.goal_index -= 1
        self.goal_index = max(self.goal_index, 0)

    def execute(self):
        # Get goal position in revolutions, scale to ticks - MagEncoder is a 1024 cpr quad
        target_position = self.goal_positions[self.goal_index] * 1024 * 4
        self.winch.set(WPI_TalonSRX.ControlMode.Position, target_position)
        print(target_position, " : ", self.winch.getQuadraturePosition())