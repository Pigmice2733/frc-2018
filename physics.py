from pyfrc.physics.drivetrains import two_motor_drivetrain

scale = 0.5


class PhysicsEngine:
    """ This is the engine which runs with the sim """

    def __init__(self, controller):
        self.controller = controller
        self.controller.add_device_gyro_channel('navxmxp_spi_4_angle')

    def update_sim(self, hal_data, now, tm_diff):
        """ Updates the simulation with new robot positions """

        left = hal_data['CAN'][0]['value']
        right = hal_data['CAN'][2]['value']

        left_pos = hal_data['CAN'][0]['quad_position'] + left * scale
        hal_data['CAN'][0]['quad_position'] = left_pos

        right_pos = hal_data['CAN'][2]['quad_position'] + right * scale
        hal_data['CAN'][2]['quad_position'] = right_pos

        rotation, speed = two_motor_drivetrain(left, right, 3, 0.025)

        self.controller.drive(speed, rotation * 0.75, tm_diff)
