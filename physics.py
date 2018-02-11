from pyfrc.physics.drivetrains import two_motor_drivetrain


class PhysicsEngine:
    """ This is the engine which runs with the sim """

    def __init__(self, controller):
        self.controller = controller
        self.controller.add_device_gyro_channel('navxmxp_spi_4_angle')
        self.srxMagTicks = 1024 * 4

    def update_sim(self, hal_data, now, tm_diff):
        """ Updates the simulation with new robot positions """

        left_speed = hal_data['CAN'][0]['value']
        right_speed = hal_data['CAN'][2]['value']

        left_distance = left_speed * (3 * tm_diff) * self.srxMagTicks
        right_distance = right_speed * (3 * tm_diff) * self.srxMagTicks

        hal_data['CAN'][0]['quad_position'] -= int(left_distance)
        hal_data['CAN'][2]['quad_position'] -= int(right_distance)

        hal_data['CAN'][0]['quad_velocity'] = -int(left_distance / tm_diff)
        hal_data['CAN'][2]['quad_velocity'] = -int(right_distance / tm_diff)

        speed, rotation = two_motor_drivetrain(left_speed, right_speed, 3, 4)

        self.controller.drive(-speed, -rotation * 0.75, tm_diff)
