from pyfrc.physics.drivetrains import two_motor_drivetrain


class PhysicsEngine:
    """ This is the engine which runs with the sim """

    def __init__(self, controller):
        self.controller = controller

    def update_sim(self, hal_data, now, tm_diff):
        """ Updates the simulation with new robot positions """

        left = hal_data['CAN'][0]['value']
        right = hal_data['CAN'][2]['value']

        rotation, speed = two_motor_drivetrain(left, right, 3, 0.025)

        self.controller.drive(speed, rotation * 0.75, tm_diff)
