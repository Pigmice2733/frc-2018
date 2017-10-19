from pyfrc.physics.drivetrains import four_motor_drivetrain


class PhysicsEngine:
    """ This is the engine which runs with the sim """

    def __init__(self, controller):
        self.controller = controller

    def update_sim(self, hal_data, now, tm_diff):
        """ Updates the simulation with new robot positions """

        fl = hal_data['CAN'][0]['value']
        bl = hal_data['CAN'][1]['value']
        fr = -hal_data['CAN'][2]['value']
        br = -hal_data['CAN'][3]['value']

        rotation, speed = four_motor_drivetrain(bl, br, fl, fr, 3, 0.025)

        self.controller.drive(speed, rotation * 0.75, tm_diff)
