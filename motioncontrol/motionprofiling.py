"""Provide motion profiles for smooth and efficient motion"""

import math

from .utils import RobotCharacteristics, phase_time, signum


class PositionProfile:
    """Motion profile representing a specific desired move

    Resolves time to optimal position
    """

    def __init__(self, robot: RobotCharacteristics, target_distance: float):
        """Create a motion profile to efficiently travel `target_distance`

        `robot`: RobotCharacteristics object describing the robot
        `target_distance`: The distance the robot should travel using the
        motion profile.
        """
        # Handle negative distances
        self.reverse = (target_distance < 0.0)
        target_distance = abs(target_distance)

        # Distance robot takes to accelerate from rest to max speed
        full_acceleration_distance = (0.5 * robot.acceleration_time * robot.max_speed)
        self.acceleration = robot.max_speed / robot.acceleration_time
        # Distance robot takes to decelerate from max speed to rest
        full_deceleration_distance = (0.5 * robot.deceleration_time * robot.max_speed)
        self.deceleration = -robot.max_speed / robot.deceleration_time

        # Distance needed to go from rest to max speed and back
        distance_for_max_speed = (full_acceleration_distance + full_deceleration_distance)

        # Trazezoidal profile - robot can reach max speed without overshooting
        if target_distance >= distance_for_max_speed:
            full_speed_time = (target_distance - distance_for_max_speed) / robot.max_speed
            self.acceleration_end_time = robot.acceleration_time
            self.deceleration_start_time = self.acceleration_end_time + \
                full_speed_time
            self.end_time = (self.deceleration_start_time + robot.deceleration_time)
            self.max_speed = robot.max_speed

        # Triangular profile - robot must start decelerating before max speed
        else:
            # acceleration distance to deceleration distance is proportional
            #  to the ratio of acceleration and deceleration times
            time_ratio = (robot.acceleration_time /
                          (robot.acceleration_time + robot.deceleration_time))
            partial_acceleration_distance = target_distance * time_ratio
            self.acceleration_end_time = math.sqrt(
                (2 * partial_acceleration_distance) / self.acceleration)
            self.deceleration_start_time = self.acceleration_end_time
            self.max_speed = self.acceleration_end_time * self.acceleration
            # Time the robot needs to stop
            deceleration_time = -self.max_speed / self.deceleration
            self.end_time = self.acceleration_end_time + deceleration_time

    # position() can be used for times after the end of the profile,
    #  this is so that if the PID hasn't got the robot to the
    #  target position by the end it can keep reducing the position
    #  error till it reaches the target position
    def position(self, time: float):
        """Get the optimal position at a specific time"""

        # Inner helper function to find the distance traveled
        #  when accelerating from an initial velocity for a time period
        def distance(initial_speed, acceleration, time):
            """Find the distance traveled when accelerating from
             `initial_speed` at `acceleration` for `time`
            """
            # delta_x = vi + 1/2(at^2)
            return (initial_speed * time) + (0.5 * acceleration * time * time)

        position = 0
        # Acceleration phase - time after start before max speed is reached
        acceleration_phase_time = phase_time(time, 0, self.acceleration_end_time)
        position += distance(0.0, self.acceleration, acceleration_phase_time)

        # Max speed phase - time since the start of the max speed pahse,
        #  before deceleration
        max_speed_phase_time = phase_time(time, self.acceleration_end_time,
                                          self.deceleration_start_time)
        position += max_speed_phase_time * self.max_speed

        # Deceleration phase - time after deceleration start and before the end
        deceleration_phase_time = phase_time(time, self.deceleration_start_time, self.end_time)
        position += distance(self.max_speed, self.deceleration, deceleration_phase_time)

        # Handle reverse (negative) directions
        return position if not self.reverse else -position

    # velocity() can be used for times after the end of the profile,
    #  this is so that if the PID hasn't got the robot to the
    #  target position by the end it can keep reducing the position
    #  error till it reaches the target position
    def velocity(self, time: float):
        sign = -1 if self.reverse else 1
        if time <= self.acceleration_end_time:
            return self.acceleration * time * sign
        elif time <= self.deceleration_start_time:
            return self.max_speed * sign
        elif time <= self.end_time:
            max_speed = (self.acceleration * self.acceleration_end_time)
            return (max_speed + (self.deceleration * (time - self.deceleration_start_time))) * sign
        else:
            return 0


class DistanceProfile:
    """Motion profile representing a specific desired move

    Resolves remaining distance to optimal velocity
    """

    def __init__(self, robot: RobotCharacteristics):
        """Create a motion profile to efficiently travel whatever distance
        is returned each step by `remaining_distance`

        `robot`: RobotCharacteristics object describing the robot
        """

        self.full_acceleration_time = robot.acceleration_time
        self.full_deceleration_time = robot.deceleration_time
        self.max_speed = robot.max_speed

        self.acceleration = robot.max_speed / robot.acceleration_time
        self.deceleration = -robot.max_speed / robot.deceleration_time

    def velocity(self, current_velocity: float, remaining_distance: float):
        """Get the velocity to travel at to efficiently travel the target distance

        `current_velocity`: Robot's current speed.
        `remaining_distance`: The distance the robot still needs to travel.

        Returns optimal velocity and optimal acceleration.
        Robot's velocity should be set to the optimal velocity at some look
        ahead time:
        velocity = optimal velocity + (optimal acceleration * look ahead time)
        """
        # Deceleration time and distance from current velocity
        deceleration_time = abs(current_velocity / self.deceleration)
        deceleration_distance = 0.5 * deceleration_time * current_velocity

        if signum(deceleration_distance) == 0:
            acceleration = self.acceleration * signum(remaining_distance)
        elif signum(deceleration_distance) != signum(remaining_distance):
            acceleration = self.deceleration * signum(deceleration_distance, separate_zero=False)
        else:
            if abs(deceleration_distance) >= abs(remaining_distance):
                acceleration = self.deceleration * signum(deceleration_distance)
            else:
                acceleration = self.acceleration * signum(deceleration_distance)
        return current_velocity, acceleration
