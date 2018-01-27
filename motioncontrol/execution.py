"""Execution provides controllers and executors to control actuation of robot
based on path tracking, motion profiling and PID control.
"""

from typing import Callable

from .pid import PIDParameters, PIDController
from .motionprofiling import PositionProfile, DistanceProfile
from .path import Path
from .utils import RobotState, RobotCharacteristics, Completed
from .utils import distance_between


class PathTracker:
    """Combines a `DistanceProfile` and a `Path` into a seamless package for
    easy path tracking
    """

    def __init__(self, path: Path,
                 robot_characteristics: RobotCharacteristics,
                 time_resolution: float,
                 absolute_error: float,
                 input_source: Callable[[], RobotState],
                 velocity_output: Callable[[float], None],
                 curvature_output: Callable[[float], None]):
        """Create a new PathTracker

        `path`: The Path for the robot to follow
        `robot_characteristics`: RobotCharacteristics of the robot
        `time_resolution`: Expected time (in seconds) between updates
        `absolute_error`: Distance from end to stop path at
        `input_source`: Callable returning the current RobotState of the robot
        `velocity_ouput`: Callable to write the optimal velocity to
        `curvature_output`: Callable to write the optimal curvature to
        """
        self.path = path
        self.input_source = input_source
        self.velocity_output = velocity_output
        self.curvature_output = curvature_output

        distance_profile = DistanceProfile(robot_characteristics)
        self.profile_executor = DistanceProfileExecutor(
            distance_profile, time_resolution,
            (lambda: distance_between(self.input_source().position,
                                      self.path.points[-1])),
            (lambda: self.input_source().velocity),
            velocity_output, absolute_error
        )

    def update(self) -> Completed:
        """Gets current RobotState and writes output. Returns
        `Completed` indictating completion status
        """
        if not self.profile_executor.update():
            curvature = self.path.get_heading(self.input_source())
            self.curvature_output(curvature)
            return Completed(done=False)
        return Completed(done=True)


class PositionProfileExecutor:
    def __init__(
            self,
            pid_params: PIDParameters,
            motion_profile: PositionProfile,
            time_source: Callable[[], float],
            input_source: Callable[[], float],
            output: Callable[[float], None],
            acceptable_error_margin: float,
    ):
        """Combines a PID controller and a position motion profile. Ties
         them together for seemless profile execution.

        Uses `input_source` to retrieve current input for motion profile,
         and `output` to write PID output. `acceptable_error_margin` is the
         acceptable percent error as a decimal.

        `time_source` should be a method to retreive current time, in the same
         units as the motion profile uses.
        """

        self.time_source = time_source
        self.input_source = input_source
        self.output = output
        self.acceptable_error_margin = acceptable_error_margin
        self.motion_profile = motion_profile

        self.pid = PIDController(pid_params, self.time_source)

        self.profile_start_time = self.time_source()

    def update(self) -> bool:
        """Updates motion profile and writes output. Returns `True`
         if profile is completed (robot is within error margin of
         target), otherwise `False`.
        """
        time_delta = self.time_source() - self.profile_start_time

        current_goal_position = self.motion_profile.position(time_delta)

        current_position = self.input_source()

        output = self.pid.get_output(current_position, current_goal_position)
        self.output(output)
        final_position = self.motion_profile.position(
            self.motion_profile.end_time)

        error = (abs(final_position - current_position) /
                 abs(final_position))
        return error < self.acceptable_error_margin


class DistanceProfileExecutor:
    def __init__(
            self,
            motion_profile: DistanceProfile,
            time_resolution: float,
            distance_input: Callable[[], float],
            velocity_input: Callable[[], float],
            output: Callable[[float], None],
            absolute_error: float,
    ):
        """Uses a distance to velocity motion profile to effciently control
        motion.

        Uses `distance_input` to retrieve the remaining distance the robot
         should travel, and `velocity_input` to get the robot's current
         velocity.

        Uses `output` to write optimal velocity. `absolute_error` is
         the acceptable error in terms of the relevant units.

        `time_resolution` is the expected amount of time between updates to
         the executor. A smaller resolution will result in improved perforance.
         If the actual time between updates differs too much from
         `time_resolution` the performance of the motion profile will suffer.
        """
        self.distance_input = distance_input
        self.velocity_input = velocity_input
        self.output = output
        self.absolute_error = absolute_error
        self.motion_profile = motion_profile
        self.time_look_ahead = time_resolution

    def update(self) -> bool:
        """Updates motion profile and writes output. Returns `True`
         if profile is completed (robot is within error margin of
         target), otherwise `False`.
        """
        remaining_distance = self.distance_input()
        current_velocity = self.velocity_input()

        velocity, acceleration = self.motion_profile.velocity(
            current_velocity, remaining_distance)

        optimal_velocity = velocity + (acceleration * self.time_look_ahead)

        if remaining_distance < self.absolute_error:
            self.output(0.0)
            return True

        self.output(optimal_velocity)
        return False
