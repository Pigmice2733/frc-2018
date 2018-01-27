import math
import typing

from motioncontrol import utils
from .utils import Point, Line, RobotState


class Action(typing.NamedTuple):
    distance: float = 0.0
    rotation: float = 0.0


class Path:
    """Represents a path between a set of points

    Provides methods to calculate desired robot heading to follow path
    """

    def __init__(self, initial_robot_state: RobotState,
                 actions: typing.List[Action]):
        """Constructs a `Path` using the `actions`, starting from
        `initial_robot_state`.
        """
        # How far apart numbers can be and still be considered equal
        self.approximation_error = 1e-3
        self.initial_state = initial_robot_state

        position = self.initial_state.position
        rotation = self.initial_state.rotation
        self.points = [position]

        for action in actions:
            rotation += math.radians(action.rotation)
            if action.distance != 0.0:
                position = Point(
                    x=position.x + (math.cos(rotation) * action.distance),
                    y=position.y + (math.sin(rotation) * action.distance))
                self.points.append(position)

    def get_heading(self, robot_state: RobotState, lookahead: float) -> float:
        """Given the current pose of the robot, calculate the optimal heading
        to follow the path.
        """
        closest, closest_path_index = self._find_closest_point(
            robot_state.position)
        goal = self._find_goal_point(closest, closest_path_index, lookahead)

        goal = utils.vehicle_coords(
            robot_state.position, math.pi / 2 - robot_state.rotation, goal)

        D = utils.distance_between(Point(), goal)
        x = goal.x

        curvature = -(2 * x) / (D * D)
        return curvature

    def _find_closest_point(self, point: Point) -> (Point, int):
        """Find the closest point on the path to the given point"""
        closest = self.points[0]
        closest_distance = utils.distance_between(closest, point)
        # Index with self.points where the closest point lays
        path_index = 0

        for i in range(0, len(self.points) - 1):
            line = Line(self.points[i], self.points[i + 1])
            candidate = utils.closest_point_on_line(line, point)
            candidate_distance = utils.distance_between(
                candidate, point)
            if candidate_distance < closest_distance:
                closest = candidate
                closest_distance = candidate_distance
                path_index = i

        return closest, path_index

    def _find_goal_point(self, closest_point: Point, path_index: int,
                         lookahead: float) -> Point:
        remaining_distance = lookahead
        goal_point = closest_point
        while (remaining_distance > self.approximation_error
               and path_index < (len(self.points) - 1)):
            path_segment = Line(
                self.points[path_index], self.points[path_index + 1])
            next_point = utils.move_point_along_line(
                path_segment, goal_point, remaining_distance)
            remaining_distance -= utils.distance_between(
                goal_point, next_point)
            goal_point = next_point
            path_index += 1
        if remaining_distance > self.approximation_error:
            return self.points[-1]
        return goal_point

    @staticmethod
    def forward(units=0):
        return Action(distance=units, rotation=0.0)

    @staticmethod
    def rotate(degrees=0):
        return Action(distance=0.0, rotation=degrees)

    @staticmethod
    def backward(units=0):
        return Path.forward(-units)
