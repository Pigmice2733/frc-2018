import math
import typing

from motioncontrol import utils

from .utils import Line, Point, RobotState


class Action(typing.NamedTuple):
    distance: float = 0.0
    rotation: float = 0.0


class PathState(typing.NamedTuple):
    center_of_rotation: Point
    curvature: float
    goal_point: Point
    path_position: Point
    path_segment: int
    remaining_distance: float


class PathTuning(typing.NamedTuple):
    default_lookahead: float
    lookahead_reduction_factor: float
    cte_dynamic_lookahead: bool
    end_stabilization_length: float


class Path:
    """Represents a path between a set of points

    Provides methods to calculate desired robot heading to follow path
    """

    def __init__(self,
                 initial_robot_state: RobotState,
                 end_angle: float,
                 default_lookahead: float,
                 lookahead_reduction_factor: float,
                 cte_dynamic_lookahead: bool,
                 end_stabilization_length: float,
                 actions: typing.List[Action] = None,
                 waypoints: typing.List[Point] = None):
        """Constructs a `Path` using the `actions`, starting from
        `initial_robot_state`.

        If a lookahead isn't supplied during a specific update,
        `default_lookahead` will be used. During the last segment of the path,
        if the default lookahead is used, it will be linearly scaled from `1`
        to `1/lookahead_reduction_factor` across that last segment.

        Setting `cte_dynamic_lookahead` to `True` will dynamically change the
        lookahead based on the cross-track-error when the robot is farther
        than the current lookahead from the path. This will decrease
        instability as the robot returns to the path, and can help prevent it
        from leaving it completely - however it may tak the robot a bit longer
        to get back to the path with this enabled.
        """
        # Error threshold to deal with floating point arithmetic
        self.approximation_error = 1e-3
        self.initial_state = initial_robot_state

        self.lookahead = default_lookahead
        self.lookahead_reduction = lookahead_reduction_factor
        self.cte_dynamic_lookahead = cte_dynamic_lookahead

        position = self.initial_state.position
        rotation = self.initial_state.rotation
        self.points = [position]

        if actions is not None:
            for action in actions:
                rotation += math.radians(action.rotation)
                if action.distance != 0.0:
                    position = Point(
                        x=position.x + (math.cos(rotation) * action.distance),
                        y=position.y + (math.sin(rotation) * action.distance))
                    self.points.append(position)
        else:
            self.points.extend(waypoints)

        self.end = self.points[-1]
        self.end_index = len(self.points) - 1

        self.pseudo_end = Point(
            x=self.end.x + (math.cos(end_angle) * end_stabilization_length),
            y=self.end.y + (math.sin(end_angle) * end_stabilization_length))
        self.points.append(self.pseudo_end)

    def get_path_state(self,
                       robot_state: RobotState,
                       lookahead: float = None) -> PathState:
        """Given the current pose of the robot and an optional lookahead
        distance, get a `PathState` object describing the optimal state.
        """
        closest, closest_segment_index = self._find_closest_point(
            robot_state.position)
        lookahead = self._compute_lookahead(
            closest,
            closest_segment_index,
            lookahead)

        absolute_goal = self._find_goal_point(
            robot_state, lookahead)

        relative_goal = utils.vehicle_coords(
            robot_state.position,
            math.pi / 2 - robot_state.rotation,
            absolute_goal)

        D = utils.distance_between(Point(), relative_goal)
        x = relative_goal.x

        curvature = (2 * x) / (D * D)

        remaining_distance = utils.distance_between(
            robot_state.position, self.end)

        robot_to_field_rotation = robot_state.rotation - math.pi / 2
        center_of_rotation = Point(None, None)
        if curvature != 0.0:
            radius = 1.0 / curvature
            center_x = (robot_state.position.x +
                        (radius * math.cos(robot_to_field_rotation)))
            center_y = (robot_state.position.y +
                        (radius * math.sin(robot_to_field_rotation)))
            center_of_rotation = utils.Point(center_x, center_y)

        return PathState(center_of_rotation=center_of_rotation,
                         curvature=curvature,
                         goal_point=absolute_goal,
                         path_position=closest,
                         path_segment=closest_segment_index,
                         remaining_distance=remaining_distance)

    def _find_goal_point(self,
                         robot_state: RobotState,
                         lookahead: float) -> Point:
        closest, closest_segment_index = self._find_closest_point(
            robot_state.position)

        for i in range(len(self.points) - 2, -1, -1):
            line = Line(self.points[i], self.points[i + 1])
            intersections = utils.circle_line_intersection(
                robot_state.position, lookahead, line)
            if intersections is not None:
                intersections = [inter for inter in intersections if utils.on_segment(inter, line)]
                candidates = sorted(intersections,
                                    key=(lambda x: utils.distance_between(x, self.points[i + 1])))
                if len(candidates) > 0:
                    return candidates[0]

        if self.cte_dynamic_lookahead:
            cross_track_error = utils.distance_between(
                robot_state.position, closest)
            lookahead = math.sqrt(
                math.pow(cross_track_error, 2) + math.pow(lookahead, 2))

        goal_point = self._find_goal_point_from_closest(
            closest, closest_segment_index, lookahead)
        return goal_point

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

    def _find_goal_point_from_closest(self,
                                      closest_point: Point,
                                      path_index: int,
                                      lookahead: float) -> Point:
        remaining_distance = lookahead
        goal_point = closest_point
        while (remaining_distance > self.approximation_error and
               path_index < (len(self.points) - 1)):
            path_segment = Line(
                self.points[path_index], self.points[path_index + 1])
            next_point = utils.move_point_along_line(
                path_segment, goal_point, remaining_distance)
            remaining_distance -= utils.distance_between(
                goal_point, next_point)
            goal_point = next_point
            path_index += 1
        if remaining_distance > self.approximation_error:
            return self.pseudo_end
        return goal_point

    def _compute_lookahead(self,
                           path_position: Point,
                           path_segment_index: int,
                           lookahead: float=None) -> float:
        lookahead_reduction = (
            1.0 if lookahead is not None else self.lookahead_reduction)
        lookahead = self.lookahead if lookahead is None else lookahead

        last_segment_index = self.end_index - 1

        if (path_segment_index == last_segment_index and
                lookahead_reduction != 1.0):
            segment_length = utils.distance_between(
                self.points[last_segment_index], self.end)
            remaining_distance = utils.distance_between(
                path_position, self.end)
            scale = (
                (segment_length + (lookahead_reduction - 1) *
                 remaining_distance) / (lookahead_reduction * segment_length))
            return lookahead * scale
        return lookahead

    @staticmethod
    def forward(units=0):
        return Action(distance=units, rotation=0.0)

    @staticmethod
    def rotate(degrees=0):
        return Action(distance=0.0, rotation=degrees)

    @staticmethod
    def backward(units=0):
        return Path.forward(-units)
