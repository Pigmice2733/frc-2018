import math
import typing

from motioncontrol import utils

from .utils import Point, RobotState


class PathState(typing.NamedTuple):
    center_of_rotation: Point
    curvature: float
    goal_point: Point
    path_position: Point
    path_segment: int
    remaining_distance: float


class PathSegment(typing.NamedTuple):
    start: Point
    end: Point
    length: float
    angle: float


class PathTuning(typing.NamedTuple):
    lookahead: float
    lookahead_reduction_factor: float = 1.0
    curvature_scaling: float = 1.0


class Path:
    """Represents a path between a set of points

    Provides methods to calculate desired robot heading to follow path
    """

    def __init__(self, tuning_parameters: PathTuning, initial_robot_state: RobotState,
                 waypoints: typing.List[Point]):
        """Path starting from `initial_robot_state` following between the `waypoints`

        Tuning parameters:

        `lookahead` will be used to select a goal point, during the last segment of the path, it
        will be linearly scaled from `lookahead` to `lookahead / lookahead_reduction_factor` across
        that last segment.

        `curvature_scaling` should be used to tune how responsive the robot is to turning - if the
        robot fails to complete curves, increase it, if it turns too sharply decrease this value.
        """
        self.initial_state = initial_robot_state

        self.lookahead = tuning_parameters.lookahead
        self.lookahead_reduction = tuning_parameters.lookahead_reduction_factor
        self.curvature_scaling = tuning_parameters.curvature_scaling

        position = self.initial_state.position
        self.segments = []
        for waypoint in waypoints:
            length = utils.distance_between(position, waypoint)
            angle = utils.angle_between(position, waypoint)
            segment = PathSegment(start=position, end=waypoint, length=length, angle=angle)
            self.segments.append(segment)
            position = waypoint

        self.end = self.segments[-1].end

        end_stabilization_length = self.lookahead * 1.1
        end_angle = self.segments[-1].angle
        self.pseudo_end = Point(
            x=self.end.x + (math.cos(end_angle) * end_stabilization_length),
            y=self.end.y + (math.sin(end_angle) * end_stabilization_length))
        self.segments.append(
            PathSegment(
                start=self.end,
                end=self.pseudo_end,
                length=end_stabilization_length,
                angle=end_angle))

    def get_path_state(self, robot_state: RobotState) -> PathState:
        """Given the current pose of the robot and an optional lookahead
        distance, get a `PathState` object describing the optimal state.
        """
        closest, closest_segment_index = self._find_closest_point(robot_state.position)
        lookahead = self._compute_lookahead(closest, closest_segment_index)

        absolute_goal = self._find_goal_point(robot_state, lookahead) or closest
        relative_goal = utils.vehicle_coords(robot_state, absolute_goal)

        D = utils.distance_between(Point(), relative_goal)
        x = relative_goal.x

        curvature = self.curvature_scaling * ((2 * x) / (D * D))

        remaining_distance = utils.distance_between(robot_state.position, self.end)

        robot_to_field_rotation = robot_state.rotation - math.pi / 2
        center_of_rotation = Point(None, None)
        if curvature != 0.0:
            radius = 1.0 / curvature
            center_x = (robot_state.position.x + (radius * math.cos(robot_to_field_rotation)))
            center_y = (robot_state.position.y + (radius * math.sin(robot_to_field_rotation)))
            center_of_rotation = utils.Point(center_x, center_y)

        return PathState(
            center_of_rotation=center_of_rotation,
            curvature=curvature,
            goal_point=absolute_goal,
            path_position=closest,
            path_segment=closest_segment_index,
            remaining_distance=remaining_distance)

    def _find_goal_point(self, robot_state: RobotState, lookahead: float) -> Point:
        """Find a goal point - point on path `lookahead` from robot that is closest by on-path
         distance to the end

        Returns `None` if no such intersection is found.
        """
        for segment in reversed(self.segments):
            intersections = utils.circle_line_intersection(robot_state.position, lookahead, segment)
            intersections = [n for n in intersections if utils.on_segment(n, segment)]
            if intersections:
                return min(intersections, key=(lambda x: utils.distance_between(x, segment.end)))

        return None

    def _find_closest_point(self, point: Point) -> (Point, int):
        """Find the closest point on the path to the given point

        Returns the closest point, and the index of the path segment it is on."""
        candidates = []

        for i, segment in enumerate(self.segments):
            candidate = utils.closest_point_on_line(segment, point)
            distance = utils.distance_between(candidate, point)
            candidates.append((candidate, distance, i))

        closest = min(candidates, key=lambda x: x[1])
        return closest[0], closest[2]

    def _compute_lookahead(self, path_position: Point, segment_index: int) -> float:
        segment = self.segments[segment_index]

        on_end_segment = segment_index == len(self.segments) - 2
        on_pseudo_segment = segment_index == len(self.segments) - 1

        if on_pseudo_segment:
            return self.lookahead / self.lookahead_reduction

        if on_end_segment:
            remaining_distance = utils.distance_between(path_position, self.end)
            scale = ((segment.length + (self.lookahead_reduction - 1) * remaining_distance) /
                     (self.lookahead_reduction * segment.length))
            return self.lookahead * scale

        return self.lookahead
