import math
import typing

from motioncontrol import utils

from .utils import Point, RobotState


class PathState(typing.NamedTuple):
    curvature: float
    goal_point: Point
    path_position: Point
    goal_direction: int
    remaining_distance: float


class Waypoint(typing.NamedTuple):
    position: Point
    lookahead: float
    curvature_scaling: float


class PathSegment(typing.NamedTuple):
    start: Point
    end: Point
    length: float
    angle: float
    start_lookahead: float
    start_curvature_scaling: float


class Path:
    """Represents a path between a set of points

    Provides methods to calculate desired robot heading to follow path
    """

    def __init__(self, waypoints: typing.List[Waypoint], end_angle: float):
        """Create a `Path` following along between the `waypoints`

        `end_angle`: Desired ending angle. The robot may not end up facing this direction at all,
        but will be biased to point this direction. The longer the lookahead of the last waypoint,
        the more likely it is that the robot will end up oriented in the desired direction.

        `waypoints` must contained at least two waypoints - a start and an end. Less than that will
        result in undefined behavior.
        """
        self.segments = []

        for i, waypoint in enumerate(waypoints):
            start = waypoint.position
            try:
                end = waypoints[i + 1].position
            except IndexError:
                last = waypoints[-1].position
                end_length = (1 + 1e-6) * waypoints[-1].lookahead
                end = Point(
                    x=last.x + (math.cos(end_angle) * end_length),
                    y=last.y + (math.sin(end_angle) * end_length))

            length = utils.distance_between(start, end)
            angle = utils.angle_between(start, end)
            segment = PathSegment(
                start=start,
                end=end,
                length=length,
                angle=angle,
                start_lookahead=waypoint.lookahead,
                start_curvature_scaling=waypoint.curvature_scaling)
            self.segments.append(segment)

    def get_path_state(self, robot_state: RobotState) -> PathState:
        """Given the current pose of the robot and an optional lookahead
        distance, get a `PathState` object describing the optimal state.
        """
        closest, closest_segment_index = self._find_closest_point(robot_state.position)
        lookahead = self._compute_lookahead(closest, closest_segment_index)
        curvature_scaling = self._compute_curvature_scaling(closest, closest_segment_index)

        absolute_goal = self._find_goal_point(robot_state, lookahead) or closest
        relative_goal = utils.vehicle_coords(robot_state, absolute_goal)

        D = utils.distance_between(Point(), relative_goal)
        x = relative_goal.x
        curvature = curvature_scaling * ((2 * x) / (D * D))

        remaining_distance = utils.distance_between(robot_state.position, self.segments[-1].end)

        return PathState(
            curvature=curvature,
            goal_point=absolute_goal,
            path_position=closest,
            goal_direction=utils.signum(relative_goal.y, separate_zero=False),
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

        start_lookahead = segment.start_lookahead
        try:
            end_lookahead = self.segments[segment_index + 1].start_lookahead
        except IndexError:
            end_lookahead = start_lookahead

        distance_to_end = utils.distance_between(path_position, segment.end)
        lookahead = utils.interpolate(end_lookahead, start_lookahead, 0, segment.length,
                                      distance_to_end)

        return lookahead

    def _compute_curvature_scaling(self, path_position: Point, segment_index: int) -> float:
        segment = self.segments[segment_index]

        start_curve_scaling = segment.start_curvature_scaling
        try:
            end_curve_scaling = self.segments[segment_index + 1].start_curvature_scaling
        except IndexError:
            end_curve_scaling = start_curve_scaling

        distance_to_end = utils.distance_between(path_position, segment.end)
        curvature_scaling = utils.interpolate(end_curve_scaling, start_curve_scaling, 0,
                                              segment.length, distance_to_end)

        return curvature_scaling
