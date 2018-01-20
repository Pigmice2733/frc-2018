"""Utils holds common functionality for motion control"""

import math
import typing


class Point(typing.NamedTuple):
    x: float = 0.0
    y: float = 0.0


class Line(typing.NamedTuple):
    start: Point
    end: Point


class RobotState(typing.NamedTuple):
    velocity: float
    position: Point = Point(0, 0)
    angle: float = math.pi / 2


class Completed(typing.NamedTuple):
    done: float = False


class RobotCharacteristics(typing.NamedTuple):
    """** Note: The units don't matter, as long as they are all the same.
    This means using seconds, meters and meters per second is ok, but
    seconds, feet and yards per minute will not work**"""
    acceleration_time: float
    deceleration_time: float
    max_speed: float
    wheel_base: float


def clamp(value, minimum, maximum):
    """Clamp value within a range - range doesn't need to be ordered"""
    if minimum > maximum:
        minimum, maximum = maximum, minimum
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def phase_time(time, start, end):
    """Return phase time - time since start of phase, but not after end"""
    return clamp(time - start, 0, end - start)


def distance_between(first, second) -> float:
    """Find the distance between two points or the length of a vector"""
    x_diff = second[0] - first[0]
    y_diff = second[1] - first[1]
    return math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2))


def line_angle(first, second) -> float:
    """Find the angle of a line between the two points"""
    dy = second.y - first.y
    dx = second.x - first.x
    theta = math.atan2(dy, dx)
    if theta < 0:
        theta += 2 * math.pi
    return theta


def slope(line: Line) -> float:
    """Compute the slope of a line"""
    return (line.end.y - line.start.y) / (line.end.x - line.start.x)


def line_from_point_slope(slope: float, point: Point) -> Line:
    """Generate line from a point and a slope"""
    next_x = point.x + 1
    next_y = point.y + slope
    return Line(point, Point(next_x, next_y))


def line_segment_clamp(segment: Line, point: Point) -> Point:
    """Clamp point onto a line segment"""
    if segment.start.x == segment.end.x:
        y = clamp(point.y, segment.start.y, segment.end.y)
        return Point(segment.start.x, y)
    x = clamp(point.x, segment.start.x, segment.end.x)
    segment_slope = slope(segment)
    # y = mx + b => b = y - mx
    y = (segment_slope * x) + (segment.start.y -
                               (segment_slope * segment.start.x))
    return Point(x, y)


def move_point_along_line(segment: Line, point: Point,
                          distance: float) -> Point:
    """Get a new point `distance` farther along `segment` than `point`"""
    v = (segment.end.x - segment.start.x, segment.end.y - segment.start.y)
    length = distance_between(segment.start, segment.end)
    if length == 0:
        return line_segment_clamp(segment, point)
    # Normalize direction vector
    u = (v[0] / length, v[1] / length)
    return line_segment_clamp(segment,
                              Point(point.x + (distance * u[0]),
                                    point.y + (distance * u[1])))


def closest_point_on_line(line_segment: Line, point: Point) -> Point:
    """Given a point and a line segment, return the closest point on the line
    to the given point
    """
    closest = None
    if line_segment.start.x == line_segment.end.x:
        closest = Point(line_segment.start.x,
                        clamp(point.y,
                              line_segment.start.y, line_segment.end.y))
    elif line_segment.start.y == line_segment.end.y:
        closest = Point(clamp(point.x, line_segment.start.x,
                              line_segment.end.x), line_segment.start.y)
    else:
        perpendicular_slope = -1 / slope(line_segment)
        closest = line_intersection(
            line_segment, line_from_point_slope(perpendicular_slope, point))
        # There should always be an intersection between perpendicular
        # lines, if none is found this code is broken
        assert closest is not None

    return line_segment_clamp(line_segment, closest)


def line_intersection(first: Line, second: Line) -> Point:
    """Find intersection between lines using Cramer's rule

    Will return `None` for coincident or parallel lines."""

    x_diff = (first[0][0] - first[1][0], second[0][0] - second[1][0])
    y_diff = (first[0][1] - first[1][1], second[0][1] - second[1][1])

    def determinant(a, b):
        return (a[0] * b[1]) - (a[1] * b[0])

    divisor = determinant(x_diff, y_diff)
    if math.fabs(divisor) < 0.000001:
        # No useful intersection - either coincident or parallel lines
        return None

    d = (determinant(*first), determinant(*second))
    x = determinant(d, x_diff) / divisor
    y = determinant(d, y_diff) / divisor

    return Point(x, y)
