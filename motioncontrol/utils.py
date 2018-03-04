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
    velocity: float = 0.0
    position: Point = Point(0, 0)
    rotation: float = math.pi / 2


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
    encoder_ticks: float
    revolutions_to_distance: float
    speed_scaling: float


def signum(value: float, separate_zero=True) -> int:
    if value < 0:
        return -1
    if value > 0:
        return 1
    return 0 if separate_zero else 1


def approximately_equal(first, second, error=1e-6):
    return math.fabs(first - second) < error


def clamp(value, minimum, maximum):
    """Clamp value within a range - range doesn't need to be ordered"""
    if minimum > maximum:
        minimum, maximum = maximum, minimum
    if value < minimum:
        return minimum
    if value > maximum:
        return maximum
    return value


def interpolate(min_out, max_out, min_in, max_in, val):
    val -= min_in
    val /= max_in - min_in
    dy = max_out - min_out
    return val * dy + min_out


def phase_time(time, start, end):
    """Return phase time - time since start of phase, but not after end"""
    return clamp(time - start, 0, end - start)


def distance_between(first, second) -> float:
    """Find the distance between two points or the length of a vector"""
    x_diff = second[0] - first[0]
    y_diff = second[1] - first[1]
    return math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2))


def angle_between(first, second) -> float:
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
    if approximately_equal(segment.start.x, segment.end.x):
        y = clamp(point.y, segment.start.y, segment.end.y)
        return Point(segment.start.x, y)
    x = clamp(point.x, segment.start.x, segment.end.x)
    segment_slope = slope(segment)
    # y = mx + b => b = y - mx
    y = (segment_slope * x) + (segment.start.y - (segment_slope * segment.start.x))
    return Point(x, y)


def move_point_along_line(segment: Line, point: Point, distance: float) -> Point:
    """Get a new point `distance` farther along `segment` than `point`"""
    v = (segment.end.x - segment.start.x, segment.end.y - segment.start.y)
    length = distance_between(segment.start, segment.end)
    if approximately_equal(length, 0):
        return line_segment_clamp(segment, point)
    # Normalize direction vector
    u = (v[0] / length, v[1] / length)
    return line_segment_clamp(segment,
                              Point(point.x + (distance * u[0]), point.y + (distance * u[1])))


def closest_point_on_line(line_segment: Line, point: Point) -> Point:
    """Given a point and a line segment, return the closest point on the line
    to the given point
    """
    closest = None
    if approximately_equal(line_segment.start.x, line_segment.end.x):
        closest = Point(line_segment.start.x,
                        clamp(point.y, line_segment.start.y, line_segment.end.y))
    elif approximately_equal(line_segment.start.y, line_segment.end.y):
        closest = Point(
            clamp(point.x, line_segment.start.x, line_segment.end.x), line_segment.start.y)
    else:
        perpendicular_slope = -1 / slope(line_segment)
        closest = line_intersection(line_segment, line_from_point_slope(perpendicular_slope, point))
        # There should always be an intersection between perpendicular
        # lines, if none is found this code is broken
        assert closest is not None

    return line_segment_clamp(line_segment, closest)


def vehicle_coords(state: RobotState, point: Point) -> Point:
    """Transform `point` into vehicle coords based on `state`
    """
    dx = point.x - state.position.x
    dy = point.y - state.position.y

    rotation_transform = math.pi / 2 - state.rotation

    x = (dx * math.cos(rotation_transform)) - (dy * math.sin(rotation_transform))
    y = (dy * math.cos(rotation_transform)) + (dx * math.sin(rotation_transform))

    return Point(x=x, y=y)


def circle_line_intersection(center: Point, radius: float, line: Line) -> typing.List[Point]:
    """Calculate intersections between the circle defined by `center` and
    `radius`, and `line`. If there are no intersections, `None` will be
    returned. Otherwise the return value will be a list of intersections.
    """
    dx = line.end.x - line.start.x
    dy = line.end.y - line.start.y
    dr = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
    D = (line.start.x - center.x) * (line.end.y - center.y) - \
        (line.end.x - center.x) * (line.start.y - center.y)

    discriminant = (math.pow(radius, 2) * math.pow(dr, 2)) - math.pow(D, 2)

    def sgn(x):
        return -1 if x < 0.0 else 1

    # No intersection
    if discriminant < 1e-6:
        return ()
    else:
        x = (D * dy) / math.pow(dr, 2)
        y = (-D * dx) / math.pow(dr, 2)

        # Tangent/degenerate intersection
        if approximately_equal(discriminant, 0.0, 1e-6):
            return (Point(x + center.x, y + center.y), )

        x_diff = sgn(dy) * dx * math.sqrt(discriminant) / math.pow(dr, 2)
        y_diff = math.fabs(dy) * math.sqrt(discriminant) / math.pow(dr, 2)

        first_intersection = Point(x + x_diff + center.x, y + y_diff + center.y)
        second_intersection = Point(x - x_diff + center.x, y - y_diff + center.y)
        return (first_intersection, second_intersection)


def line_intersection(first: Line, second: Line) -> Point:
    """Find intersection between lines using Cramer's rule

    Will return `None` for coincident or parallel lines."""

    x_diff = (first[0][0] - first[1][0], second[0][0] - second[1][0])
    y_diff = (first[0][1] - first[1][1], second[0][1] - second[1][1])

    def determinant(a, b):
        return (a[0] * b[1]) - (a[1] * b[0])

    divisor = determinant(x_diff, y_diff)
    if math.fabs(divisor) < 1e-6:
        # No useful intersection - either coincident or parallel lines
        return None

    d = (determinant(first.start, first.end), determinant(second.start, second.end))
    x = determinant(d, x_diff) / divisor
    y = determinant(d, y_diff) / divisor

    return Point(x, y)


def on_segment(point: Point, segment: Line) -> bool:
    if abs(clamp(point.x, segment.start.x, segment.end.x) - point.x) > 1e-4:
        return False
    if abs(segment.start.x - segment.end.x) < 1e-4:
        return abs(clamp(point.y, segment.start.y, segment.end.y) - point.y) < 1e-4
    segment_slope = slope(segment)
    y = (segment_slope * point.x) + (segment.start.y - (segment_slope * segment.start.x))
    return abs(point.y - y) < 1e-4


def tank_drive_odometry(current_wheel_distances: typing.Tuple[float, float],
                        previous_wheel_distances: typing.Tuple[float, float],
                        current_orientation: float, previous_orientation: float, position: Point,
                        velocity: float) -> RobotState:
    """Tank drive odometry - use sensors to estimate position

    Models driving as arc segments with arc length equal to the distance driven.
    """
    delta_left = current_wheel_distances[0] - previous_wheel_distances[0]
    delta_right = current_wheel_distances[1] - previous_wheel_distances[1]
    distance = (delta_left + delta_right) / 2

    delta_theta = current_orientation - previous_orientation
    if abs(delta_theta) > math.pi:
        delta_theta = math.copysign(2 * math.pi - abs(delta_theta), -delta_theta)

    if abs(delta_theta) < 1e-6:
        delta_x = round(distance * math.cos(current_orientation), 6)
        delta_y = round(distance * math.sin(current_orientation), 6)
        new_position = Point(position.x + delta_x, position.y + delta_y)
        return RobotState(velocity=velocity, position=new_position, rotation=current_orientation)

    radius = distance / abs(delta_theta)
    chord_angle = previous_orientation + delta_theta / 2
    chord_length = 2 * radius * math.sin(abs(delta_theta) / 2)

    delta_x = round(chord_length * math.cos(chord_angle), 6)
    delta_y = round(chord_length * math.sin(chord_angle), 6)

    new_position = Point(position.x + delta_x, position.y + delta_y)
    return RobotState(velocity=velocity, position=new_position, rotation=current_orientation)


def tank_drive_wheel_velocities(wheel_base: float, forward_speed: float,
                                curvature: float) -> (float, float):
    if math.fabs(curvature) > 1e-6:
        radius = 1.0 / curvature
        left_radius = (radius + wheel_base / 2)
        right_radius = (radius - wheel_base / 2)

        v_left = (left_radius / radius) * forward_speed
        v_right = (right_radius / radius) * forward_speed

        return v_left, v_right
    return forward_speed, forward_speed
