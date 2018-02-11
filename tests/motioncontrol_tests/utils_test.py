"""Test module for motioncontrol.utils.py"""

import math

import pytest

from motioncontrol import utils


def test_approximately_equal():
    assert utils.approximately_equal(1.000000000000, 1.00000000000001)
    assert utils.approximately_equal(-2.000000000001, -1.999999999999)
    assert not utils.approximately_equal(-2.001, -1.9998)
    assert not utils.approximately_equal(-5, 5)
    assert not utils.approximately_equal(-7.9999, 7.9999)


def test_clamp():
    assert utils.clamp(0.5, 0.0, 1.0) == 0.5
    assert utils.clamp(0.5, 1.0, 2.0) == 1.0
    assert utils.clamp(0.5, 2.0, 1.0) == 1.0
    assert utils.clamp(-0.5, -1.0, -0.8) == -0.8
    assert utils.clamp(-2.0, -3.0, -4.0) == -3.0


def test_phase_time():
    assert utils.phase_time(2.6, 1.3, 3.0) == 1.3
    assert utils.phase_time(2.6, 1.3, 2.0) == 0.7
    assert utils.phase_time(2.6, 3.0, 4.0) == 0.0


def test_distance_between():
    p1 = utils.Point(0.0, 0.0)
    p2 = utils.Point(1.0, 1.0)
    p3 = utils.Point(-1.0, -1.0)
    p4 = utils.Point(-4.0, 3.0)

    assert utils.distance_between(p1, p2) == pytest.approx(math.sqrt(2.0))
    assert utils.distance_between(p1, p3) == pytest.approx(math.sqrt(2.0))
    assert utils.distance_between(p4, p2) == pytest.approx(math.sqrt(29.0))
    assert utils.distance_between(p4, p3) == pytest.approx(5.0)


def test_line_angle():
    origin = utils.Point(0.0, 0.0)

    positive_horizontal = utils.Point(5.0, 0.0)
    negative_horizontal = utils.Point(-13.0, 0.0)
    negative_vertical = utils.Point(0.0, -67.0)
    quadrant_three = utils.Point(-1 / 2, -math.sqrt(3) / 2)
    quadrant_four = utils.Point(1 / 2, -math.sqrt(3) / 2)
    quadrant_two = utils.Point(-math.sqrt(3) / 2, 1 / 2)

    assert utils.line_angle(origin, positive_horizontal) == pytest.approx(0.0)
    assert utils.line_angle(
        origin, negative_horizontal) == pytest.approx(math.pi)
    assert utils.line_angle(
        origin, negative_vertical) == pytest.approx(3 * math.pi / 2)
    assert utils.line_angle(
        origin, quadrant_three) == pytest.approx(4 * math.pi / 3)
    assert utils.line_angle(
        origin, quadrant_four) == pytest.approx(5 * math.pi / 3)
    assert utils.line_angle(
        origin, quadrant_two) == pytest.approx(5 * math.pi / 6)


def test_slope():
    zero = utils.Line(utils.Point(), utils.Point(1.0, 0.0))
    one = utils.Line(utils.Point(), utils.Point(1.0, 1.0))
    negative = utils.Line(utils.Point(), utils.Point(4.0, -27.0))
    reversed_one = utils.Line(utils.Point(), utils.Point(-1.0, -1.0))

    assert utils.slope(zero) == 0.0
    assert utils.slope(one) == 1.0
    assert utils.slope(negative) == -27 / 4
    assert utils.slope(reversed_one) == 1.0


def test_line_from_point_slope():
    origin = utils.Point(0.0, 0.0)

    positive = utils.line_from_point_slope(1.0, origin)
    assert (positive.end.y - positive.start.y) / \
        (positive.end.x - positive.start.x) == 1.0
    cross = ((origin.x - positive.start.x) *
             (positive.end.y - positive.start.y) -
             (origin.y - positive.start.y) *
             (positive.end.x - positive.start.x))
    # Checks whether specified point is on line
    assert cross == pytest.approx(0.0)

    negative = utils.line_from_point_slope(-3.2, origin)
    assert (negative.end.y - negative.start.y) / \
        (negative.end.x - negative.start.x) == -3.2
    cross = ((origin.x - negative.start.x) *
             (negative.end.y - negative.start.y) -
             (origin.y - negative.start.y) *
             (negative.end.x - negative.start.x))
    # Checks whether specified point is on line
    assert cross == pytest.approx(0.0)


def test_line_segment_clamp():
    line_segment = utils.Line(utils.Point(-1.0, 1.0), utils.Point(2.0, -3.0))

    on_segment = utils.Point(0.0, -1 / 3)
    assert utils.line_segment_clamp(
        line_segment, on_segment) == pytest.approx(on_segment)

    off_segment = utils.Point(0.0, 2.0)
    assert utils.line_segment_clamp(
        line_segment, off_segment) == pytest.approx(on_segment)


def test_move_point_along_line():
    line_segment = utils.Line(utils.Point(-1.0, 1.0), utils.Point(2.0, -3.0))

    assert (utils.move_point_along_line(
        line_segment, utils.Point(0.0, -1 / 3), 10 / 3) ==
        pytest.approx(utils.Point(2.0, -3.0)))

    assert (utils.move_point_along_line(
        line_segment, utils.Point(0.0, -1 / 3), -20) ==
        pytest.approx(utils.Point(-1.0, 1.0)))

    assert (utils.move_point_along_line(
        utils.Line(utils.Point(), utils.Point()),
        utils.Point(1.0, 1.0),
        78.3
    ) == pytest.approx(utils.Point()))


def test_closest_point_on_line():
    line_segment = utils.Line(utils.Point(0.0, 2.0), utils.Point(2.0, 0.0))

    point_less = utils.Point(0.0, 0.0)
    assert utils.closest_point_on_line(
        line_segment, point_less) == pytest.approx(utils.Point(1.0, 1.0))

    point_more = utils.Point(1.0, 2.0)
    assert utils.closest_point_on_line(
        line_segment, point_more
    ) == pytest.approx(utils.Point(0.5, 1.5))

    point_off = utils.Point(-1.0, 4.0)
    assert utils.closest_point_on_line(
        line_segment, point_off) == pytest.approx(utils.Point(0.0, 2.0))

    horizontal_line = utils.Line(utils.Point(0.0, 0.0), utils.Point(5.0, 0.0))
    assert utils.closest_point_on_line(horizontal_line, utils.Point(
        1.0, 4.0)) == pytest.approx(utils.Point(1.0, 0.0))

    vertical_line = utils.Line(utils.Point(0.0, 0.0), utils.Point(0.0, -10.0))
    assert utils.closest_point_on_line(vertical_line, utils.Point(
        1.0, -12.0)) == pytest.approx(utils.Point(0.0, -10.0))


def test_vehicle_coords():
    origin = utils.Point(12, 12)
    point = utils.Point(12, 14)
    coords = utils.vehicle_coords(origin, math.pi / 2 - math.pi / 2, point)
    assert coords == pytest.approx(utils.Point(0, 2))
    coords = utils.vehicle_coords(origin, math.pi / 2 + math.pi / 2, point)
    assert coords == pytest.approx(utils.Point(0, -2))
    coords = utils.vehicle_coords(origin, math.pi / 2 - math.pi / 4, point)
    x = - math.sqrt(2)
    y = math.sqrt(2)
    assert coords == pytest.approx(utils.Point(x, y))
    origin = utils.Point(-10, 10)
    point = utils.Point(-8, 10)
    coords = utils.vehicle_coords(
        origin, math.pi / 2 - (3 * math.pi / 2), point)
    assert coords == pytest.approx(utils.Point(-2, 0))


def test_circle_line_intersection():
    center = utils.Point(-3.2, 5.6)
    radius = 4.2
    line = utils.Line(utils.Point(1, -10), utils.Point(1, 10))
    assert utils.circle_line_intersection(center, radius, line) == [
        pytest.approx(utils.Point(1, 5.6))]
    center = utils.Point(0.0, 0.0)
    radius = 1.0
    line = utils.Line(utils.Point(-2, -2), utils.Point(2, 2))
    assert ((utils.circle_line_intersection(center, radius, line) == [
        pytest.approx(utils.Point(math.sqrt(2) / 2, math.sqrt(2) / 2)),
        pytest.approx(utils.Point(-math.sqrt(2) / 2, -math.sqrt(2) / 2))]) or
        (utils.circle_line_intersection(center, radius, line) == [
            pytest.approx(utils.Point(-math.sqrt(2) / 2, -math.sqrt(2) / 2)),
            pytest.approx(utils.Point(math.sqrt(2) / 2, math.sqrt(2) / 2))]))


def test_line_intersection():
    horizontal_origin = utils.Line(
        utils.Point(-5.0, 0.0), utils.Point(5.0, 0.0))
    vertical_origin = utils.Line(utils.Point(
        0.0, 10.0), utils.Point(0.0, -10.0))

    assert (utils.line_intersection(
        horizontal_origin, vertical_origin) ==
        pytest.approx(utils.Point(0.0, 0.0)))
    assert (utils.line_intersection(
        vertical_origin, horizontal_origin) ==
        pytest.approx(utils.Point(0.0, 0.0)))

    assert (utils.line_intersection(
        utils.Line(utils.Point(0.5, 0.5), utils.Point(1.5, 0.5)),
        utils.Line(utils.Point(1.0, 0.0), utils.Point(1.0, 2.0))
    ) == pytest.approx(utils.Point(1.0, 0.5)))

    assert (utils.line_intersection(
        utils.Line(utils.Point(-2.0, -1.0), utils.Point(0.0, -2.0)),
        utils.Line(utils.Point(-1.5, -2.0), utils.Point(0.0, 2.5)),
    ) == pytest.approx(utils.Point(-1.2857142857142858, -1.3571428571428572)))


def test_on_segment():
    horizontal = utils.Line(utils.Point(-1, 2.33333), utils.Point(5, 2.33333))
    vertical = utils.Line(utils.Point(-5.66, -56), utils.Point(-5.66, -58))
    angled = utils.Line(utils.Point(-12, -5), utils.Point(-2, 5))

    assert utils.on_segment(utils.Point(-0.5, 2.33333), horizontal)
    assert not utils.on_segment(utils.Point(-0.5, 2.32), horizontal)
    assert not utils.on_segment(utils.Point(5.1, 2.33333), horizontal)

    assert utils.on_segment(utils.Point(-5.66, -57), vertical)
    assert not utils.on_segment(utils.Point(-5.67, -57), vertical)
    assert not utils.on_segment(utils.Point(-5.66, 57), vertical)

    assert utils.on_segment(utils.Point(-7, 0), angled)
    assert utils.on_segment(utils.Point(-12, -5), angled)
    assert not utils.on_segment(utils.Point(-7, 1), angled)
    assert not utils.on_segment(utils.Point(-7, -1), angled)
    assert not utils.on_segment(utils.Point(-13, -3), angled)
    assert not utils.on_segment(utils.Point(1, 0), angled)


def test_tank_drive_odometry():
    previous_wheel_distances = (1.0, 2.0)
    initial_position = utils.Point(-2.0, 3.0)
    initial_orientation = math.pi / 2
    velocity = 12.0

    orientations = [0, math.pi, math.pi / 4]
    positions = [utils.Point(-1.0, 4.0),
                 utils.Point(-3.0, 4.0),
                 utils.Point(-1.0 - math.sqrt(2) / 2, 3.0 + math.sqrt(2) / 2)]
    current_wheel_distances = [(1.0 + math.pi, 2.0),
                               (1.0, 2.0 + math.pi), (1.0 + math.pi / 2, 2.0)]

    for orientation, position, current_wheel_distances in zip(orientations,
                                                              positions,
                                                              current_wheel_distances):
        next_state = utils.RobotState(
            velocity=velocity,
            position=position,
            rotation=orientation)

        calculated_state = utils.tank_drive_odometry(
            current_wheel_distances,
            previous_wheel_distances,
            orientation,
            initial_orientation,
            initial_position,
            velocity)

        assert calculated_state.position == pytest.approx(next_state.position)
        assert calculated_state.rotation == pytest.approx(next_state.rotation)
        assert calculated_state.velocity == pytest.approx(next_state.velocity)


def test_tank_drive_wheel_velocities():
    wheel_base = 2
    forward_speed = 5.0

    assert utils.tank_drive_wheel_velocities(
        wheel_base, forward_speed, 0.0) == (forward_speed, forward_speed)

    assert utils.tank_drive_wheel_velocities(
        wheel_base, forward_speed, -0.25) == pytest.approx((3.75, 6.25))

    assert utils.tank_drive_wheel_velocities(
        wheel_base, forward_speed, 0.25) == pytest.approx((6.25, 3.75))
