import math
import _thread
from typing import Callable, List

import wpilib

from networktables.networktable import NetworkTable

from motioncontrol.utils import RobotState, Point

from utils import NetworkTablesSender


class Selector:
    path_data = {}

    @staticmethod
    def set_up(autonomous_chooser_table: NetworkTable, path_tracking_table: NetworkTable,
               path_selection_table: NetworkTable, path_tracking_sender: NetworkTablesSender,
               path_selection_sender: NetworkTablesSender,
               robot_state_output: Callable[[RobotState], None],
               is_robot_disabled: Callable[[], bool]):
        Selector.autonomous_chooser_table = autonomous_chooser_table
        Selector.path_tracking_table = path_tracking_table
        Selector.path_selection_table = path_selection_table
        Selector.path_tracking_sender = path_tracking_sender
        Selector.path_selection_sender = path_selection_sender
        Selector.robot_state_output = robot_state_output
        Selector.is_robot_disabled = is_robot_disabled

        default_mode = Selector.autonomous_chooser_table.getString("default", None)
        Selector._update_selected_autonomous(default_mode)

        Selector._register_network_tables_listeners()

    @staticmethod
    def game_message() -> str:
        return wpilib.DriverStation.getInstance().getGameSpecificMessage()

    @staticmethod
    def starting_position() -> str:
        return Selector.path_selection_table.getString("starting_position", None)

    @staticmethod
    def add_new_path(mode_name, default_state, starting_positions, waypoints):
        Selector.path_data[mode_name] = {
            'waypoints': {},
            'starting_positions': {
                'default': default_state,
            }
        }

        for state in starting_positions:
            Selector.path_data[mode_name]['starting_positions'][state[0]] = state[1]
            Selector.path_data[mode_name]['waypoints'][state[0]] = waypoints[state[0]]

    @staticmethod
    def mirror_waypoints(waypoints: List[Point], field_width: float) -> List[Point]:
        def mirrored_point(point: Point) -> Point:
            return Point(field_width - point.x, point.y)

        mirrored_waypoints = [mirrored_point(point) for point in waypoints]
        return mirrored_waypoints

    @staticmethod
    def _register_network_tables_listeners():
        def mode_listener(source, key, value, isNew):
            return _thread.start_new_thread(Selector._update_selected_autonomous, (value, ))

        def initial_state_listener(source, key, value, isNew):
            return _thread.start_new_thread(Selector._update_starting_state, (value, ))

        Selector.autonomous_chooser_table.addEntryListener(
            mode_listener, immediateNotify=True, localNotify=True, key="selected")

        Selector.path_selection_table.addEntryListener(
            initial_state_listener, immediateNotify=True, localNotify=True, key="starting_position")

    @staticmethod
    def _update_selected_autonomous(mode: str):
        Selector.path = Selector.path_data.get(mode, None)
        if Selector.path is None:
            Selector._update_path('None', None)
            return

        default_state_name = Selector.path['starting_positions']['default']
        initial_state = Selector.path['starting_positions'][default_state_name]

        waypoints = Selector.path['waypoints'][default_state_name]

        starting_positions = [
            key for key in Selector.path['starting_positions'].keys() if key != 'default'
        ]

        Selector.path_selection_sender.send(starting_positions, 'starting_positions')
        Selector._update_path(waypoints, initial_state)

    @staticmethod
    def _update_starting_state(state_name: str):
        initial_state = Selector.path['starting_positions'][state_name]
        waypoints = Selector.path['waypoints'][state_name]

        Selector._update_path(waypoints, initial_state)

    @staticmethod
    def _update_path(waypoints, initial_state):
        if not Selector.is_robot_disabled():
            return

        if waypoints is not None and initial_state is not None:
            Selector.path_tracking_sender.send(waypoints, "path")
            Selector.path_tracking_sender.send(initial_state, "robot_state")
            Selector.robot_state_output(initial_state)
            return
        Selector.path_tracking_sender.send([], 'path')
        Selector.path_selection_sender.send([], 'starting_positions')
        Selector.path_tracking_sender.send(
            RobotState(position=Point(), rotation=math.pi / 2), 'robot_state')
        Selector.robot_state_output(RobotState(position=Point(), rotation=math.pi / 2))
