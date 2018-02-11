import math
import _thread
from typing import Callable, List

from networktables.networktable import NetworkTable

from motioncontrol.path import Path
from motioncontrol.utils import RobotState, Point

from utils import NetworkTablesSender


class Selector:
    path_data = {}
    starting_position = None

    def __init__(self,
                 autonomous_chooser_table: NetworkTable,
                 path_tracking_table: NetworkTable,
                 path_selection_table: NetworkTable,
                 path_tracking_sender: NetworkTablesSender,
                 path_selection_sender: NetworkTablesSender,
                 robot_state_output: Callable[[RobotState], None],
                 is_robot_disabled: Callable[[], bool]):
        self.autonomous_chooser_table = autonomous_chooser_table
        self.path_tracking_table = path_tracking_table
        self.path_selection_table = path_selection_table
        self.path_tracking_sender = path_tracking_sender
        self.path_selection_sender = path_selection_sender
        self.robot_state_output = robot_state_output
        self.is_robot_disabled = is_robot_disabled

        default_mode = self.autonomous_chooser_table.getString("default", None)
        self._update_selected_autonomous(default_mode)

        self._register_network_tables_listeners()

    @staticmethod
    def add_new_path(mode_name,
                     default_state,
                     initial_states,
                     waypoints):
        Selector.path_data[mode_name] = {
            'waypoints': {},
            'initial_states': {
                'default': default_state,
            }
        }

        for state in initial_states:
            Selector.path_data[mode_name]['initial_states'][state[0]] = state[1]
            Selector.path_data[mode_name]['waypoints'][state[0]] = waypoints[state[0]]

    @staticmethod
    def mirror_waypoints(waypoints: List[Point], field_width: float) -> List[Point]:
        def mirrored_point(point: Point) -> Point:
            return Point(field_width - point.x, point.y)

        mirrored_waypoints = [mirrored_point(point) for point in waypoints]
        return mirrored_waypoints

    def _register_network_tables_listeners(self):
        def mode_listener(source, key, value, isNew): return _thread.start_new_thread(
            self._update_selected_autonomous, (value,))

        def initial_state_listener(source, key, value, isNew): return _thread.start_new_thread(
            self._update_starting_state, (value,))

        self.autonomous_chooser_table.addEntryListener(
            mode_listener, immediateNotify=True, localNotify=True, key="selected")

        self.path_selection_table.addEntryListener(
            initial_state_listener, immediateNotify=True, localNotify=True, key="initial_state")

    def _update_selected_autonomous(self, mode: str):
        self.path = Selector.path_data.get(mode, None)
        if self.path is None:
            self._update_path('None', None)
            return

        default_state_name = self.path['initial_states']['default']
        Selector.starting_position = default_state_name
        initial_state = self.path['initial_states'][default_state_name]

        waypoints = self.path['waypoints'][default_state_name]

        initial_states = [key for key in self.path['initial_states'].keys() if key != 'default']

        self.path_selection_sender.send(initial_states, 'initial_states')
        self._update_path(waypoints, initial_state)

    def _update_starting_state(self, state_name: str):
        Selector.starting_position = state_name
        initial_state = self.path['initial_states'][state_name]
        waypoints = self.path['waypoints'][state_name]

        self._update_path(waypoints, initial_state)

    def _update_path(self,
                     waypoints,
                     initial_state):
        if not self.is_robot_disabled():
            return

        if waypoints is not None and initial_state is not None:
            path = Path(initial_state, 0, waypoints=waypoints)
            self.path_tracking_sender.send(path.points, "path")
            self.path_tracking_sender.send(path.initial_state, "robot_state")
            self.robot_state_output(path.initial_state)
            return
        self.path_tracking_sender.send([], 'path')
        self.path_selection_sender.send([], 'initial_states')
        self.path_tracking_sender.send(RobotState(
            position=Point(), rotation=math.pi / 2), 'robot_state')
        self.robot_state_output(RobotState(position=Point(), rotation=math.pi / 2))
