import math
import _thread

from networktables.networktable import NetworkTable

from motioncontrol.path import Path
from motioncontrol.utils import RobotState, Point

from utils import NetworkTablesSender

from components.drivetrain import Drivetrain


class Selector:
    autonomous_chooser_table = NetworkTable
    path_tracking_table = NetworkTable
    path_selection_table = NetworkTable
    path_tracking_sender = NetworkTablesSender
    path_selection_sender = NetworkTablesSender
    drivetrain = Drivetrain

    path_data = {}

    def setup(self):
        default_mode = self.autonomous_chooser_table.getString("default", None)
        self._update_selected_autonomous(default_mode)

        self._register_network_tables_listeners()

    @staticmethod
    def add_new_path(mode_name,
                     waypoints,
                     default_state,
                     initial_states):
        print("New", mode_name)
        Selector.path_data[mode_name] = {
            'waypoints': waypoints,
            'initial_states': {
                'default': default_state,
            }
        }

        for state in initial_states:
            Selector.path_data[mode_name]['initial_states'][state[0]] = state[1]

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
        print(mode)
        self.path = Selector.path_data.get(mode, None)
        if self.path is None:
            self._update_path('None', None, False)
            return

        default_state_name = self.path['initial_states']['default']
        initial_state = self.path['initial_states'][default_state_name]

        waypoints = self.path['waypoints']

        initial_states = [key for key in self.path['initial_states'].keys() if key != 'default']

        self.path_selection_sender.send(initial_states, 'initial_states')
        self._update_path(waypoints, initial_state, False)

    def _update_starting_state(self, state_name: str):
        default_state_name = self.path['initial_states']['default']
        default_state = self.path['initial_states'][default_state_name]
        initial_state = self.path['initial_states'].get(state_name, default_state)

        waypoints = self.path['waypoints']

        self._update_path(waypoints, initial_state, initial_state == default_state)

    def _update_path(self,
                     waypoints,
                     initial_state,
                     mirror):
        if waypoints is not None and initial_state is not None:
            path = Path(initial_state, 0, waypoints, mirror=mirror)
            self.path_tracking_sender.send(path.points, "path")
            self.path_tracking_sender.send(path.initial_state, "robot_state")
            self.drivetrain.robot_state = path.initial_state
            self.drivetrain.navx.setAngleAdjustment(
                math.degrees(-self.drivetrain.robot_state.rotation))
            return
        self.path_tracking_sender.send([], "path")
        self.path_selection_sender.send([], 'initial_states')
        self.path_tracking_sender.send(RobotState(
            position=Point(), rotation=math.pi / 2), "robot_state")
        self.drivetrain.robot_state = RobotState(position=Point(), rotation=math.pi / 2)
        self.drivetrain.navx.setAngleAdjustment(math.degrees(-self.drivetrain.robot_state.rotation))

    def execute(self):
        pass
