import json
from typing import Callable

from networktables.networktable import NetworkTable
from wpilib import Timer


class RateLimiter:
    """Limit an action to once per configurable interval"""

    def __init__(self, interval: float, action: Callable):
        self.interval = interval
        self.action = action
        self.last_execution = Timer.getFPGATimestamp() * 1000

    def execute(self, *args):
        time = Timer.getFPGATimestamp() * 1000
        if time - self.last_execution > self.interval:
            self.action(args)
            self.last_execution = time


class NetworkTablesSender:
    """Send data over NetworkTables

    Can handle tuples, strings, numbers, bools and lists
    Tuples will be decomposed and sent in NetworkTables in a nested structure
    identical to that of the tuple. Lists will be sent as a json dump.
    """

    def __init__(self, table: NetworkTable):
        self.table = table

    def send(self, value, name: str):
        """Send a value into NetworkTables using `name` as the root of the
        path.
        """
        if isinstance(value, tuple):
            for key in value._fields:
                self._send_value(key, getattr(value, key), name + "/")
        else:
            self._send_value("", value, name)

    def _send_value(self, key: str, value, path: str):
        success = True
        if isinstance(value, tuple):
            path = path + key + "/"
            for key in value._fields:
                self._send_value(key, getattr(value, key), path)
        elif isinstance(value, int) or isinstance(value, float):
            success = self.table.putNumber(path + key, value)
        elif isinstance(value, bool):
            success = self.table.putBoolean(path + key, value)
        elif isinstance(value, str):
            success = self.table.putString(path + key, value)
        elif isinstance(value, list):
            str_array = [json.dumps(x) for x in value]
            if all(isinstance(n, str) for n in value):
                str_array = value
            success = self.table.putStringArray(path + key, str_array)
        else:
            success = self.table.putString(path + key, json.dumps(value))

        if not success:
            self.table.delete(path + key)
            self._send_value(key, value, path)
