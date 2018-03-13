import json
from enum import Enum

from networktables import NetworkTables


class NTStreamer:
    """Send data over NetworkTables

    NamedTubles be will sent with each value under a subtable created from the tuple key.

    Lists will be sent as string arrays created by calling json.dumps on each element.

    Numbers, bools and strings will be sent as such, everything that doesn't it into one of these
    categories will be sent as a string created by calling json.dumps.

    Identical values will not to sent so as not to clog network bandwidth. NTStreamer is a one way
    connection.
    """

    def __init__(self, initial_value, key: str, table: str = 'components',
                 round_digits: int = None):
        self.value = initial_value
        self.key = key
        self.table = NetworkTables.getTable(table)
        self.rounding = round_digits

        self._send(self.value, self.key)

    def send(self, value):
        """Send `value` into NetworkTables if it is a new value"""
        if value == self.value:
            return
        self.value = value
        if isinstance(value, tuple):
            for key in value._fields:
                self._send(getattr(value, key), key, self.key + "/")
        else:
            self._send(value, self.key)

    def _send(self, value, key: str, path: str = "", retry_on_fail=True):
        success = True
        if isinstance(value, tuple):
            path = path + key + "/"
            for key in value._fields:
                self._send(getattr(value, key), key, path)
        elif isinstance(value, int) or isinstance(value, float):
            value = round(value, self.rounding) if self.rounding is not None else value
            success = self.table.putNumber(path + key, value)
        elif isinstance(value, Enum):
            success = self.table.putString(path + key, value.value)
        elif isinstance(value, bool):
            success = self.table.putBoolean(path + key, value)
        elif isinstance(value, str):
            success = self.table.putString(path + key, value)
        elif isinstance(value, list):
            if all(isinstance(n, str) for n in value):
                str_array = value
            else:
                str_array = [json.dumps(x) for x in value]
            success = self.table.putStringArray(path + key, str_array)
        else:
            success = self.table.putString(path + key, json.dumps(value))

        if not success and retry_on_fail:
            self.table.delete(path + key)
            self._send(value, key, path, retry_on_fail=False)
