import json


class NetworkTablesStreamer:
    def __init__(self, table):
        self.table = table

    def stream(self, value, name):
        """value must be a tuple"""
        if isinstance(value, tuple):
            for key in value._fields:
                self._send_value(key, getattr(value, key), name + "/")
        elif isinstance(value, list):
            self._send_value("", value, name)

    def _send_value(self, key, value, path):
        if isinstance(value, tuple):
            for key in value._fields:
                self._send_value(key, getattr(value, key), path)
        elif isinstance(value, int) or isinstance(value, float):
            self.table.putNumber(path + key, value)
        elif isinstance(value, bool):
            self.table.putBoolean(path + key, value)
        elif isinstance(value, str):
            self.table.putString(path + key, value)
        else:
            self.table.putString(path + key, json.dumps(value))
