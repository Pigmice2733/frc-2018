import json


class NetworkTablesSender:
    """Send data over NetworkTables

    Can handle tuples, strings, numbers, bools and lists
    Tuples will be decomposed and sent in NetworkTables in a nested structure
    identical to that of the tuple. Lists will be sent as a json dump.
    """

    def __init__(self, table):
        self.table = table

    def send(self, value, name):
        """Send a value into NetworkTables using `name` as the root of the
        path.
        """
        if isinstance(value, tuple):
            for key in value._fields:
                self._send_value(key, getattr(value, key), name + "/")
        else:
            self._send_value("", value, name)

    def _send_value(self, key, value, path):
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
            success = self.table.putStringArray(path + key, str_array)
        else:
            success = self.table.putString(path + key, json.dumps(value))

        if not success:
            self.table.delete(path + key)
            self._send_value(key, value, path)
