class NetworkTablesTupleStreamer:
    def __init__(self, table):
        self.table = table

    def stream(self, value):
        """value must be a tuple"""
        assert isinstance(value, tuple)
        for key in value._fields:
            key, getattr(value, key)

    def _send_value(self, key, value):
        if isinstance(value, tuple):
            for key in value._fields:
                self._send_value(key, getattr(value, key))
        elif isinstance(value, int) or isinstance(value, float):
            self.table.putNumber(key, value)
        elif isinstance(value, bool):
            self.table.putBoolean(key, value)
        else:
            self.table.putString(key, value)
