"""PID control of motors and other output"""

from typing import Callable, NamedTuple

from .utils import clamp


class PIDCoefficients(NamedTuple):
    """Coefficients for PID controller

    `p`: Porportioanl term coefficient.
    `i`: Integral term coefficient.
    `d`: Derivative term coeffcient.
    """
    p: float
    i: float = 0.0
    d: float = 0.0


class PIDParameters(NamedTuple):
    """PID controller parameters

    `coefs`: Controller coefficents.
    `output_max`/`output_min`: Range to constrain output within.
    `input_max`/`input_min`: Range to expect input in - used only for
        continuous mode.
    `continuous`: Treat setpoint/input as a continuous range wrapping around
        at `input_max`/`input_min`.
    """
    coefs: PIDCoefficients
    output_max: float = 1.0
    output_min: float = -1.0
    input_max: float = None
    input_min: float = None
    continuous: bool = False


class PIDController:
    """PID controller for a process.

    See https://en.wikipedia.org/wiki/PID_controller for
    overview/control algorithm

    Supports both continuous and non-continuous input/setpoint
    ranges. For example, controlling motor speed is a
    non-continuous process, while position of a motor would be
    continuous as the position "wraps around" at some point.
    """

    def __init__(self, parameters: PIDParameters, time_source: Callable[[], float]):
        """PID controller constructor

        `parameters`: PIDParameters for controller
        """

        # PID control coefficients
        self._coefs = parameters.coefs

        # Internal variables used in the control alogorithm
        self._integral_term = 0.0
        # System time in seconds at last update
        self._previous_time = 0.0
        # PID input at last update
        self._previous_input = 0.0

        # Whether the setpoint/input range is continous
        self._continuous = parameters.continuous
        # Range of input/setpoint values to expect
        self._input_max = parameters.input_max
        self._input_min = parameters.input_min
        # Acceptable output range
        self._output_max = parameters.output_max
        self._output_min = parameters.output_min

        self.time_source = time_source

    def get_output(self, current_input: float, setpoint: float) -> float:
        """Get PID output for process

        `current_input`:
            The current PID input
        `setpoint`:
            Desired output of process/input to PID
        """

        current_time = self.time_source()

        # Time elapsed since last update
        time_change = current_time - self._previous_time

        # The current error
        current_error = self._get_continuous_error(setpoint - current_input)

        self._integral_term += self._coefs.i * (current_error * time_change)
        self._integral_term = clamp(self._integral_term, self._output_min, self._output_max)

        # Protect againsts ZeroDivisionError caused
        #  by time resolution in RobotPy simulator
        if time_change <= 0.0:
            time_change = 0.001

        derivative = (current_input - self._previous_input) / time_change

        self._previous_input = current_input
        self._previous_time = current_time

        output = ((self._coefs.p * current_error) + self._integral_term +
                  (self._coefs.d * derivative))
        return clamp(output, self._output_min, self._output_max)

    def reset(self):
        """Reset internal control variables

        Should be used if the PID is disabled for a time,
        then re-enabled.
        """
        self._previous_input = 0.0
        self._integral_term = 0.0
        self._previous_time = 0.0

    def _get_continuous_error(self, error):
        if self._continuous:
            if abs(error) > (self._input_max - self._input_min) / 2.0:
                if error > 0.0:
                    return error - (self._input_max - self._input_min)
                return error + (self._input_max - self._input_min)
        return error
