"""
Class for using an Adeept HAT line-tracking module.

To use, add the following line to the top of your module:

    from adeept_components_linetracker import LineTracker
"""

from typing import Callable, List

from RPi import GPIO
from adeept_components import IRSensor

# ======================================================================
# LINETRACKER CLASS DEFINITION
# ======================================================================

class LineTracker():
    """
    Monitor the state of a line-tracking module.

    Connect the line-tracking module to the "Tracking" port on the
    Adeept HAT.

    The line-tracking module has three infrared emitters & sensors on
    it.  Each sensor is connected to a pin on the GPIO.  A white surface
    will set the pin LOW; a black surface will set the pin HIGH.

    Sensitivity can be adjusted by turning the on-board potentiometer
    with a Phillips screwdriver.

    Instances of this class can be set to detect either black lines on a
    white surface (default) or white lines on a black surface.

    Two means of detection are supported:  polling and callback
    functions.
    """

    CALLBACK_TYPE = Callable[[bool, bool, bool], None]

    def __init__(self, pin_left:  int, pin_middle:  int, pin_right:  int,
                 line_is_white:  bool = False) -> None:
        """
        Manage an infrared sensor.
        """

        self._PIN_LEFT:   IRSensor = IRSensor(pin_left, self._on_change)
        self._PIN_MIDDLE: IRSensor = IRSensor(pin_middle, self._on_change)
        self._PIN_RIGHT:  IRSensor = IRSensor(pin_right, self._on_change)

        if line_is_white is True:
            self._detection_state = GPIO.LOW
        elif line_is_white is False:
            self._detection_state = GPIO.HIGH
        else:
            raise ValueError(f"line_is_white ({line_is_white}) must be either "
                             f"True or False.")

        self._callbacks:  List[Callable[[bool, bool, bool], None]] = []

    # ------------------------------------------------------------------

    def line_is_white(self) -> None:
        """
        Sets the module to track a white line on a black background.
        """

        self._detection_state = GPIO.LOW

    # ------------------------------------------------------------------

    def line_is_black(self) -> None:
        """
        Sets the module to track a black line on a white background.
        """

        self._detection_state = GPIO.HIGH

    # ------------------------------------------------------------------

    @property
    def left(self) -> bool:
        """
        Get the state of the left IR sensor.

        Returns
        -------
        True if a line was detected, False if otherwise.
        """

        return self._PIN_LEFT.state == self._detection_state

    # ------------------------------------------------------------------

    @property
    def middle(self) -> bool:
        """
        Get the state of the middle IR sensor.

        Returns
        -------
        True if a line was detected, False if otherwise.
        """

        return self._PIN_MIDDLE.state == self._detection_state

    # ------------------------------------------------------------------

    @property
    def right(self) -> bool:
        """
        Get the state of the right IR sensor.

        Returns
        -------
        True if a line was detected, False if otherwise.
        """

        return self._PIN_RIGHT.state == self._detection_state

    # ------------------------------------------------------------------

    def add_callback(self, callback:  CALLBACK_TYPE) -> None:
        """
        Add a function to be called when an IR sensor's state changes.

        A callback function can only be added once; subsequent attempts
        to add it will be silently ignored.

        Callback functions should exit as soon as possible since state
        changes can occur rapidly.

        Parameters
        ----------
        callback:  CALLBACK_TYPE
            The function to be called when an IR sensor's state changes.
        """

        if callback not in self._callbacks:
            self._callbacks.append(callback)

    # ------------------------------------------------------------------

    def remove_callback(self, callback:  CALLBACK_TYPE) -> None:
        """
        Remove a prevously-added callback function.

        Attempts to remove a callback function that isn't registered
        will be silently ignored.

        Parameters
        ----------
        callback:  CALLBACK_TYPE
            The callback function to be removed.
        """

        if callback in self._callbacks:
            self._callbacks.remove(callback)

    # ------------------------------------------------------------------

    def _on_change(self) -> None:
        for listener in self._callbacks:
            listener(self.left, self.middle, self.right)
