"""
Classes for controlling Adeept HAT buzzers.

To use, add one of the following lines (depending on which type of
buzzer is connected to that HAT) to the top of your module:

    from adeept_components_buzzers import BuzzerActive
    from adeept_components_buzzers import BuzzerPassive
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

from typing import Union

from RPi import GPIO

from adeept_components import _validate_gpio_pin_number

# ======================================================================
# BUZZERACTIVE CLASS DEFINITION
# ======================================================================

class BuzzerActive:
    """
    Control an active buzzer.

    Connect the buzzer to the "Buzzer" port on the Adeept HAT.

    An active buzzer has a built-in oscillator and only needs a steady
    DC current to make a sound.  It is simpler to operate than a
    passive buzzer but there is no way to change its frequency.

    Parameters
    ----------
    pin:  int
        The GPIO pin (out) that controls the buzzer.

    Raises
    ------
    ValueError
        "pin" isn't a valid GPIO BCM pin
    """

    def __init__(self, pin:  int) -> None:
        """
        Prepare an active buzzer for use.
        """

        _validate_gpio_pin_number(pin, "pin")

        self._PIN = pin

        GPIO.setup(self._PIN, GPIO.OUT, initial = GPIO.LOW)

    # ------------------------------------------------------------------

    def start(self) -> None:
        """
        Make the buzzer buzz.
        """

        GPIO.output(self._PIN, GPIO.HIGH)

    # ------------------------------------------------------------------

    def stop(self) -> None:
        """
        Silence the buzzer.
        """

        GPIO.output(self._PIN, GPIO.LOW)

# ======================================================================
# BUZZERPASSIVE CLASS DEFINITION
# ======================================================================

class BuzzerPassive:
    """
    Control a passive buzzer.

    Connect the buzzer to the "Buzzer" port on the Adeept HAT.

    A passive buzzer needs an oscillating current to make a sound.  It
    is slightly more complex to operate than an active buzzer but its
    frequency can be changed.

    Parameters
    ----------
    pin:  int
        The GPIO pin (output) that controls the buzzer's frequency.

    Attributes
    ----------
    frequency

    Raises
    ------
    ValueError
        "pin" isn't a valid GPIO BCM pin
    """

    # Class Constants
    # ---------------
    # _PWM_DUTY_CYCLE:  int
    #     The PWM duty cycle to use when making the buzzer buzz.  It
    #     should be a square wave otherwise the buzzer may sound weak
    #     or strange and burn out prematurely.
    #
    # Instance Constants
    # ------------------
    # _PIN:  int
    #     The GPIO pin (output) that controls the buzzer's frequency.
    #
    # Instance Variables
    # ------------------
    # _frequency:  int
    #     The frequency that the buzzer is currently buzzing at (in
    #     hertz).
    # _controller:  Union[RPi.GPIO.PWM, None]
    #     The PWM contoller for the buzzer (used to change the buzzer's
    #     frequency), if any.
    #
    # Class Invariants
    # ----------------
    # If _controller is None then _frequency MUST be 0.
    #
    # Design Notes
    # ------------
    # A GPIO.PWM isntance cannot be created in a stopped state.
    # Therefore, an instance isn't created until it's first needed (i.e.
    # when set_frequency() is called with a non-zero argument).

    _PWM_DUTY_CYCLE:  int = 50      # makes a nice, even square PWM wave

    # ------------------------------------------------------------------

    def __init__(self, pin:  int) -> None:
        """
        Prepare a passive buzzer for use.
        """

        _validate_gpio_pin_number(pin, "pin")

        self._PIN:        int = pin
        self._frequency:  int = 0

        GPIO.setup(self._PIN, GPIO.OUT, initial = GPIO.LOW)

        self._controller:  Union[GPIO.PWM, None] = None

    # ------------------------------------------------------------------

    def set_frequency(self, new_frequency:  int) -> None:
        """
        Set the frequency that the buzzer is to sound at.

        The buzzer can be silenced by setting its frequency to 0 hertz.

        Parameters
        ----------
        frequency:  int
            The buzzer's new frequency (in hertz).

        Raises
        ------
        ValueError
            "new_frequency" is negative.

        Notes
        -----
        According to `Wikipedia
        <https://en.wikipedia.org/wiki/Hearing_range>`, human hearing
        range is commonly given as 20Hz to 20,000Hz.
        """

        if new_frequency < 0:
            raise ValueError(f"\"new_frequency\" ({new_frequency}) must be " \
                             "a whole number.")

        assert (self._controller is not None) or (self._frequency == 0)

        # If "new_frequency" is 0 then silence the buzzer by stopping
        # PWM (if the PWM controller exists) -- otherwise, set the
        # buzzer's frequency to "new_frequency" (creating a new PWM
        # controller if necessary).  If the buzzer was previously silent
        # then start PWM.

        if new_frequency == 0:
            if self._controller is not None:
                self._controller.stop()
        else:
            if self._controller is None:
                self._controller = GPIO.PWM(self._PIN, new_frequency)
            else:
                self._controller.ChangeFrequency(new_frequency)

            if self._frequency == 0:
                self._controller.start(self._PWM_DUTY_CYCLE)

        # Remember the new frequency.

        self._frequency = new_frequency

    # ------------------------------------------------------------------

    def stop(self) -> None:
        """
        Silence the buzzer.
        """

        # Code re-use at its finest.

        self.set_frequency(0)

    # ------------------------------------------------------------------

    frequency = property(lambda self:  self._frequency, set_frequency, None,
                         "The buzzer's frequency (in hertz).")
