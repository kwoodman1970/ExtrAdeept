"""
Classes for controlling Adeept HAT buzzers.

To use, add one of the following lines (as appropriate) to the top of
your module:

    from adeept_components_buzzers import Buzzer
    from adeept_components_buzzers import BuzzerActive
    from adeept_components_buzzers import BuzzerPassive

Class Listings
--------------
Buzzer
BuzzerActive
BuzzerPassive
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

from typing import Union

from RPi import GPIO

from adeept_components import _validate_gpio_pin_number

# ======================================================================
# BUZZER CLASS DEFINITION
# ======================================================================

class Buzzer:
    """
    Base class for buzzer classes.

    Connect the buzzer to the "Buzzer" port on the Adeept HAT.

    Parameters
    ----------
    pin:  int
        The GPIO pin (output) that controls the buzzer.

    Raises
    ------
    ValueError
        "pin" isn't a valid GPIO BCM pin.
    """

    # Private Attributes
    # ------------------
    # _PIN:  int
    #     The GPIO pin (output) that controls the buzzer.  Descendant
    #     classes can use this constant.

    def __init__(self, pin:  int) -> None:
        """
        Prepare a buzzer for use.
        """

        _validate_gpio_pin_number(pin, "pin")

        self._PIN = pin

    # ------------------------------------------------------------------

    def stop(self) -> None:
        """
        Silence the buzzer.
        """

        # Descendant classes MUST override this method with their own
        # implementation.

# ======================================================================
# BUZZERACTIVE CLASS DEFINITION
# ======================================================================

class BuzzerActive(Buzzer):
    """
    Control an active buzzer.

    Connect the buzzer to the "Buzzer" port on the Adeept HAT.

    Acive buzzers have two connections:  POSITIVE (the longer pin, and
    can be connected to a voltage source) and NEGATIVE (the shorter pin,
    and can be connected to a ground).  As power flows through the
    buzzer, a built-in oscillator vibrates at a fixed frequency.

    A GPIO data pin doesn't have enough power on its own to drive an
    active buzzer -- therefore, it must be connected to a transister
    that's also connected to a +3.3V or +5V pin.  The buzzer can then be
    turned on and off by setting the data pin high and low,
    respectively.

    Parameters
    ----------
    pin:  int
        The GPIO pin (output) that controls the buzzer.

    Raises
    ------
    ValueError
        "pin" isn't a valid GPIO BCM pin.
    """

    def __init__(self, pin:  int) -> None:
        """
        Prepare an active buzzer for use.
        """

        Buzzer.__init__(self, pin)
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

class BuzzerPassive(Buzzer):
    """
    Control a passive buzzer.

    Connect the buzzer to the "Buzzer" port on the Adeept HAT.

    Passive buzzers have two connections that are connected to an
    alternating current source.  The frequency at which the current
    alternates is the frequency of the buzzer's sound.

    A GPIO data pin doesn't have enough power on its own to drive a
    passive buzzer -- therefore, it must be connected to a transister
    that's also connected to a +3.3V or +5V pin.  The buzzer can then be
    turned on by using pulse-width modulation on the data pin and turned
    off by stopping pulse-width modulation (a square PWM wave is the
    closest that the GPIO can come to an alternating current).

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
        "pin" isn't a valid GPIO BCM pin.

    Notes
    -----
    According to `Wikipedia
    <https://en.wikipedia.org/wiki/Hearing_range>`, human hearing range
    is commonly given as 20Hz to 20,000Hz.  Furthermore, different
    passive buzzers have different frequency ranges.
    """

    # Private Class Attributes
    # ------------------------
    # _PWM_DUTY_CYCLE:  int
    #     The pulse-width modulation duty cycle to use when making the
    #     buzzer buzz.  It should be a square wave otherwise the buzzer
    #     may sound weak or strange and burn out prematurely.
    #
    # Private Attributes
    # ------------------
    # _frequency:  int
    #     The frequency that the buzzer is currently buzzing at (in
    #     hertz).
    # _controller:  Union[RPi.GPIO.PWM, None]
    #     The pulse-width modulation contoller for the buzzer (used to
    #     change the buzzer's frequency), if any.
    #
    # Class Invariants
    # ----------------
    # If `_controller`` is None then `_frequency`` MUST be 0.
    #
    # Design Notes
    # ------------
    # A GPIO.PWM instance cannot be created with a frequency of 0 hertz.
    # Therefore, an instance isn't created until it's first needed (i.e.
    # when `set_frequency()` is called with a non-zero argument).

    _PWM_DUTY_CYCLE:  int = 50      # makes a nice, even square PWM wave

    # ------------------------------------------------------------------

    def __init__(self, pin:  int) -> None:
        """
        Prepare a passive buzzer for use.
        """

        Buzzer.__init__(self, pin)
        GPIO.setup(self._PIN, GPIO.OUT, initial = GPIO.LOW)

        self._frequency:   int                   = 0
        self._controller:  Union[GPIO.PWM, None] = None

    # ------------------------------------------------------------------

    def set_frequency(self, new_frequency:  int) -> None:
        """
        Set the frequency that the buzzer is to sound at.

        The buzzer can be silenced by setting its frequency to 0 hertz.

        Parameters
        ----------
        new_frequency:  int
            The buzzer's new frequency (in hertz).

        Raises
        ------
        ValueError
            "new_frequency" is negative.
        """

        if new_frequency < 0:
            raise ValueError(f"\"new_frequency\" ({new_frequency}) must be " \
                             "a whole number.")

        assert (self._controller is not None) or (self._frequency == 0)

        # If "new_frequency" is 0 then silence the buzzer by stopping
        # pulse-width modulation (if the PWM controller exists) --
        # otherwise, set the buzzer's frequency to "new_frequency"
        # (creating a PWM controller if necessary).  If the buzzer was
        # previously silent then start pulse-width modulation.

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
