"""
Classes for controlling Adeept HAT components.

This module contains classes for controlling the following components:

* direct-current drive motors
* ultrasonic rangefinders
* RGB LED's
* infrared sensors
* on/off devices (e.g. LED lamps)
* IIC devices (e.g. OLED displays)
* acceleration sensors

To use one or more of the classes in this module (see the **Class
Listings** section), add the following line to the top of your module::

    from adeept_components import *list_of_classes*

Different HAT's support different sets of components.

Class Listings
--------------
DriveMotor
UltrasonicSensor
IRSensor
RGB_LED
SingleSignal
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

from threading import Event
import time
from typing import Callable, List, Union

from RPi import GPIO

# ======================================================================
# PRIVATE SUBROUTINES
# ======================================================================

def _validate_gpio_pin_number(pin_number:  int, parameter_name:  str) -> None:
    """
    Confirm that a pin number is a valid GPIO BCM pin number.

    Parameters
    ----------
    pin_number:  int
        The pin number to validate
    parameter_name:  str
        The name of the caller's parameter that `pin_number` came from.
        If an exception is raised then this will be included in the
        error message.

    Raises
    ------
    ValueError
        `pin_number` isn't a valid GPIO BCM pin number.

    Notes
    -----
    Adeept HAT's connect to a 40-pin GPIO, which means that (as of this
    writing) Raspberry Pi models 2, 3 & 4 are supported.  Valid GPIO BCM
    pin numbers are within the same range for all of these models.  See
    `Raspberry Pi Pinout <https://pinout.xyz>`_ for details.
    """

    # Constants
    # ---------
    # GPIO_MIN_PIN:  int
    #     The minimum valid GPIO BCM pin number.
    # GPIO_MAX_PIN:  int
    #     The maximum valid GPIO BCM pin number.

    GPIO_MIN_PIN: int = 2
    GPIO_MAX_PIN: int = 27

    if (pin_number < GPIO_MIN_PIN) or (pin_number > GPIO_MAX_PIN):
        raise ValueError(f"\"${parameter_name}\" ({pin_number}) must be from "
                         f"${GPIO_MIN_PIN} to ${GPIO_MAX_PIN}")

# ======================================================================
# DRIVEMOTOR CLASS DEFINITION
# ======================================================================

class DriveMotor:
    """
    Control a DC drive motor that's connected to an L298 controller.

    Connect the drive motor to either the "Motor A" or "Motor B" port on
    the Adeept HAT.

    Parameters
    ----------
    enable_pin:  int
        The GPIO pin (output) that connects to the L298 controller's
        ENABLE connection.
    input_pin_1:  int
    input_pin_2:  int
        The GPIO pins (output) that connect to the L298 controller's
        INPUT connections.  If the motor rotates opposite to the desired
        direction then swap these two arguments.
    scale_factor:  float (optional)
        An adjustment for when one drive motor is slightly faster than
        another and needs to be slowed down (for example, to keep a
        vehicle moving in a straight line).  It must be least 0.0 and no
        greater than 1.0.  The default value is 1.0.

    Attributes
    ----------
    speed

    Raises
    ------
    ValueError
        One or more arguments are invalid.

    Notes
    -----
    An L298 controller has a VOLTAGE SUPPLY (Vs) connection (up to 42V,
    2A) and two sets of H-bridge connections.  Each H-bridge has an
    ENABLE connection, two INPUT connections and two OUTPUT connections.
    If ENABLE is high then power from VOLTAGE SUPPLY is sent to the
    OUTPUT connections according to the INPUT signals with the following
    effects on the drive motor::

    +-------+-------+--------+--------+----------+
    |INPUT 1|INPUT 2|OUTPUT 1|OUTPUT 2|  Effect  |
    +=======+=======+========+========+==========+
    |  Low  |  Low  |   0V   |   0V   |Fast Brake|
    +-------+-------+--------+--------+----------+
    | High  |  Low  |   Vs   |   0V   | Forward  |
    +-------+-------+--------+--------+----------+
    |  Low  | High  |   0V   |   Vs   | Reverse  |
    +-------+-------+--------+--------+----------+
    | High  | High  |   Vs   |   Vs   |Fast Brake|
    +-------+-------+--------+--------+----------+

    "Forward" and "reverse" are arbitrary directions in this table.

    If ENABLE is low then the motor will receive no power and it will
    freewheel.  Furthermore, PWM can be used on the ENABLE connection to
    control the speed of the motor.

    See STMicroelectronics' `Dual full-bridge driver
    <https://www.st.com/resource/en/datasheet/l298.pdf>` datasheet (with
    particular attention to Figure 6) for details.

    On an Adeept HAT, the L298 controller's VOLTAGE SUPPLY is connected
    either to the HAT's "Vin" port or GPIO power pins, the ENABLE &
    INPUT connections are connected to GPIO I/O pins and the OUTPUT
    connections are connected to the HAT's "Motor A" and "Motor B"
    ports.
    """

    # Class Private Attributes
    # ------------------------
    # _PWM_FREQUENCY:  int
    #     The PWM frequency to use with the drive motor.
    #
    # Private Attributes
    # ------------------
    # _ENABLE_PIN:  int
    #     The GPIO pin (output) that connects to the L298 controller's
    #     ENABLE connection.
    #
    # _INPUT_PIN_1:  int
    # _INPUT_PIN_2:  int
    #     The GPIO pins (output) that connect to the L298 controller's
    #     INPUT connections.  If the motor rotates opposite to the
    #     desired direction then swap these two values.
    #
    # _SCALE_FACTOR:  float
    #     An adjustment for when one drive motor is slightly faster than
    #     another and needs to be slowed down (for example, to keep a
    #     vehicle moving in a straight line).
    #
    # _CONTROLLER:  RPi.GPIO.PWM
    #     A PWM controller for setting the speed of the motor.
    #
    # _speed:  int, None
    #     The drive motor's current speed (-100 to 100).  A value of 0
    #     means that the motor is freewheeling.  A value of "None" means
    #     that an electromotive brake has been applied to the motor.

    # _PWM_FREQUENCY:  int = 2000
    _PWM_FREQUENCY:  int = 1000

    def __init__(self, enable_pin:  int, input_pin_1:  int, input_pin_2:  int,
                 scale_factor:  float = 1.0) -> None:
        """
        Prepare an L298-connected DC drive motor for use.
        """

        _validate_gpio_pin_number(enable_pin, "enable_pin")
        _validate_gpio_pin_number(input_pin_1, "input_pin_1")
        _validate_gpio_pin_number(input_pin_2, "input_pin_2")

        if (scale_factor <= 0.0) or (scale_factor > 1.0):
            raise ValueError(f"\"scale_factor\" ({scale_factor}) must be "
                             f"at least 0.0 and no greater than 1.0")

        # Declare private properties.

        self._ENABLE_PIN:    int   = enable_pin
        self._INPUT_PIN_1:   int   = input_pin_1
        self._INPUT_PIN_2:   int   = input_pin_2
        self._SCALE_FACTOR:  float = scale_factor

        GPIO.setup(self._ENABLE_PIN,  GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(self._INPUT_PIN_1, GPIO.OUT, initial = GPIO.HIGH)
        GPIO.setup(self._INPUT_PIN_2, GPIO.OUT, initial = GPIO.LOW)

        # try:
        #     self._CONTROLLER = GPIO.PWM(self._ENABLE_PIN, self._PWM_FREQUENCY)
        # except:
        #     pass

        self._controller = GPIO.PWM(self._ENABLE_PIN, self._PWM_FREQUENCY)
        self._speed      = 0

        self._controller.start(self._speed)

    # ------------------------------------------------------------------

    def set_speed(self, speed:  int) -> None:
        """
        Set the speed of the drive motor.

        Parameters
        ----------
        speed:  int
            The new speed for the drive motor (must be from -100 to
            100).  Negative values will cause the motor to rotate in
            reverse.  A value of 0 will turn off the power to the drive
            motor and let it freewheel.

        Raises
        ------
        ValueError
            "speed" is an invalid value.
        """

        if abs(speed) > 100:
            raise ValueError(f"\"speed\" ({speed}) must be from -100 to 100.")

        # Duty cycle must always be a positive integer -- therefore,
        # direction of rotation is determined by which INPUT pin is HIGH
        # and which one is LOW.

        if speed >= 0:
            GPIO.output(self._INPUT_PIN_1, GPIO.HIGH)
            GPIO.output(self._INPUT_PIN_2, GPIO.LOW)
        else:
            GPIO.output(self._INPUT_PIN_1, GPIO.LOW)
            GPIO.output(self._INPUT_PIN_2, GPIO.HIGH)

        new_duty_cycle = round(abs(speed * self._SCALE_FACTOR))
        self._speed    = speed

        self._controller.ChangeDutyCycle(new_duty_cycle)

    # ------------------------------------------------------------------

    def brake(self) -> None:
        """
        Apply an electromotive brake to the motor.
        """

        GPIO.output(self._INPUT_PIN_1, GPIO.LOW)
        GPIO.output(self._INPUT_PIN_2, GPIO.LOW)
        self._controller.ChangeDutyCycle(100)

        self._speed = None

    # ------------------------------------------------------------------

    speed = property(lambda self:  self._speed, set_speed, None,
                     "The speed of the drive motor (None means brake "
                     "is applied).")

# ======================================================================
# HC-SR04 ULTRSONIC SENSOR CLASS DEFINITION
# ======================================================================

class UltrasonicSensor:
    """
    Control an HC-SR04 ultrasonic distance sensor.

    Connect this component to the "Ultrasonic" port on the Adeept HAT.
    Do NOT connect it to any other port -- doing so will likely damage
    it!

    Parameters
    ----------
    trigger_pin:  int
        The GPIO pin (output) that connects to the HC-SR04 sensor's
        TRIGGER connection.
    echo_pin:  int
        The GPIO pin (input) that connects to the HC-SR04 sensor's
        ECHO connection.

    Raises
    ------
    ValueError
        One or more arguments are invalid.

    Notes
    -----
    An HC-SR04 module has a TRIGGER connection and an ECHO connection.
    When TRIGGER is high for at least 10us, the module will emit eight
    40Khz ultrasonic pulses, then set ECHO high.  It will then set ECHO
    low either when it detects an ultrasonic echo or after 38ms has
    elapsed (whichever occurs first).

    This sequence can take up to 60ms.

    If an ultrasonic echo was detected then the distance can be
    calculated by multiplying the total time that ECHO was high by the
    speed of sound, then dividing by two.

    See `HC-SR04 datasheet
    <https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf>`_
    (one of many sources) for details.

    IMPORTANT NOTE ABOUT ACCURACY:

    It isn't always possible to make accurate measurements due to the
    way that Python and Linux can interrupt a program to deal with other
    processess.  This can be mitigated by taking multiple readings and
    using smoothing algorithms.
    """

    # Class Private Attributes
    # ------------------------
    # _TRIGGER_INTERVAL
    #     The minimum interval between calls to `get_distance()`.
    # _TRIGGER_DURATION:  float
    #     How long to keep TRIGGER high in order to activate the
    #     distance finder.
    # _ECHO_TIMEOUT:  float
    #     How long to wait for an echo before giving up.
    # _SPEED_OF_SOUND:  float
    #     The speed of sound in metres per second at 20°C.
    #
    # Private Attributes
    # ------------------
    # _TRIGGER_PIN:  int
    #     The GPIO pin (output) that's connected to TRIGGER.
    # _ECHO_PIN:  int
    #     The GPIO pin (input) that's connected to ECHO.
    # _last_trigger_time:  float
    #     The time at which the last entire cycle started to be
    #     triggered.

    _TRIGGER_INTERVAL:  float =   0.065
    _TRIGGER_DURATION:  float =   0.00001
    _ECHO_TIMEOUT:      float =   0.037
    _SPEED_OF_SOUND:    float = 343.42

    # ------------------------------------------------------------------

    def __init__(self, trigger_pin:  int, echo_pin:  int) -> None:
        """
        Prepare an HC_SR04 ultrasonic distance sensor for use.
        """

        _validate_gpio_pin_number(trigger_pin, "trigger_pin")
        _validate_gpio_pin_number(echo_pin, "echo_pin")

        self._TRIGGER_PIN:  int = trigger_pin
        self._ECHO_PIN:     int = echo_pin

        self._last_trigger_time: float = 0.0

        GPIO.setup(self._TRIGGER_PIN,  GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(self._ECHO_PIN, GPIO.IN)

    # ------------------------------------------------------------------

    def get_distance(self) -> Union[float, None]:
        """
        Detect the distance to an object.

        Returns
        -------
        Union[float, None]
            The distance detected by the component in meters/second
            (residents of the United States of America, Liberia and
            Myanmar can convert this to feet/second by multiplying this
            value by 3.28084 feet/meter).  If no distance was detected
            (i.e.  the component timed-out) then `None` is returned.
        """

        start_time:  float = 0.0
        end_time:    float = 0.0
        blocker:     Event = Event()

        def _on_edge(_:  int):
            nonlocal start_time
            nonlocal end_time
            nonlocal blocker

            if start_time == 0.0:
                start_time = time.time()
            elif end_time == 0.0:
                end_time = time.time()

                blocker.set()

        # First, make sure that enough time has pased since the last
        # time the sensor was triggered.

        time.sleep(max(0.0, self._last_trigger_time + self._TRIGGER_INTERVAL
                       - time.time()))

        self._last_trigger_time = time.time()

        # This method uses the GPIO event detection callback mechanism
        # to track edge events on the ECHO pin.  Only one type of edge
        # event (rising, falling or both) can be tracked on any given
        # GPIO pin at a time -- therefore, a single callback function
        # must be used.
        #
        # Since it's possible for edge events to be missed, timeouts on
        # blocking operations must be used.

        GPIO.add_event_detect(self._ECHO_PIN, GPIO.BOTH, callback = _on_edge)

        GPIO.output(self._TRIGGER_PIN,GPIO.HIGH)
        time.sleep(self._TRIGGER_DURATION)
        GPIO.output(self._TRIGGER_PIN, GPIO.LOW)

        echo_detected:  bool = blocker.wait(self._TRIGGER_INTERVAL)

        GPIO.remove_event_detect(self._ECHO_PIN)

        # The time from when the response pin goes high to when it goes
        # low is the delay betwen transmitting and receiving the
        # ultrasonic signals.

        if echo_detected:
            duration:  float = end_time - start_time

            if duration <= self._ECHO_TIMEOUT:
                return duration * self._SPEED_OF_SOUND / 2.0

        return None

# ======================================================================
# IRSENSOR CLASS DEFINITION
# ======================================================================

class IRSensor():
    """
    Monitor a single infrared sensor on a line-tracking module.

    Connect the line-tracking module to the "Tracking" port on the
    Adeept HAT.

    The line-tracking module has three infrared emitters & sensors on
    it.  Each sensor is connected to a pin on the GPIO.  A black surface
    will set the pin HIGH; a white surface will set the pin LOW.

    Sensitivity can be adjusted by turning the on-board potentiometer
    with a Phillips screwdriver.

    Parameters
    ----------
    pin:  int
        The GPIO pin (input) that's connected to the infrared sensor.

    Attributes
    ----------
    sensing_black:  bool

    Raises
    ------
    ValueError
        `pin` isn't a valid GPIO BCM pin.

    See Also
    --------
    adeept_linetracker.LineTracker

    Notes
    -----
    This class is separate from the LineTracker class because the
    infrared sensors on the line-tracking module could be used for
    purposes other than tracking lines.
    """

    # Private Attributes
    # ------------------
    # _callbacks:  List[Callable[[bool], None]]
    #     A list of callback functions to call when the sensor changes
    #     state.
    # _state:  int
    #     The current state of the sensor (GPIO.HIGH means black is
    #     detected, GPIO.LOW means that it isn't).

    def __init__(self, pin:  int) -> None:
        """
        Prepare an infrared sensor for use.
        """

        _validate_gpio_pin_number(pin, "pin")

        GPIO.setup(pin, GPIO.IN)

        self._state:      int                               = GPIO.input(pin)
        self._callbacks:  List[Callable[[int, bool], None]] = []

        GPIO.add_event_detect(pin, GPIO.BOTH, callback = self._on_change)

    # ------------------------------------------------------------------

    def add_callback(self, callback:  Callable[[int, bool], None]) -> bool:
        """
        Add a callback function for when the sensor's state changes.

        All callbacks will be called from a separate thread and will be
        called consecutively (not concurrently) in the order in which
        they were added.

        Parameters
        ----------
        callback:  Callable[[int, bool], None]
            The callback function to call.  It must accept the
            following arguments:

            pin:  int
                The sensor's GPIO pin.
            sensing_black:  bool
                True if black is now detected or False if it isn't now
                detected.

        Returns
        -------
        True if black is currently detected or False if it isn't.  This
        could be useful if the caller needs to perform any
        initializations based on the sensor's current state.

        Raises
        ------
        ValueError
            `callback` has already been added.

        See Also
        --------
        remove_callback
        """

        if callback in self._callbacks:
            raise ValueError("\"callback\" has already been added.")

        self._callbacks.append(callback)

        return self._state

    # ------------------------------------------------------------------

    def remove_callback(self, callback:  Callable[[bool], None]) -> None:
        """
        Remove a callback function for when the sensor's state changes.

        Parameters
        ----------
        callback:  Callable[[int, bool], None]
            The callback function to call.  It must accept the
            following arguments:

            pin:  int
                The sensor's GPIO pin.
            sensing_black:  bool
                True if black is now detected or False if it isn't now
                detected.

        Raises
        ------
        ValueError
            `callback` hasn't been added.

        See Also
        --------
        add_callback
        """

        try:
            self._callbacks.remove(callback)
        except ValueError:
            pass

    # ------------------------------------------------------------------

    def _on_change(self, pin:  int) -> None:
        """
        Event handler for when the infrared sensor changes state.

        Parameters
        ----------
        pin:  int
            The GPIO pin (input) whose state has changed.
        """

        self._state = GPIO.input(pin)

        for callback in self._callbacks:
            callback(pin, self._state)

    # ------------------------------------------------------------------

    sensing_black = property(lambda self:  self._state == GPIO.HIGH, None,
                             None, "Is the infrared sensor detecting a black "
                             "surface?")

# ======================================================================
# RGB_LED CLASS DEFINITION
# ======================================================================

class RGB_LED:
    """
    Control an RGB LED (not to be confused with a NeoPixel).

    Connect the RGB LED (MUST be of the anode variety -- the cathode
    variety will not work) to either the "RGB1" or "RGB2" port on the
    Adeept HAT.

    An RGB LED uses three GPIO pins -- one for each of its
    primary-colour emitters.  The brightness of each emitter is
    controlled through pulse-width modulation.

    Parameters
    ----------
    red_pin:    int
    green_pin:  int
    blue_pin:   int
        The GPIO pins (output) for each of the three primary colours.

    Attributes
    ----------
    colour:  int

    Raises
    ------
    ValueError
        At least one of the arguments isn't a valid GPIO BCM pin.
    """

    # Private Attributes
    # ------------------
    # _red:    _Emitter
    # _green:  _Emitter
    # _blue:   _Emitter
    #     Controllers for each of the RGB LED's coloured emitters.

    class _Emitter:
        """
        Control a single emitter in an RGB LED.

        Parameters
        ----------
        pin:  int
            The GPIO pin (output) for the emitter.

        Raises
        ------
        ValueError
            `pin` isn't a valid GPIO BCM pin.
        """

        # Class Private Attributes
        # ------------------------
        # _PWM_FREQUENCY:  int
        #     The frequency (in hertz) that pulse-width modulation is to
        #     operate at.  Its value is high enough to not be noticeable
        #     by the human eye.
        #
        # Private Attributes
        # ------------------
        # _controller:  GPIO.PWM
        #     Pulse-width modulator controller for the emitter.
        #
        # Notes
        # -----
        # The RGB ports on the HAT have a +3.3V pin instead of a ground
        # pin (which is why only anode RGB LED's are compatible) and
        # brightness is controlled by the GPIO pin pushing back
        # against the power pin.  In other words, the GPIO pin acts
        # like a brake pedal instead of an accelerator pedal --
        # increased pressure causes decreased activity.
        #
        # Thus, a high signal on the GPIO pin will darken the emitter
        # and a low signal will brighten it.  This principle also
        # applies to pulse-width modulation.

        _PWM_FREQUENCY:  int = 240

        def __init__(self, pin:  int) -> None:
            """
            Prepare an RGB LED for use.
            """

            _validate_gpio_pin_number(pin, "pin")

            GPIO.setup(pin, GPIO.OUT, initial = GPIO.HIGH)

            self._controller = GPIO.PWM(pin, self._PWM_FREQUENCY)
            self._controller.start(100)

        # --------------------------------------------------------------

        def set_brightness(self, brightness:  int) -> None:
            """
            Change the emitter's brightness.

            Parameters
            ----------
            brightness:  int
                The intensity of the LED's emitter (must be an integer
                from 0x00 to 0xFF)
            """

            assert (brightness >= 0x00) and (brightness <= 0xFF)

            # The brightness is scaled to the range of the pulse-width
            # modulator's duty cycle (0% to 100%).  Also, the duty cycle
            # scales inversely -- not directly -- with brightness.

            self._controller.ChangeDutyCycle(100
                                             - round((brightness * 100) / 0xFF))

    # ------------------------------------------------------------------

    def __init__(self, pin_red:  int, pin_green:  int, pin_blue:  int) -> None:
        """
        Prepare an RGB LED for use.
        """

        self._red    = self._Emitter(pin_red)
        self._green  = self._Emitter(pin_green)
        self._blue   = self._Emitter(pin_blue)
        self._colour = 0x00000000

    # ------------------------------------------------------------------

    def set_colour(self, colour:  int) -> None:
        """
        Set the colour of the RGB LED.

        Parameters
        ----------
        colour:  int
            The new colour for the RGB LED (must be a 24-bit RGB
            integer).

        Raises
        ------
        ValueError
            `colour` is outside the range of 0x00000000 to 0x00FFFFFF.
        """

        if (colour < 0x00000000) or (colour > 0x00FFFFFF):
            raise ValueError("\"colour\" ("
                             + (f"0x{colour:X}" if colour >= 0
                                else f"-0x{-colour:X}")
                             + ") is not a 24-bit RGB integer")

        self._red.set_brightness((colour & 0x00FF0000) >> 16)
        self._green.set_brightness((colour & 0x0000FF00) >> 8)
        self._blue.set_brightness(colour & 0x000000FF)

        self._colour = colour

    # ------------------------------------------------------------------

    def off(self) -> None:
        """
        Turn off the RGB LED.
        """

        # Code re-use at its finest.

        self.set_colour(0x000000)

    # ------------------------------------------------------------------

    colour = property(lambda self:  self._colour, set_colour, None,
                      "The colour of the RGB LED.")

# ======================================================================
# SINGLESIGNAL CLASS DEFINITION
# ======================================================================

class SingleSignal():
    """
    Control a component that responds to a single GPIO signal.

    For all Adeept HAT's (as of this writing), this includes the three
    on-board LED's.

    For the Adeept Motor HAT 1.0, this includes a transistored
    buzzer.

    For the Adeept Robot HAT, this includes 5V DC components.  They're
    usually LED lamps but other components can be controlled as well
    (such as a non-transistored buzzer).

    Connect the component(s) to the "Buzzer", "Port1", "Port2" or
    "Port3" ports (as appropriate) on the Adeept HAT.

    Parameters
    ----------
    signal_pin:  int
        The GPIO pin (output) that's conected to the component.

    Attributes
    ----------
    state:  bool
    frequency:  float
    duty_cyctle:  float

    Raises
    ------
    ValueError
        `signal_pin` isn't a valid GPIO BCM pin.

    Notes
    -----
    The signal can either be on/off or use pulse-width modulation.
    Activating one signal type will de-activate the other type.
    """

    # Private Attributes
    # ------------------
    # _SIGNAL_PIN:  int
    #     The GPIO pin (output) that turns the component(s) on and off.

    def __init__(self, signal_pin:  int) -> None:
        """
        Prepare a switch port for use.
        """

        _validate_gpio_pin_number(signal_pin, "signal_pin")

        self._SIGNAL_PIN:  int                   = signal_pin
        self._controller:  Union[GPIO.PWM, None] = None
        self._state:       bool                  = False
        self._frequency:   float                 = 0.0
        self._duty_cycle:  int                   = 0

        GPIO.setup(self._SIGNAL_PIN, GPIO.OUT, initial = GPIO.LOW)

    # ------------------------------------------------------------------

    def set_state(self, state:  bool) -> None:
        """
        Set a new on/off state for the component(s).

        if pulse-width modulation is active then it's stopped.

        Parameters
        ----------
        state:  bool
            The new state (True means on, False means off).
        """

        self._state = state

        if self._frequency > 0.0:
            self._controller.stop()

            self._frequency = 0.0

        GPIO.output(self._SIGNAL_PIN, GPIO.HIGH if state else GPIO.LOW)

    # ------------------------------------------------------------------

    def set_frequency(self, frequency:  float) -> None:
        """
        Set a new frequency for pulse-width modulation.

        If pulse-width modulation isn't active then a frequency that's
        greater than 0.0 will activate it; if it is active then a
        frequency of 0.0 will stop it.

        Parameters
        ----------
        frequency:  float
            The new frequency (in hertz) for pulse-width modulation
            (cannot be negative).

        Raises
        ------
        ValueError
            `frequency` is negative.
        """

        if frequency < 0.0:
            raise ValueError(f"\"frequency\" ({frequency}) cannot be " \
                             f"negative.")

        assert (self._controller is not None) or (self._frequency == 0.0)

        if frequency != self._frequency:
            if self._state and (self._frequency == 0.0):
                GPIO.output(self._SIGNAL_PIN, GPIO.LOW)

                self._state = False

            if self._controller is not None:
                if frequency == 0.0:
                    self._controller.stop()
                else:
                    self._controller.ChangeFrequency(frequency)

                    if self._frequency == 0.0:
                        self._controller.start(self._duty_cycle)
            elif frequency > 0.0:
                self._controller = GPIO.PWM(self._SIGNAL_PIN, frequency)

                self._controller.start(self._duty_cycle)

            self._frequency = frequency

    # ------------------------------------------------------------------

    def set_duty_cycle(self, duty_cycle:  float) -> None:
        """
        Set a new duty cycle for pulse-width modulation.

        If pulse-width modulation isn't active then the new duty cycle
        will be applied when it becomes active.

        Parameters
        ----------
        duty_cycle : float
            The new duty cycle for pulse-width modulation (must be from
            0.0 to 100.0).

        Raises
        ------
        ValueError
            `duty_cycle` is out of range.
        """

        if (duty_cycle < 0.0) or (duty_cycle > 100.0):
            raise ValueError(f"\"duty_cycle\" ({duty_cycle}) must be from " \
                             f"0.0 to 100.0.")

        if duty_cycle != self._duty_cycle:
            self._duty_cycle = duty_cycle

            if self._frequency > 0.0:
                self._controller.ChangeDutyCycle(self._duty_cycle)

    # ------------------------------------------------------------------

    state = property(lambda self:  self._state, set_state, None,
                     "The on/off signal state (True means on, False means "
                     "off).")

    # ------------------------------------------------------------------

    frequency = property(lambda self:  self._frequency, set_frequency, None,
                         "The pulse-width modulation frequency (0.0 means not "
                         "active).")

    # ------------------------------------------------------------------

    duty_cycle = property(lambda self:  self._duty_cycle, set_duty_cycle, None,
                          "The pulse-width modulation duty cycle.")
