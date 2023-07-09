"""
adeept_components.py -- classes for controlling Adeept HAT components.

This module contains classes for controlling the following components:

  * direct-current drive motors
  * ultrasonic rangefinders
  * buzzers
  * RGB LED's
  * NeoPixels
  * line trackers
  * on/off devices (e.g. LED lamps)
  * IIC devices (e.g. OLED displays)
  * acceleration sensors

To use one or more of the classes this module (see the **Class
Listings** section), add the following line to the top of your module::

    from adeept_components import *list_of_classes*

Different HAT's support different sets of components.

Class Listings
--------------
DriveMotor
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

import atexit
import time
from typing import Callable, List, Union, Optional

from RPi import GPIO
from rpi_ws281x import Adafruit_NeoPixel

# ======================================================================
# PRIVATE FUNCTIONS
# ======================================================================

def _validate_gpio_pin_number(pin_number:  int, parameter_name:  str) -> None:
    # Confirm that a pin number is a valid GPIO BCM pin number.
    #
    # Parameters
    # ----------
    # pin_number:  int
    #     The pin number to validate
    # parameter_name:  str
    #     The name of the caller's parameter that `pin_number` came
    #     from.  If an exception is raised then this will be included in
    #     the error message.
    #
    # Raises
    # ------
    # ValueError
    #     `pin_number` isn't a valid GPIO BCM pin number.

    # Adeept HAT's connect to a 40-pin GPIO, which means that (as of
    # this writing) Raspberry Pi models 2, 3 & 4 are supported.  Valid
    # GPIO BCM pin numbers are within the same range for all of these
    # models.  See `Raspberry Pi Pinout <https://pinout.xyz>` for
    # details.
    #
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
                         f"${GPIO_MIN_PIN} to ${GPIO_MIN_PIN}")

# ======================================================================
# DRIVEMOTOR CLASS DEFINITION
# ======================================================================

class DriveMotor:
    """
    Control a DC drive motor that's connected to an L298N controller.

    Connect the drive motor to either the "Motor A" or "Motor B" port on
    the Adeept HAT.

    Parameters
    ----------
    enable_pin:  int
        The GPIO pin (output) that connects to the L298N controller's
        ENABLE connection.
    input_pin_1:  int
    input_pin_2:  int
        The GPIO pins (output) that connect to the L298N controller's
        INPUT connections.  If the motor rotates opposite to the desired
        direction then swap these two arguments.
    scale_factor:  float
        An adjustment for when one drive motor is slightly faster than
        another and needs to be slowed down (for example, to keep a
        vehicle moving in a straight line).  It must be greater than 0.0
        and no greater than 1.0 -- the default value is 1.0.

    Attributes
    ----------
    speed

    Notes
    -----
    An L298N controller has two sets of H-bridge connections.  Each
    H-bridge has an ENABLE connection, two INPUT connections and two
    OUTPUT connections.  If ENABLE is high then power is sent to the
    OUTPUT connections according to the INPUT signals.  The INPUT
    signals have the following effect on a DC motor::

        +-------+-------+----------+
        |INPUT 1|INPUT 2|  Effect  |
        +=======+=======+==========+
        |      Low      |Fast brake|
        +-------+-------+----------+
        |  Low  | High  |  Revese  |
        +-------+-------+----------+
        | High  |  Low  | Forward  |
        +-------+-------+----------+
        |     High      |Fast brake|
        +---------------+----------+

    "Forward" and "reverse" are arbitrary directions.

    If ENABLE is low then the motor will receive no power and it will
    freewheel.  Furthermore, PWM can be used on the ENABLE connection to
    control the speed of the motor.

    On an Adeept HAT, the L298N controller's ENABLE and INPUT
    connections are connected to GPIO pins and the OUTPUT connections
    are connected (along with a voltage boost from VIN) to the "Motor A"
    and "Motor B" ports.

    Raises
    ------
    ValueError
        One or more arguments are invalid.
    """

    # Class Private Properties
    # ------------------------
    # _PWM_FREQUENCY:  int
    #     The PWM frequency to use with the drive motor.
    #
    # Private Properties
    # ------------------
    # _ENABLE_PIN:  int
    #     The GPIO pin (output) that connects to the L298N controller's
    #     ENABLE connection.
    #
    # _INPUT_PIN_1:  int
    # _INPUT_PIN_2:  int
    #     The GPIO pins (output) that connect to the L298N controller's
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
        Prepare an L298N-connected DC drive motor for use.
        """

        _validate_gpio_pin_number(enable_pin, "enable_pin")
        _validate_gpio_pin_number(input_pin_1, "input_pin_1")
        _validate_gpio_pin_number(input_pin_2, "input_pin_2")

        if (scale_factor <= 0.0) or (scale_factor > 1.0):
            raise ValueError(f"\"scale_factor\" ({scale_factor}) must be "
                             f"greater than 0.0 and not more than 1.0")

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

        self._CONTROLLER = GPIO.PWM(self._ENABLE_PIN, self._PWM_FREQUENCY)
        self._speed      = 0

        self._CONTROLLER.start(self._speed)

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

        self._CONTROLLER.ChangeDutyCycle(new_duty_cycle)

    # ------------------------------------------------------------------

    def brake(self) -> None:
        """
        Apply an electromotive brake to the motor.
        """

        GPIO.output(self._INPUT_PIN_1, GPIO.LOW)
        GPIO.output(self._INPUT_PIN_2, GPIO.LOW)
        self._CONTROLLER.ChangeDutyCycle(100)

        self._speed = None

    # ------------------------------------------------------------------

    @property
    def speed(self) -> Union[int, None]:
        """
        Get the speed of the drive motor.

        Returns
        -------
            The motor's current speed (-100 to 100), or `None` if an
            electromotive brake is currently being applied.
        """

        return self._speed

# ======================================================================
# HC-SR04 ULTRSONIC SENSOR CLASS DEFINITION
# ======================================================================

class UltrasonicSensor:
    """
    Control an HC-SR04 ultrasonic distance sensor.

    Connect this component to the "Ultrasonic" port on the Adeept HAT.
    Do NOT connect it to any other port -- doing so will likely damage
    it!

    An HC-SR04 module has a TRIGGER connection and an ECHO connection.
    When TRIGGER is high for at least 10us, the module will emit eight
    40Khz ultrasonic pulses, then set ECHO high.  It will then set ECHO
    low either when it detects an ultrasonic echo or after 38ms has
    elapsed (whichever occurs first).

    This sequence can take up to 60ms.

    If an ultrasonic echo was detected then the distance can be
    calculated by multiplying the total time that ECHO was high by the
    speed of sound, then dividing by two.

    IMPORTANT NOTE ABOUT ACCURACY
    =============================

    It isn't always possible to make accurate measurements due to the
    way that Python and Linux can interrupt a program to deal with other
    processess.  This can be mitigated by taking multiple readings and
    using smoothing algorithms.

    METHODS
    =======

    __init__() -> None:
        Prepare an HC_SR04-connected ultrasonic distance sensor for use.

    get_distance() -> Union[float, None]:
        Detect the distance to an object.
    """

    # Class Private Properties
    # ------------------------    #
    #     The minium interval between pings.
    #
    # _TRIGGER_DURATION:  float
    # Private Properties
    # ------------------    #
    # _ECHO_TIMEOUT:  float
    #     How long to wait for an echo before giving up.
    #
    # _SPEED_OF_SOUND:  float
    #     The speed of sound in metres per second at 20°C.
    #
    # PRIVATE PROPERTIES
    # ==================
    #
    # _TRIGGER_PIN:  int
    #     The GPIO pin (output) that's connected to TRIGGER.
    #
    # _ECHO_PIN:  int
    #     The GPIO pin (input) that's connected to ECHO.
    #
    # _last_trigger_time:  float
    #     The last time at which the entire sequence started to be
    #     triggered.

    _TRIGGER_INTERVAL:  float =   0.065
    _TRIGGER_DURATION:  float =   0.000011
    _ECHO_TIMEOUT:      float =   0.037
    _SPEED_OF_SOUND:    float = 343.42

    # ------------------------------------------------------------------

    def __init__(self, trigger_pin:  int, echo_pin:  int) -> None:
        """
        Prepare an HC_SR04-connected ultrasonic distance sensor for use.

        PARAMETERS
        ==========

        trigger_pin:  int
            The GPIO pin (output) that connects to the HC-SR04 sensor's
            TRIGGER connection.

        echo_pin:  int
            The GPIO pin (input) that connects to the HC-SR04 sensor's
            ECHO connection.

        All GPIO pins must be from 0 to 27 (BCM numbering).

        RAISES
        ======

        ValueError
            One or more arguments are invalid.
        """

        _validate_gpio_pin_number(trigger_pin, "trigger_pin")
        _validate_gpio_pin_number(echo_pin, "echo_pin")

        self._TRIGGER_PIN = trigger_pin
        self._ECHO_PIN    = echo_pin

        self._last_trigger_time = 0.0

        GPIO.setup(self._TRIGGER_PIN,  GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(self._ECHO_PIN, GPIO.IN)

    # ------------------------------------------------------------------

    def get_distance(self) -> Union[float, None]:
        """
        Detect the distance to an object.

        RETURNS
        =======

        The distance detected by the component in meters/second
        (residents of the United States of America, Liberia and Myanmar
        can convert this to feet/second by multiplying this value by
        3.28084 feet/meter).

        If no distance was detected (i.e. the component timed-out) then
        "None" is returned.
        """

        # First, make sure that enough time has pased since the last
        # time the sensor was triggered.

        time.sleep(max(0.0, self._last_trigger_time + self._TRIGGER_INTERVAL
                       - time.time()))

        self._last_trigger_time = time.time()

        GPIO.output(self._TRIGGER_PIN,GPIO.HIGH)
        time.sleep(self._TRIGGER_DURATION)
        GPIO.output(self._TRIGGER_PIN, GPIO.LOW)

        # The time from when the response pin goes high to when it goes
        # low is the delay betwen transmitting and receiving the
        # ultrasonic signals.

        while GPIO.input(self._ECHO_PIN) == GPIO.LOW:
            pass

        start_time = time.time()

        while GPIO.input(self._ECHO_PIN) == GPIO.HIGH:
            pass

        total_time = time.time() - start_time

        if total_time < self._ECHO_TIMEOUT:
            return total_time * self._SPEED_OF_SOUND / 2.0
        else:
            return None

    # ------------------------------------------------------------------

    def get_distance2(self) -> Union[float, None]:
        """
        Detect the distance to an object.

        RETURNS
        =======

        The distance detected by the component in meters/second
        (residents of the United States of America, Liberia and Myanmar
        can convert this to feet/second by multiplying this value by
        3.28084 feet/meter).

        If no distance was detected (i.e. the component timed-out) then
        "None" is returned.
        """

        # First, make sure that enough time has pased since the last
        # time the sensor was triggered.

        time.sleep(max(0.0, self._last_trigger_time + self._TRIGGER_INTERVAL
                       - time.time()))

        self._last_trigger_time = time.time()

        GPIO.output(self._TRIGGER_PIN,GPIO.HIGH)
        time.sleep(self._TRIGGER_DURATION)
        GPIO.output(self._TRIGGER_PIN, GPIO.LOW)

        # The time from when the response pin goes high to when it goes
        # low is the delay betwen transmitting and receiving the
        # ultrasonic signals.

        if GPIO.wait_for_edge(self._TRIGGER_PIN, GPIO.RISING,
                              self._TRIGGER_INTERVAL * 1000.0) is not None:

            start_time = time.time()

            if GPIO.wait_for_edge(self._TRIGGER_PIN, GPIO.FALLING,
                                  self._ECHO_TIMEOUT * 1000.0) is not None:

                return (time.time() - start_time) * self._SPEED_OF_SOUND / 2.0
            else:
                return None
        else:
            return None

# ======================================================================
# NEOPIXEL STRIP CLASS DEFINITION
# ======================================================================

class NeoPixelStrip(Adafruit_NeoPixel):
    """
    Control a strip of NeoPixels (not to be confused with RGB LED's).

    Connect the first NeoPixel to the "WS2812" port on the Adeept HAT.

    A WS2812 NeoPixel is an... uh... okay, it IS actually an RGB LED --
    but with its own controller.  It has a DI conection for data input
    and a DO connection for data output.  They can be daisy-chained and
    only the first one is connected to the HAT.  Each one's colour can
    be individually specified.

    IMPORTANT NOTE:  Instances of this class need to run with root
    privileges because the base class's supporting libraries use DMA.

    METHODS
    =======

    __init__() -> None:
        Prepare the strip of NeoPixels for use.

    fill() -> None:
        Set the colour(s) for some or all of the NeoPixels.

        This method was added to make the base class more like its C++
        counterpart.

    clear() -> None:
        Turn all NeoPixels off.

        This method was added to make the base class more like its C++
        counterpart.
    """

    # Class Private Properties
    # ------------------------    #
    #     DMA channel to use (DMA is why root privileges are required).
    #
    # _FREQUENCY:  int
    # Private Properties
    # ------------------    #
    #     PWM channnel on the data pin to use.
    #
    # _BRIGHTNESS:  int
    #     Scale factor for brightness.
    #
    # _INVERT:  bool
    #     Invert the signal line.

    _DMA_CHANNEL:  int  = 10
    _FREQUENCY:    int  = 800000
    _PWM_CHANNEL:  int  = 0
    _BRIGHTNESS:   int  = 255
    _INVERT:       bool = False

    # ------------------------------------------------------------------

    def __init__(self, data_pin:  int, num_pixels:  int) -> None:
        """
        Prepare the strip of NeoPixels for use.

        PARAMETERS
        ==========

        data_pin:  int
            The GPIO pin (output) that connects to the first NeoPixel's
            DI connection.

        numPixels:  int
            The number of NeoPixels in the strip (must be greater than
            zero).

        RAISES
        ======

        ValueError
            One or more arguments are invalid.

        RuntimeError
            The base class could not initialize the NeoPixels.
        """

        _validate_gpio_pin_number(data_pin, "trigger_pin")

        if num_pixels <= 0:
            raise ValueError(f"\"num_pixels\" ({num_pixels}) must be "
                             f"greater than 0")

        # The base class does most of the heavy lifting, and the
        # parameters for its constructor are known ahead of time.

        Adafruit_NeoPixel.__init__(self, num_pixels, data_pin, self._FREQUENCY,
                                   self._DMA_CHANNEL, self._INVERT,
                                   self._BRIGHTNESS, self._PWM_CHANNEL)
        self.begin()
        atexit.register(self._neo_pixel_strip_atexit)

    # ------------------------------------------------------------------

    def fill(self, color:  Union[int, List[int]],
             pixels:  Optional[List[int]] = None) -> None:
        """
        Set the colour(s) for some or all of the NeoPixels.

        Changes will not take effect until ".show()" is called.

        PARAMETERS
        ==========

        color:  Union[int, List[int]]
            The new colour or colours (24-bit RGB or 32-bit WRGB,
            depending on the model of NeoPixel) to set "pixels" to.

            If "color" is a single integer then that colour is applied
            to all of the NeoPixels in "pixels".

            If "color" is a list then all of the colors in that list are
            applied to "pixels" in the same order.  If there are fewer
            colours than pixels then the pattern in "color" will repeat.

            Colours can't be negative.

        pixels:  Optional[List[int]] = None
            A list of NeoPixel indices.  If "None" then all NeoPixels
            will be changed.

            All elements of "pixels" must be at least 0 and less than
            the number of NeoPixels in the strip.

        RAISES
        ======

        ValueError
            One or more arguments are invalid.

        """

        # CONSTANTS
        # =========
        #
        # NUM_NEO_PIXELS:  int
        #     The number of NeoPixels in the strip.

        NUM_NEO_PIXELS = self.numPixels()

        if pixels is None:
            pixels2 = range(NUM_NEO_PIXELS)
        else:
            for element in pixels:
                if (element < 0) or (element >= NUM_NEO_PIXELS):
                    raise ValueError(f"Index in \"pixels\" ({element}) out of "
                                     f"range (0-{NUM_NEO_PIXELS})")

            pixels2 = pixels

        if isinstance(color, list):
            for element in color:
                if element < 0:
                    raise ValueError(f"Colour in \"color\" ({element}) can't "
                                     f"be negative")

            # for i in range(len(pixels2)):
            #     self.setPixelColor(pixels2[i], color[i % len(color)])

            for i, neo_pixel in enumerate(pixels2):
                self.setPixelColor(neo_pixel, color[i % len(color)])
        else:
            if color < 0:
                raise ValueError(f"\"color\" ({color}) can't be "
                                    f"negative")

            for i in pixels2:
                self.setPixelColor(i, color)

    # ------------------------------------------------------------------

    def clear(self):
        """
        Turn all NeoPixels off.

        Changes will not take effect until ".show()" is called.
        """

        self.fill(0x000000)

    # ------------------------------------------------------------------

    def _neo_pixel_strip_atexit(self) -> None:
        # Darken all NeoPixels when program terminates.
        #
        # Register this method with "atexit".

        self.clear()
        self.show()

# ======================================================================
# LINETRACKER CLASS DEFINITION
# ======================================================================

class LineTracker():
    """
    Read the outputs of a line-tracking module.

    Connect the line-tracking module to the "Tracking" port on the
    Adeept HAT.

    The line-tracking module has three infrared emitters & sensors on
    it.  Each sensor is connected to a pin on the GPIO.  A white surface
    will set the pin HIGH; a black surface will set the pin LOW.

    Sensitivity can be adjusted by turning the on-board potentiometer
    with a Phillips screwdriver.

    Instances of this class can be set to detect either black lines on a
    white surface (default) or white lines on a black surface.

    Two means of detection are supported:  polling and callback
    functions.
    """

    class _IRSensor():
        """
        Manage an infrared sensor.

        METHODS
        =======

        __init__() -> None:
            Prepare an infred sensor for use.

        _monitor() -> None:
            Event handler for when the infrared sensor changes state.


        PROPERTIES
        ==========

        state:  int
            The state of the IR sensor (GPIO.LOW or GPIO.HIGH).
        """

        def __init__(self, pin:  int,
                     on_change_any:  Callable[[], None]) -> None:
            """
            Prepare an infrared sensor for use.

            PARAMETERS
            ==========

            pin:  int
                The GPIO pin that's connected to the infrared sensor.

            monitor:  Callable[[], None]
                Event handler for when any of the infrared sensors
                changes state.
            """

            _validate_gpio_pin_number(pin, "pin")

            GPIO.setup(pin, GPIO.IN)

            self._PIN:            int                = pin
            self._ON_CHANGE_ANY:  Callable[[], None] = on_change_any
            self._state:          int                = GPIO.input(pin)

            GPIO.add_event_detect(pin, GPIO.BOTH)
            GPIO.add_event_callback(pin, self._on_change)

        @property
        def state(self) -> int:
            """
            The current state of the infrared sensor.

            RETURNS
            =======

            GPIO.LOW or GPIO.HIGH.
            """

            return self._state

        def _on_change(self, pin:  int) -> None:
            """
            Event handler for when the infrared sensor changes state.
            """

            assert pin == self._PIN, \
                   f"pin {pin} is not the same as self._PIN {self._PIN}"

            self._state = GPIO.input(self._PIN)

            self._ON_CHANGE_ANY()


    def __init__(self, pin_left:  int, pin_middle:  int, pin_right:  int,
                 line_is_white:  bool = False) -> None:
        """
        Manage an infrared sensor.

        METHODS
        =======

        __init__()
            Prepare an infred sensor for use.

        _monitor()
            Event handler for when the infrared sensor changes state.


        PROPERTIES
        ==========

        state:  int
            The state of the IR sensor (GPIO.LOW or GPIO.HIGH).
        """


        self._PIN_LEFT:   self._IRSensor = self._IRSensor(pin_left,
                                                          self._on_change)
        self._PIN_MIDDLE: self._IRSensor = self._IRSensor(pin_middle,
                                                          self._on_change)
        self._PIN_RIGHT:  self._IRSensor = self._IRSensor(pin_right,
                                                          self._on_change)

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

        RETURNS
        =======

        True if a line was detected, False if otherwise.
        """

        return self._PIN_LEFT.state == self._detection_state

    # ------------------------------------------------------------------

    @property
    def middle(self) -> bool:
        """
        Get the state of the middle IR sensor.

        RETURNS
        =======

        True if a line was detected, False if otherwise.
        """

        return self._PIN_MIDDLE.state == self._detection_state

    # ------------------------------------------------------------------

    @property
    def right(self) -> bool:
        """
        Get the state of the right IR sensor.

        RETURNS
        =======

        True if a line was detected, False if otherwise.
        """

        return self._PIN_RIGHT.state == self._detection_state

    # ------------------------------------------------------------------

    def add_callback(self,
                     callback:  Callable[[bool, bool, bool], None]) -> None:
        """
        Add a function to be called when an IR sensor's state changes.

        A callback function can only be added once; subsequent attempts
        to add it will be silently ignored.

        PARAMETERS
        ==========

        callback:  Callable[[bool, bool, bool], None]
            The function to be called when an IR sensor's state changes.
            It must be of the following form:

                name(left:  bool, middle:  bool, right:  bool) -> None

            where "left", "middle" and "right" are the detection states
            of the IR sensors.  Be aware that more than one sensor can
            be True at any given time.

            Callback functions should exit as soon as possible since
            state changes can occur rapidly.
        """

        if callback not in self._callbacks:
            self._callbacks.append(callback)

    # ------------------------------------------------------------------

    def remove_callback(self,
                        callback:  Callable[[bool, bool, bool], None]) -> None:
        """
        Remove a prevously-added callback function.

        Attempts to remove a callback function that isn't registered
        will be silently ignored.

        PARAMETERS
        ==========

        callback:  Callable[[bool, bool, bool], None]
            The callback function to be removed.  It must be of the
            following form:

                name(left:  bool, middle:  bool, right:  bool) -> None
        """


        if callback in self._callbacks:
            self._callbacks.remove(callback)

    # ------------------------------------------------------------------

    def _on_change(self) -> None:
        for listener in self._callbacks:
            listener(self.left, self.middle, self.right)

# ======================================================================
# BUZZERACTIVE CLASS DEFINITION
# ======================================================================

class BuzzerActive:
    """
    This class controls an active buzzer.

    An active buzzer has a built-in oscillator and only needs a steady
    DC current to make a sound.  It is simpler to operate than a
    passive buzzer but there is no way to change its frequency.

    Connect the buzzer to the "Buzzer" port on the Adeept HAT.
    """

    def __init__(self, pin:  int, lowIsOff:  bool = False) -> None:
        """
        This constructor sets up an active buzzer.

        NOTE:  Most active buzzers are wired so that a high signal from
        the GPIO silences them and a low signal makes them buzz (please
        don't ask why -- it doesn't make sense to me, either, but it's
        because of the transistors that are used).  If, however, the
        buzzer that's connected to the HAT behaves differently then set
        "lowIsOff" to True.

        ARGUMENTS
        =========

        pin      -- the GPIO pin that controls the buzzer
        lowIsOff -- a low GPIO signal turns the buzzer off instead of
                    on

        RAISES
        ======

        ValueError -- raised if "pin" isn't a valid GPIO BCM pin
        """

        self._pin = pin

        if lowIsOff:
            self._off = GPIO.LOW
            self._on  = GPIO.HIGH
        else:
            self._off = GPIO.HIGH
            self._on  = GPIO.LOW

        GPIO.setup(self._pin, GPIO.OUT, initial = self._off)

    # ------------------------------------------------------------------

    def start(self) -> None:
        """
        This method makes the buzzer buzz.
        """

        GPIO.output(self._pin, self._on)

    # ------------------------------------------------------------------

    def stop(self) -> None:
        """
        This method silences the buzzer.
        """

        GPIO.output(self._pin, self._off)

# ======================================================================
# BUZZERPASSIVE CLASS DEFINITION
# ======================================================================

class BuzzerPassive:
    """
    This class controls a passive buzzer.

    A passive buzzer needs an oscillating current to make a sound.  It
    is slightly more complex to operate than an active buzzer but its
    frequency can be changed.

    Connect the buzzer to the "Buzzer" port on the Adeept HAT.
    """

    _PWMDutyCycle = 50             # makes a nice, even PWM square wave

    def __init__(self, pin:  int) -> None:
        """
        This constructor sets up a passive buzzer.

        ARGUMENTS
        =========

        pin -- the GPIO pin that controls the buzzer's frequency

        RAISES
        ======

        ValueError -- raised if "pin" isn't a valid GPIO BCM pin
        """

        self._pin          = pin
        self._PWMFrequency = 0

        GPIO.setup(self._pin, GPIO.OUT, initial = GPIO.LOW)

        self._CONTROLLER = GPIO.PWM(self._pin, 0)

    # ------------------------------------------------------------------

    def setFrequency(self, frequency:  int) -> None:
        """
        This method sets the frequency that the buzzer is to sound at.

        The buzzer can be silenced by setting its frequency to 0.

        ARGUMENTS
        =========

        frequency -- the buzzer's new frequency (in hertz)

        RAISES
        ======

        ValueError -- raised if "frequency" is negative
        """

        if (type(frequency) != int) or (frequency < 0):
            raise ValueError(f"\"frequency\" ({frequency}) must be a whole " \
                             "number.")

        # If "frequency" is 0 then the buzzer is silenced -- otherwise,
        # the buzzer is set to "frequency".  If the buzzer had
        # previously been silenced then it must be started again.

        if frequency == 0:
            self._CONTROLLER.stop()
        else:
            self._CONTROLLER.ChangeFrequency(frequency)

            if self._PWMFrequency == 0:
                self._CONTROLLER.start(self._PWMDutyCycle)

        # Remember the new frequency.

        self._PWMFrequency = frequency

    # ------------------------------------------------------------------

    def stop(self) -> None:
        """
        This method silences the buzzer.
        """

        self._CONTROLLER.stop()

# ======================================================================
# RGB_LED CLASS DEFINITION
# ======================================================================

class RGB_LED:
# This class is an interface for an RGB LED (not to be confused with a NeoPixel).
#
# An RGB LED uses three GPIO pins -- one for each of its primary color emiters.  The brightness
# of each emiter is controlled through pulse-width modulation.
#
# Connect the RGB LED to either the "RGB1" or "RGB2" port on the Adeept Motor HAT.

    _PWMFrequency = 50                        # the PWM frequency that the RGB LED operates at

    def __init__(self, pinRed, pinGreen, pinBlue):
    # This constructor initializes an RGB LED.
    #
    # "portNumber" is the RGB port on the HAT and must be either 1 or 2.

        # First, the GPIO pins are sorted out.  There are three pins per RGB LED corresponding
        # to the three primary colors.

        self._pinRed   = pinRed
        self._pinGreen = pinGreen
        self._pinBlue  = pinBlue

        self._pins = {"Red":pinRed, "Green":pinGreen, "Blue":pinBlue}

        # Next, the pins are initialized.  Brighness is controlled by pulse-wave modulation so
        # each pin's PWM object is obtained and initialized as well.

        for currentPin in self._pins:
            GPIO.setup(currentPin, GPIO.OUT, initial = GPIO.LOW)

        self._PWMRed   = GPIO.PWM(pinRed,   self._PWMFrequency)
        self._PWMGreen = GPIO.PWM(pinGreen, self._PWMFrequency)
        self._PWMBlue  = GPIO.PWM(pinBlue,  self._PWMFrequency)

        self._PWMRed.start(0)
        self._PWMGreen.start(0)
        self._PWMBlue.start(0)

    # ------------------------------------------------------------------

    def setColor(self, color):
    # This method sets the color of the RGB LED.
    #
    # "color" must be a 24-bit RGB integer.

        # First, the three primary colors are extracted from "color".  Next, each pin's PWM
        # duty cycle is set to its corresponding primary color.
        #
        # Note that the range for the duty cycle is from 0 to 100, so some scaling is
        # necessary.

        redValue   = (color & 0xff0000) >> 16
        greenValue = (color & 0x00ff00) >> 8
        blueValue  = (color & 0x0000ff)

        self._PWMRed.ChangeDutyCycle((redValue * 100) / 255)
        self._PWMGreen.ChangeDutyCycle((greenValue * 100) / 255)
        self._PWMBlue.ChangeDutyCycle((blueValue * 100) / 255)

    # ------------------------------------------------------------------

    def off(self):
    # This method turns off the RGB LED.

        for i in self._pins:
            GPIO.output(i, GPIO.LOW)

# ======================================================================
# PORT CLASS DEFINITION
# ======================================================================

class Port():
# This class is an interface for a switch port.
#
# A port can be switched on and off.  Its most commonly conected
# component is an LED lamp, but it can turn other things on and off as
# well.
#
# Connect the LED lamp to either the "Port1", "Port2" or "Port3" ports on the Adeept Motor HAT.
# Do not ask why those ports are labelled like that, or else you may attract the attention of
# the Thought Police.  You've been warned.

    def __init__(self, controlPin):
    # This constructor sets up an LED lamp for use.
    #
    # "portID" must be an integer from 1 to 3 and determines the port on the HAT to control.

        # The GPIO pin that powers the LED lamp is cross-referenced from the port number.

        self._controlPin = controlPin

        GPIO.setup(self._controlPin, GPIO.OUT, initial = GPIO.LOW)

    # ------------------------------------------------------------------

    def on(self):
    # This method turns the LED lamp on.

        GPIO.output(self._controlPin, GPIO.HIGH)

    # ------------------------------------------------------------------

    def off(self):
    # This method turns the LED lamp off.

        GPIO.output(self._controlPin, GPIO.LOW)
