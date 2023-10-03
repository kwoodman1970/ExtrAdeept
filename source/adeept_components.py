"""
Classes for controlling Adeept HAT components.

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
from threading import Event
import time
from typing import Callable, List, Union, Optional

from RPi import GPIO
from rpi_ws281x import Adafruit_NeoPixel
from _rpi_ws281x.ws import WS2812_STRIP, WS2811_TARGET_FREQ

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
        vehicle moving in a straight line).  It must be greater than 0.0
        and no greater than 1.0 -- the default value is 1.0.

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
        Union[int, None]
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

        def on_edge(pin:  int):
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

        GPIO.add_event_detect(self._ECHO_PIN, GPIO.BOTH, callback = on_edge)

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
# NEOPIXEL STRIP CLASS DEFINITION
# ======================================================================

class NeoPixelStrip(Adafruit_NeoPixel):
    """
    Control a strip of NeoPixels (not to be confused with RGB LED's).

    Connect the first NeoPixel to the "WS2812" port on the Adeept HAT.

    This class extends the Adafruit_NeoPixel class to make it more like
    its C++ counterpart.

    IMPORTANT NOTE:  Instances of this class need to run with superuser
    privileges because the base class's supporting libraries use DMA.

    Parameters
    ----------
    numPixels:  int
        The number of NeoPixels in the strip (must be greater than
        zero).
    data_pin:  int
        The GPIO pin (output) that connects to the first NeoPixel's DI
        connection.
    strip_type:  int (optional)
        The type of NeoPixels being used.  Import an `appropriate
        constant from _rpi_ws281x
        <https://github.com/jgarff/rpi_ws281x/blob/master/ws2811.h#L46>`_
        (such as `WS2811_STRIP_GRB` or `WS2812_STRIP`).  The default
        value is `WS2812_STRIP`.

    Other Parameters
    ----------------
    freq_hz:  int (optional)
        The frequency of the display signal (in hertz).
    dma:  int (optional)
        The DMA channel to use.
    brightness:  int (optional)
        A scale factor for the brightness of each LED in the strip (0 is
        the darkest; 255 is the brightest).
    channel:  int (optional)
        The PWM channel to use.
    gamma:  List[float] (optional)
        A custom gamma correction array based on a gamma correction
        factor (must have 256 elements).

    Raises
    ------
    ValueError
        One or more arguments are invalid.
    RuntimeError
        The base class could not initialize the NeoPixels.

    Notes
    -----
    A NeoPixel is an... uh... okay, it IS actually an RGB LED -- but
    with its own controller.  It has a DI conection for data input and a
    DO connection for data output.  They can be daisy-chained and only
    the first one is connected to the HAT.  Each one's colour can be
    individually specified.

    There is a dearth of documentation for the modules that control
    NeoPixels, but the `Adafruit NeoPixel Class Reference
    <https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html>`_
    (for C++) is a good place to start.
    """

    # Class Private Attributes
    # ------------------------
    # _DMA:  int
    #     The DMA channel to use (DMA is why superuser privileges are
    #     required).
    # _PWM_CHANNEL:  int
    #     The PWM channnel on the data pin to use.
    # _BRIGHTNESS:  int
    #     Scale factor for brightness.
    # _INVERT:  bool
    #     Invert the signal line?.

    _DMA_CHANNEL:  int  = 10
    _PWM_CHANNEL:  int  = 0
    _BRIGHTNESS:   int  = 255
    _INVERT:       bool = False

    # ------------------------------------------------------------------

    def __init__(self, num_pixels:  int, data_pin:  int,
                 strip_type:  int = WS2812_STRIP,
                 freq_hz:  int = WS2811_TARGET_FREQ, dma:  int = _DMA_CHANNEL,
                 brightness:  int = _BRIGHTNESS, channel:  int = _PWM_CHANNEL,
                 gamma:  Optional[List[float]] = None) -> None:
        """
        Prepare the strip of NeoPixels for use.
        """

        _validate_gpio_pin_number(data_pin, "data_pin")

        if num_pixels <= 0:
            raise ValueError(f"\"num_pixels\" ({num_pixels}) must be "
                             f"greater than 0")

        # The base class does most of the heavy lifting.

        Adafruit_NeoPixel.__init__(self, num_pixels, data_pin, freq_hz, dma,
                                   self._INVERT, brightness, channel,
                                   strip_type, gamma)
        atexit.register(self._neo_pixel_strip_atexit)

    # ------------------------------------------------------------------

    def fill(self, color:  int = 0, first:  int = 0, count:  int = 0) -> None:
        """
        Set the colour(s) for some or all of the NeoPixels.

        Changes will not take effect until ".show()" is called.

        This method was added to make the base class more like its C++
        counterpart.

        Parameters
        ----------
        color:  int (optional)
            The new colour or colours (24-bit RGB or 32-bit WRGB,
            depending on the model of NeoPixel) to set "pixels" to.  The
            default value is 0 (black).
        first:  int (optional)
            0-based index of the first NeoPixel to change.  It must be
            from 0 to the number of NeoPixels in the strip minus 1).
            The default value is 0 (the first NeoPixel).
        count:  int (optional)
            The total number of NeoPixels to change.  It must be from 0
            (meaning all NeoPixels from `count` upward) to the number of
            NeoPixels in the strip.  The default is 0.

        Raises
        ------
        ValueError
            One or more arguments are invalid.
        """

        # Constants
        # ---------
        # NUM_NEO_PIXELS:  int
        #     The number of NeoPixels in the strip.

        NUM_NEO_PIXELS = self.numPixels()

        # Validate the parameters.

        if (first < 0) or (first >  NUM_NEO_PIXELS - 1):
            raise ValueError(f"\"first\" ({first}) out of range (0-"
                                f"{NUM_NEO_PIXELS - 1})")

        if (count < 0) or (count > NUM_NEO_PIXELS - first):
            raise ValueError(f"\"count\" ({count}) out of range (0-"
                                f"{NUM_NEO_PIXELS - first})")

        # Change the requested NeoPixels

        for i in range(first, count if count != 0 else NUM_NEO_PIXELS):
            self.setPixelColor(i, color)

    # ------------------------------------------------------------------

    def clear(self):
        """
        Turn all NeoPixels off.

        Changes will not take effect until ".show()" is called.

        This method was added to make the base class more like its C++
        counterpart.

        Raises
        ------
        ValueError
            One or more arguments are invalid.
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
# IRSENSOR CLASS DEFINITION
# ======================================================================

class IRSensor():
    """
    Monitor a single infrared sensor on a line-tracking module.

    Connect the line-tracking module to the "Tracking" port on the
    Adeept HAT.

    The line-tracking module has three infrared emitters & sensors on
    it.  Each sensor is connected to a pin on the GPIO.  A white surface
    will set the pin LOW; a black surface will set the pin HIGH.

    Sensitivity can be adjusted by turning the on-board potentiometer
    with a Phillips screwdriver.

    Parameters
    ----------
    pin:  int
        The GPIO pin (input) that's connected to the infrared
        sensor.

    on_change_any:  Optional[Callable[[], None]]
        Event handler for when any of the infrared sensors changes
        state.

    Attributes
    ----------
    state

    See Also
    --------
    adeept_linetracker.LineTracker

    Notes
    -----
    This class is separate from the LineTracker class because the
    infrared sensors on the line-tracking module could be used for
    purposes other than tracking lines.
    """

    def __init__(self, pin:  int,
                 on_change_any:  Optional[Callable[[], None]]) -> None:
        """
        Prepare an infrared sensor for use.
        """

        _validate_gpio_pin_number(pin, "pin")

        GPIO.setup(pin, GPIO.IN)

        self._PIN:            int                          = pin
        self._ON_CHANGE_ANY:  Optional[Callable[[], None]] = on_change_any
        self._state:          int                          = GPIO.input(pin)

        GPIO.add_event_detect(pin, GPIO.BOTH)
        GPIO.add_event_callback(pin, self._on_change)

    @property
    def state(self) -> int:
        """
        Return the current state of the infrared sensor.

        Returns
        -------
        GPIO.LOW (white surface detected) or GPIO.HIGH (black surface
        detected).
        """

        return self._state

    def _on_change(self, pin:  int) -> None:
        # Event handler for when the infrared sensor changes state.
        #
        # Parameters
        # ----------
        # pin:  int
        #     The GPIO pin (input) whose state has changed (should be
        #     the same as pin passed to the constructor).

        assert pin == self._PIN, \
               f"pin {pin} is not the same as self._PIN {self._PIN}"

        self._state = GPIO.input(self._PIN)

        if self._ON_CHANGE_ANY is not None:
            self._ON_CHANGE_ANY()

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

        # Class Attributes
        # ----------------
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
            # scales inversely with brightness.

            self._controller.ChangeDutyCycle(100
                                             - round((brightness * 100) / 0xFF))

    def __init__(self, pin_red:  int, pin_green:  int, pin_blue:  int) -> None:
        """
        Prepare an RGB LED for use.
        """

        self._red   = self._Emitter(pin_red)
        self._green = self._Emitter(pin_green)
        self._blue  = self._Emitter(pin_blue)

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

    # ------------------------------------------------------------------

    def off(self) -> None:
        """
        Turn off the RGB LED.
        """

        # Code re-use at its finest.

        self.set_colour(0x000000)

# ======================================================================
# PORT CLASS DEFINITION
# ======================================================================

class Port():
    """
    Control a switch port.

    Connect the component to either the "Port1", "Port2" or "Port3"
    ports on the Adeept HAT.  Do not ask why those ports are labelled
    this way or you may attract the attention of the Thought Police.
    You've been warned.

    A port can be switched on and off.  The most commonly connected
    component is an LED lamp, but other components can be connected as
    well.

    Parameters
    ----------
    control_pin:  int
        The GPIO pin (output) that turns the port on and off.

    Attributes
    ----------
    state

    Raises
    ------
    ValueError
        `control_pin` isn't a valid GPIO BCM pin.
    """

    # Private Attributes
    # ------------------
    # _CONTROL_PIN:  int
    #     The GPIO pin (output) that turns the port on and off.
    # _state:  bool
    #     The current state of the port (True means on, False means off)

    def __init__(self, control_pin:  int) -> None:
        """
        Prepare a switch port for use.
        """

        _validate_gpio_pin_number(control_pin, "control_pin")

        self._CONTROL_PIN:  int  = control_pin
        self._state:        bool = False

        GPIO.setup(self._CONTROL_PIN, GPIO.OUT, initial = GPIO.LOW)

    # ------------------------------------------------------------------

    def set_state(self, new_state:  bool) -> None:
        """
        Set a new state for the port.

        Parameters
        ----------
        new_state:  bool
            The port's new state (True means on, False means off).
        """

        GPIO.output(self._CONTROL_PIN, GPIO.HIGH if new_state else GPIO.LOW)

        self._state = new_state

    # ------------------------------------------------------------------

    state = property(lambda self:  self._state, set_state, None,
                     "The state of the port (True means on, False means off)")
