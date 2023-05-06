#!/usr/bin/python3
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

Different HAT's support different sets of components.

HOW TO USE
==========

EXPORTS
=======

DriveMotor:  Class
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

import sys
import time
from typing import Union, Optional

#import RPi.GPIO as GPIO
from RPi import GPIO
from rpi_ws281x import Adafruit_NeoPixel

# ======================================================================
# PRIVATE FUNCTIONS
# ======================================================================

def _validate_gpio_pin_number(pin_number:  int, var_name:  str) -> None:
    GPIO_MIN_PIN = 0
    GPIO_MAX_PIN = 27

    if (pin_number < GPIO_MIN_PIN) or (pin_number > GPIO_MAX_PIN):
        raise ValueError(f"\"${var_name}\" ({pin_number}) must be from "
                            f"${GPIO_MIN_PIN} to ${GPIO_MIN_PIN}")

# ======================================================================
# DRIVEMOTOR CLASS DEFINITION
# ======================================================================

class DriveMotor:
    """
    Control a drive motor that's connected to an L298N controller.

    Connect the drive motor to either the "Motor A" or "Motor B" port on
    the Adeept HAT.

    An L298N controller has two sets of H-bridge connections.  Each
    H-bridge has an ENABLE connection, two INPUT connections and two
    OUTPUT connections.  If ENABLE is high then power is sent to the
    OUTPUT connections according to the INPUT signals.  With a DC motor
    connected, the INPUT signals have the following effect:

               |INPUT 2|INPUT 2|
               |is low |is high|
        -------+-------+-------+
        INPUT 1|Fast   |Revese |  "Forward" and "Reverse" are arbitrary
        is low |brake  |       |  directions here
        -------+-------+-------|
        INPUT 1|Forward|Fast   |
        is high|       |brake  |
        -------+-------+-------+

    If ENABLE is low then the motor will receive no power and it will
    free-wheel.

    PWM can be used on the ENABLE connection to control the speed of the
    motor.

    On an Adeept HAT, the ENABLE and INPUT connections are connected to
    GPIO pins and the OUTPUT connections are connected to the "Motor A"
    and "Motor B" ports.

    METHODS
    =======

    __init__() -> None:
        Prepare a drive motor for use.

    speed() -> Union[int, None]:
        Get the speed of the drive motor.

    set_speed() -> None:
        Set the speed of the drive motor.

    stop() -> None:
        Turn off the power to the drive motor and let it freewheel.
    """

    # CLASS PRIVATE PROPERTIES
    # =========================
    #
    # _PWM_FREQUENCY:  int
    #     The PWM frequency to use with the drive motor.
    #
    # PRIVATE PROPERTIES
    # ==================
    #
    # _ENABLE_PIN:  int
    #     The GPIO pin that connects to the L298N controller's ENABLE
    #     connection.
    #
    # _INPUT_PIN_1:  int
    # _INPUT_PIN_2:  int
    #     The GPIO pins that connect to the L298N controller's INPUT
    #     connections.  If the motor rotates opposite to the desired
    #     direction then swap these two values.
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
    #     means that an electromotive brake has been applied to the
    #     motor.  A value of "None" means that the motor is
    #     freewheeling.

    #_PWM_FREQUENCY:  int = 1000
    _PWM_FREQUENCY:  int = 2000

    def __init__(self, enable_pin:  int, input_pin_1:  int, input_pin_2:  int,
                 scale_factor:  float = 1.0) -> None:

        """
        Prepare a drive motor for use.

        PARAMETERS
        ==========

        enable_pin:  int
            The GPIO pin that connects to the L298N controller's ENABLE
            connection.

        input_pin_1:  int
        input_pin_2:  int
            The GPIO pins that connect to the L298N controller's
            INPUT connections.  If the motor rotates opposite to the
            desired direction then swap these two arguments.

        All GPIO pins must be from 0 to 27 (BCM numbering).

        scale_factor:  float
            An adjustment for when one drive motor is slightly faster
            than another and needs to be slowed down (for example, to
            keep a vehicle moving in a straight line).  It must be
            greater 0.0 and no greater than 1.0.

        RAISES
        ======

        ValueError
            One or more arguments are invalid.
        """

        _validate_gpio_pin_number(enable_pin, "enable_pin")
        _validate_gpio_pin_number(input_pin_1, "input_pin_1")
        _validate_gpio_pin_number(input_pin_2, "input_pin_2")

        if (scale_factor <= 0.0) or (scale_factor > 1.0):
            raise ValueError(f"\"scale_factor\" ({scale_factor}) must be "
                             f"greater than 0.0 and not more than 1.0")

        # Declare private properties.

        self._ENABLE_PIN:  int     = enable_pin
        self._INPUT_PIN_1:  int    = input_pin_1
        self._INPUT_PIN_2:  int    = input_pin_2
        self._SCALE_FACTOR:  float = scale_factor

        GPIO.setup(self._ENABLE_PIN,  GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(self._INPUT_PIN_1, GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(self._INPUT_PIN_2, GPIO.OUT, initial = GPIO.LOW)

        try:
            self._CONTROLLER = GPIO.PWM(self._ENABLE_PIN, self._PWM_FREQUENCY)
        except:
            pass

        self._CONTROLLER.start(0)

        self._speed = None

# ---------------------------------------------------------------------

    def speed(self) -> Union[int, None]:
        """
        Get the speed of the drive motor.
        """

        return self._speed
# ---------------------------------------------------------------------

    def set_speed(self, speed:  int) -> None:
        """
        Set the speed of the drive motor.

        PARAMETERS
        ==========

        speed:  int
            The new speed for the drive motor (must be from -100 to
            100).  Negative values will cause the motor to rotate in
            reverse.  A value of 0 will apply an electromotive brake to
            the motor.

        RAISES
        ======

        ValueError
            "speed" is given an invalid value.
        """

        if (speed < -100) or (speed > 100):
            raise ValueError(f"\"speed\" ({speed}) must be from -100 to 100.")

        if speed == 0:
            GPIO.output(self._INPUT_PIN_1, GPIO.HIGH)
            GPIO.output(self._INPUT_PIN_2, GPIO.HIGH)

            new_duty_cycle = 100
        else:
            # Duty cycle must always be a positive integer -- therefore,
            # direction of rotation is determined by which INPUT pin is
            # HIGH and which one is LOW.

            if speed > 0:
                GPIO.output(self._INPUT_PIN_1, GPIO.HIGH)
                GPIO.output(self._INPUT_PIN_2, GPIO.LOW)
            else:
                GPIO.output(self._INPUT_PIN_1, GPIO.LOW)
                GPIO.output(self._INPUT_PIN_2, GPIO.HIGH)

            new_duty_cycle = round(abs(speed * self._SCALE_FACTOR))

        self._CONTROLLER.ChangeDutyCycle(new_duty_cycle)

        self._speed = speed

# ---------------------------------------------------------------------

    def stop(self) -> None:
        """
        Turn off the power to the drive motor and let it freewheel.
        """

        GPIO.output(self._INPUT_PIN_1, GPIO.LOW)
        GPIO.output(self._INPUT_PIN_2, GPIO.LOW)
        GPIO.output(self._ENABLE_PIN,  GPIO.LOW)

        self._speed = None

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
    low either when it detects an ultrasonic echo or after 38ms.
    (whichever occurs first).

    This sequence can take up to 60ms.

    If an ultrasonic echo was detected then the distance can be
    calculated by multiplying the total time that ECHO was high by the
    speed of sound, then dividing by two.
    """

    _TRIGGER_DURATION =   0.000011  # how long to trigger the trigger pin for to start a sonic ping
    _SPEED_OF_SOUND   = 343.42      # in metres per second at 20°C
    _ECHO_TIMEOUT     =   0.037     # how long to wait for an echo before giving up
    _TRIGGER_INTERVAL =   0.065     # interval between pings

# ---------------------------------------------------------------------

    def __init__(self, trigger_pin:  int, echo_pin:  int) -> None:
    # This constructor sets up the HC_SR04 device for use.

        self._TRIGGER_PIN = trigger_pin
        self._ECHO_PIN    = echo_pin

        self._last_trigger_time = 0.0

        GPIO.setup(self._TRIGGER_PIN,  GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(self._ECHO_PIN, GPIO.IN)

# ---------------------------------------------------------------------

    def ping(self, num_attempts:  int = 1) -> float:
    # This method returns a distance in metres, or sys.maxsize if the distance is infinte.
    #
    # "numAttempts" is an integer indicating the number of pings to emit without hearing an
    # echo before giving up.  If the device is having trouble receiving an echo then set
    # "numAttempts" to a value greater than one.

        # This is the main loop.  Dring each iteration, it will emit an ultrasonic ping up to
        # "numAttermpts" times.  If no echo is received by the end of the last iteration then
        # infinity is assumed.

        i        = 0
        detected = False

        while (i < num_attempts) and not detected:
            delay    = self._ping()
            detected = (delay < self._ECHO_TIMEOUT)

            if not detected:
                time.sleep(self._TRIGGER_INTERVAL)

                i += 1

        return delay * self._SPEED_OF_SOUND / 2.0 if detected \
            else sys.float_info.max

# ---------------------------------------------------------------------

    def _ping(self) -> float:

        time.sleep(min(0.0,
                       time.time() - self._last_trigger_time
                       + self._TRIGGER_INTERVAL))

        GPIO.output(self._TRIGGER_PIN,GPIO.HIGH)
        time.sleep(self._TRIGGER_DURATION)
        GPIO.output(self._TRIGGER_PIN, GPIO.LOW)

        # The time from when the response pin goes high to when it goes low is the delay
        # betwen transmitting and receiving the ping.

        while not GPIO.input(self._ECHO_PIN):
            pass

        startTime = time.time()

        while GPIO.input(self._ECHO_PIN):
            pass

        endTime = time.time()

        return endTime - startTime

# ======================================================================
# NEOPIXEL STRIP CLASS DEFINITION
# ======================================================================

class NeoPixelStrip(Adafruit_NeoPixel):
# This class is an interface for a strip of NeoPixels (not to be confused with RGB LED's).
#
# A NeoPixel is an... uh... okay, it is actually an RGB LED -- but with a WS2812 controller.
# They can be daisy-chained so that only the first one need be connected to the HAT, and each
# one can be individually addressed and controlled.
#
# This class adds two methods -- "fill()" and "clear()" -- to the base class to make it more
# like its C++ counterpart.
#
# There is no destructor because the base class has no destructor and cleans up after itself
# using a different method.  If you want all of the NeoPixels to go dark at the end of your
# program's execution then you'll have to call ".clear()" and ".show()" manually.
#
# Connect the first NeoPixel to the "WS2812" port on the Adeept Motor HAT.
#
# IMPORTANT NOTE:  Instances of this class need to run with root privileges due to the nature
# of the base class's supporting libraries.

    _DMAChannel = 10     # DMA channel to use (DMA is why root privileges are required)
    _frequency  = 800000 # data stream frequency in Hz
    _PWMChannel = 0      # PWM channnel on the data pin to use
    _brightness = 255    # scale factor for brightness
    _invert     = False  # invert the signal line?

# ---------------------------------------------------------------------

    def __init__(self, dataPin, numPixels):
    # This constructor initializes the strip of NeoPixels for use.
    #
    # "numPixels" is the number of NeoPixels in the strip.

        # The base class does most of the heavy lifting, and the parameters for its constructor
        # are known ahead of time.

        Adafruit_NeoPixel.__init__(self, numPixels, dataPin, self._frequency,
            self._DMAChannel, self._invert, self._brightness, self._PWMChannel)
        self.begin()

# ---------------------------------------------------------------------

    def fill(self, color, pixels = None):
    # This method fills part or all of a strip of NeoPixels with the specified color(s).
    #
    # If "color" is a single integer (a 24-bit RGB or 32-bit WRGB integer, depending on the
    # model of NeoPixel) then that color is applied to all of the NeoPixels in "pixels".
    #
    # If "color" is a list then all of the colors in that list are applied to "pixels" in
    # order.  If there are fewer colors than pixels then the pattern in "color" will repeat.
    #
    # "pixels" is a list of NeoPixel indices.  If no list is specified then all NeoPixels will
    # be changed.
    #
    # Changes will not take effect until "self.show()" is called.

        if pixels is None:
            pixels2 = range(self.numPixels())
        else:
            pixels2 = pixels

        if isinstance(color, list):
            for i in range(len(pixels2)):
                self.setPixelColor(pixels2[i], color[i % len(color)])
        else:
            for i in pixels2:
                self.setPixelColor(i, color)

# ---------------------------------------------------------------------

    def clear(self):
    # This method turns off all of the NeoPixels.
    #
    # Changes will not take effect until ".show()" is called.

        self.fill(0x000000)

# ======================================================================
# LINETRACKER CLASS DEFINITION
# ======================================================================

class LineTracker():
# This class is an interface for a tracking module.

    def __init__(self, pinLeft, pinMiddle, pinRight, lineIsWhite = False):

        self._pinLeft           = pinLeft
        self._pinMiddle         = pinMiddle
        self._pinRight          = pinRight

        if lineIsWhite == True:
            self._detectionSignal = GPIO.LOW
        elif lineIsWhite == False:
            self._detectionSignal = GPIO.HIGH
        else:
            raise ValueError("lineIsWhite must be either True or False.")

        GPIO.setup(pinLeft, GPIO.IN)
        GPIO.setup(pinMiddle, GPIO.IN)
        GPIO.setup(pinRight, GPIO.IN)

# ---------------------------------------------------------------------

    def lineIsWhite(self):
        self._detectionSignal = GPIO.LOW

# ---------------------------------------------------------------------

    def lineIsBlack(self):
        self._detectionSignal = GPIO.HIGH

# ---------------------------------------------------------------------

    def left(self):
        return GPIO.input(self._pinLeft) == self._detectionSignal

# ---------------------------------------------------------------------

    def middle(self):
        return GPIO.input(self._pinMiddle) == self._detectionSignal

# ---------------------------------------------------------------------

    def right(self):
        return GPIO.input(self._pinRight) == self._detectionSignal

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

# ---------------------------------------------------------------------

    def start(self) -> None:
        """
        This method makes the buzzer buzz.
        """

        GPIO.output(self._pin, self._on)

# ---------------------------------------------------------------------

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

# ---------------------------------------------------------------------

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

# ---------------------------------------------------------------------

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

# ---------------------------------------------------------------------

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

# ---------------------------------------------------------------------

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

# ---------------------------------------------------------------------

    def on(self):
    # This method turns the LED lamp on.

        GPIO.output(self._controlPin, GPIO.HIGH)

# ---------------------------------------------------------------------

    def off(self):
    # This method turns the LED lamp off.

        GPIO.output(self._controlPin, GPIO.LOW)
