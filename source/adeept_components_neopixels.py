"""
Class for controlling Adeept HAT NeoPixels.

To use, add the following line to the top of your module:

    from adeept_components_neopixelstrip import NeoPixelStrip

Notes
-----
`Adafruit_NeoPixel` (the base class for the `NeoPixelStrip` class)
doesn't follow `PEP8 naming conventions
<https://www.python.org/dev/peps/pep-0008/>_` -- instead, it uses the
names that are in the `Adafruit NeoPixel Class Reference
<https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html>`_
(which is reasonable).

Therefore, the members of `NeoPixelStrip` follow the same naming
convention -- except that accursed single-letter parameter names are
changed to more-descriptive names.
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

import atexit
from colorsys import hsv_to_rgb
import math
from typing import List, Optional, Tuple

from rpi_ws281x import Adafruit_NeoPixel, Color

# ======================================================================
# MODULE SUBROUTINES
# ======================================================================

def _build_args(dictionary:  dict, value_name:  str, value:  any) -> None:
    """
    Add a name-value pair to a dictionary if the value isn't None.

    Parameters
    ----------
    args:  dict
        The dictionary of name-value pairs that the name-value pair will
        be added to if the value isn't None.
    value_name:  str
        The name of the name-value pair.
    value:  any
        The value of the name-value pair.
    """

    if value is not None:
        dictionary[value_name] = value

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
    num:  int
        The number of NeoPixels in the strip (must be greater than
        zero).
    pin:  int
        The GPIO pin (output) that connects to the first NeoPixel's DI
        connection.
    strip_type:  int (optional)
        The type of NeoPixels being used.  Use an `appropriate constant
        imported from _rpi_ws281x
        <https://github.com/jgarff/rpi_ws281x/blob/master/ws2811.h#L46>`_
        (such as `SK6812_STRIP_RGBW`, `WS2811_STRIP_GRB` or
        `WS2812_STRIP`).  The default value is `WS2812_STRIP`.

    Attributes
    ----------
    numLEDs
    numBytes
    pin
    brightness

    Other Parameters
    ----------------
    freq_hz:  int (optional)
        The frequency of the display signal (in hertz).
    dma:  int (optional)
        The DMA channel to use.
    invert:  bool (optional)
        Invert the signal line?
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

    # Private Constants
    # -----------------
    # _PIN:  int
    #     The GPIO pin (output) that connects to the first NeoPixel's
    #     DI connection.

    # ------------------------------------------------------------------

    def __init__(self, num:  int, pin:  int, strip_type:  Optional[int] = None,
                 freq_hz:  Optional[int] = None, dma:  Optional[int] = None,
                 invert:  Optional[bool] = None,
                 brightness:  Optional[int] = None,
                 channel:  Optional[int] = None,
                 gamma:  Optional[List[float]] = None) -> None:
        """
        Prepare the strip of NeoPixels for use.
        """

        if num <= 0:
            raise ValueError(f"\"num\" ({num}) must be greater than 0")

        self._PIN:  int = pin

        # Rather than keep track of the default values of the parameters
        # for the base class's constructor, dynamically build the
        # arguments list to pass to that constructor.

        args:  dict = {"self":  self, "num":  num, "pin":  pin}

        _build_args(args, "strip_type", strip_type)
        _build_args(args, "freq_hz", freq_hz)
        _build_args(args, "dma", dma)
        _build_args(args, "invert", invert)
        _build_args(args, "brightness", brightness)
        _build_args(args, "channel", channel)
        _build_args(args, "gamma", gamma)

        # The base class does most of the heavy lifting.

        Adafruit_NeoPixel.__init__(**args)
        atexit.register(self._neo_pixel_strip_atexit)

    # ------------------------------------------------------------------

    @staticmethod
    def sine8(angle:  int) -> int:
        """
        An 8-bit integer sine wave function.

        It's not directly compatible with standard trigonometric units
        like radians or degrees.

        Parameters
        ----------
        angle:  int
            Input angle, 0 to 255; 256 would loop back to zero,
            completing the circle (equivalent to 360 degrees or 2 pi
            radians).  One can therefore use an unsigned 8-bit variable
            and simply add or subtract, allowing it to
            overflow/underflow and it still does the expected contiguous
            thing.

        Returns
        -------
        Sine result, 0 to 255 (subtract 128 to convert to a signed
        integer, but you'll most likely want unsigned as this output is
        often used for pixel brightness in animation effects).

        Notes
        -----
        See the `AdaFruit_NeoPixel Class Reference
        <https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html#afb00024000da81bcb6378c7052f77be3>`_.
        """

        return round((math.sin((angle % 256) * 2.0 * math.pi / 256.0) + 1.0)
                     * 127.5)


    # ------------------------------------------------------------------

    @staticmethod
    def ColorHSV(hue:  int, sat: int = 255, val:  int = 255) -> Color:
        """
        Convert hue, saturation and value to a packed 32-bit RGB color.

        The returned value can be passed to `setPixelColor()` or other
        RGB-compatible functions.

        Parameters
        ----------
        hue:  int
            An unsigned 16-bit value, 0 to 65535, representing one full
            loop of the color wheel, which allows 16-bit hues to "roll
            over" while still doing the expected thing (and allowing
            more precision than the wheel() function that was common to
            prior NeoPixel examples).
        sat:  int
            Saturation, 8-bit value, 0 (min or pure grayscale) to 255
            (max or pure hue). Default of 255 if unspecified.
        val:  int
            Value (brightness), 8-bit value, 0 (min / black / off) to
            255 (max or full brightness).  Default of 255 if
            unspecified.

        Returns
        -------
        Packed 32-bit RGB with the most significant byte set to 0 -- the
        white element of WRGB pixels is NOT utilized.

        Result is linearly but not perceptually correct, so you may want
        to pass the result through the gamma32() function (or your own
        gamma-correction operation) else colors may appear washed out.
        This is not done automatically by this function because coders
        may desire a more refined gamma-correction function than the
        simplified one-size-fits-all operation of gamma32().

        Diffusing the LEDs also really seems to help when using
        low-saturation colors.

        Raises
        ------
        ValueError
            One or more arguments are invalid.

        Notes
        -----
        See the `AdaFruit_NeoPixel Class Reference
        <https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html#a1f16aee5b96e16e62598f826e292b23b>`_.

        This method uses `colorsys.hsv_to_rgb()` to perform the
        conversion instead of translating the `ColorHSV` method in the
        `C++ source file
        <https://github.com/adafruit/Adafruit_NeoPixel/blob/master/Adafruit_NeoPixel.cpp>`_
        to Python.
        """

        # Constants
        # ---------
        # RGB_TYPE
        #     A type hint alias for the result of `hsv_to_rgb()`.
        #
        # Variables
        # ---------
        # primaries:  RGB_TYPE
        #     A tuple of red, green and blue values (each value ranging
        #     from 0.0 to 1.0) converted from hue, saturation and value.

        if (sat < 0) or (sat >  0xFF):
            raise ValueError(f"\"sat\" (0x{hex(sat)}) out of range "
                             f"(0x00-0xFF)")

        if (val < 0) or (val >  0xFF):
            raise ValueError(f"\"v\" (0x{hex(val)}) out of range "
                             f"(0x00-0xFF)")

        RGB_TYPE = Tuple[float, float, float]

        primaries:  RGB_TYPE = hsv_to_rgb((hue % 0x10000) / 0x10000,
                                          sat / 0xFF, val / 0xFF)

        return Color(round(primaries[0] * 0xFF), round(primaries[1] * 0xFF),
                     round(primaries[2] * 0xFF))

    # ------------------------------------------------------------------

    def fill(self, color:  int = 0, first:  int = 0, count:  int = 0) -> None:
        """
        Fill all or part of the NeoPixel strip with a color.

        Parameters
        ----------
        color:  int (optional)
            32-bit color value.  Most significant byte is white (for
            RGBW pixels) or ignored (for RGB pixels), next is red, then
            green, and least significant byte is blue.  If all arguments
            are unspecified, this will be 0 (off).
        first:  int (optional)
            Index of first pixel to fill, starting from 0.  Must be
            in-bounds, no clipping is performed.  0 if unspecified.
        count:  int (optional)
            Number of pixels to fill, as a positive value.  Passing 0 or
            leaving unspecified will fill to end of strip.

        Raises
        ------
        ValueError
            One or more arguments are invalid.

        Notes
        -----
        See the `AdaFruit_NeoPixel Class Reference
        <https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html#a310844b3e5580056edf52ce3268d8084>`_.
        """

        # Validate the parameters.

        if (color < 0) or (color >  0xFFFFFFFF):
            raise ValueError(f"\"color\" (0x{hex(color)}) out of "
                             f"range (0-0xFFFFFFFF)")

        if (first < 0) or (first >  self.numLEDs - 1):
            raise ValueError(f"\"first\" ({first}) out of range (0-"
                             f"{self.numLEDs - 1})")

        if (count < 0) or (count > self.numLEDs - first):
            raise ValueError(f"\"count\" ({count}) out of range (0-"
                             f"{self.numLEDs - first})")

        # Change the requested NeoPixels

        for i in range(first, count if count != 0 else self.numLEDs):
            self.setPixelColor(i, color)

    # ------------------------------------------------------------------

    def clear(self) -> None:
        """
        Fill the whole NeoPixel strip with 0 / black / off.

        Notes
        -----
        See the `AdaFruit_NeoPixel Class Reference
        <https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html#ae259682b202be0acd258d6879f4c7121>`_.
        """

        self.fill(0x000000)

    # ------------------------------------------------------------------

    def rainbow(self, first_hue:  int, reps:  int, saturation: int = 255,
                brightness:  int = 255) -> None:
        """
        Fill NeoPixel strip with one or more cycles of hues.

        Everyone loves the rainbow swirl so much, now it's canon!

        Parameters
        ----------
        first_hue:  int
            Hue of first pixel, 0-65535, representing one full cycle of
            the color wheel.  Each subsequent pixel will be offset to
            complete one or more cycles over the length of the strip.
        reps:  int
            Number of cycles of the color wheel over the length of the
            strip.  Default is 1.  Negative values can be used to
            reverse the hue order.
        saturation:  int (optional)
            Saturation, 0-255 = gray to pure hue, default = 255.
        brightness:  int (optional)
            Brightness/value, 0-255 = off to max, default = 255.  This
            is distinct and in combination with any configured global
            strip brightness.

        Raises
        ------
        ValueError
            One or more arguments are invalid.

        Notes
        -----
        See the `AdaFruit_NeoPixel Class Reference
        <https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html#a914f61b78e4d36a03c91372ceded2d46>`_.

        The `gammify` parameter isn't supported because gamma tables can
        be provided to this class's constructor and the `setGamma()`
        method.
        """

        # Variables
        # ---------
        # hue:  int
        #     Calculated hue for current NeoPixel in iteration.
        # color:  int
        #     Calculated RGB color for current NeoPixel in iteration.

        for i in range(self.numLEDs):
            hue:    int = first_hue + (i * reps * 0x10000) // self.numLEDs
            color:  int = self.ColorHSV(hue, saturation, brightness)

            self.setPixelColor(i, color)

    # ------------------------------------------------------------------

    def _neo_pixel_strip_atexit(self) -> None:
        """
        Darken all NeoPixels when program terminates.

        Register this method with "atexit".
        """

        self.clear()
        self.show()

    # ------------------------------------------------------------------

    numLEDs = property(Adafruit_NeoPixel.numPixels, None, None,
                       "Number of RGB LEDs in strip.")

    # ------------------------------------------------------------------

    numBytes = property(Adafruit_NeoPixel.__len__, None, None,
                        "Size of 'pixels' buffer.")

    # ------------------------------------------------------------------

    pin = property(lambda self:  self._pin, None, None, "Output pin number.")

    # ------------------------------------------------------------------

    brightness = property(Adafruit_NeoPixel.getBrightness,
                          Adafruit_NeoPixel.setBrightness, None,
                          "Strip brightness 0-255 (stored as +1).")
