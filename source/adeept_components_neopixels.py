"""
Class for controlling Adeept HAT NeoPixels.

To use, add the following line to the top of your module:

    from adeept_components_neopixelstrip import NeoPixelStrip
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

import atexit
import math
from typing import List, Optional

from rpi_ws281x import Adafruit_NeoPixel

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
        x:  int
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
        See `AdaFruit_NeoPixel Class
        Reference<https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html#afb00024000da81bcb6378c7052f77be3>`_.
        """

        return round((math.sin((angle % 256) * 2.0 * math.pi / 256.0) + 1.0)
                     * 127.5)

    # ------------------------------------------------------------------

    def fill(self, color:  int = 0, first:  int = 0, count:  int = 0) -> None:
        """
        Fill all or part of the NeoPixel strip with a color.

        Changes will not take effect until ".show()" is called.

        This method was added to make the base class more like its C++
        counterpart.

        Parameters
        ----------
        color:  int (optional)
            The new color or colors (24-bit RGB or 32-bit WRGB,
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

        Notes
        -----
        See `AdaFruit_NeoPixel Class
        Reference<https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html#a310844b3e5580056edf52ce3268d8084>`_.
        """

        # Constants
        # ---------
        # NUM_NEO_PIXELS:  int
        #     The number of NeoPixels in the strip.

        NUM_NEO_PIXELS = self.numPixels()

        # Validate the parameters.

        if (color < 0) or (color >  0xFFFFFFFF):
            raise ValueError(f"\"color\" (0x{hex(color)}) out of "
                             f"range (0-0xFFFFFFFF)")

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

    def clear(self) -> None:
        """
        Fill the whole NeoPixel strip with 0 / black / off.

        Notes
        -----
        See `AdaFruit_NeoPixel Class
        Reference<https://adafruit.github.io/Adafruit_NeoPixel/html/class_adafruit___neo_pixel.html#ae259682b202be0acd258d6879f4c7121>`_.
        """

        self.fill(0x000000)

    # ------------------------------------------------------------------

    def _neo_pixel_strip_atexit(self) -> None:
        """
        Darken all NeoPixels when program terminates.

        Register this method with "atexit".
        """

        self.clear()
        self.show()
