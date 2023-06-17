"""
adeept_position_functions.py -- calculate positions based on timing.

A position function calculates a position when given a range, a duration
and a time index.  These functions are typically called at regular
intervals.

To use, add the following line to the top of your module:

    from adeept_position_functions import *list_of_functions*

Notes
-----
See `12 Principles of Animation:  Timing and Spacing principle
<https://animost.com/tutorials/timing-and-spacing-principle/>`_
for an excellent article on how different position functions are used to
create different motions.

Examples
--------
Move the servo on port 0 at a constant speed:

>>> from adeept_components_servo import Servo
>>> from adeept_position_functions.py import linear_position
>>> my_servo = Servo(0)
>>> my_servo.moveBy(linear_position, 1.0, pwm:  200)
>>> my_servo.wait()
>>> my_servo.moveBy(linear_position, 2.5, angle:  150.0)
>>> my_servo.wait()

Move the servo on port 0 with a custom position function (which uses the
rising part of a cosine wave):

>>> import math
>>> from adeept_components_servo import Servo
>>> def cospos(range:  int, duration:  float, time_index: float) -> int:
...     angle = (1.0 + time_index / duration) * pi
...     return round((math.cos(angle) / 2.0 + 0.5) * range)
>>> my_servo = Servo(0)
>>> my_servo.moveBy(cospos, 1.0, angle:  180.0)
>>> my_servo.wait()
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

from typing import Callable

# ======================================================================
# TYPE DECLARATIONS
# ======================================================================

PositionFunction = Callable[[int, float, float], int]
"""
Position function template.

This is the format for all position calback functions used in this
project.

Parameters
----------
range:  int
    A zero-based range (can be negative).
duration:  float
    a zero-based duration (will always be greater than zero).
time_index:  float
    The point in time to calculate the position at (will always be
    within `duration`).

Returns
-------
int
    The resulting calculated position (need not be within `range`).
"""

# ----------------------------------------------------------------------

def linear_position(range:  int, duration:  float, time_index: float) -> int:
    """
    Calculate a position based on constant velocity.

    See `TimingFunction` for information on parameters and return
    values.

    Notes
    -----
    This function uses the following well-known kinetics formulae::

        v = d / t
        d = v * t

    where `d` is distance, `v` is velocity and `t` is time.
    """

    assert duration > 0.0
    assert (time_index >= 0.0) and (time_index <= duration)

    velocity = range / duration

    return round(velocity * time_index)

# ----------------------------------------------------------------------

def ease_out_position(range:  int, duration:  float, time_index: float) -> int:
    """
    Calculate a position based on constant acceleration.

    See `TimingFunction` for information on parameters and return
    values.

    Notes
    -----
    This function uses the following well-known kinetics formulae::

        a = d / t^2
        d = a * t^2

    where `d` is distance, `a` is acceleration and `t` is time.
    """

    assert duration > 0.0
    assert (time_index >= 0.0) and (time_index <= duration)

    acceleration = range / (duration ** 2)

    return round(acceleration * (time_index ** 2))

# ----------------------------------------------------------------------

def ease_in_position(range:  int, duration:  float, time_index: float) -> int:
    """
    Calculate a position based on constant deceleration.

    See `TimingFunction` for information on parameters and return
    values.

    Notes
    -----
    Deceleration is merely the reverse of acceleration.
    """

    assert duration > 0.0
    assert (time_index >= 0.0) and (time_index <= duration)

    return range - ease_out_position(range, duration,
                                       duration - time_index)

# ----------------------------------------------------------------------

def easy_ease_position(range:  int, duration:  float,
                       time_index: float) -> int:
    """
    Calculate a position based on constant acceleration & deceleration.

    See `TimingFunction` for information on parameters and return
    values.

    Notes
    -----
    Constant acceleration occurs over the first half of `duration` &
    `range` and deceleration over the second half.
    """

    assert duration > 0.0
    assert (time_index >= 0.0) and (time_index <= duration)

    half_range     = range // 2
    half_duration  = duration / 2.0

    if time_index <= half_duration:
        return ease_out_movement(half_range, half_duration, time_index)
    else:
        return ease_in_position(range - half_range, half_duration,
                              time_index - half_duration) \
               + half_range
