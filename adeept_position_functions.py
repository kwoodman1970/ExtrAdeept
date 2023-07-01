"""
adeept_position_functions.py -- calculate positions based on timing.

A position function calculates a position when given an interval (a.k.a.
range or distance), a duration and a time index.  These functions are
typically called at regular intervals.

To use one or more of the position functions in this module, add the
following line to the top of your module:

    from adeept_position_functions import *list_of_functions*

If you want to create your own routine that calls a position function
then add the following line to the top of your module in order to use
`PositionFunction` as a type hint:

    from adeept_position_functions import PositionFunction

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

If the function were to be graphed then the origin would be (0.0, 0.0)
and the endpoint would be (`duration`, `interval`).

A position function need not be pure, but (in the majority of cases) it
should return the same result for the same inputs as long as something
is using it.

Parameters
----------
interval:  int
    A zero-based interval (can be negative).
duration:  float
    A zero-based duration (will always be greater than zero).
time_index:  float
    The point in time to calculate the position at (will always be
    within the interval [0.0, `duration`]).

Returns
-------
int
    The resulting calculated position (need not be within the interval
    [0 .. `interval`]).
"""

# ----------------------------------------------------------------------

def linear_position(interval:  int, duration:  float, time_index: float) -> int:
    """
    Calculate a position based on constant velocity.

    See `PositionFunction` for information on parameters and return
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

    velocity = interval / duration

    return round(velocity * time_index)

# ----------------------------------------------------------------------

def ease_out_position(interval:  int, duration:  float,
                      time_index: float) -> int:
    """
    Calculate a position based on constant acceleration.

    See `PositionFunction` for information on parameters and return
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

    acceleration = interval / (duration ** 2)

    return round(acceleration * (time_index ** 2))

# ----------------------------------------------------------------------

def ease_in_position(interval:  int, duration:  float,
                     time_index: float) -> int:
    """
    Calculate a position based on constant deceleration.

    See `PositionFunction` for information on parameters and return
    values.

    Notes
    -----
    Deceleration is merely the reverse of acceleration.
    """

    assert duration > 0.0
    assert (time_index >= 0.0) and (time_index <= duration)

    return interval - ease_out_position(interval, duration,
                                       duration - time_index)

# ----------------------------------------------------------------------

def easy_ease_position(interval:  int, duration:  float,
                       time_index: float) -> int:
    """
    Calculate a position based on constant acceleration & deceleration.

    See `PositionFunction` for information on parameters and return
    values.

    Notes
    -----
    Constant acceleration occurs over the first half of `duration` and
    deceleration over the second half.
    """

    assert duration > 0.0
    assert (time_index >= 0.0) and (time_index <= duration)

    half_interval = interval // 2
    half_duration = duration / 2.0

    if time_index <= half_duration:
        return ease_out_position(half_interval, half_duration, time_index)
    else:
        return ease_in_position(interval - half_interval, half_duration,
                                time_index - half_duration) \
               + half_interval
