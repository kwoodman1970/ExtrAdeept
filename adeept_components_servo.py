#!/usr/bin/python3
"""
adeept_components_servo.py -- class for controlling Adeept HAT servos.

To use, add the following line to the top of your module:

    from adeept_components_servo import Servo
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

import threading
import time
from typing import Callable, Optional, Union

import Adafruit_PCA9685

# ======================================================================
# PRIVATE CONSTANT
# ======================================================================

# _SERVOS_LOCK:  threading.Lock()
#     Use this mutex to prevent multiple threads from accessing the
#     PCA9685 controller at the same time.

_SERVOS_LOCK:  threading.Lock = threading.Lock()

# ======================================================================
# SERVO CLASS DEFINITION
# ======================================================================

class Servo:
    """
    Control a PCA9685-connected servo motor.

    Connect the servo motor to one of the 16 tricolor ports on the
    Adeept HAT.

    Three styles of movement are implemented:  instantaneous, constant
    velocity (looks stiff and mechanical), and constant acceleration/
    deceleration (looks smooth and natural).

    NOTE:  if an optional arguement for the constructor isn't provided
    then the default value for the Adeept Micro Servo AD002 will be
    used.  If you're using a different servo then consult its
    documentation for the correct arguments.

    Parameters
    ----------
    port_num:  int
        The port on the PCA9685 controller that the servo is connected
        to (0 to 15).

    Attributes
    ----------
    MIN_PWM:  int
    MAX_PWM:  int
    ANGLE_RANGE:  float
    pwm:  int
    angle:  float

    Other Parameters
    ----------------
    frequency:  int, default `class._DEFAULT_FREQUENCY`
        The servo's PWM frequency (refer to the servo's specifications).
    pwm_offset:  int, default 0
        An adjustment for PWM values sent to the servo.
    min_pwm:  int, default `class._DEFAULT_MIN_PWM`
        The minumum PWM that can be sent to the servo (refer to the
        servo's specifications, but can be higher to limit the servo's
        range of motion).  Must be less than `max_pwm` (see below).
    max_pwm:  int, default `class._DEFAULT_MAX_PWM`
        The maximum PWM that can be sent to the servo (refer to the
        servo's specifications, but can be lower to limit the servo's
        range of motion).  Must be greater than `min_pwm` (see above).
    angle_range:  float, default 180.0
        The number of degrees that the servo rotates between `min_pwm`
        and `max_pwm`.  Must be greater than 0.0.
    initial_pwm:  int
        The initial PWM to send to the servo (such as a starting
        position or a home position).  The default value is
        (`class._DEFAULT_MIN_PWM` + `class._DEFAULT_MAX_PWM`) // 2).
        Must be between `min_pwm` and `max_pwm` (see above).

    Raises
    ------
    ValueError
        One or more arguments are invalid.
    """

    # Class Private Attibutes
    # -----------------------
    # _DEFAULT_FREQUENCY:  int
    #     The Adeept Micro Servo AD002's PWM frequency.
    # _DEFAULT_MIN_PWM:  int
    #     The Adeept Micro Servo AD002's minumum PWM.
    # _DEFAULT_MAX_PWM:  int
    #     The Adeept Micro Servo AD002's maximum PWM.
    # _DEFAULT_ANGLE_RANGE:  float
    #     The number of degrees that the Adeept Micro Servo AD002 can
    #     rotate.
    # _MOVE_INTERVAL:  float
    #     The interval between sending new PWM's to the servo.  This
    #     value was arrived at through experimentation.
    #
    # Private Attributes
    # ------------------
    # _CONTROLLER:  Adafruit_PCA9685.PCA9685
    #     A reference to the PCA9685 controller.  This is the object
    #     that controls the servo motors.
    # _PORT_NUM:  int
    #     The port on the PCA9685 controller that the servo is
    #     connected to.
    # _PWM_OFFSET:  int
    #     The PWM offset to pass to the PCA9685 controller when
    #     setting a new PWM value.
    # _PWM_RANGE:  int
    #     `MAX_PWM` - `MIN_PWM`.  This constant is useful for some
    #     calculations.
    # _current_pwm:  int
    #     The PWM value that the servo was last set to.  Be aware that
    #     this value can be changed by other threads -- call ".wait(0)"
    #     to see if the servo is currently being moved.
    # _thread:  threading.Lock, None
    #     The thread that's currently moving the servo.
    # _stop_moving:  bool
    #     Has a request been made to stop a moving servo?

    # According to their documentation, the Adeept Micro Servo AD002's
    # duty cycle range is 100 to 560.  However, experimentation has
    # shown that 100 to 500 is a more accurate range.  This may explain
    # why the "zero" position in Adeept's code is 300.

    _DEFAULT_FREQUENCY:    int   =  50
    _DEFAULT_MIN_PWM:      int   = 100
    _DEFAULT_MAX_PWM:      int   = 500
    _DEFAULT_ANGLE_RANGE:  float = 180.0
    _MOVE_INTERVAL:        float =   0.01

    # -------------------------------------------------------------------

    def __init__(self, port_num:  int, frequency:  int = _DEFAULT_FREQUENCY,
                 pwm_offset:  int = 0, min_pwm:  int = _DEFAULT_MIN_PWM,
                 max_pwm:  int = _DEFAULT_MAX_PWM,
                 angle_range:  float = _DEFAULT_ANGLE_RANGE,
                 initial_pwm:  int
                 = (_DEFAULT_MIN_PWM + _DEFAULT_MAX_PWM) // 2) -> None:

        """Prepare a PCA9685-connected servo for use."""

        # Validate the method's arguments.

        if (port_num < 0) or (port_num > 15):
            raise ValueError(f"\"port_num\" ({port_num}) must be from 0 to "
                             f"15.")

        if min_pwm >= max_pwm:
            raise ValueError(f"\"min_pwm\" ({min_pwm}) must be less than "
                             f"\"max_pwm\" ({max_pwm}).")

        if float(angle_range) <= 0.0:
            raise ValueError(f"\"angle_range\" ({angle_range}) must be "
                             f"positive.")

        if (initial_pwm < min_pwm) or (initial_pwm > max_pwm):
            raise ValueError(f"\"initial_pwm\" ({initial_pwm}) must be "
                             f"between \"min_pwm\" ({min_pwm}) and "
                             f"\"max_pwm\" ({max_pwm}).")

        # Declare the private properties.

        self._CONTROLLER:  Adafruit_PCA9685.PCA9685 \
                           = Adafruit_PCA9685.PCA9685()

        self._PORT_NUM:     int                           = port_num
        self._PWM_OFFSET:   int                           = pwm_offset
        self._MIN_PWM:      int                           = min_pwm
        self._MAX_PWM:      int                           = max_pwm
        self._ANGLE_RANGE:  float                         = angle_range
        self._PWM_RANGE:    int                           = max_pwm - min_pwm
        self._current_pwm:  int                           = initial_pwm
        self._thread:       Union[threading.Thread, None] = None
        self._stop_moving:  bool                          = False

        # Initialize the servo.

        with _SERVOS_LOCK:
            self._CONTROLLER.set_pwm_freq(frequency)
            self._CONTROLLER.set_pwm(port_num, pwm_offset, initial_pwm)

    # ------------------------------------------------------------------

    def move_to(self, pwm:  Optional[int] = None,
                angle:  Optional[float] = None) -> None:

        """
        Move the servo to a new position immediately.

        This method operates synchronously (unlike the other "move...()"
        methods in this class) and will return after the signal is sent
        to the servo.

        Either `pwm` or `angle` must be provided, but not both.

        Parameters
        ----------
        pwm:  int, optional
            The new PWM value to set the servo to.  If provided, it
            must be between `self.MIN_PWM` and `self.MAX_PWM`.
        angle:  float, optional
            The new angle to set the servo to.  If provided, it must be
            between 0 and `self.ANGLE_RANGE`.

        Raises
        ------
        ValueError
            One or more arguments are invalid.
        """

        # Stop the servo if it's moving.

        self.stop_moving()
        self.wait()

        self._stop_moving = False

        # Validate the method's arguments and determine the new PWM.

        self._current_pwm = self._validated_pwm(pwm, angle)

        # Set the servo to the new position immediately.

        with _SERVOS_LOCK:
            self._CONTROLLER.set_pwm(self._PORT_NUM, self._PWM_OFFSET,
                                     self._current_pwm)

        return

    # ------------------------------------------------------------------

    def move_by(self, timing_function:  Callable[[int, float, float], int],
                stop_time:  float,
                pwm:  Optional[int] = None,
                angle:  Optional[float] = None) -> None:

        new_pwm = self._validated_pwm(pwm, angle)

        self.stop_moving()
        self.wait()

        self._stop_moving = False
        self._thread = threading.Thread(target = self._move_by,
                                        args = (timing_function, new_pwm,
                                                stop_time))

        self._thread.start()
    # ------------------------------------------------------------------

    def move_by_velocity(self, pwm:  Optional[int] = None,
                         angle:  Optional[float] = None,
                         velocity:  Optional[float] = None,
                         angular_velocity:  Optional[float] = None,
                         duration:  Optional[float] = None,
                         stop_time:  Optional[float] = None) -> None:

        """
        Move the servo to a new position at a fixed velocity.

        DEPRECATED -- use move_by(linear_timing, ...) instead.

        The servo will move in a stiff, mechanical-looking manner.

        This method operates asynchronously and returns immediately
        while a separate thread controls the servo.

        Either `pwm` or `angle` must be provided, but not both.  Also,
        exactly one of `velocity`, `angular_velocity`, `duration` or
        `stop_time` must be provided.

        Parameters
        ----------
        pwm:  int, optional
            The new PWM value to set the servo to.  If provided, it
            must be between `self.MIN_PWM` and `self.MAX_PWM`.
        angle:  float, optional
            The new angle to set the servo to.  If provided, it
            must be between 0 and `self.ANGLE_RANGE`.
        velocity:  float, optional
            The speed (in PWM/s) at which the servo is to rotate.  If
            provided, it must be greater than 0.
        angular_velocity:  float, optional
            The speed (in degrees/s) at which the servo is to rotate.
            If provided, it must be greater than 0.0.
        duration:  float, optional
            The time period over which the server is to rotate.  If
            provided, it cannot be negative.  A value of 0.0 is the
            same as moving the servo immediately (like the `moveTo()`
            method but done asynchronously.
        stop_time:  float, optional
            The time at which the servo is to stop moving.

        Raises
        ------
        ValueError
            One or more arguments are invalid.

        Notes
        -----
        Calculations for movement by constant velocity are based on the
        following well-known kinetics formulae:

            d = v * t

            v = d / t

            t = d / v

        where `d` is distance, `v` is velocity and `t` is time.
        """

        # The thread method needs to know the PWM to move the servo to
        # and the time at which to stop moving.  The first argument is
        # easy to determine; the second will need to be calculated if
        # "stop_time" wasn't provided.
        #
        # Variables
        # ---------
        # new_pwm:  int
        #     The PWM value (calculated from either `pwm` or `angle`) to
        #     be passed to the thread.
        # num_time_args:  int
        #     A count of how many time-related arguments were passed
        #     (used for validating arguments)
        # target_time:  float
        #     The time that the servo is supposed to stop moving at
        #     (will be passed to the thread).

        # Validate the method's arguments and determine the new PWM &
        # target time.

        new_pwm:        int = self._validated_pwm(pwm, angle)
        num_time_args:  int = 0 if velocity is None else 1

        num_time_args += 0 if angular_velocity is None else 1
        num_time_args += 0 if duration is None else 1
        num_time_args += 0 if stop_time is None else 1

        if num_time_args != 1:
            raise ValueError("One (and only one) of \"velocity\", "
                             "\"angular_velocity\", \"duration\" or "
                             "\"stop_time\" must be provided.")

        if velocity is not None:
            if velocity <= 0.0:
                raise ValueError(f"\"velocity\" ({velocity}) must be "
                      f"positive.")
            else:
                target_time = time.time() \
                              + abs(new_pwm - self._current_pwm) \
                              / velocity
        elif angular_velocity is not None:
            if angular_velocity <= 0.0:
                raise ValueError(f"\"angular_velocity\" "
                      f"({angular_velocity}) must be positive.")
            else:
                target_time = time.time() \
                              + abs(new_pwm - self._current_pwm) \
                              / self._PWM_RANGE * self._ANGLE_RANGE \
                              / angular_velocity
        elif duration is not None:
            if duration < 0.0:
                raise ValueError(f"\"duration\" ({duration}) can't be "
                                 f"negative.")
            else:
                target_time = time.time() + duration
        else:
            target_time = stop_time

        self.move_by(linear_timing, target_time, pwm = new_pwm)

    # ------------------------------------------------------------------

    def move_by_acceleration(self, pwm:  Optional[int] = None,
                             angle:  Optional[float] = None,
                             acceleration:  Optional[float] = None,
                             angular_acceleration:  Optional[float] = None,
                             duration:  Optional[float] = None,
                             stop_time:  Optional[float] = None) -> None:

        """
        Move the servo to a new position at a fixed acceleration.

        DEPRECATED -- use move_by(ease_in_out_timing, ...) instead.

        The servo will move in a smooth, natural-looking manner.

        This method operates asynchronously and returns immediately
        while a separate thread controls the servo.

        Either `pwm` or `angle` must be provided, but not both.  Also,
        exactly one of `acceleration`, `angular_acceleration`,
        `duration` and `stop_time` must be provided.

        Parameters
        ----------
        pwm:  int, optional
            The new PWM value to set the servo to.  If provided, it
            must be between `self.MIN_PWM` and `self.MAX_PWM`.
        angle:  float, optional
            The new angle to set the servo to.  If provided, it
            must be between 0 and `self.ANGLE_RANGE`.
        acceleration:  float, optional
            The rate of acceleration (in PWM/s^2) at which the servo is
            to rotate HALFWAY (after that, the servo will decelerate at
            the same rate until it stops).  If provided, it must be
            greater than 0.
        angular_acceleration:  float, optional
            The rate of acceleration (in degrees/s^2) at which the servo
            is to rotate HALFWAY (after that, the servo will decelerate
            at the same rate until it stops).  If provided, it must be
            greater than 0.0.
        duration:  float, optional
            The time period over which the server is to rotate.  If
            provided, it cannot be negative.  A value of 0.0 is the
            same as moving the servo immediately (like the `moveTo()`
            method but done asynchronously.
        stop_time:  float, optional
            The time at which the servo is to stop moving.

        Raises
        ------
        ValueError
            One or more arguments are invalid.

        Notes
        -----
        Calulations for movement by constant acceleration/deceleration
        are based on the following well-known kinetics formulae:

            d = a * t^2 / 2

            a = d * 2 / t^2

            t = sqrt(d * 2 / a)

        where `d` is distance, `a` is acceleration and `t` is time.
        However, there's an acceleration phase & a deceleration phase,
        and the servo rotates half the provided/calculated distance over
        half of provided/calculated time during each phase.  With that
        consideration, the aforementioned formulae need to be re-written
        as:

            d = a * t^2 / 4
            a = d * 4 / t^2
            t = sqrt(d / a) * 2

        where `d` is (total) distance, `a` is acceleration and `t` is
        (total) time (deceleration is the negative of acceleration).
        """

        # The thread method needs to know the PWM to move the servo to
        # and the time at which to stop moving.  The first argument is
        # easy to determine; the second will need to be calculated if
        # "stop_time" wasn't provided.
        #
        # Variables
        # ---------
        # new_pwm:  int
        #     The PWM value (calculated from either `pwm` or `angle`) to
        #     be passed to the thread.
        # num_time_args:  int
        #     A count of how many time-related arguments were passed
        #     (used for validating arguments)
        # target_time:  float
        #     The time that the servo is supposed to stop moving at
        #     (will be passed to the thread).

        # Validate the method's arguments and determine the new PWM &
        # target time.

        new_pwm:  int       = self._validated_pwm(pwm, angle)
        num_time_args:  int = 0 if acceleration is None else 1

        num_time_args += 0 if angular_acceleration is None else 1
        num_time_args += 0 if duration is None else 1
        num_time_args += 0 if stop_time is None else 1

        if num_time_args != 1:
            raise ValueError("One (and only one) of \"acceleration\", "
                             "\"angular_acceleration\", \"duration\" or "
                             "\"stop_time\" must be provided.")

        if acceleration is not None:
            if acceleration <= 0.0:
                raise ValueError(f"\"acceleration\" ({acceleration}) "
                                 f"must be positive.")
            else:
                target_time = time.time() \
                              + ((abs(new_pwm - self._current_pwm) \
                              / acceleration) ** 0.5) * 2.0
        elif angular_acceleration is not None:
            if angular_acceleration <= 0.0:
                raise ValueError(f"\"angular_acceleration\" "
                      f"({angular_acceleration}) must be positive.")
            else:
                target_time = time.time() \
                              + ((abs(new_pwm - self._current_pwm) \
                              / self._PWM_RANGE * self._ANGLE_RANGE \
                              / angular_acceleration) ** 0.5) * 2.0
        elif duration is not None:
            if duration < 0.0:
                raise ValueError(f"\"duration\" ({duration}) can't be "
                                 f"negative.")
            else:
                target_time = time.time() + duration
        else:
            target_time = stop_time

        # Stop the servo if it's moving, then start a new movement.

        self.move_by(easy_ease_timing, target_time, pwm = new_pwm)

    # ------------------------------------------------------------------

    def stop_moving(self) -> None:
        """Tell the servo to stop if it is moving."""

        # Setting the "._stop_moving" member to True will tell an active
        # thread to exit at its earliest convenience.

        self._stop_moving = True

    # ------------------------------------------------------------------

    def wait(self, timeout:  Optional[float] = None) -> bool:
        """
        Wait for a moving servo to be ready for a new position.

        This method doesn't necessarily wait for a moving servo to come
        to a complete stop -- it only waits until the servo doesn't
        have any pending positions to be sent to it.  If the servo is
        moving then this method will wait for the last position to be
        sent to it before returning; if it isn't moving then it will
        return immediately.

        Parameters
        ----------
        timeout:  float, optional
            How long to wait before returning early.  If provided then
            it must be at least 0.0

        Returns
        -------
        bool
            True if the servo is ready to be given a new position, and
            False if it isn't.  The latter value could be returned if
            the servo is currently moving and a value for `timeout` was
            provided.

        Raises
        ------
        ValueError
            `timeout`, if provided, is invalid.
        """

        # Variables
        # ---------
        # ended:  bool
        #     If there's a controlling thread in play then has it ended?
        #     If there isn't then this will be False.

        ended:  bool = True

        # Validate the method's argument.

        if (timeout is not None) and (timeout < 0.0):
            raise ValueError(f"\"timeout\" ({timeout}) cannot be negative.")

        # If there's a controlling thread in play then see if it's
        # ended.  If it hasn't then wait until either it ends or
        # `timeout` has elapsed.

        if self._thread is not None:
            self._thread.join(timeout)

            ended = not self._thread.is_alive()

            if ended:
                self._thread      = None
                self._stop_moving = False
        else:
            self._stop_moving = False

        return ended

    # ------------------------------------------------------------------

    def power_down(self) -> None:
        """
        Turn off the power to the servo so that it can rotate freely.
        """

        self.stop_moving()
        self.wait()

        self._stop_moving = False

        # Setting the servo's PWM to 0 will let it rotate freely

        with _SERVOS_LOCK:
            self._CONTROLLER.set_pwm(self._PORT_NUM, self._PWM_OFFSET, 0)

    # ------------------------------------------------------------------

    @property
    def MIN_PWM(self) -> int:
        """
        Get the servo's minimum PWM value.

        Returns
        -------
        int
            The servo's minimum PWM value.
        """

        return self._MIN_PWM

    # ------------------------------------------------------------------

    @property
    def MAX_PWM(self) -> int:
        """
        Get the servo's maximum PWM value.

        Returns
        -------
        int
            The servo's maximum PWM value.
        """

        return self._current_pwm

    # ------------------------------------------------------------------

    @property
    def ANGLE_RANGE(self) -> int:
        """
        Get the servo's angle range (in degrees).

        Returns
        -------
        float
            The servo's angle range (in degrees).
        """

        return self._ANGLE_RANGE

    # ------------------------------------------------------------------

    @property
    def pwm(self) -> int:
        """
        Get the servo's current PWM value.

        Returns
        -------
        int
            The servo's curret PWM value.
        """

        return self._current_pwm

    # ------------------------------------------------------------------

    @property
    def angle(self) -> float:
        """
        Get the servo's current angle (in degrees).

        Returns
        -------
        float
            The servo's curret angle.
        """

        return (self._current_pwm - self._MIN_PWM) / self._PWM_RANGE \
               * self._ANGLE_RANGE

    # ------------------------------------------------------------------

    def _validated_pwm(self, pwm:  Optional[int],
                      angle:  Optional[float]) -> int:

        # Validate either `pwm` or `angle` and return the PWM value.
        #
        # Make sure that either `pwm` or `angle` (but not both) are
        # within the servo's range and return the appropriate PWM
        # value.  This is a convenience for methods that handle the
        # servo's movements.
        #
        # Parameters
        # ----------
        # pwm:  int, optional
        #     The new PWM value to set the servo to.  If provided, it
        #     must be between `self.MIN_PWM` and `self.MAX_PWM`.
        # angle:  float, optional
        #     The new angle to set the servo to.  If provided, it
        #     must be between 0 and `self.ANGLE_RANGE`.
        #
        # Returns
        # -------
        # int
        #     `pwm`, or `angle` converted to PWM.
        #
        # Raises
        # ------
        # ValueError
        #     One or more arguments are invalid.

        # Variables
        # ---------
        # new_pwm:  int
        #     The value to return.

        new_pwm:  int = 0

        # Validate the method's arguments and determine the correct
        # value to return.

        if (pwm is None) == (angle is None):
            raise ValueError("Either \"pwm\" or \"angle\" (but not "
                             "both) must be provided.")

        if pwm is not None:
            if (pwm < self._MIN_PWM) or (pwm > self._MAX_PWM):
                raise ValueError(f"\"pwm\" ({pwm}) must be between "
                                 f"\".MIN_PWM\" ({self._MIN_PWM}) and "
                                 f"\".MAX_PWM\" ({self._MAX_PWM}).")

            new_pwm = pwm
        else:
            if (angle < 0.0) or (angle > self._ANGLE_RANGE):
                raise ValueError(f"\"angle\" ({angle}) must be "
                                 f"between 0 and \".ANGLE_RANGE\" "
                                 f"({self._ANGLE_RANGE}).")

            new_pwm = round(angle / self._ANGLE_RANGE * self._PWM_RANGE) \
                      + self._MIN_PWM

        return new_pwm

    # ------------------------------------------------------------------

    def _move_by(self, timing_function:  Callable[[int, float, float], int],
                 new_pwm:  int, stop_time:  float) -> None:

        # Move the servo according to a timing function.
        #
        # This method is intended to be a thread body.
        #
        # Parameters
        # ----------
        #
        # new_pwm:  int
        #     The final PWM value to set the servo to.  It must be
        #     between "self.MIN_PWM" and "self.MAX_PWM".
        #
        # stop_time:  float
        #     The time at which to stop moving the servo.  If it's in
        #     the past then movement is instantaneous.
        #
        # Variables
        # ---------
        #
        # num_secs:  float
        #     The total time needed to complete the movement.
        #
        # speed:  float
        #     How fast the servo should rotate during the movement (in
        #     PWM's/second)
        #
        # start_pwm:  int
        #     The movement's initial PWM.
        #
        # current_secs:  float:
        #     The current time in the movement.

        duration:  float   = stop_time - time.time()
        start_pwm:  int    = self._current_pwm
        pwm_range:  int    = new_pwm - start_pwm
        start_time:  float = time.time()
        time_index:  float = self._MOVE_INTERVAL

        # This is the main loop.  During each iteration, the servo's
        # position is set to the PWM according to the timing function.
        # The thread then sleeps for "self._MOVE_INTERVAL" seconds.
        #
        # The loop exits when "end_time" is reached or when
        # "stop_moving" is set.

        while not self._stop_moving and (time_index < duration):
            self._current_pwm = timing_function(pwm_range, duration,
                                                        time_index) \
                                + start_pwm
            self._current_pwm = min(max(self._current_pwm, self.MIN_PWM),
                                    self.MAX_PWM)

            with _SERVOS_LOCK:
                self._CONTROLLER.set_pwm(self._PORT_NUM, self._PWM_OFFSET,
                                         self._current_pwm)

            time.sleep(max(0, start_time + time_index - time.time()))

            time_index += self._MOVE_INTERVAL

        self._current_pwm = new_pwm

        with _SERVOS_LOCK:
            self._CONTROLLER.set_pwm(self._PORT_NUM, self._PWM_OFFSET,
                                     self._current_pwm)

# ----------------------------------------------------------------------

# https://animost.com/tutorials/timing-and-spacing-principle/#How_to_Implement_Spacing_Into_an_Animation

def linear_timing(pwm_range:  int, duration:  float, current_time: float) -> int:
    velocity = pwm_range / duration

    return round(velocity * current_time)

# ----------------------------------------------------------------------

def ease_out_timing(pwm_range:  int, duration:  float, current_time: float) -> int:
    # d = a * t^2
    # a = d / t^2

    acceleration  = pwm_range / (duration ** 2)

    return round(acceleration * (current_time ** 2))

# ----------------------------------------------------------------------

def ease_in_timing(pwm_range:  int, duration:  float, current_time: float) -> int:
    return pwm_range - ease_out_timing(pwm_range, duration,
                                       duration - current_time)

# ----------------------------------------------------------------------

def easy_ease_timing(pwm_range:  int, duration:  float, current_time: float) -> int:
    half_pwm_range = pwm_range // 2
    half_duration  = duration / 2.0

    if current_time <= half_duration:
        return ease_out_timing(half_pwm_range, half_duration, current_time)
    else:
        return ease_in_timing(pwm_range - half_pwm_range, half_duration,
                              current_time - half_duration) \
               + half_pwm_range
