#!/usr/bin/python3
"""
adeept_components_servo.py -- class for controlling Adeept HAT servos.

HOW TO USE
==========

Add the following line to the top of your module:

    from adeept_components_servo import Servo

The "Servo" class will then be availabele for instanciation.

EXPORTS
=======

Servo:  class
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

import threading
import time
from typing import Optional

import Adafruit_PCA9685

# ======================================================================
# PRIVATE CONSTANT
# ======================================================================

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

    PROPERTIES
    ==========

    MIN_PWM:  int
        The minimum PWM that the servo can move to.

    MAX_PWM:  int
        The maximum PWM that the servo can move to.

    ANGLE_RANGE:  float
        The number of degrees that the servo rotates between
        "MIN_PWM" and "MAX_PWM".

    METHODS
    =======

    __init__() -> None
        Prepare the servo for use.

    move_to() -> None
        Move the servo to a new position immediately.

    move_by_velocity() -> None
        Move the servo to a new position at a fixed velocity.

    move_by_acceleration() -> None
        Move the servo to a new position at a fixed acceleration.

    stop_moving() -> None
        Tell a servo to stop if it is moving.

    wait() -> bool
        Wait for a moving servo to be ready for a new position.

    get_pwm() -> int
        Get the servo's current PWM value.

    get_angle() -> float
        Get the servo's current angle.

    power_down() -> None
        Turn off the power to the servo so that it can rotate freely.
    """

    # THE MATHEMATICS OF MOVEMENT
    # ===========================
    #
    # Movement by constant velocity is based on the following well-known
    # kinetics formulae:
    #
    #     d = v * t
    #     v = d / t
    #     t = d / v
    #
    # where "d" is distance, "v" is velocity and "t" is time.
    #
    # Movement by constant acceleration/deceleration is based on the
    # following well-known kinetics formulae:
    #
    #     d = a * t^2 / 2
    #     a = d * 2 / t^2
    #     t = sqrt(d * 2 / a)
    #
    # where "d" is distance, "a" is acceleration and "t" is time.
    # There's an acceleration phase & a deceleration phase, and the
    # servo rotates half the required distance over half of required
    # time in each phase.  With that consideration, the aforementioned
    # formulae need to be re-written as:
    #
    #     d = a * t^2 / 4
    #     a = d * 4 / t^2
    #     t = sqrt(d / a) * 2
    #
    # where "d" is (total) distance, "a" is acceleration and "t" is
    # (total) time (deceleration is the negative of acceleration).
    #
    # Many methods in this class include calculations based on these six
    # formulae, substituting PWM or angle for "d".
    #
    # CLASS PRIVATE PROPERTIES
    # ========================
    #
    # _DEFAULT_FREQUENCY:  int
    #     The Adeept Micro Servo AD002's PWM frequency.
    #
    # _DEFAULT_MIN_PWM:  int
    #     The Adeept Micro Servo AD002's minumum PWM.
    #
    # _DEFAULT_MAX_PWM:  int
    #     The Adeept Micro Servo AD002's maximum PWM.
    #
    # _DEFAULT_ANGLE_RANGE:  float
    #     The number of degrees that the Adeept Micro Servo AD002 can
    #     rotate.
    #
    # _MOVE_INTERVAL:  float
    #     The interval between sending new PWM's to the servo.  This
    #     value was arrived at through experimentation.
    #
    # PRIVATE PROPERTIES
    # ==================
    #
    # _CONTROLLER:  Adafruit_PCA9685.PCA9685
    #     A reference to the PCA9685 controller.  This is the object
    #     that controls the servo motors.
    #
    # _PORT_NUM:  int
    #     The port on the PCA9685 controller that the serve is
    #     connected to.
    #
    # _PWM_OFFSET:  int
    #     The PWM offset to pass to the PCA9685 controller when
    #     setting a new PWM value.
    #
    # _PWM_RANGE:  int
    #     MAX_PWM - MIN_PWM.  This constant is useful for some
    #     calculations.
    #
    # _current_pwm:  int
    #     The PWM value that the servo was last set to.  Be aware that
    #     this value can be changed by other threads -- call ".wait(0)"
    #     to see if the servo is currently being moved.
    #
    # _thread:  threading.Lock, None
    #     The thread that's currently moving the servo.
    #
    # _stop_moving:  bool
    #     Has a request been made to stop a moving servo?
    #
    # PRIVATE METHODS
    # ===============
    #
    # def _validated_pwm() -> int:
    #     Validate either "pwm" or "angle" and return the PWM value.
    #
    # def _move_by_velocity() -> None:
    #     Move this servo at a constant velocity in a thread.
    #
    # def _move_by_acceleration() -> None:
    #     Move this servo at a constant ac/deceleration in a thread.

    # According to their documentation, the Adeept Micro Servo AD002's
    # duty cycle range is 100 to 560.  However, experimentation has
    # shown that 100 to 500 is a more accurate range.  This may explain
    # why the "zero" position in Adeept's code is 300.

    _DEFAULT_FREQUENCY:    int   =  50
    _DEFAULT_MIN_PWM:      int   = 100
    _DEFAULT_MAX_PWM:      int   = 500
    _DEFAULT_ANGLE_RANGE:  float = 180.0
    _MOVE_INTERVAL:        float =   0.01

# ---------------------------------------------------------------------

    def __init__(self, port_num:  int, frequency:  int = _DEFAULT_FREQUENCY,
                 pwm_offset:  int = 0, min_pwm:  int = _DEFAULT_MIN_PWM,
                 max_pwm:  int = _DEFAULT_MAX_PWM,
                 angle_range:  float = _DEFAULT_ANGLE_RANGE,
                 initial_pwm:  int
                 = (_DEFAULT_MIN_PWM + _DEFAULT_MAX_PWM) // 2) -> None:

        """
        Prepare a PCA9685-connected servo for use.

        PARAMETERS
        ==========

        port_num:  int
            The port on the PCA9685 controller that the servo is
            connected to (0 - 15).

        frequency:  int
            The servo's PWM frequency (refer to the servo's
            specifications).

        pwm_offset:  int
            An adjustment for PWM values sent to the servo (typically
            0).

        min_pwm:  int
            The minumum PWM that can be sent to the servo (refer to
            the servo's specifications, but can be higher to limit the
            servo's range of motion).  Must be less than "max_pwm" (see
            below).

        max_pwm:  int
            The maximum PWM that can be sent to the servo (refer to
            the servo's specifications, but can be lower to limit the
            servo's range of motion).  Must be greater than "min_pwm"
            (see above).

        angle_range:  float
            The number of degrees that the servo rotates between
            "min_pwm" and "max_pwm" (typically 180.0).  Must be greater
            than 0.0.

        initial_pwm:  int
            The initial PWM to send to the servo (such as a starting
            position or a home position).  Must be between "min_pwm"
            and "max_pwm" (see above).

        RAISES
        ======

        ValueError
            One or more arguments are invalid.
        """

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

        # Declare prviate properties.

        self._CONTROLLER:  Adafruit_PCA9685.PCA9685 \
                           = Adafruit_PCA9685.PCA9685()

        self._PORT_NUM:     int                      = port_num
        self._PWM_OFFSET:   int                      = pwm_offset
        self._PWM_RANGE:    int                      = max_pwm - min_pwm
        self._current_pwm:  int                      = initial_pwm
        self._thread:       Optional[threading.Lock] = None
        self._stop_moving:  bool                     = False

        # Declare public properties.

        self.MIN_PWM:      int    = min_pwm
        self.MAX_PWM:      int    = max_pwm
        self.ANGLE_RANGE:  float  = angle_range

        # Set up the servo using "_SERVOS_LOCK" to ensure that no other
        # threads are accessing the PCA9685 controller at the same
        # time.

        with _SERVOS_LOCK:
            self._CONTROLLER.set_pwm_freq(frequency)
            self._CONTROLLER.set_pwm(port_num, pwm_offset, initial_pwm)

# ---------------------------------------------------------------------

    def move_to(self, pwm:  Optional[int] = None,
                angle:  Optional[float] = None) -> None:

        """
        Move the servo to a new position immediately.

        This method operates synchronously (unlike the other "move...()"
        methods in this class) and will return after the signal is sent
        to the servo.

        PARAMETERS
        ==========

        pwm:  int, None
            The new PWM value to set the servo to.  If specified, it
            must be between "self.MIN_PWM" and "self.MAX_PWM".

        angle:  float, None
            The new angle to set the servo to.  If specified, it must be
            between 0 and "self.ANGLE_RANGE".

        Either "pwm" or "angle" must be specified, but not both.

        RAISES
        ======

        ValueError
            One or more arguments are invalid.
        """

        # Stop the servo if it's moving.

        self.stop_moving()
        self.wait()

        self._stop_moving = False

        # Validate the method's arguments and determine the new pwm.

        try:
            self._current_pwm = self._validated_pwm(pwm, angle)
        except ValueError:
            # Make it look like this method generated this exception
            # instead of the validator.

            raise

        # Move the servo to the new position immediately using
        # "_servos_lock" to ensure that no other threads are accessing
        # the PCA9685 controller at the same time.

        with _SERVOS_LOCK:
            self._CONTROLLER.set_pwm(self._PORT_NUM, self._PWM_OFFSET,
                                     self._current_pwm)

        return

# ---------------------------------------------------------------------

    def move_by_velocity(self, pwm:  Optional[int] = None,
                         angle:  Optional[float] = None,
                         velocity:  Optional[float] = None,
                         angular_velocity:  Optional[float] = None,
                         duration:  Optional[float] = None,
                         stop_time:  Optional[float] = None) -> None:

        """
        Move the servo to a new position at a fixed velocity.

        The servo will move in a stiff, mechanical-looking manner.

        This method operates asynchronously and returns immediately
        while a separate thread controls the servo.

        PARAMETERS
        ==========

        pwm:  int, None
            The new PWM value to set the servo to.  If specified, it
            must be between "self.MIN_PWM" and "self.MAX_PWM".

        angle:  float, None
            The new angle to set the servo to.  If specified, it
            must be between 0 and "self.ANGLE_RANGE".

        velocity:  float, None
            The speed (in PWM/s) at which the servo is to rotate.  If
            specified, it must be greater than 0.

        angular_velocity:  float, None
            The speed (in degrees/s) at which the servo is to rotate.
            If specified, it must be greater than 0.0.

        duration:  float, None
            The time period over which the server is to rotate.  If
            specified, it cannot be negative.  A value of 0.0 is the
            same as moving the servo immediately (like the "moveTo()"
            method but done asynchronously.

        stop_time:  float, None
            The time at which the servo is to stop moving.

        Either "pwm" or "angle" must be specified, but not both.  Also,
        exactly one of "velocity", "angular_velocity", "duration" or
        "stop_time" must be specified.

        RAISES
        ======

        ValueError
            One or more arguments are invalid.
        """

        # The thread method needs to know the PWM to move the servo to
        # and the time at which to stop moving.  The first argument is
        # easy to determine; the second will need to be calculated if
        # "stop_time" wasn't specified.
        #
        # VARIABLES
        # =========
        #
        # new_pwm:  int
        #     The PWM value (calculated from either "pwm" or "angle") to
        #     be passed to the thread.
        #
        # num_args:  int
        #     A count of how many time-related arguments were passed
        #     (used for validating arguments)
        #
        # target_time:  float
        #     The time that the servo is supposed to stop moving at
        #     (will be passed to the thread).

        new_pwm:   int
        num_args:  int = 0 if velocity is None else 1

        # Validate the method's arguments and determine the new PWM &
        # target time.

        try:
            new_pwm = self._validated_pwm(pwm, angle)
        except ValueError:
            # Make it look like this method generated this exception
            # instead of the validator.

            raise

        num_args += 0 if angular_velocity is None else 1
        num_args += 0 if duration is None else 1
        num_args += 0 if stop_time is None else 1

        if num_args != 1:
            raise ValueError("One (and only one) of \"velocity\", "
                             "\"angular_velocity\", \"duration\" or "
                             "\"stop_time\" must be specified.")

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
                              / self._PWM_RANGE * self.ANGLE_RANGE \
                              / angular_velocity
        elif duration is not None:
            if duration < 0.0:
                raise ValueError(f"\"duration\" ({duration}) can't be "
                                 f"negative.")
            else:
                target_time = time.time() + duration
        else:
            target_time = stop_time

        # Stop the servo if it's moving, then start a new movement.

        self.stop_moving()
        self.wait()

        self._stop_moving = False
        self._thread = threading.Thread(target = self._move_by_velocity,
                                        args = (new_pwm, target_time))

        self._thread.start()

# ---------------------------------------------------------------------

    def move_by_acceleration(self, pwm:  Optional[int] = None,
                             angle:  Optional[float] = None,
                             acceleration:  Optional[float] = None,
                             angular_acceleration:  Optional[float] = None,
                             duration:  Optional[float] = None,
                             stop_time:  Optional[float] = None) -> None:

        """
        Move the servo to a new position at a fixed acceleration.

        The servo will move in a smooth, natural-looking manner.

        This method operates asynchronously and returns immediately
        while a separate thread controls the servo.

        PARAMETERS
        ==========

        pwm:  int, None
            The new PWM value to set the servo to.  If specified, it
            must be between "self.MIN_PWM" and "self.MAX_PWM".

        angle:  float, None
            The new angle to set the servo to.  If specified, it
            must be between 0 and "self.ANGLE_RANGE".

        acceleration:  float, None
            The rate of acceleration (in PWM/s^2) at which the servo is
            to rotate HALFWAY (after that, the servo will decelerate at
            the same rate until it stops).  If specified, it must be
            greater than 0.

        angular_acceleration:  float, None
            The rate of acceleration (in degrees/s^2) at which the servo
            is to rotate HALFWAY (after that, the servo will decelerate
            at the same rate until it stops).  If specified, it must be
            greater than 0.0.

        duration:  float, None
            The time period over which the server is to rotate.  If
            specified, it cannot be negative.  A value of 0.0 is the
            same as moving the servo immediately (like the "moveTo()"
            method but done asynchronously.

        stop_time:  float, None
            The time at which the servo is to stop moving.

        Either "pwm" or "angle" must be specified, but not both.  Also,
        exactly one of "acceleration", "angular_acceleration",
        "duration" and "stop_time" must be specified.

        RAISES
        ======

        ValueError
            One or more arguments are invalid.
        """

        # The thread method needs to know the PWM to move the servo to
        # and the time at which to stop moving.  The first argument is
        # easy to determine; the second will need to be calculated if
        # "stop_time" wasn't specified.
        #
        # VARIABLES
        # =========
        #
        # new_pwm:  int
        #     The PWM value (calculated from either "pwm" or "angle") to
        #     be passed to the thread.
        #
        # num_args:  int
        #     A count of how many time-related arguments were passed
        #     (used for validating arguments)
        #
        # target_time:  float
        #     The time that the servo is supposed to stop moving at
        #     (will be passed to the thread).

        new_pwm:   int
        num_args:  int = 0 if acceleration is None else 1

        # Validate the method's arguments and determine the new PWM &
        # target time.

        try:
            new_pwm = self._validated_pwm(pwm, angle)
        except ValueError:
            # Make it look like this method generated this exception
            # instead of the validator.

            raise

        num_args += 0 if angular_acceleration is None else 1
        num_args += 0 if duration is None else 1
        num_args += 0 if stop_time is None else 1

        if num_args != 1:
            raise ValueError("One (and only one) of \"acceleration\", "
                             "\"angular_acceleration\", \"duration\" or "
                             "\"stop_time\" must be specified.")

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
                              / self._PWM_RANGE * self.ANGLE_RANGE \
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

        self.stop_moving()
        self.wait()

        self._stop_moving = False
        self._thread = threading.Thread(target = self._move_by_acceleration,
                                        args = (new_pwm, target_time))

        self._thread.start()

# ---------------------------------------------------------------------

    def stop_moving(self) -> None:
        """Tell a servo to stop if it is moving."""

        # Setting the ".stop_moving" member to True will tell an active
        # thread to exit at its earliest convenience.

        self._stop_moving = True

# ---------------------------------------------------------------------

    def wait(self, timeout:  Optional[float] = None) -> bool:
        """
        Wait for a moving servo to be ready for a new position.

        This method doesn't necessarily wait for a moving servo to come
        to a complete stop -- it only waits until the servo doesn't
        have any pending positions to be sent to it.  If the servo is
        moving then this method will wait for the last position to be
        sent to it before returning; if it isn't moving then it will
        return immediately.

        PARAMETERS
        ==========

        timeout:  float, None
            How long to wait before returning early.  If specified then
            it must be at least 0.0

        RETURNS
        =======

        True if the servo is ready to be given a new position, and False
        if it isn't.

        RAISES
        ======

        ValueError
            One or more arguments are invalid.
        """

        ended:  bool = False

        if (timeout is not None) and (timeout < 0.0):
            raise ValueError(f"\"timeout\" ({timeout}) cannot be negative.")

        if self._thread is not None:
            self._thread.join(timeout)

            ended = not self._thread.is_alive()

            if ended:
                self._thread = None

        return ended

# ---------------------------------------------------------------------

    def get_pwm(self) -> int:
        """
        Get the servo's current PWM value.

        RETURNS
        =======

        The servo's curret PWM.
        """

        return self._current_pwm

# ---------------------------------------------------------------------

    def get_angle(self) -> float:
        """
        Get the servo's current angle.

        RETURNS
        =======

        The servo's curret angle.
        """

        return (self._current_pwm - self.MIN_PWM) / self._PWM_RANGE \
               * self.ANGLE_RANGE

# ---------------------------------------------------------------------

    def power_down(self) -> None:
        """
        Turn off the power to the servo so that it can rotate freely.
        """

        self.stop_moving()
        self.wait()

        self._stop_moving = False

        with _SERVOS_LOCK:
            self._CONTROLLER.set_pwm(self._PORT_NUM, self._PWM_OFFSET, 0)

# ---------------------------------------------------------------------

    def _validated_pwm(self, pwm:  Optional[int],
                      angle:  Optional[float]) -> int:

        # Validate either "pwm" or "angle" and return the PWM value.
        #
        # Make sure that either "pwm" or "angle" (but not both) are
        # within the servo's limits and return the appropriate PWM
        # value.  This is a convience for methods that handle the
        # servo's movements.
        #
        # PARAMETERS
        # ==========
        #
        # pwm:  int, None
        #     The new PWM value to set the servo to.  If specified, it
        #     must be between "self.MIN_PWM" and "self.MAX_PWM".
        #
        # angle:  float, None
        #     The new angle to set the servo to.  If specified, it
        #     must be between 0 and "self.ANGLE_RANGE".
        #
        # Either "pwm" or "angle" must be specified, but not both.
        #
        # RETURNS
        # =======
        #
        # "pwm", or "angle" converted to PWM.
        #
        # RAISES
        # ======
        #
        # ValueError
        #     One or more arguments are invalid.

        new_pwm:  int = 0

        if (pwm is None) == (angle is None):
            raise ValueError("Either \"pwm\" or \"angle\" (but not "
                             "both) must be specified.")

        if pwm is not None:
            if (pwm < self.MIN_PWM) or (pwm > self.MAX_PWM):
                raise ValueError(f"\"pwm\" ({pwm}) must be between "
                                 f"\".MIN_PWM\" ({self.MIN_PWM}) and "
                                 f"\".MAX_PWM\" ({self.MAX_PWM}).")

            new_pwm = pwm
        else:
            if (angle < 0.0) or (angle > self.ANGLE_RANGE):
                raise ValueError(f"\"angle\" ({angle}) must be "
                                 f"between 0 and \".ANGLE_RANGE\" "
                                 f"({self.ANGLE_RANGE}).")

            new_pwm = round(angle / self.ANGLE_RANGE * self._PWM_RANGE) \
                      + self.MIN_PWM

        return new_pwm

# ---------------------------------------------------------------------

    def _move_by_velocity(self, new_pwm:  int, end_time:  float) -> None:
        # Move this servo at a constant velocity in a thread.
        #
        # This method is intended to be a thread body.
        #
        # PARAMETERS
        # ==========
        #
        # new_pwm:  int
        #     The final PWM value to set the servo to.  It must be
        #     between "self.MIN_PWM" and "self.MAX_PWM".
        #
        # end_time:  float
        #     The time at which to stop moving the servo.  If it's in
        #     the past then movement is instantaneous.
        #
        # VARIABLES
        # =========
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

        num_secs = end_time - time.time()

        if num_secs > self._MOVE_INTERVAL * 4:
            # The formula for constant velocity maps to the local
            # variables as follows:
            #
            #     .current_pwm = start_pwm + velocity * current_secs

            velocity     = (new_pwm - self._current_pwm) / num_secs
            start_pwm    = self._current_pwm
            start_time   = time.time()
            current_secs = self._MOVE_INTERVAL

            # This is the main loop.  During each iteration, the servo's
            # position is set to the PWM according to the aforementioned
            # formula.  The thread then sleeps for "MOVE_INTERVAL"
            # seconds.
            #
            # The loop exits when "end_time" is reached or when
            # "stop_moving" is set.

            while not self._stop_moving and (self._current_pwm != new_pwm):
                if (start_time + current_secs) < end_time:
                    self._current_pwm = start_pwm \
                                        + round(velocity * current_secs)
                else:
                    self._current_pwm = new_pwm

                with _SERVOS_LOCK:
                    self._CONTROLLER.set_pwm(
                        self._PORT_NUM, self._PWM_OFFSET, self._current_pwm)

                time.sleep(max(0, start_time + current_secs - time.time()))

                current_secs += self._MOVE_INTERVAL
        else:
            self._current_pwm = new_pwm

        with _SERVOS_LOCK:
            self._CONTROLLER.set_pwm(
                self._PORT_NUM, self._PWM_OFFSET, new_pwm)

# ---------------------------------------------------------------------

    def _move_by_acceleration(self, new_pwm:  int, end_time:  float) -> None:
        # Move this servo at a constant ac/deceleration in a thread.
        #
        # This method is intended to be a thread body.
        #
        # PARAMETERS
        # ==========
        #
        # new_pwm:  int
        #     The final PWM value to set the servo to.  It must be
        #     between "self.MIN_PWM" and "self.MAX_PWM".
        #
        # end_time:  float
        #     The time at which to stop moving the servo.  If it's in
        #     the past then movement is instantaneous.
        #
        # VARIABLES
        # =========
        #
        # num_secs:  float
        #     The total time needed to complete the movement.
        #
        # acceleration:  float
        #     How fast the servo should rotate during the movement (in
        #     PWM's/second^2)
        #
        # start_pwm:  integer
        #     The movement's initial PWM.
        #
        # halfway_point:  integer
        #     The PWM value that's halfway between "start_pwm" and
        #     "new_pwm".
        #
        # halfway_time:  float
        #     The time at which acceleration changes to deceleration.
        #
        # current_secs:  float:
        #     The current time in the movement.

        num_secs = end_time - time.time()

        if num_secs > self._MOVE_INTERVAL * 4:
            # The formula for constant acceleration maps to the local
            # variables as follows:
            #
            #     .current_pwm = start_pwm
            #                    + acceleration * current_secs ^ 2 / 2
            #
            # Deceleration is the negative of acceleration.

            acceleration  = (new_pwm - self._current_pwm) * 4.0 \
                            / (num_secs ** 2)
            start_pwm     = self._current_pwm
            start_time    = time.time()
            halfway_point = round((start_pwm + new_pwm) / 2)
            halfway_time  = start_time + (num_secs / 2.0)
            current_secs  = self._MOVE_INTERVAL

            # This is the loop for the acceleration stage.  During each
            # iteration, the servo's position is set to the PWM
            # according to the aforementioned formula.  The thread then
            # sleeps for "MOVE_INTERVAL" seconds.
            #
            # The loop exits when "end_time" is reached or when
            # "stop_moving" is set.

            while not self._stop_moving \
                    and (self._current_pwm != halfway_point):
                if (start_time + current_secs) < halfway_time:
                    self._current_pwm = \
                        start_pwm \
                        + round(acceleration * (current_secs ** 2) / 2)
                else:
                    self._current_pwm = halfway_point

                with _SERVOS_LOCK:
                    self._CONTROLLER.set_pwm(
                        self._PORT_NUM, self._PWM_OFFSET, self._current_pwm)

                time.sleep(max(0, start_time + current_secs - time.time()))

                current_secs += self._MOVE_INTERVAL

            # This is the loop for the deceleration stage.  It works
            # like the loop for the acceleration phase but slows the
            # servo down instead of speeding it up.

            while not self._stop_moving and (self._current_pwm != new_pwm):
                if (start_time + current_secs) < end_time:
                    self._current_pwm = new_pwm \
                        - round(acceleration * (num_secs - current_secs) ** 2 \
                        / 2)
                else:
                    self._current_pwm = new_pwm

                with _SERVOS_LOCK:
                    self._CONTROLLER.set_pwm(
                        self._PORT_NUM, self._PWM_OFFSET, self._current_pwm)

                time.sleep(max(0, start_time + current_secs - time.time()))

                current_secs += self._MOVE_INTERVAL

        else:
            self._current_pwm = new_pwm

            with _SERVOS_LOCK:
                self._CONTROLLER.set_pwm(
                    self._PORT_NUM, self._PWM_OFFSET, new_pwm)
