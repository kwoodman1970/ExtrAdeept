"""
Class for controlling servos motors.

To use, add the following line to the top of your module:

    from adeept_components_servo import Servo

Class Listings
--------------
Servo
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

import threading
from typing import Optional
import time

from adeept_position_functions import PositionFunction

# ======================================================================
# SERVO CLASS DEFINITION
# ======================================================================

class Servo:
    """
    Control a servo motor.

    This is a base class that defines the interface for servo
    control.  It can be extended by other classes that define servos
    with specific characteristics.

    Parameters
    ----------
    pwm_offset : int
        An adjustment for PWM values sent to the servo.
    min_pwm : int
        The minimum PWM that can be sent to the servo (refer to the
        servo's specifications, but can be higher to limit the servo's
        range of motion). Must be less than `max_pwm`.
    max_pwm : int
        The maximum PWM that can be sent to the servo (refer to the
        servo's specifications, but can be lower to limit the servo's
        range of motion). Must be greater than `min_pwm`.
    angle_range : float
        The number of degrees that the servo rotates between `min_pwm`
        and `max_pwm`. Must be greater than 0.0.
    initial_pwm : int
        The initial PWM to send to the servo (such as a starting
        position or a home position). Must be between `min_pwm` and
        `max_pwm`.

    Attributes
    ----------
    PWM_OFFSET : int
    MIN_PWM : int
    MAX_PWM : int
    ANGLE_RANGE : float
    is_moving : bool
    angle : float
    stop_pwm : int
    stop_angle : float
    stop_time : Optional[float]

    Raises
    ------
    ValueError
        One or more arguments are invalid.
    """

    # Private Attributes
    # ------------------
    # _PWM_OFFSET : int
    #     The PWM offset to use when setting a new PWM value.
    # _MIN_PWM : int
    #     The minimum PWM value for this servo.
    # _MAX_PWM : int
    #     The maximum PWM value for this servo.
    # _ANGLE_RANGE : float
    #     The total angle range of the servo in degrees.
    # _PWM_RANGE : int
    #     `_MAX_PWM` - `_MIN_PWM`. This constant is useful for some
    #     calculations.
    # _position_function : Optional[PositionFunction]
    #     The current position function being used for movement, or None
    #     if the servo is not moving.
    # _movement_start_time : Optional[float]
    #     The time when the current movement started, or None if not moving.
    # _movement_stop_time : Optional[float]
    #     The time when the current movement should stop, or None if not moving.
    # _movement_start_pwm : Optional[int]
    #     The PWM value when the current movement started, or None if not moving.
    # _movement_stop_pwm : Optional[int]
    #     The PWM value when the current movement finishes.

    # -------------------------------------------------------------------

    def __init__(self, pwm_offset: int, min_pwm: int, max_pwm: int,
                 angle_range: float, initial_pwm: int) -> None:
        """Initialize the servo."""

        # Validate the method's arguments.

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

        # Store the parameters.

        self._PWM_OFFSET: int = pwm_offset
        self._MIN_PWM: int = min_pwm
        self._MAX_PWM: int = max_pwm
        self._ANGLE_RANGE: float = float(angle_range)
        self._PWM_RANGE: int = max_pwm - min_pwm

        # Initialize the servo state.

        self._lock = threading.RLock()
        self._position_function: Optional[PositionFunction] = None
        self._movement_start_time: Optional[float] = None
        self._movement_stop_time: float = time.time()
        self._movement_start_pwm: Optional[int] = None
        self._movement_stop_pwm: int = initial_pwm

    # -------------------------------------------------------------------

    def move_to(self, pwm: int,
                position_function: Optional[PositionFunction] = None,
                stop_time: Optional[float] = None) -> None:
        """
        Move the servo to a new position using a position function.

        If no position function is provided, the servo moves instantly.

        Parameters
        ----------
        pwm : int
            The new PWM value to set the servo to (must be between
            `self.MIN_PWM` and `self.MAX_PWM`).
        position_function : PositionFunction, optional
            The position function that governs the servo's movement.
        stop_time : float, optional
            The time at which the servo is to stop moving (must be
            provided if `position_function` is provided).

        Raises
        ------
        ValueError
            One or more arguments are invalid.
        """

        if (pwm < self._MIN_PWM) or (pwm > self._MAX_PWM):
            raise ValueError(f"\"pwm\" ({pwm}) must be between "
                             f"\"self.MIN_PWM\" ({self._MIN_PWM}) and "
                             f"\"self.MAX_PWM\" ({self._MAX_PWM}).")

        if (position_function is None) != (stop_time is None):
            raise ValueError("\"stop_time\" must be provided if and only if "
                             "a position function is provided.")

        with self._lock:
            if position_function is None:
                # Instant movement
                self._movement_start_pwm = None
                self._movement_start_time = None
                self._movement_stop_time = time.time()
            else:
                assert stop_time is not None

                # Gradual movement using position function
                self._movement_start_pwm = self.pwm()
                self._movement_start_time = time.time()
                self._movement_stop_time = stop_time

            self._movement_stop_pwm = pwm
            self._position_function = position_function

    # -------------------------------------------------------------------

    def move_to_angle(self, angle: float,
                     position_function: Optional[PositionFunction] = None,
                     stop_time: Optional[float] = None) -> None:
        """
        Move the servo to a new angle using a position function.

        If no position function is provided, the servo moves instantly.

        Parameters
        ----------
        angle : float
            The new angle to set the servo to (must be between
            `0.0` and `self.ANGLE_RANGE`).
        position_function : PositionFunction, optional
            The position function that governs the servo's movement.
        stop_time : float, optional
            The time at which the servo is to stop moving (must be
            provided if `position_function` is provided).

        Raises
        ------
        ValueError
            One or more arguments are invalid.
        """

        if (angle < 0.0) or (angle > self._ANGLE_RANGE):
            raise ValueError(f"\"angle\" ({angle}) must be between "
                             f"0.0 and \"self.ANGLE_RANGE\" ({self._ANGLE_RANGE}).")

        # Convert angle to PWM
        pwm = int(self._MIN_PWM + (angle / self._ANGLE_RANGE) * self._PWM_RANGE)

        self.move_to(pwm, position_function, stop_time)

    # -------------------------------------------------------------------

    def stop_moving(self) -> None:
        """
        Tell the servo to stop if it is moving.

        This clears any active position function and movement parameters.
        The servo will remain at its current position.
        """

        with self._lock:
            self._movement_stop_pwm = self.pwm()
            self._movement_stop_time = time.time()
            self._movement_start_pwm = None
            self._movement_start_time = None
            self._position_function = None

    # -------------------------------------------------------------------

    def wait(self, timeout: Optional[float] = None) -> bool:
        """
        Wait for a moving servo to complete its movement.

        This method blocks until the servo has finished moving or the
        timeout expires. If the servo is not moving, it returns
        immediately.

        Parameters
        ----------
        timeout : float, optional
            How long to wait before returning early (in seconds).
            If provided, it must be at least 0.0.

        Returns
        -------
        bool
            True if the servo has finished moving (or was not moving),
            False if the timeout expired while the servo was still moving.

        Raises
        ------
        ValueError
            `timeout`, if provided, is invalid.
        """

        if (timeout is not None) and (timeout < 0.0):
            raise ValueError(f"\"timeout\" ({timeout}) must be at least 0.0.")

        if not self.is_moving:
            return True

        start_wait_time = time.time()

        while self.is_moving:
            if timeout is not None:
                elapsed = time.time() - start_wait_time
                if elapsed >= timeout:
                    return False

            time.sleep(0.01)  # Small sleep to avoid busy-waiting

        return True

    # -------------------------------------------------------------------

    def pwm(self, at_time: Optional[float] = None) -> int:
        """
        Get the current PWM value of the servo, based on the current
        position function.

        Parameters
        ----------
        at_time : float, optional
            The time at which the PWM value would be.  If not provided
            then the current PWM value is returned.

        Returns
        -------
        int
            The PWM value of the servo at the requested time.
        """

        with self._lock:
            target_time:  float = time.time() if at_time is None else at_time

            if self._position_function is None:
                return self._movement_stop_pwm
            elif target_time >= self._movement_stop_time:
                return self._movement_stop_pwm
            else:
                assert self._movement_start_time is not None
                assert self._movement_start_pwm is not None

                if target_time <= self._movement_start_time:
                    return self._movement_start_pwm
                else:
                    elapsed_time:  float = target_time - self._movement_start_time
                    pwm_range:  int = self._movement_stop_pwm - self._movement_start_pwm
                    new_pwm:  int = self._movement_start_pwm \
                        + self._position_function(pwm_range,
                                                  self._movement_stop_time - self._movement_start_time,
                                                  elapsed_time)

                    return min(max(new_pwm, self._MIN_PWM), self._MAX_PWM)

    # -------------------------------------------------------------------

    @property
    def PWM_OFFSET(self) -> int:
        """
        Get the servo's PWM offset.

        Returns
        -------
        int
            The servo's PWM offset.
        """

        return self._PWM_OFFSET

    # -------------------------------------------------------------------

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

    # -------------------------------------------------------------------

    @property
    def MAX_PWM(self) -> int:
        """
        Get the servo's maximum PWM value.

        Returns
        -------
        int
            The servo's maximum PWM value.
        """

        return self._MAX_PWM

    # -------------------------------------------------------------------

    @property
    def ANGLE_RANGE(self) -> float:
        """
        Get the servo's angle range (in degrees).

        Returns
        -------
        float
            The servo's angle range (in degrees).
        """

        return self._ANGLE_RANGE


    # -------------------------------------------------------------------

    @property
    def is_moving(self) -> bool:
        """
        Check if the servo is currently moving.

        Returns
        -------
        bool
            True if the servo is currently executing a movement with a
            position function, False otherwise.
        """

        with self._lock:
            return self._position_function is not None \
                and self._movement_stop_time > time.time()

    # -------------------------------------------------------------------

    @property
    def angle(self) -> float:
        """
        Get the current angle of the servo.

        Returns
        -------
        float
            The current angle of the servo.
        """

        with self._lock:
            return (self.pwm() - self._MIN_PWM) * self._ANGLE_RANGE \
                / self._PWM_RANGE

    # -------------------------------------------------------------------

    @property
    def stop_pwm(self) -> int:
        """
        Get the target PWM value for the current or most recent move.

        Returns
        -------
        int
            The target PWM value for the current or most recent move.
        """

        with self._lock:
            return self._movement_stop_pwm

    # -------------------------------------------------------------------

    @property
    def stop_angle(self) -> float:
        """
        Get the target angle for the current or most recent move.

        Returns
        -------
        float
            The target angle for the current or most recent move.
        """

        with self._lock:
            return (self._movement_stop_pwm - self._MIN_PWM) \
                * self._ANGLE_RANGE / self._PWM_RANGE

    # -------------------------------------------------------------------

    @property
    def stop_time(self) -> Optional[float]:
        """
        Get the time at which the current move will end, or None if the
        servo is not currently moving.

        Returns
        -------
        Optional[float]
            The time at which the current move will end, or None if the
            servo is not currently moving.
        """

        with self._lock:
            if self.is_moving:
                return self._movement_stop_time
            return None
