
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

from typing import Optional
import time

from adeept_position_functions import PositionFunction
from adeept_components_servo_controller import ServoController

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
    controller : ServoController
        The servo controller instance managing this servo.
    port_num : int
        The port on the controller that the servo is connected to.
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
        and `max_pwm`. Must be greater than 0.0 (default is 180.0).
    initial_pwm : int
        The initial PWM to send to the servo (such as a starting
        position or a home position). Must be between `min_pwm` and
        `max_pwm`.

    Attributes
    ----------
    MIN_PWM : int
    MAX_PWM : int
    ANGLE_RANGE : float
    pwm : int
    angle : float

    Raises
    ------
    ValueError
        One or more arguments are invalid.
    """

    # Private Attributes
    # ------------------
    # _controller : ServoController
    #     The servo controller object that this servo is connected to.
    # _PORT_NUM : int
    #     The port number on the controller that this servo is connected to.
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
    # _current_pwm : int
    #     The PWM value that the servo was last set to.
    # _position_function : Optional[PositionFunction]
    #     The current position function being used for movement, or None
    #     if the servo is not moving.
    # _movement_start_time : Optional[float]
    #     The time when the current movement started, or None if not moving.
    # _movement_stop_time : Optional[float]
    #     The time when the current movement should stop, or None if not moving.
    # _movement_start_pwm : Optional[int]
    #     The PWM value when the current movement started, or None if not moving.
    # _movement_target_pwm : Optional[int]
    #     The target PWM value for the current movement, or None if not moving.

    # -------------------------------------------------------------------

    def __init__(self, controller: ServoController, port_num: int,
                 pwm_offset: int, min_pwm: int, max_pwm: int,
                 angle_range: float, initial_pwm: int) -> None:
        """Initialize the servo."""

        # Validate the method's arguments.

        if port_num < 0 or port_num >= controller.get_max_num_servos():
            raise ValueError(f"\"port_num\" ({port_num}) must be from 0 to "
                           f"{controller.get_max_num_servos() - 1}")

        if not controller.port_is_available(port_num):
            raise ValueError(f"Port {port_num} is already in use.")

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

        self._controller: ServoController = controller
        self._PORT_NUM: int = port_num
        self._PWM_OFFSET: int = pwm_offset
        self._MIN_PWM: int = min_pwm
        self._MAX_PWM: int = max_pwm
        self._ANGLE_RANGE: float = float(angle_range)
        self._PWM_RANGE: int = max_pwm - min_pwm

        # Initialize the servo state.

        self._current_pwm: int = initial_pwm
        self._position_function: Optional[PositionFunction] = None
        self._movement_start_time: Optional[float] = None
        self._movement_stop_time: Optional[float] = None
        self._movement_start_pwm: Optional[int] = None
        self._movement_target_pwm: Optional[int] = None

        # Register this servo with the controller.

        controller.add_servo(port_num, self)

        # Set the initial position.

        with controller.get_lock():
            controller.set_pwm(port_num, pwm_offset, initial_pwm)

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

        # Stop any current movement
        self.stop_moving()

        if position_function is None:
            # Instant movement
            self._current_pwm = pwm
            with self._controller.get_lock():
                self._controller.set_pwm(self._PORT_NUM, self._PWM_OFFSET, pwm)
        else:
            # Gradual movement using position function
            self._position_function = position_function
            self._movement_start_time = time.time()
            self._movement_stop_time = stop_time
            self._movement_start_pwm = self._current_pwm
            self._movement_target_pwm = pwm

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

        self._position_function = None
        self._movement_start_time = None
        self._movement_stop_time = None
        self._movement_start_pwm = None
        self._movement_target_pwm = None

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

        if not self.is_moving():
            return True

        start_wait_time = time.time()

        while self.is_moving():
            if timeout is not None:
                elapsed = time.time() - start_wait_time
                if elapsed >= timeout:
                    return False

            time.sleep(0.01)  # Small sleep to avoid busy-waiting

        return True

    # -------------------------------------------------------------------

    def is_moving(self) -> bool:
        """
        Check if the servo is currently moving.

        Returns
        -------
        bool
            True if the servo is currently executing a movement with a
            position function, False otherwise.
        """

        return self._position_function is not None

    # -------------------------------------------------------------------


    def _update_position(self, current_time: float) -> None:
        """
        Update the servo's position based on its position function.

        **INTERNAL METHOD - Friend of ServoController**

        This method implements a "friend class" pattern where
        ServoController is granted privileged access to update servo
        positions.  While the leading underscore indicates this is a
        private method, it is intentionally called by ServoController's
        background update thread.

        **Do not call this method directly from user code.**  The
        servo's position is automatically updated by the controller's
        background thread when a position function is active.

        Parameters
        ----------
        current_time : float
            The current time (from time.time()).

        Thread Safety
        -------------
        The servo's position is updated by the ServoController's
        background thread.  Reading the current position (via the `pwm`
        or `angle` properties) from another thread is safe due to
        Python's GIL, but the value may be slightly out of date.
        """

        if not self.is_moving():
            return

        assert(self._movement_start_time is not None)
        assert(self._movement_stop_time is not None)
        assert(self._movement_start_pwm is not None)
        assert(self._movement_target_pwm is not None)
        assert(self._position_function is not None)

        # Calculate time index for position function
        time_index: float = current_time - self._movement_start_time
        duration: float = self._movement_stop_time - self._movement_start_time

        # Check if movement is complete
        if time_index >= duration:
            self._current_pwm = self._movement_target_pwm
            with self._controller.get_lock():
                self._controller.set_pwm(self._PORT_NUM, self._PWM_OFFSET,
                                    self._current_pwm)
            self.stop_moving()
        elif time_index >= 0.0:
            # Only update if movement has actually started
            # Calculate intermediate position
            pwm_range:  int = self._movement_target_pwm - self._movement_start_pwm
            new_pwm:  int = int(self._position_function(pwm_range, duration, time_index) \
                    + self._movement_start_pwm)

            self._current_pwm = min(max(new_pwm, self._MIN_PWM), self._MAX_PWM)
            with self._controller.get_lock():
                self._controller.set_pwm(self._PORT_NUM, self._PWM_OFFSET, self._current_pwm)

    # -------------------------------------------------------------------

    def get_port_num(self) -> int:
        """
        Get the port number this servo is connected to.

        Returns
        -------
        int
            The port number on the controller.
        """
        return self._PORT_NUM

    # -------------------------------------------------------------------

    MIN_PWM = property(lambda self: self._MIN_PWM, None, None,
                      "The servo's minimum PWM value.")

    # -------------------------------------------------------------------

    MAX_PWM = property(lambda self: self._MAX_PWM, None, None,
                      "The servo's maximum PWM value.")

    # -------------------------------------------------------------------

    ANGLE_RANGE = property(lambda self: self._ANGLE_RANGE, None, None,
                          "The servo's angle range (in degrees).")

    # -------------------------------------------------------------------

    pwm = property(lambda self: self._current_pwm, None, None,
                  "The servo's current PWM value.")

    # -------------------------------------------------------------------

    angle = property(lambda self: (self._current_pwm - self._MIN_PWM) \
                    * self._ANGLE_RANGE / self._PWM_RANGE, None, None,
                    "The servo's current angle (in degrees).")

# ======================================================================
