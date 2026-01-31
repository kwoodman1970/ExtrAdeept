
"""
Base class for servo controllers used with Adeept HATs.

This module provides a base class that manages servo controllers and
provides centralized position updates for all connected servos.

To use, add the following line to the top of your module:

    from adeept_components_servo_controller import ServoController

Class Listings
--------------
ServoController
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

import threading
import time
from typing import Optional, List

# ======================================================================
# SERVO CONTROLLER BASE CLASS DEFINITION
# ======================================================================

class ServoController:
    """
    Base class for servo controllers.

    This class manages servo controllers and provides centralized
    position updates for all connected servos through a single
    background thread.

    The controller maintains a list of servo objects and calls their
    update_position() method at regular intervals. This allows each
    servo to maintain its own state while benefiting from centralized
    thread management.

    Attributes
    ----------
    _lock : threading.Lock
        Mutex to prevent multiple threads from accessing the controller
        simultaneously.
    _max_num_servos : int
        Maximum number of servos this controller can manage.
    _servos : List[Optional[object]]
        List of servo objects registered with this controller.
    _update_thread : threading.Thread
        Background thread that updates all servo positions.
    _running : bool
        Flag indicating whether the update thread is running.
    """

    _UPDATE_INTERVAL: float = 0.01  # 10ms update interval (100Hz)

    # -------------------------------------------------------------------

    def __init__(self, max_num_servos: int) -> None:
        """
        Initialize the servo controller.

        Parameters
        ----------
        max_num_servos : int
            The maximum number of servos this controller can manage.
        """
        self._lock = threading.Lock()
        self._max_num_servos: int = max_num_servos
        self._servos: List[Optional[object]] = [None] * max_num_servos

        # Thread control
        self._running: bool = False
        self._update_thread: Optional[threading.Thread] = None

        # Start the update thread
        self._start_update_thread()

    # -------------------------------------------------------------------

    def _start_update_thread(self) -> None:
        """Start the background thread that updates servo positions."""
        if not self._running:
            self._running = True
            self._update_thread = threading.Thread(
                target=self._update_loop,
                daemon=True
            )
            self._update_thread.start()

    # -------------------------------------------------------------------

    def _update_loop(self) -> None:
        """
        Main loop for updating servo positions.

        This runs in a background thread and calls update_position()
        on all registered servos at regular intervals.
        """
        while self._running:
            start_time = time.time()
            current_time = start_time

            # Update each servo that is registered
            for servo in self._servos:
                if servo is not None:
                    servo._update_position(current_time)

            # Sleep for the remainder of the update interval
            elapsed = time.time() - start_time
            sleep_time = max(0, self._UPDATE_INTERVAL - elapsed)
            time.sleep(sleep_time)

    # -------------------------------------------------------------------

    def shutdown(self) -> None:
        """
        Shutdown the controller and stop the update thread.

        This should be called when the controller is no longer needed.
        All servos should be stopped before calling this method.
        """
        self._running = False
        if self._update_thread is not None:
            self._update_thread.join(timeout=1.0)

    # -------------------------------------------------------------------

    def add_servo(self, port_num: int, servo: object) -> object:
        """
        Register a servo with this controller.

        Parameters
        ----------
        port_num : int
            The port number where the servo is connected.
        servo : object
            The servo object to register.

        Returns
        -------
        object
            The servo object that was registered.

        Raises
        ------
        ValueError
            If the port number is invalid or already in use.
        """
        if port_num < 0 or port_num >= self._max_num_servos:
            raise ValueError(f"Port number {port_num} is out of range "
                           f"(must be from 0 to {self._max_num_servos - 1}).")

        if self._servos[port_num] is not None:
            raise ValueError(f"Port {port_num} is already in use by another servo.")

        self._servos[port_num] = servo

        return servo

    # -------------------------------------------------------------------

    def remove_servo(self, port_num: int) -> None:
        """
        Unregister a servo from this controller.

        The servo should be stopped before calling this method.

        Parameters
        ----------
        port_num : int
            The port number to clear.

        Raises
        ------
        ValueError
            If the port number is invalid.
        """
        if port_num < 0 or port_num >= self._max_num_servos:
            raise ValueError(f"Port number {port_num} is out of range "
                           f"(must be from 0 to {self._max_num_servos - 1}).")

        self._servos[port_num] = None

    # -------------------------------------------------------------------

    def get_servo(self, port_num: int) -> Optional[object]:
        """
        Get the servo registered at a specific port.

        Parameters
        ----------
        port_num : int
            The port number to query.

        Returns
        -------
        object or None
            The servo object at the specified port, or None if no servo
            is registered there.

        Raises
        ------
        ValueError
            If the port number is invalid.
        """
        if port_num < 0 or port_num >= self._max_num_servos:
            raise ValueError(f"Port number {port_num} is out of range "
                           f"(must be from 0 to {self._max_num_servos - 1}).")

        return self._servos[port_num]

    # -------------------------------------------------------------------

    def port_is_available(self, port_num: int) -> bool:
        """
        Check if a port is available for use.

        Parameters
        ----------
        port_num : int
            The port number to check.

        Returns
        -------
        bool
            True if the port is available, False otherwise.

        Raises
        ------
        ValueError
            If the port number is invalid.
        """
        if port_num < 0 or port_num >= self._max_num_servos:
            raise ValueError(f"Port number {port_num} is out of range "
                           f"(must be from 0 to {self._max_num_servos - 1}).")

        return self._servos[port_num] is None

    # -------------------------------------------------------------------

    def get_max_num_servos(self) -> int:
        """
        Get the maximum number of servos this controller supports.

        Returns
        -------
        int
            The maximum number of servos.
        """
        return self._max_num_servos

    # -------------------------------------------------------------------

    def get_lock(self) -> threading.Lock:
        """
        Get the controller's lock for thread-safe operations.

        Servos should use this lock when calling set_pwm() to ensure
        thread-safe access to the hardware.

        Returns
        -------
        threading.Lock
            The lock object for this controller.
        """
        return self._lock

    # -------------------------------------------------------------------

    def set_pwm_freq(self, frequency: int) -> None:
        """
        Set the PWM frequency for the controller.

        This method must be implemented by subclasses to configure
        the hardware-specific PWM frequency.

        Parameters
        ----------
        frequency : int
            The PWM frequency in Hz.

        Raises
        ------
        NotImplementedError
            If the subclass does not implement this method.
        """
        raise NotImplementedError("Subclasses must implement set_pwm_freq")

    # -------------------------------------------------------------------

    def set_pwm(self, port_num: int, offset: int, pwm_value: int) -> None:
        """
        Set the PWM value for a specific port.

        This method must be implemented by subclasses to send PWM
        signals to the hardware.

        Parameters
        ----------
        port_num : int
            The port number (0 to max_num_servos-1).
        offset : int
            PWM offset value (hardware-specific).
        pwm_value : int
            The PWM duty cycle value.

        Raises
        ------
        NotImplementedError
            If the subclass does not implement this method.
        """
        raise NotImplementedError("Subclasses must implement set_pwm")

    # -------------------------------------------------------------------

    def set_all_pwm(self, offset: int, pwm_value: int) -> None:
        """
        Set the PWM value for all ports simultaneously.

        This method can be optionally implemented by subclasses if
        the hardware supports setting all channels at once.

        Parameters
        ----------
        offset : int
            PWM offset value (hardware-specific).
        pwm_value : int
            The PWM duty cycle value.

        Raises
        ------
        NotImplementedError
            If the subclass does not implement this method.
        """
        raise NotImplementedError("Subclasses may implement set_all_pwm")

# ======================================================================
