"""
Adafruit PCA9685 servo controller for Adeept HATs.

This module provides servo controller functionality for the Adafruit
PCA9685 servo driver.  It manages servo position updates through a
background thread and provides thread-safe hardware access.

To use, add the following line to the top of your module:

    from adeept_components_servo_controller import AdafruitPCA9685

Class Listings
--------------
AdafruitPCA9685
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

import threading
import time
from typing import Optional, List
from Adafruit_PCA9685 import PCA9685, PCA9685_ADDRESS
from adeept_components_servo import Servo

# ======================================================================
# ADAFRUITPCA9685 CLASS DEFINITION
# ======================================================================

class AdafruitPCA9685:
    """
    Controller for the Adafruit PCA9685 servo driver.

    This class manages the Adafruit PCA9685 servo driver and provides
    centralized position updates for all connected servos through a
    single background thread.

    The instance maintains a list of servo objects and calls their
    pwm() methods at regular intervals. This allows each
    servo to maintain its own state while benefiting from centralized
    thread management.

    Parameters
    ----------
    pwm_frequency : int, optional
        The PWM frequency in Hz (default: 50).
    i2c_address : int, optional
        The I2C address of the PCA9685 (default: 0x40).
    i2c_bus : int, optional
        The I2C bus number (default: None, which uses the default bus).

    Attributes
    ----------
    MAX_NUM_SERVOS : int
        Maximum number of servos this controller can manage (16 for
        PCA9685).

    Raises
    ------
    IOError
        If there is an error initializing the PCA9685 controller (e.g.,
        I2C communication failure).
    """

    # Private Attributes
    # ------------------
    #
    # _DEFAULT_FREQUENCY : int
    #     Default PWM frequency (in hertz) for the servos (50 for
    #     PCA9685).
    # _SERVOS_INITIAL_PWM : int
    #     Initial PWM value for all servos (300 for center position).
    # _UPDATE_INTERVAL : float
    #     Time interval (in seconds) at which to update all servo
    #     positions (0.01 for 100Hz, determined through
    #     experimentation).
    # _controller : Adafruit_PCA9685.PCA9685
    #     The underlying Adafruit PCA9685 controller instance.
    # _lock : threading.Lock
    #     Mutex to prevent multiple threads from accessing the
    #     controller simultaneously.
    # _servos : List[Optional[Servo]]
    #     List of servo objects registered with this controller.
    # _last_pwms : List[Optional[int]]
    #     Cached PWM values for each port to avoid redundant hardware
    #     writes.
    # _update_thread : Optional[threading.Thread]
    #     Background thread that updates all servo positions.
    # _activated : bool
    #     Flag indicating whether all servos have been added and the
    #     controller has been activated.
    # _running : bool
    #     Flag indicating whether the update thread is running.

    _DEFAULT_FREQUENCY:  int = 50
    _SERVOS_INITIAL_PWM:  int = 300
    _UPDATE_INTERVAL:  float = 0.01
    MAX_NUM_SERVOS:  int = 16

    # -------------------------------------------------------------------

    def __init__(self, pwm_frequency: int = _DEFAULT_FREQUENCY,
                 i2c_address: int = PCA9685_ADDRESS,
                 i2c_bus: Optional[int] = None) -> None:
        """
        Initialize the Adafruit PCA9685 controller.
        """

        # Initialize the hardware controller
        self._controller:  PCA9685 = PCA9685(i2c_address, i2c_bus)
        self._controller.set_pwm_freq(pwm_frequency)
        self._controller.set_all_pwm(0, self._SERVOS_INITIAL_PWM)

        # Initialize servo management
        self._lock:  threading.Lock = threading.Lock()
        self._servos:  List[Optional[Servo]] = [None] * self.MAX_NUM_SERVOS
        self._last_pwms:  List[Optional[int]] = [self._SERVOS_INITIAL_PWM] * self.MAX_NUM_SERVOS

        # Initialize update thread
        self._running:  bool = False
        self._update_thread:  Optional[threading.Thread] = None
        self._activated:  bool = False

    # -------------------------------------------------------------------

    def _start_update_thread(self) -> None:
        """
        Start the background thread that updates servo positions.

        If the update thread is already running then this method takes
        no further action.  This method is thread-safe and can be called
        multiple times without issue.

        Notes
        -----
        The update thread is created as a daemon thread, so it will
        automatically terminate when the main program exits.
        """

        with self._lock:
            if self._update_thread is None or not self._update_thread.is_alive():
                self._running = True
                self._update_thread = threading.Thread(
                    target = self._update_loop,
                    daemon = True
                )

                self._update_thread.start()

    # -------------------------------------------------------------------

    def _stop_update_thread(self, timeout:  Optional[float] = None) -> bool:
        """
        Stop the background thread that updates servo positions.

        Parameters
        ----------
        timeout : float, optional
            Maximum time (in seconds) to wait for the update thread to
            stop (default:  None, which waits indefinitely).

        Returns
        -------
        bool
            True if the update thread successfully stopped, False if the
            thread hasn't stopped running before the request timed out.
        """

        self._running = False

        if self._update_thread is not None:
            self._update_thread.join(timeout)

            return not self._update_thread.is_alive()

        return True

    # -------------------------------------------------------------------

    def _update_loop(self) -> None:
        """
        Main loop for updating servo positions.

        This method runs in a background thread and continuously updates
        all registered servos at the configured update interval.  For
        each servo, it queries the current PWM value and sends it to the
        hardware controller if it has changed.

        The loop maintains a consistent update rate by scheduling the
        next update time and sleeping for the remaining duration.  If
        servo updates take longer than the update interval, the next
        cycle starts immediately.

        Notes
        -----
        This method should not be called directly.  It is automatically
        started by the `activate()` method and stopped by the
        `shutdown()` or `stop_all_servos()` methods.
        """

        current_time = time.time()

        while self._running:
            # Update each servo that is registered

            with self._lock:
                for i, servo in enumerate(self._servos):
                    if servo is not None:
                        try:
                            new_pwm = servo.pwm(time.time())

                            if new_pwm != self._last_pwms[i]:
                                self._controller.set_pwm(i, servo.PWM_OFFSET, new_pwm)
                                self._last_pwms[i] = new_pwm
                        except Exception as exception:
                            print(f"Error updating servo at port {i}: {exception}")

            # Sleep for the remainder of the update interval

            current_time += self._UPDATE_INTERVAL

            time.sleep(max(0, current_time - time.time()))

    # -------------------------------------------------------------------

    def activate(self) -> None:
        """
        Start controlling the servos.

        Must be called after all servos have been added.
        """

        self._activated = True
        self._start_update_thread()

    # -------------------------------------------------------------------

    def shutdown(self, timeout:  Optional[float] = 1.0) -> None:
        """
        Shut the controller down.

        This method should be called when the instance is no longer
        needed (such as the program exiting) to ensure a clean shutdown.

        Parameters
        ----------
        timeout : float, optional
            Maximum time to wait for the update thread to stop (default:
            1.0 seconds).
        """
        # Stop the update thread first
        if not self._stop_update_thread(timeout):
            print("Warning: Update thread did not stop within timeout period")

        # Now safe to access hardware directly
        with self._lock:
            self._controller.set_all_pwm(0, 0)

            for i in range(self.MAX_NUM_SERVOS):
                self._last_pwms[i] = None

    # -------------------------------------------------------------------

    def add_servo(self, port_num:  int, servo:  Servo) -> Servo:
        """
        Register a servo with this controller.

        Parameters
        ----------
        port_num : int
            The port number (0 to MAX_NUM_SERVOS - 1) where the servo
            is connected.
        servo : Servo
            The servo object to register.

        Returns
        -------
        Servo
            The servo object that was registered.

        Raises
        ------
        RuntimeError
            The controller has already been activated and is running.
            No servos can be added after activation.
        ValueError
            The port number is invalid or already in use.
        TypeError
            The servo parameter is not a Servo instance.
        """

        if self._activated:
            raise RuntimeError("Cannot add servos after activation")

        if port_num < 0 or port_num >= self.MAX_NUM_SERVOS:
            raise ValueError(f"Port number {port_num} is out of range "
                           f"(0-{self.MAX_NUM_SERVOS - 1})")

        if not isinstance(servo, Servo):
            raise TypeError(f"servo must be a Servo instance, not a {type(servo).__name__}")

        with self._lock:
            if self._servos[port_num] is not None:
                raise ValueError(f"Port {port_num} is already in use")

            initial_pwm = servo.pwm()
            self._servos[port_num] = servo
            self._last_pwms[port_num] = initial_pwm

            self._controller.set_pwm(port_num, servo.PWM_OFFSET, initial_pwm)

        return servo

    # -------------------------------------------------------------------

    def get_servo(self, port_num:  int) -> Optional[Servo]:
        """
        Get the servo instance registered at a specific port.

        Parameters
        ----------
        port_num : int
            The port number (0 to MAX_NUM_SERVOS - 1) to query.

        Returns
        -------
        Optional[Servo]
            The servo object at the specified port, or None if no servo
            is registered there.

        Raises
        ------
        ValueError
            The port number is invalid.
        """
        if port_num < 0 or port_num >= self.MAX_NUM_SERVOS:
            raise ValueError(f"Port number {port_num} is out of range "
                           f"(0-{self.MAX_NUM_SERVOS - 1})")

        with self._lock:
            return self._servos[port_num]

    # -------------------------------------------------------------------

    def port_is_available(self, port_num:  int) -> bool:
        """
        Check if a port is available for a servo to be connected to it.

        Parameters
        ----------
        port_num : int
            The port number (0 to MAX_NUM_SERVOS - 1) to check.

        Returns
        -------
        bool
            True if the port is available, False otherwise.

        Raises
        ------
        ValueError
            The port number is invalid.
        """

        if port_num < 0 or port_num >= self.MAX_NUM_SERVOS:
            raise ValueError(f"Port number {port_num} is out of range "
                           f"(0-{self.MAX_NUM_SERVOS - 1})")

        with self._lock:
            return self._servos[port_num] is None

    # -------------------------------------------------------------------

    def get_lock(self) -> threading.Lock:
        """
        Get the lock used for thread-safe hardware access.

        Returns
        -------
        threading.Lock
            The lock object for this controller.
        """

        return self._lock

    # -------------------------------------------------------------------

    def set_all_pwm(self, offset:  int, pwm_value:  int) -> None:
        """
        Set the PWM value for all ports.

        Parameters
        ----------
        offset : int
            PWM offset value.
        pwm_value : int
            The PWM duty cycle value.
        """

        with self._lock:
            self._controller.set_all_pwm(offset, pwm_value)

            for i in range(self.MAX_NUM_SERVOS):
                self._last_pwms[i] = None

    # -------------------------------------------------------------------

    def stop_all_servos(self, timeout:  Optional[float] = None) -> bool:
        """
        Stop the movement of all servos.

        Parameters
        ----------
        timeout : Optional[float]
            Maximum time (in seconds) to wait for the servos to stop
            moving (default:  None, which waits indefinitely).

        Returns
        -------
        bool
            True if the update thread stopped successfully within the
            timeout period, False otherwise.
        """

        if not self._stop_update_thread(timeout):
            return False

        with self._lock:
            for port_num, servo in enumerate(self._servos):
                if servo is not None:
                    servo.stop_moving()
                    pwm_value = servo.pwm()
                    self._controller.set_pwm(port_num, servo.PWM_OFFSET, pwm_value)
                    self._last_pwms[port_num] = pwm_value

        return True
