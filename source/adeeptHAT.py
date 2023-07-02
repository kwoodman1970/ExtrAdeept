#!/usr/bin/python3
"""
adeeptHAT.py -- objects for controlling Adeept HAT components.

This module contains objects for controlling the following HATs:

 * Adeept Motor HAT 1.0
 * Adeept Motor HAT 2.0
 * Adeept Robot HAT

It is possible to replace one type of HAT with another with only minor
changes required to your code, provided that the new HAT supports the
same components.

HOW TO USE
==========

The HAT objects in this module need to be told which components are
connected to them using their various "add...()" methods.  Those
components can then be controlled using the HAT object's properties.

For example, suppose that the machine is a simple car with one drive
motor for the drive wheels and one servo for steering.  The drive motor
is connected to the "Motor A" connector and the servo is connected to
the "Servo Port 0" connector on a Motor HAT 1.0.  Declaring code might
look like this:

    from adeeptHat import AdeeptMotorHAT1 as _adeeptHAT

    driveWheels = _adeeptHAT.addDriveMotor("A")
    steering    = _adeeptHAT.addServo(0)

To control the car's speed and direction, all that needs to be done is
to call the "driveWheels.setSpeed()" and "steering.move_to()" methods.

Suppose that the HAT fails (wah!) and is replaced with an Adeept Motor
HAT 2.0.  The "AdeeptMotorHAT2" object would need to be imported
instead, but no other changes would be needed to the code.

EXPORTS
=======

AdeeptMotorHAT1:  object

AdeeptMotorHAT2:  object

AdeeptRobotHAT:  object
"""

# ======================================================================
# IMPORTS FROM OTHER MODULES
# ======================================================================

import atexit
from typing import Union

from adeept_components import *
from adeept_components_servo import Servo

# ======================================================================
# PRIVATE VARIABLES
# ======================================================================

_initialized:  bool = False
_buzzer:  Union[None, BuzzerActive, BuzzerPassive] = None
_driveMotors = []
_neoPixels   = None
_servos      = []

# ======================================================================
# PRIVATE FUNCTIONS
# ======================================================================

def _initHAT():
# This function initializes the Adeept HAT for use.

    global _initialized

    if not _initialized:
        GPIO.setwarnings(False)
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)

        _initialized = True

@atexit.register
def _endHAT():
# This function cleans up the Adeept HAT after use.

    global _initialized
    global _buzzer
    global _driveMotors
    global _servos

    print("_endHAT() called.")

    if _initialized:

        if not (_buzzer is None):
            _buzzer.stop()

        print("_endHAT() silenced the buzzer.")

        for driveMotor in _driveMotors:
            driveMotor.stop()

        print("_endHAT() stopped all drive motors.")

        for servo in _servos:
            servo.power_down()

        print("_endHAT() powered down all servos.")

        GPIO.cleanup()

        print("_endHAT() cleaned up GPIO.")

        _intialized = False

# ======================================================================
# ADEEPTMOTORHAT1 CLASS DEFINITION
# ======================================================================

# On the Adeept Motor HAT 1.0, the buzzer's control pin is GPIO 7.

# To be continued...

# ======================================================================
# ADEEPTMOTORHAT2 CLASS DEFINITION
# ======================================================================

class adeeptMotorHAT2:

# This object provides access to all of the components that can be
# connected to an Adeept Motor HAT v2.0.

    @staticmethod
    def addDriveMotor(portID:  str, reverseDirection:  bool, \
                      scaleFactor:  float = 1.0) -> DriveMotor:
        # Drive motors are expected to rotate counter-clockwise when set to a positve
        # speed value.  If the motor rotates clockwise instead then change
        # "reverseDirection".

        global _driveMotors

        _initHAT()

        # The pin assignments given in Chapter 11 of the Adeept RaspTank Tutorial PDF
        # (created on 2020-12-11) appear to be for the opposite motor ports.  The pin
        # assignments used here are for the correct ports.


        if portID == "A":
            if reverseDirection:
                newMotor = DriveMotor(17, 18, 27, scaleFactor)
            else:
                newMotor = DriveMotor(17, 27, 18, scaleFactor)
        elif portID == "B":
            if reverseDirection:
                newMotor = DriveMotor(4, 15, 14, scaleFactor)
            else:
                newMotor = DriveMotor(4, 14, 15, scaleFactor)
        else:
            raise ValueError("portID must be either \"A\" or \"B\".")

        _driveMotors.append(newMotor)

        return newMotor

    @staticmethod
    def addServo(\
        portNum:     int, \
        frequency:   int = Servo._DEFAULT_FREQUENCY, \
        PWMOffset:   int = 0, \
        minPWM:      int = Servo._DEFAULT_MIN_PWM, \
        maxPWM:      int = Servo._DEFAULT_MAX_PWM, \
        angleRange:  int = Servo._DEFAULT_ANGLE_RANGE) -> Servo:

        # The user is free to conduct their own
        # experiments to find accurate ranges for their own servos
        # (and caution is strongly advised when using values outside the manufacturer's specified range because that
        # may damage the servo's gear assembly).

        global _servos

        _initHAT()

        if (portNum < 0) or (portNum > 15):
            raise ValueError("portNum must be from 0 to 15.")

        newServo = Servo(portNum, frequency, PWMOffset, minPWM, maxPWM, angleRange, (minPWM + maxPWM) // 2)

        _servos.append(newServo)

        return newServo

    @staticmethod
    def addUltrasonicSensor():
        _initHAT()

        return UltrasonicSensor(11, 8)

    @staticmethod
    def addNeoPixelStrip(numPixels):
        _initHAT()

        global _neoPixels

        try:
            _neoPixels = NeoPixelStrip(12, numPixels)
        except RuntimeError:
            logging.warning("NeoPixels are not accessable.")
            _neoPixels = None

        return _neoPixels

    @staticmethod
    def addLineTracker():
        _initHAT()

        return LineTracker(20, 16, 19)

    @staticmethod
    def addRGB_LED(portNum):
        _initHAT()

        if portNum == 1:
            return RGB_LED(22, 23, 24)
        elif portNum == 2:
            return RGB_LED(10, 9, 25)
        else:
            raise ValueError("portNum must be either 1 or 2.")

# ======================================================================
# ADEEPTROBOTHAT CLASS DEFINITION
# ======================================================================

# To be continued...

# ======================================================================
# TEST AREA
# ======================================================================

if __name__ == "__main__":
    if False:
        #servo = Servo(14, 50, 0, 100, 500, 180)
        servo = adeeptMotorHAT2.addServo(14)

        servo.move_to(angle = 45)
        time.sleep(.5)
        servo.move_to(angle = 90)
        time.sleep(.5)
        servo.move_to(angle = 135)
        time.sleep(.5)
        servo.move_to(pwm = 300)

    if False:
        #servo = Servo(14, 50, 0, 100, 500, 180)
        servo = adeeptMotorHAT2.addServo(14)

        servo.move_to(pwm = 70)
        time.sleep(.5)
        servo.move_to(pwm = 300)
        time.sleep(.5)
        servo.move_to(pwm = 530)
        time.sleep(.5)
        servo.move_to(pwm = 300)

    if True:
        shoulder = adeeptMotorHAT2.addServo(12)
        elbow = adeeptMotorHAT2.addServo(13)

        try:
            shoulder.move_to(angle = 90)
            elbow.move_to(angle = 90)
            time.sleep(2)

            while True:
                shoulder.move_by_acceleration(angle = 170, duration = 1)
                elbow.move_by_acceleration(angle = 170, duration = 1)

                elbow.wait()
                shoulder.wait()
                time.sleep(.5)

                shoulder.move_by_acceleration(angle = 10, duration = 1)
                elbow.move_by_acceleration(angle = 10, duration = 1)

                elbow.wait()
                shoulder.wait()
                time.sleep(.5)

        except:
            elbow.wait()
            shoulder.wait()

            elbow.move_by_acceleration(angle = 180, duration = .5)
            shoulder.move_by_acceleration(angle = 180, duration = .5)
            elbow.move_by_acceleration(pwm = 560, duration = .5)
            shoulder.move_by_acceleration(pwm = 560, duration = .5)

            elbow.wait()
            shoulder.wait()

    if False:
        #servo = Servo(14, 50, 0, 100, 500, 180)
        shoulder = adeeptMotorHAT2.addServo(12)
        elbow = adeeptMotorHAT2.addServo(13)

        try:
            shoulder.move_to(angle = 30)
            elbow.move_to(angle = 30)
            time.sleep(1)

            elbow.move_by_acceleration(angle = 60, duration = 0.5)
            time.sleep(0.25)

            shoulder.move_by_acceleration(angle = 0, duration = 0.5)

            while True:
                elbow.move_by_acceleration(angle = 0, duration = 0.5)
                shoulder.move_by_acceleration(angle = 30, duration = 1)
                elbow.wait()
                time.sleep(.5)

                elbow.move_by_acceleration(angle = 60, duration = 0.5)
                shoulder.move_by_acceleration(angle = 0, duration = 1)
                elbow.wait()
                time.sleep(.5)

        except:
            elbow.wait()
            shoulder.wait()

            elbow.move_by_acceleration(angle = 180, duration = .5)
            shoulder.move_by_acceleration(angle = 180, duration = .5)

            elbow.wait()
            shoulder.wait()

    if False:
        #motorA = DriveMotor("A", True)
        #motorB = DriveMotor("B", False)
        motorA = adeeptMotorHAT2.getDriveMotor("A", False)
        motorB = adeeptMotorHAT2.getDriveMotor("B", True)

        motorA.setSpeed(100)
        time.sleep(1)

        motorB.setSpeed(100)
        time.sleep(1)

        motorA.stop()
        time.sleep(1)

        motorB.stop()
        time.sleep(1)

        motorA.setSpeed(-100)
        motorB.setSpeed(-100)
        time.sleep(1)

        motorA.stop()
        motorB.stop()

    if False:
        #sonar = UltrasonicSensor()
        sonar = adeeptMotorHAT2.getUltrasonicSensor()

        try:
            while True:
                print(round(sonar.ping(5), 2))
                time.sleep(1)
        except:
            print()

    if False:
        #neopixels = NeoPixelStrip(12, 12)
        neopixels = adeeptMotorHAT2.getNeoPixelStrip(12)

        try:
            while True:
                for i in range(neopixels.numPixels()):
                    neopixels.setPixelColor(i, 0xFFFF00)
                    neopixels.show()
                    time.sleep(.5)
                    neopixels.setPixelColor(i, 0x000000)

        except:
            neopixels.clear()
            neopixels.show()
            print()

    if False:
        #neopixels = NeoPixelStrip(12, 12)
        neopixels = adeeptMotorHAT2.getNeoPixelStrip(12)

        try:
            while True:
                neopixels.fill(0xFF0000, 3, 6)
                neopixels.show()
                time.sleep(.5)
                neopixels.fill(0xFFFF00, 3, 6)
                neopixels.show()
                time.sleep(.5)
                neopixels.fill(0x00FF00, 3, 6)
                neopixels.show()
                time.sleep(.5)
                neopixels.fill(0x00FFFF, 3, 6)
                neopixels.show()
                time.sleep(.5)
                neopixels.fill(0x0000FF, 3, 6)
                neopixels.show()
                time.sleep(.5)
                neopixels.fill(0xFF00FF, 3, 6)
                neopixels.show()
                time.sleep(.5)
        except:
            neopixels.clear()
            neopixels.show()
            print()
