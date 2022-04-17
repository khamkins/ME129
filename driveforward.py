#!/usr/bin/env python
#
#   drive.py
#

# Imports
import pigpio
import sys
import time

# Define the motor pins.
MTR1_LEGA = 7
MTR1_LEGB = 8

MTR2_LEGA = 5
MTR2_LEGB = 6

#
#   Main
#
if __name__ == "__main__":

    ############################################################
    # Prepare the GPIO connetion (to command the motors).
    print("Setting up the GPIO...")

    # Initialize the connection to the pigpio daemon (GPIO interface).
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)

    # Set up the four pins as output (commanding the motors).
    io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
    io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
    io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
    io.set_mode(MTR2_LEGB, pigpio.OUTPUT)

    # Prepare the PWM.  The range gives the maximum value for 100%
    # duty cycle, using integer commands (1 up to max).
    io.set_PWM_range(MTR1_LEGA, 255)
    io.set_PWM_range(MTR1_LEGB, 255)
    io.set_PWM_range(MTR2_LEGA, 255)
    io.set_PWM_range(MTR2_LEGB, 255)

    # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
    # to see whether there is a difference?
    io.set_PWM_frequency(MTR1_LEGA, 1000)
    io.set_PWM_frequency(MTR1_LEGB, 1000)
    io.set_PWM_frequency(MTR2_LEGA, 1000)
    io.set_PWM_frequency(MTR2_LEGB, 1000)

    # Clear all pins, just in case.
    io.set_PWM_dutycycle(MTR1_LEGA, 0)
    io.set_PWM_dutycycle(MTR1_LEGB, 0)
    io.set_PWM_dutycycle(MTR2_LEGA, 0)
    io.set_PWM_dutycycle(MTR2_LEGB, 0)

    print("GPIO ready...")

    ############################################################
    # Forward/Backward
    print("Driving Forward")

    io.set_PWM_dutycycle(MTR1_LEGA, 150)
    io.set_PWM_dutycycle(MTR2_LEGB, 150)
    time.sleep(1)

    io.set_PWM_dutycycle(MTR1_LEGA, 0)
    io.set_PWM_dutycycle(MTR2_LEGB, 0)
    time.sleep(.5)

    io.set_PWM_dutycycle(MTR1_LEGA, 150)
    io.set_PWM_dutycycle(MTR2_LEGA, 150)
    time.sleep(1.1)
    
    io.set_PWM_dutycycle(MTR1_LEGA, 0)
    io.set_PWM_dutycycle(MTR2_LEGA, 0)
    time.sleep(.5)
    
    io.set_PWM_dutycycle(MTR1_LEGA, 150)
    io.set_PWM_dutycycle(MTR2_LEGB, 150)
    time.sleep(1)
    
    io.set_PWM_dutycycle(MTR1_LEGA, 0)
    io.set_PWM_dutycycle(MTR2_LEGB, 0)
    time.sleep(.5)

    
    io.set_PWM_dutycycle(MTR1_LEGA, 150)
    io.set_PWM_dutycycle(MTR2_LEGA, 150)
    time.sleep(1.1)


    ############################################################
    # Turn Off.
    # Note the PWM still stay at the last commanded value.  So you
    # want to be sure to set to zero before the program closes.  else
    # your robot will run away...
    print("Turning off...")

    # Clear the PINs (commands).
    io.set_PWM_dutycycle(MTR1_LEGA, 0)
    io.set_PWM_dutycycle(MTR1_LEGB, 0)
    io.set_PWM_dutycycle(MTR2_LEGA, 0)
    io.set_PWM_dutycycle(MTR2_LEGB, 0)

    # Also stop the interface.
    io.stop()
