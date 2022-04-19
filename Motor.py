#!/usr/bin/env python
#
#   Motor.py creates a Motor object
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
    

class Motor:
    
    def __init__(self):
        

        # Prepare the GPIO connetion (to command the motors).
        print("Setting up the GPIO...")

        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = pigpio.pi()
        if not self.io.connected:
            print("Unable to connection to pigpio daemon!")
            sys.exit(0)

        # Set up the four pins as output (commanding the motors).
        self.io.set_mode(MTR1_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR1_LEGB, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGA, pigpio.OUTPUT)
        self.io.set_mode(MTR2_LEGB, pigpio.OUTPUT)

        # Prepare the PWM.  The range gives the maximum value for 100%
        # duty cycle, using integer commands (1 up to max).
        self.io.set_PWM_range(MTR1_LEGA, 255)
        self.io.set_PWM_range(MTR1_LEGB, 255)
        self.io.set_PWM_range(MTR2_LEGA, 255)
        self.io.set_PWM_range(MTR2_LEGB, 255)

        # Set the PWM frequency to 1000Hz.  You could try 500Hz or 2000Hz
        # to see whether there is a difference?
        self.io.set_PWM_frequency(MTR1_LEGA, 1000)
        self.io.set_PWM_frequency(MTR1_LEGB, 1000)
        self.io.set_PWM_frequency(MTR2_LEGA, 1000)
        self.io.set_PWM_frequency(MTR2_LEGB, 1000)

        # Clear all pins, just in case.
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

        print("GPIO ready...")


    def shutdown(self):
        # Clear all pins, just in case.
        self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
        self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

        self.io.stop()

    def set(self, leftdutycycle, rightdutycycle):
        if(leftdutycycle > 1) or (leftdutycycle < -1) or (rightdutycycle > 1) or (rightdutycycle < -1) :
            print("command out of bounds")
            self.shutdown

        if leftdutycycle > 0:
            self.io.set_PWM_dutycycle(MTR1_LEGA, int(leftdutycycle*255))
            self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        else:
            self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR1_LEGB, -int(leftdutycycle*255))

        if rightdutycycle > 0:
            self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR2_LEGB, int(rightdutycycle*255))
        else:
            self.io.set_PWM_dutycycle(MTR2_LEGA, -int(rightdutycycle*255))
            self.io.set_PWM_dutycycle(MTR2_LEGB, 0)


    
    def setlinear(self, speed):
        PWM = (speed + 0.153)/0.738
        self.set(PWM, PWM)
        
        
    def setspin(self, speed):
        #set spin based on speed. Positive speed indicates clockwise rotation
        
        PWM = (abs(speed) + 153.0)/546.0
        if speed > 0:
            self.set(PWM, -PWM)
        else:
            self.set(-PWM, PWM)
            
            
    def setvel(self, linear, spin):
        PWM_fwd = (abs(linear) + 0.153)/0.738 + .2
        PWM_spin = (abs(spin) + 153.0)/546.0 
              
        if linear >= 0 and spin >= 0 :
            PWM_L = PWM_fwd + PWM_spin
            PWM_R = PWM_fwd 
            
        if linear >= 0 and spin <= 0 :
            PWM_L = PWM_fwd - PWM_spin
            PWM_R = PWM_fwd + PWM_spin
            
        print(PWM_L, PWM_R)
        motors.set(PWM_L, PWM_R)  
        
    
    
#
#   Main
#
if __name__ == "__main__":
    motors = Motor()
    
    try:
        print("now move")
        t = 3
        motors.setvel(3/2/t,360/t )
        time.sleep(t*2)
    
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
    
    motors.shutdown()

