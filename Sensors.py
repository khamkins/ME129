#!/usr/bin/env python
#
#   Sensors.py creates a Motor object with sensors
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

# define sensors pins
IR_LEFT = 18
IR_MIDDLE = 15
IR_RIGHT = 14

    

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


        # Set up three IR pins as input
        self.io.set_mode(IR_LEFT, pigpio.INPUT)
        self.io.set_mode(IR_MIDDLE, pigpio.INPUT)
        self.io.set_mode(IR_RIGHT, pigpio.INPUT)
        

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
#         if(leftdutycycle > 1) or (leftdutycycle < -1) or (rightdutycycle > 1) or (rightdutycycle < -1) :
#             print("command out of bounds")
#             self.shutdown
        if leftdutycycle > 1 :
            leftdutycycle = 0.99
        if rightdutycycle > 1 :
            rightdutycycle = 0.99
        if leftdutycycle < -1:
            leftdutycycle = -0.99
        if rightdutycycle < -1 :
            rightdutycycle = -0.99

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
        PWM_fwd = (abs(linear) + 0.153)/0.738 + 0.2
        if spin > 90 :
            PWM_spin = (abs(spin) + 153.0 )/546.0 
        
        else :
            PWM_spin = (abs(spin))/546.0
        
        if linear == 0:
            PWM_fwd = 0
        if spin == 0:
            PWM_spin = 0
        
        
        if linear >= 0 and spin >= 0 :
            PWM_L = PWM_fwd + PWM_spin
            PWM_R = PWM_fwd - PWM_spin
            
        if linear >= 0 and spin <= 0 :
            PWM_L = PWM_fwd - PWM_spin
            PWM_R = PWM_fwd + PWM_spin
            
        print(PWM_L, PWM_R)
        motors.set(PWM_L, PWM_R)  
        
    
    def ircheck(self):
        return self.io.read(IR_LEFT)*4 + self.io.read(IR_MIDDLE)*2 + self.io.read(IR_RIGHT)
        
        
    def lost(self, ir_old):
        degree = 140
        vel = 0
        t = 0
        while self.ircheck() == 0 :
            if t == 5000:
                degree = degree -.5
                vel = vel + 0.05
                t = 0
            else:
                t = t+1
            if ir_old == 1 :
                self.setvel(vel, degree)
            else:
                self.setvel(vel, -degree)
            print(degree)
            
#
#   Main
#
if __name__ == "__main__":
    motors = Motor()
    searching = True
    try:
        ir_old = motors.ircheck()
        while True:
            if searching and ir_old == 0:
                motors.lost(1)
            elif ir_old != 0:
                searching = False
            
            # read sensors
            ir_curr = motors.ircheck()
            
            if ir_curr == 2 :
                motors.setvel(0.2, 0)
            
            elif (ir_old == 2 or ir_old == 3) and ir_curr == 3 : #drifted left
                motors.setvel(0.2, 20)
            
            elif (ir_old == 2 or ir_old == 6) and ir_curr == 6 :#drifted right
                motors.setvel(0.2, -20)
            
            elif (ir_old == 2 or ir_old == 3 or ir_old == 1) \
                 and ir_curr ==  1:      #a lot left
                motors.setvel(0.2, 60)
            
            elif (ir_old == 2 or ir_old == 6 or ir_old == 4)\
                 and ir_curr == 4 :  #a lot right
                motors.setvel(0.2, -60)
            
            elif ir_curr == 0:  # past end
                if(ir_old == 2 or ir_old == 3 or ir_old == 6):
                    motors.lost(1)
            
                # if we're lost, spiral
                elif ir_old == 1:
                    motors.lost(1)
                    ir_curr = motors.ircheck()
                elif ir_old == 4:
                    motors.lost(4)
                    ir_curr = motors.ircheck()
                    
            
            # state 5 - branch in road
            elif ir_curr == 5 :
                motors.setvel(0.2, 90)    # go right for now
            
            #sees all of them
            elif ir_curr == 7 :
                motors.setspin(120)   #spin in place
            
            #update past IR status
            ir_old = ir_curr
            
            
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
    
    motors.shutdown()


