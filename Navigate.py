#!/usr/bin/env python
# 
#   Sensors.py creates a Motor object with sensors
#
# 

# Imports
import pigpio
import sys
import time
import random

# Define the motor pins.
MTR1_LEGA = 7
MTR1_LEGB = 8
MTR2_LEGA = 5
MTR2_LEGB = 6

# define sensors pins
IR_LEFT = 18
IR_MIDDLE = 15
IR_RIGHT = 14

# Global Constants:
# Headings
NORTH = 0
WEST = 1
SOUTH = 2
EAST = 3
HEADING = {NORTH:'North', WEST:'West', SOUTH:'South', EAST: 'East', None:'None'} # For printing

# Street status
UNKNOWN = 'Unknown'
NOSTREET = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED = 'Connected'

# Global Variables:
intersections = [] # List of intersections
turnstaken = [] #list of turns taken
lastintersection = None # Last intersection visited
long = 0 # Current east/west coordinate
lat = -1 # Current north/south coordinate
heading = 0 # Current heading
    

class Intersection:
    # Initialize - create new intersection at (long, let)
    def __init__(self, long, lat):
        # Save the parameters.
        self.long = long
        self.lat = lat
        # Status of streets at the intersection, in NWSE directions.
        self.streets = [UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN]
        # Direction to head from this intersection in planned move.
        self.headingToTarget = None
        # You are welcome to implement an arbitrary number of
        # "neighbors" to more directly match a general graph.
        # But the above should be sufficient for a regular grid.
        # Add this to the global list of intersections to make it searchable.
        global intersections
        if intersection(long, lat) is not None:
            raise Exception("Duplicate intersection at (%2d,%2d)" % (long,lat))
        intersections.append(self)
        # Print format.
        
    def __repr__(self):
        return ("(%2d, %2d) N:%s W:%s S:%s E:%s - head %s\n" %
               (self.long, self.lat, self.streets[0],
                self.streets[1], self.streets[2], self.streets[3],
                HEADING[self.headingToTarget]))
    
    


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
        if spin > 90 or spin < -90:
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
            
        #print(PWM_L, PWM_R)
        motors.set(PWM_L, PWM_R)  
        
    
    def ircheck(self):
        return self.io.read(IR_LEFT)*4 + self.io.read(IR_MIDDLE)*2 + self.io.read(IR_RIGHT)
        
        
    def lost(self, ir_old):
        print('Lost!')
        degree = 140
        vel = 0
        t = 0
        while self.ircheck() == 0 or self.ircheck()==1 or self.ircheck()==4 :
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
            #print(degree)
            
            
    #drive forward until you see an intersection        
    def drive(self):
        ir_old = self.ircheck()
        while True:            
            # read sensors
            ir_curr = self.ircheck()
            
            if ir_curr == 2 :
                self.setvel(0.2, 0)
            
            elif (ir_old == 2 or ir_old == 3) and ir_curr == 3 : #drifted left
                self.setvel(0.2, 40)
            
            elif (ir_old == 2 or ir_old == 6) and ir_curr == 6 :#drifted right
                self.setvel(0.2, -40)
                        
            elif (ir_old == 2 or ir_old == 3 or ir_old == 1) \
                 and ir_curr ==  1:      #a lot left
                motors.setvel(0.2, 60)
            
            elif (ir_old == 2 or ir_old == 6 or ir_old == 4)\
                 and ir_curr == 4 :  #a lot right
                motors.setvel(0.2, -60)
                
            elif ir_curr == 0:  # past end
                if ir_old == 4:
                    self.lost(4)
                    ir_curr = self.ircheck()
                else:
                    self.lost(1)
                    ir_curr = self.ircheck()
                    
            #sees all of them
            elif ir_curr == 7 :
                #drive straight to get axels over intersection
                self.setvel(0.2, 0)
                time.sleep(0.45)
                self.setvel(0,0)
                break
            #update past IR status
            ir_old = ir_curr
            
    
    def turn(self, dir):
        global heading
        new_dir = dir
        if dir == 1 or dir == -3:
            new_dir =1
            self.setvel(0, -360)
            time.sleep(.32)
        if dir == 2 or dir == -2:
            new_dir = 2
            self.setvel(0, -360)
            time.sleep(.64)
        if dir == 3 or dir == -1:
            new_dir=3
            self.setvel(0, 360)
            time.sleep(.33)
        
        self.setvel(0, 0)
        time.sleep(1)
        #update heading
        heading = (heading + new_dir) % 4
        
        
    def goHome(self):
        global long
        global lat
        global heading
        
        if long == 0 and lat == 0:
                    print('made it home!')
                    return
        while True:
            for i in reversed(range(len(turnstaken))):
                self.turn(turnstaken[i]-heading)
                self.drive()
                [long, lat] = shift(long, lat, heading)
                print(long, lat)
                if long == 0 and lat == 0:
                    print('made it home!')
                    return
        
        
    def sample(self):     # find number of streets coming from an intersection
        results = [False, False, False, False]
        self.setvel(0, 0)
        time.sleep(1.0)
        self.turn(1)
        if self.ircheck() != 0:
            results[heading] = True
        self.turn(3)
        if self.ircheck() != 0:
            results[heading] = True
        self.turn(3)
        if self.ircheck() != 0:
            results[heading] = True
        self.turn(1)
        results[(heading + 2) % 4] = True
            
        return results
    
    def unexplored(self):
        for i in intersections:
            for j in i.streets:
                if j == UNEXPLORED:
                    print('still unexplored')
                    return True
        return False
    
    
        # New longitude/latitude value after a step in the given heading.
def shift(long, lat, heading):
    if heading % 4 == NORTH:
        return (long, lat+1)
    elif heading % 4 == WEST:
        return (long-1, lat)
    elif heading % 4 == SOUTH:
        return (long, lat-1)
    elif heading % 4 == EAST:
        return (long+1, lat)
    else:
        raise Exception('This cant be')
        
# Find the intersection
def intersection(long, lat):
    list = [i for i in intersections if i.long == long and i.lat == lat]
    if len(list) == 0:
        return None
    if len(list) > 1:
        raise Exception("Multiple intersections at (%2d,%2d)" % (long, lat))
    return list[0]
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
            
            motors.drive()
            
            [long, lat] = shift(long, lat, heading)
            print(len(intersections))
            if intersection(long, lat) == None:
                inter = Intersection(long, lat)
                temp = motors.sample()
                for i in range(4):
                    if temp[i] == True:
                        inter.streets[i] = UNEXPLORED
                    else:
                        inter.streets[i] = NOSTREET                   
            else:
                inter = intersection(long, lat)
                
            if lastintersection != None:
                lastintersection.streets[heading] = CONNECTED
                inter.streets[(heading+2)%4] = CONNECTED
                inter.headingToTarget = (heading+2)%4
                turnstaken.append(inter.headingToTarget)
            streetind = []
            streetcnct = []
            for i in range(len(inter.streets)):
                if inter.streets[i] == UNEXPLORED:
                    streetind.append(i)
                elif inter.streets[i] == CONNECTED:
                    streetcnct.append(i)
            if len(streetind) == 0:
                #check for any unexplored streets on the map
                if motors.unexplored():   
                    motors.turn(random.choice(streetcnct) - heading)
                else:
                    motors.goHome()
                    motors.shutdown()
                                
            else:
                motors.turn(streetind[0]-heading)
            print(repr(inter))
            lastintersection = inter
            
            
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
    
    motors.shutdown()



