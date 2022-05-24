#!/usr/bin/env python
# 
#   Ultrasonic.py adds an ultrasonic class
#
# 

# Imports
import pigpio
import sys
import time
import random
import traceback
import statistics
import threading

# Define the motor pins.
MTR1_LEGA = 7
MTR1_LEGB = 8
MTR2_LEGA = 5
MTR2_LEGB = 6

# define Ultrasonic codes
CHANNEL_TRIGGER_1 = 13
CHANNEL_ECHO_1 = 16

CHANNEL_TRIGGER_2 = 19
CHANNEL_ECHO_2 = 20

CHANNEL_TRIGGER_3 = 26
CHANNEL_ECHO_3 = 21

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
HEADING = {NORTH: 'North', WEST: 'West', SOUTH: 'South', EAST: 'East', None: 'None'}  # For printing

# Street status
UNKNOWN = 'Unknown'
NOSTREET = 'NoStreet'
UNEXPLORED = 'Unexplored'
CONNECTED = 'Connected'

# Global Variables:
intersections = []  # List of intersections
turnstaken = []  # list of turns taken
lastintersection = None  # Last intersection visited
long = 0  # Current east/west coordinate
lat = -1  # Current north/south coordinate
heading = 0  # Current heading

last_trigger = 0
rise_ticks = 0
dtick = 0
stopflag = False

cur_dist = [0,0,0]

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
            raise Exception("Duplicate intersection at (%2d,%2d)" % (long, lat))
        intersections.append(self)
        # Print format.

    def __repr__(self):
        return ("(%2d, %2d) N:%s W:%s S:%s E:%s - head %s\n" %
                (self.long, self.lat, self.streets[0],
                 self.streets[1], self.streets[2], self.streets[3],
                 HEADING[self.headingToTarget]))


class Ultrasonic:

    def __init__(self, io, trig, echo, sens_dir):
#         if not self.io.connected:
#             print("Unable to connection to pigpio daemon!")
#             sys.exit(0)

        # Set up the two pins as output/input.
        io.set_mode(trig, pigpio.OUTPUT)
        io.set_mode(echo, pigpio.INPUT)
        self.io = io
        # Set up the callbacks.
        self.cbrise = io.callback(echo, pigpio.RISING_EDGE, self.rising)
        self.cbfall = io.callback(echo, pigpio.FALLING_EDGE, self.falling)
        
        self.trig = trig
        self.echo = echo
        self.curr_distance = 0
        self.sens_dir = sens_dir

    def rising(self, gpio, level, tick):
        global rise_ticks
        rise_ticks = tick

    def falling(self, gpio, level, tick):
        global dtick
        global cur_dist
        dtick = tick - rise_ticks
        #if dtick < 0:
         #   dtick += (1 << 32)
        self.curr_distance = 343.0 / 2.0 * dtick * 10.0**(-4.0)
        cur_dist[self.sens_dir-1] = self.curr_distance
        
    def trigger(self):
        global last_trigger
        self.io.write(self.trig, 0)
        time.sleep(0.000005)
        self.io.write(self.trig, 1)
        time.sleep(0.000020)
        self.io.write(self.trig, 0)
        last_trigger = time.time
        
        

    def shutdown(self):
        self.cbrise.cancel()
        self.cbfall.cancel()


class Motor:

    def __init__(self, io):

        # Prepare the GPIO connetion (to command the motors).
        print("Setting up the GPIO...")

        # Initialize the connection to the pigpio daemon (GPIO interface).
        self.io = io
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
        if leftdutycycle > 1:
            leftdutycycle = 0.99
        if rightdutycycle > 1:
            rightdutycycle = 0.99
        if leftdutycycle < -1:
            leftdutycycle = -0.99
        if rightdutycycle < -1:
            rightdutycycle = -0.99

        if leftdutycycle > 0:
            self.io.set_PWM_dutycycle(MTR1_LEGA, int(leftdutycycle * 255))
            self.io.set_PWM_dutycycle(MTR1_LEGB, 0)
        else:
            self.io.set_PWM_dutycycle(MTR1_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR1_LEGB, -int(leftdutycycle * 255))

        if rightdutycycle > 0:
            self.io.set_PWM_dutycycle(MTR2_LEGA, 0)
            self.io.set_PWM_dutycycle(MTR2_LEGB, int(rightdutycycle * 255))
        else:
            self.io.set_PWM_dutycycle(MTR2_LEGA, -int(rightdutycycle * 255))
            self.io.set_PWM_dutycycle(MTR2_LEGB, 0)

    def setlinear(self, speed):
        PWM = (speed + 0.153) / 0.738
        self.set(PWM, PWM)

    def setspin(self, speed):
        # set spin based on speed. Positive speed indicates clockwise rotation

        PWM = (abs(speed) + 153.0) / 546.0
        if speed > 0:
            self.set(PWM, -PWM)
        else:
            self.set(-PWM, PWM)

    def setvel(self, linear, spin):
        PWM_fwd = (abs(linear) + 0.153) / 0.738 + 0.2
        if spin > 90 or spin < -90:
            PWM_spin = (abs(spin) + 153.0) / 546.0

        else:
            PWM_spin = (abs(spin)) / 546.0

        if linear == 0:
            PWM_fwd = 0
        if spin == 0:
            PWM_spin = 0

        if linear >= 0 and spin >= 0:
            PWM_L = PWM_fwd + PWM_spin + .05
            PWM_R = PWM_fwd - PWM_spin

        if linear >= 0 and spin <= 0:
            PWM_L = PWM_fwd - PWM_spin + .05
            PWM_R = PWM_fwd + PWM_spin

        if linear < 0 and spin >= 0:
            PWM_L = -PWM_fwd + PWM_spin - .05
            PWM_R = -PWM_fwd - PWM_spin

        if linear < 0 and spin <= 0:
            PWM_L = -PWM_fwd - PWM_spin - .05
            PWM_R = -PWM_fwd + PWM_spin

        # print(PWM_L, PWM_R)
        motors.set(PWM_L, PWM_R)

    def ircheck(self):
        return self.io.read(IR_LEFT) * 4 + self.io.read(IR_MIDDLE) * 2 + self.io.read(IR_RIGHT)

    def lost(self, ir_old):
        print('Lost!')
        degree = 200
        vel = 0
        t = 0
        while self.ircheck() == 0 or self.ircheck() == 1 or self.ircheck() == 4:
            if t == 5000:
                degree = degree - .5
                vel = vel + 0.05
                t = 0
            else:
                t = t + 1
            if ir_old == 1:
                self.setvel(vel, degree)
            else:
                self.setvel(vel, -degree)
            # print(degree)

    # drive forward until you see an intersection
    def drive(self):
        ir_old = self.ircheck()
        while True:
            # read sensors
            ir_curr = self.ircheck()

            if ir_curr == 2:
                self.setvel(0.25, 0)

            elif (ir_old == 2 or ir_old == 3) and ir_curr == 3:  # drifted left
                self.setvel(0.3, 40)

            elif (ir_old == 2 or ir_old == 6) and ir_curr == 6:  # drifted right
                self.setvel(0.3, -40)

            elif (ir_old == 2 or ir_old == 3 or ir_old == 1) \
                    and ir_curr == 1:  # a lot left
                motors.setvel(0.3, 60)

            elif (ir_old == 2 or ir_old == 6 or ir_old == 4) \
                    and ir_curr == 4:  # a lot right
                motors.setvel(0.3, -60)

            elif ir_curr == 0:  # past end
                if ir_old == 4:
                    self.lost(4)
                    ir_curr = self.ircheck()
                else:
                    self.lost(1)
                    ir_curr = self.ircheck()

            # sees all of them
            elif ir_curr == 7:
                # drive straight to get axels over intersection
                self.setvel(0.2, 0)
                time.sleep(0.45)
                self.setvel(0, 0)
                break
            # update past IR status
            ir_old = ir_curr

    def turn(self, dir):
        global heading
        new_dir = dir
        if dir == 1 or dir == -3:
            new_dir = 1
            self.setvel(0, -360)
            time.sleep(.38)
        if dir == 2 or dir == -2:
            new_dir = 2
            self.setvel(0, -360)
            time.sleep(.75)
        if dir == 3 or dir == -1:
            new_dir = 3
            self.setvel(0, 360)
            time.sleep(.38)

        self.setvel(0, 0)
        time.sleep(1)
        # update heading
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
                self.turn(turnstaken[i] - heading)
                self.drive()
                [long, lat] = shift(long, lat, heading)
                print(long, lat)
                if long == 0 and lat == 0:
                    print('made it home!')
                    return

    def sample(self):  # find number of streets coming from an intersection
        results = [False, False, False, False]
        self.setvel(0, 0)
        time.sleep(1.0)
        self.turn(1)
        if self.ircheck() != 0:
            while self.ircheck() != 2:
                if self.ircheck() == 3 or self.ircheck() == 1:  # drifted left
                    self.setvel(0, 200)

                else:  # drifted right
                    self.setvel(0, -200)

            self.setvel(0, 0)
            time.sleep(0.2)

            results[heading] = True

        self.turn(3)
        if self.ircheck() != 0:
            while self.ircheck() != 2:
                if self.ircheck() == 3 or self.ircheck() == 1:  # drifted left
                    self.setvel(0, 200)

                else:  # drifted right
                    self.setvel(0, -200)
            self.setvel(0, 0)
            time.sleep(0.2)
            results[heading] = True

        self.turn(3)
        if self.ircheck() != 0:
            while self.ircheck() != 2:
                if self.ircheck() == 3 or self.ircheck() == 1:  # drifted left
                    self.setvel(0, 200)

                else:  # drifted right
                    self.setvel(0, -200)
            self.setvel(0, 0)
            time.sleep(0.2)
            results[heading] = True

        self.turn(1)
        results[(heading + 2) % 4] = True

        return results

    def unexplored(self):
        for i in reversed(intersections):
            for j in i.streets:
                if j == UNEXPLORED:
                    print('still unexplored')
                    return i
        return None

    def toTarget(self, targetlong, targetlat):
        # clear heading to target
        # global intersections
        global long
        global lat
        for i in intersections:
            i.headingToTarget = None
        unprocessed = []
        curr = intersection(targetlong, targetlat)
        while curr.lat != lat or curr.long != long:
            if curr.streets[0] == CONNECTED:
                new = intersection(curr.long, curr.lat + 1)
                if new.headingToTarget == None:
                    unprocessed.append(new)
                    new.headingToTarget = ((0 + 2) % 4)

            if curr.streets[1] == CONNECTED:
                new = intersection(curr.long - 1, curr.lat)
                if new.headingToTarget == None:
                    unprocessed.append(new)
                    new.headingToTarget = ((1 + 2) % 4)

            if curr.streets[2] == CONNECTED:
                new = intersection(curr.long, curr.lat - 1)
                if new.headingToTarget == None:
                    unprocessed.append(new)
                    new.headingToTarget = ((2 + 2) % 4)

            if curr.streets[3] == CONNECTED:
                new = intersection(curr.long + 1, curr.lat)
                if new.headingToTarget == None:
                    unprocessed.append(new)
                    new.headingToTarget = ((3 + 2) % 4)

            curr = unprocessed.pop(0)

        # follow to target
        while long != targetlong or lat != targetlat:
            inter = intersection(long, lat)
            print(repr(inter))
            self.turn(inter.headingToTarget - heading)
            self.drive()
            time.sleep(0.5)
            [long, lat] = shift(long, lat, heading)

        self.setvel(0, 0)

        # New longitude/latitude value after a step in the given heading.


def shift(long, lat, heading):
    if heading % 4 == NORTH:
        return (long, lat + 1)
    elif heading % 4 == WEST:
        return (long - 1, lat)
    elif heading % 4 == SOUTH:
        return (long, lat - 1)
    elif heading % 4 == EAST:
        return (long + 1, lat)
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

def stopcontinual():
    global stopflag
    stopflag = True
    
def runcontinual(ultra1, ultra2, ultra3):
    global stopflag
    
    stopflag = False
    
    while not stopflag:
        ultra1.trigger()
        ultra2.trigger()
        ultra3.trigger()
        #time.sleep(2)
        time.sleep(0.08 + 0.04 * random.random())
     

#
#   Main
#
if __name__ == "__main__":
    
    io = pigpio.pi()
    motors = Motor(io)
        
    ultra1 = Ultrasonic(io, CHANNEL_TRIGGER_1, CHANNEL_ECHO_1, 1)
    ultra2 = Ultrasonic(io, CHANNEL_TRIGGER_2, CHANNEL_ECHO_2, 2)
    ultra3 = Ultrasonic(io, CHANNEL_TRIGGER_3, CHANNEL_ECHO_3, 3)

    
    thread = threading.Thread(target=runcontinual,args=(ultra1, ultra2, ultra3))
    thread.start()
    
    try:
        lst = []
        i = 0
        avg_dist = cur_dist[1]
        ir_old = motors.ircheck()
            
        
        while True:
            
            print(cur_dist)
            
            d = cur_dist[2]
            d_des = 20
            e = d - d_des
            k = 0.015
            u = -k*e
            PWMleft = max(0.5, min(0.9, 0.7 - u))
            PWMright = max(0.5, min(0.9, 0.7+u)) + 0.025
            print(PWMleft)
            print(PWMright)
            motors.set(PWMleft, PWMright)
            
            
#             if cur_dist[1] <= 20:
#                 print('too close')
#                 motors.setvel(-0.2,0)   
#             elif cur_dist[0] <= 20 and cur_dist[2] > 20:
#                 motors.setvel(0.2, 180)
#             elif cur_dist[0] > 20 and cur_dist[2] <= 20:
#                 motors.setvel(0.2, -180)
#             elif cur_dist[0] <= 20 and cur_dist[2] <= 20:
#                 motors.setvel(0.2, 0)
#                 
#             else:
#                 print('go')
#                 motors.setvel(0.2, 0)
        
            
    
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))

    
    
    stopcontinual()
    thread.join()

    print('motor shutdown')
    motors.shutdown()

    ultra1.shutdown()
    ultra2.shutdown()
    ultra3.shutdown()
