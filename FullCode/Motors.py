
import pigpio
import sys
import time

from Intersection import * 
from GlobalVar import *

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
        self.set(PWM_L, PWM_R)

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
                self.setvel(0.3, 60)

            elif (ir_old == 2 or ir_old == 6 or ir_old == 4) \
                    and ir_curr == 4:  # a lot right
                self.setvel(0.3, -60)

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
