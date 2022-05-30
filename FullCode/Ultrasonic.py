
import pigpio
import time

from GlobalVar import *

last_trigger = 0
rise_ticks = 0
dtick = 0

class Ultrasonic:

    def __init__(self, io, trig, echo, sens_dir):
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
        # if dtick < 0:
        #   dtick += (1 << 32)
        self.curr_distance = 343.0 / 2.0 * dtick * 10.0 ** (-4.0)
        cur_dist[self.sens_dir - 1] = self.curr_distance

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
