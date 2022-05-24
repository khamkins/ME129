#!/usr/bin/env python3
#
#   ultrasound.py
#
#   Basic ultrasound access.
#   pins, to drive and read the ultrasound.
#

# Imports
import pigpio
import time
import traceback


# Global Definitions (Contants) - GPIO pins.
ULTRA1_TRIG = 13
ULTRA2_TRIG = 19
ULTRA3_TRIG = 26

ULTRA1_ECHO = 16
ULTRA2_ECHO = 20
ULTRA3_ECHO = 21


#
#   Ultrasound Object
#
class Ultrasound:
    # Initialization
    def __init__(self, io, chtrig, checho):
        # Remember the parameters.
        self.io     = io
        self.chtrig = chtrig
        self.checho = checho

        # Set up the two pins as output/input.
        io.set_mode(chtrig, pigpio.OUTPUT)
        io.set_mode(checho, pigpio.INPUT)

        # Clear the trigger (just to be safe).
        io.write(chtrig, 0)

        # Set the state and current reading to nothing.
        self.distance = 0.123
        self.risetick = 0

        # Set up the callbacks.
        self.cbs = [self.io.callback(checho, pigpio.RISING_EDGE,  self.rising),
                    self.io.callback(checho, pigpio.FALLING_EDGE, self.falling)]
        
        # Report.
        print("Set ultrasound channels %d-%d" % (chtrig, checho))

    # Cleanup.
    def shutdown(self):
        # Simply cancel the callback functions, waiting until all is done.
        for cb in self.cbs:
            cb.cancel()
        time.sleep(0.1)

    # Callbacks.
    def rising(self, gpio, level, tick):
        self.risetick = tick
    def falling(self, gpio, level, tick):
        self.distance = (0.5 * 0.000343) * float(tick - self.risetick)

    # Trigger a reading
    def trigger(self):
        # Trigger, hold low for 2us, then high for >= 10us.
        self.io.write(self.chtrig, 0)
        time.sleep(0.000002)
        self.io.write(self.chtrig, 1)
        time.sleep(0.000010)
        self.io.write(self.chtrig, 0)

    # Grab the latest reading.
    def reading(self):
        return self.distance


#
#   Main
#
if __name__ == "__main__":

    ############################################################
    # Prepare the GPIO connection
    print("Setting up the GPIO...")
    
    # Initialize the connection to the pigpio daemon (GPIO interface).
    io = pigpio.pi()
    if not io.connected:
        raise Exception("Unable to connection to pigpio daemon!")
    print("GPIO connected...")

    ############################################################
    # Create the ultrasound
    print("Preparing the ultrasounds...")
    ultra1 = Ultrasound(io, ULTRA1_TRIG, ULTRA1_ECHO)
    ultra2 = Ultrasound(io, ULTRA2_TRIG, ULTRA2_ECHO)
    ultra3 = Ultrasound(io, ULTRA3_TRIG, ULTRA3_ECHO)

    ultra = ultra1

    ############################################################
    # Grab lots of readings.
    print("Running...")
    try:
        while True:
            # Trigger a reading:
            ultra.trigger()

            # Wait.
            time.sleep(0.1)

            # Print
            reading = ultra.reading()
            print("Distance = %6.3fm" % reading)

    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    ############################################################
    # Shutdown.
    print("Disconnecting the ultrasounds...")
    ultra1.shutdown()
    ultra2.shutdown()
    ultra3.shutdown()

    ############################################################
    # Turn Off.
    print("Shutting down...")
    io.stop()
