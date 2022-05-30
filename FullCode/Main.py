from Intersection import *
from Ultrasonic import *
from Motors import *
from GlobalVar import *

#define global variables
turnstaken = []  # list of turns taken
lastintersection = None  # Last intersection visited
long = 0  # Current east/west coordinate
lat = -1  # Current north/south coordinate
heading = 0  # Current heading

if __name__ == "__main__":

    io = pigpio.pi()
    motors = Motor(io)

    ultra1 = Ultrasonic(io, CHANNEL_TRIGGER_1, CHANNEL_ECHO_1, 1)
    ultra2 = Ultrasonic(io, CHANNEL_TRIGGER_2, CHANNEL_ECHO_2, 2)
    ultra3 = Ultrasonic(io, CHANNEL_TRIGGER_3, CHANNEL_ECHO_3, 3)

    thread = threading.Thread(target=runcontinual, args=(ultra1, ultra2, ultra3))
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
            u = -k * e
            PWMleft = max(0.5, min(0.9, 0.7 - u))
            PWMright = max(0.5, min(0.9, 0.7 + u)) + 0.025
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
