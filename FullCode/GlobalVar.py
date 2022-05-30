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

# # Global Variables:
# intersections = []  # List of intersections
# turnstaken = []  # list of turns taken
# lastintersection = None  # Last intersection visited
# long = 0  # Current east/west coordinate
# lat = -1  # Current north/south coordinate
# heading = 0  # Current heading

# last_trigger = 0
# rise_ticks = 0
# dtick = 0
stopflag = False

cur_dist = [0,0,0]
