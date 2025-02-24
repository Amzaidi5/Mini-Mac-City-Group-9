import serial
import time
import threading

BAUDRATE = 9600
TIMEOUT = 1 #seconds to wait before assuming no data incoming

RNG_DELAY_MS = 100 #milliseconds between tag-anchor pings (H/W parameter)
DIST_OUTPUT_PERIOD_MS = 1000 #milliseconds between distance outputs

anchor_distance_L = 0 #distance to left anchor
anchor_distance_R = 0 #distance to right anchor

centered = False #flag

tag_port_L = 'COM4' #edit port name as needed
tag_port_R = 'COM3' #edit port name as needed


class UWB_Tag:
    def __init__(self, port, identifier):
        self.port = port #specify port upon creation
        self.id = identifier #specify "left" or "right" tag
        #self.distance_to_anchor = 0 #can change this default value if needed

    #def get_anchor_distance(self):
    #    return self.distance_to_anchor

    def __str__(self):
        return f"{self.id} tag connected to port {self.port}\n"

    def read_anchor_distance(self):
        try:
            S = serial.Serial(self.port, BAUDRATE, timeout = TIMEOUT)
            buffer = [] #accumulates distance readings over 1 second
            counter = 0 #tracks number of readings obtained in 1 second

            while not centered:
                if S.in_waiting > 0:
                    msg = S.readline()

                    try:
                        distance = msg.decode('utf-8').strip()
                        distance = max(0, float(distance)) #treat negative readings as 0 m
                        counter += 1
                        buffer.append(distance)

                        if counter >= DIST_OUTPUT_PERIOD_MS/RNG_DELAY_MS:
                            if self.id == 'left':
                                anchor_distance_L = sum(buffer)/len(buffer)
                                print(anchor_distance_L)
                                buffer = [] #clear
                                counter = 0 #reset

                            elif self.id == 'right':
                                anchor_distance_R = sum(buffer)/len(buffer)
                                print(anchor_distance_R)
                                buffer = [] #clear
                                counter = 0 #reset

                    except UnicodeDecodeError as e_UDE:
                            print(f"Unicode Decode Error: {e_UDE}")

        except serial.SerialException as e_SE:
            print(f"Serial Exception: {e_SE}")

        finally:
            if S.is_open:
                S.close()
                print(f"{self.port} closed")


#worker function 1
def get_left_anchor_distance():
    """
    creates a left UWB_Tag object and runs the method to read distance to left anchor
    """

    tag_L = UWB_Tag(tag_port_L, 'left')
    print(tag_L)
    tag_L.read_anchor_distance()

    return

#worker function 2
def get_right_anchor_distance():
    """
    creates a right UWB_Tag object and runs the method to read distance to right anchor
    """

    tag_R = UWB_Tag(tag_port_R, 'right')
    print(tag_R)
    tag_R.read_anchor_distance()

    return

def centering():
    d_left = threading.Thread(target = get_left_anchor_distance)
    d_right = threading.Thread(target = get_right_anchor_distance)

    d_left.start()
    d_right.start()

    d_left.join()
    d_right.join()

    if anchor_distance_L < anchor_distance_R:
        print("move right")

    if anchor_distance_R < anchor_distance_L:
        print("move left")

    if anchor_distance_L == anchor_distance_R:
        centered = True
        print("centered - yippee!")

    print(f"L: {anchor_distance_L} m, R: {anchor_distance_R} m")
