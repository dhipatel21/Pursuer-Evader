import sys
import lcm
from lcmtypes import pose_xyt_t
from lcm import LCM
import time

def handler(channel, data):
    message = pose_xyt_t.decode(data)
    print("hello world!")

    return message

lcm = LCM()
subscription_1 = lcm.subscribe("test", handler)

# subscription_2 = lcm.subscribe("cam_2", handler)

# subscription_3 = lcm.subscribe("cam_3", handler)

while True:
    lcm.handle_timeout(1)