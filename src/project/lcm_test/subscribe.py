import sys
import lcm
sys.path.append("../../lcmtypes")
from pose_xyt_t import pose_xyt_t
from lcm import LCM
from lcmtypes import pose_xyt_t

def handler(channel, data):
    message = cam_t.decode(data)

    return message

lcm = LCM()
subscription_1 = lcm.subscribe("cam", handler)

# subscription_2 = lcm.subscribe("cam_2", handler)

# subscription_3 = lcm.subscribe("cam_3", handler)

while True:
    lcm.handle()