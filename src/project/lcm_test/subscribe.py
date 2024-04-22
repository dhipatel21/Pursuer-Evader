from lcm import LCM
from lcmtypes import pose_xyt_t

def handler(channel, data):
    message = pose_xyt_t.decode(data)
    print(message.x, message.y, message.theta)

lcm = LCM()
subscription_1 = lcm.subscribe("cam", handler)

# subscription_2 = lcm.subscribe("cam_2", handler)

# subscription_3 = lcm.subscribe("cam_3", handler)

try:
    while True:
        lcm.handle()
except KeyboardInterrupt:
    pass