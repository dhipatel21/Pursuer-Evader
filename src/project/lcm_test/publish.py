from lcm import LCM

from lcmtypes import pose_xyt_t

lcm = LCM()

while True:
    message = pose_xyt_t()
    message.x = 10
    lcm.publish("test", message.encode())

