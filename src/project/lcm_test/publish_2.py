from lcm import LCM
import time
from cam_t import cam_t

lcm = LCM()

i = 0
while True:
    i = i + 1
    if i % 3 == 0:
        message = cam_t()
        message.distance = 2
        lcm.publish("test", message.encode())

        time.sleep(1)
    else:
        message = cam_t()
        message.distance = -1
        lcm.publish("test", message.encode())
        time.sleep(1)