from lcm import LCM

from cam_t import cam_t

lcm = LCM()

while True:
    message = cam_t()
    message.distance = 10
    lcm.publish("test", message.encode())

