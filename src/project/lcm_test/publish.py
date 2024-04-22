from lcm import LCM

from msg import msg

lcm = LCM()
i = 0
while True:
    i += 1
    message = msg()
    message.var = i
    lcm.publish("test", message.encode())

    message.var = 10
    lcm.publish("test 2", message.encode())

