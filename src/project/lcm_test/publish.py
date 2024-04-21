import time
from message import message
from lcm import LCM

def main():
    lcm = LCM()

    while True:
        msg = message()
        msg.timestamp = int(time.time() * 1000000)  # Current timestamp in microseconds
        msg.value = 42
        msg.name = "Hello, LCM!"

        lcm.publish("example_channel", msg.encode())
        time.sleep(1)  # Publish message every 1 second

if __name__ == "__main__":
    main()