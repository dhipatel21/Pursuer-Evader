from message import message # IMPORT MESSAGE OBJECTS
from lcm import LCM # IMPORT LCM

def message_handler(channel, data):
    msg = message.decode(data) # DECODE 
    print(f"Received message: {msg.name}, value: {msg.value}")

def main():
    lcm = LCM()
    lcm.subscribe("example_channel", message_handler) # MESSAGE HANDLER FUNCTION RUNS WHEN A MESSAGE IS PUBLISHED TO CHANNEL

    try:
        while True:
            lcm.handle() # EVERYTIME LCM HANDLE IS CALLED, THE SUBSCRIBED CHANNEL IS CHECKED
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()