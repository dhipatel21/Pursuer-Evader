# make sure to run:
    # $ sudo pip install hid 
    # $ sudo pip install hidapi

import hid

mic = hid.Device(0x2752, 0x1C)

count = 0
while (True):
    a = mic.read(60)
    if (a[0] == 0x06 and a[1] == 0x36):
        print(f"\rVAD: {a[2]} Angle: {a[3]*255+a[4]} Dir: {a[5]}", end="", flush=True)
        count += 1
        print(count)
        if (a[2] == 0):
            count = 0
        if count > 1:   # attempt to make sure we follow the correct sound, bass boost/distortion works best
            print("moving")
        # If VAD == 1 then voice is detected
        # Angle ranges from 0-360 with