# make sure to run:
    # $ sudo pip install hid 
    # $ sudo pip install hidapi

import hid
import time
import sounddevice as sd
import numpy as np
from lcmtypes import pose_xyt_t
from lcm import LCM

# Define constants
RATE = 48000  # Sampling rate
BLOCK_SIZE = 1024  # Block size for audio input
THRESHOLD_FREQUENCY = 0  # TODO: Adjust based on testing, Frequency threshold in Hz

try:
    mic = hid.Device(0x2752, 0x1C)
except:
    print("Unable to find mic")
    exit()

count = 0

lcm = LCM()

# Callback function for audio input
def audio_callback(indata, frames, time, status):
    a = mic.read(60)
    if (a[0] == 0x06 and a[1] == 0x36):
        # if (a[2] == 1):
        print(f"\rVAD: {a[2]} Angle: {a[3]*255+a[4]} Dir: {a[5]}", end="", flush=True)
        fft_data = np.fft.fft(indata[:, 0])
        frequencies = np.fft.fftfreq(len(fft_data), 1.0 / RATE)

        # Find the dominant frequency
        dominant_frequency = frequencies[np.argmax(np.abs(fft_data))]
        
        if dominant_frequency >= THRESHOLD_FREQUENCY:
            # Message handling
            msg = pose_xyt_t()
            msg.theta = a[3]*255+a[4]

            lcm.publish("good mic", msg.encode())

            print(f" Sound detected at frequency: {dominant_frequency:.2f} Hz")

        with sd.InputStream(device=0, channels=1, samplerate=RATE,
                            blocksize=BLOCK_SIZE, callback=audio_callback):

            print("Listening for audio...")
            input("Press Enter to exit.")