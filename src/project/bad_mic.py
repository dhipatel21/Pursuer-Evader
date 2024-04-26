import sounddevice as sd
import numpy as np
import time
import sys
from lcm import LCM
from lcmtypes import mbot_motor_command_t, pose_xyt_t

# Define constants
RATE = 44100  # Sampling rate
BLOCK_SIZE = 1024  # Block size for audio input
THRESHOLD_FREQUENCY = 3500  # TODO: Adjust based on testing, Frequency threshold in Hz

# Callback function for audio input
def audio_callback(indata, frames, time, status):
    # Calculate FFT (Fast Fourier Transform)
    fft_data = np.fft.fft(indata[:, 0])
    frequencies = np.fft.fftfreq(len(fft_data), 1.0 / RATE)

    # Find the dominant frequency
    dominant_frequency = frequencies[np.argmax(np.abs(fft_data))]

    # Message Handling
    lcm = LCM()
    msg = pose_xyt_t()
    msg.x = dominant_frequency

    lcm.publish("BAD_MICROPHONE_CHANNEL", msg.encode())
    
    if dominant_frequency >= THRESHOLD_FREQUENCY:
        print(dominant_frequency)
        sd.stop()
        msg = pose_xyt_t()
        msg.x = 1
        msg.y = 1
        msg.theta = 1
        msg.utime = 1

        lcm.publish("PE_SHUTDOWN", msg.encode())

# Main function
def main():
    # Find the index of the USB microphone
    mic_index = -1
    # print(sd.query_devices())     # use to find mic index
    for i in range(len(sd.query_devices())):
        str = "USB PnP Sound Device: Audio (hw:0,0)"
        if (sd.query_devices()[i]["name"] == str):
                mic_index = i
                break
    
    if mic_index == -1:
        exit(1)
    # Start audio input stream
    with sd.InputStream(device=mic_index, channels=1, samplerate=RATE,
                         blocksize=BLOCK_SIZE, callback=audio_callback):
        print("Listening for audio...")
        input("Press Enter to exit.")
    
if __name__ == "__main__":
    main()
