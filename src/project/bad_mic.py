import sounddevice as sd
import numpy as np

import time
from bad_mic_msg import bad_mic_msg
from lcm import LCM

# Define constants
RATE = 44100  # Sampling rate
BLOCK_SIZE = 1024  # Block size for audio input
THRESHOLD_FREQUENCY = 3000  # TODO: Adjust based on testing, Frequency threshold in Hz

# Callback function for audio input
def audio_callback(indata, frames, time, status):
    # Calculate FFT (Fast Fourier Transform)
    fft_data = np.fft.fft(indata[:, 0])
    frequencies = np.fft.fftfreq(len(fft_data), 1.0 / RATE)

    # Find the dominant frequency
    dominant_frequency = frequencies[np.argmax(np.abs(fft_data))]
    if dominant_frequency >= THRESHOLD_FREQUENCY:
        print(f"Sound detected at frequency: {dominant_frequency:.2f} Hz")

        # Message Handling
        lcm = LCM()
        msg = bad_mic_msg()
        msg.timestamp = int(time.time() * 1000000)  # Current timestamp in microseconds
        msg.frequency = dominant_frequency

        lcm.publish("bad mic", msg.encode())

# Main function
def main():
    # Find the index of the USB microphone
    mic_index = 1
    # print(sd.query_devices())     # use to find mic index
    # Start audio input stream
    with sd.InputStream(device=mic_index, channels=1, samplerate=RATE,
                         blocksize=BLOCK_SIZE, callback=audio_callback):
        print("Listening for audio...")
        input("Press Enter to exit.")
    
if __name__ == "__main__":
    main()
