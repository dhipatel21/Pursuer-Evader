import sounddevice as sd
import numpy as np

# Define constants
RATE = 44100  # Sampling rate
BLOCK_SIZE = 1024  # Block size for audio input
THRESHOLD = 1  # TODO: Adjust based on testing
THRESHOLD_FREQUENCY = 1000  # TODO: Adjust based on testing, Frequency threshold in Hz
PRODUCT_ID = 0x08BB  # Replace with the product ID of your USB microphone
VENDOR_ID = 0x2902  # Replace with the vendor ID of your USB microphone

# Callback function for audio input
def audio_callback(indata, frames, time, status):
    # if status:
    #     print(f"Error in audio input: {status}")
    # volume = (np.linalg.norm(indata) * 10) - 0.2  # Calculate volume (RMS amplitude)
    # if volume > THRESHOLD:
    #     print(f"Sound detected: {volume:.2f}")
    if status:
        print(f"Error in audio input: {status}")
    volume = np.linalg.norm(indata) * 10  # Calculate volume (RMS amplitude)
    if volume > THRESHOLD:
        # Calculate FFT (Fast Fourier Transform)
        fft_data = np.fft.fft(indata[:, 0])
        frequencies = np.fft.fftfreq(len(fft_data), 1.0 / RATE)
        # Find the dominant frequency
        dominant_frequency = frequencies[np.argmax(np.abs(fft_data))]
        if dominant_frequency > THRESHOLD_FREQUENCY:
            print(f"Sound detected at frequency: {dominant_frequency:.2f} Hz")

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
