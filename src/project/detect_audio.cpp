#include <iostream>
#include <portaudio.h>

#define SAMPLE_RATE (44100)
#define FRAMES_PER_BUFFER (256)
#define THRESHOLD (.01)     // will need to change based on testing

// Audio callback function
static int audioCallback(const void *inputBuffer, void *outputBuffer,
                         unsigned long framesPerBuffer,
                         const PaStreamCallbackTimeInfo *timeInfo,
                         PaStreamCallbackFlags statusFlags,
                         void *userData) {
    // Check if there is audio data present in the input buffer
    const float *input = (const float *)inputBuffer;
    for (unsigned int i = 0; i < framesPerBuffer; i++) {
        if (input[i] > THRESHOLD || input[i] < -THRESHOLD) {
            std::cout << "audio detected\n";
            return paContinue; // Audio detected, continue capturing
        }
    }
    std::cout << "no audio\n";
    return paContinue; // No audio detected, continue capturing
}

// Function to detect audio input
void detectAudioInput() {
    PaError err;
    PaStream *stream;

    // Initialize PortAudio
    err = Pa_Initialize();
    if (err != paNoError) {
        std::cerr << "Error: PortAudio initialization failed\n";
        return;
    }

    // Open audio stream for capturing from the default input device
    err = Pa_OpenDefaultStream(&stream, 1, 0, paFloat32, SAMPLE_RATE,
                               FRAMES_PER_BUFFER, audioCallback, NULL);
    if (err != paNoError) {
        std::cerr << "Error: Failed to open audio stream\n";
        Pa_Terminate();
        return;
    }

    // Start audio stream
    err = Pa_StartStream(stream);
    if (err != paNoError) {
        std::cerr << "Error: Failed to start audio stream\n";
        Pa_CloseStream(stream);
        Pa_Terminate();
        return;
    }

    // Wait for audio input to be detected
    std::cout << "Listening for audio input...\n";
    while (true) {
        // Sleep for a short duration to conserve CPU
        Pa_Sleep(100);
    }

    // Stop and close audio stream
    Pa_StopStream(stream);
    Pa_CloseStream(stream);

    // Terminate PortAudio
    Pa_Terminate();
}

int main() {
    // Detect audio input from USB microphone
    detectAudioInput();

    return 0;
}
