#include <cstdlib>
#include <string>
#include <iostream>
#include "functions.hpp"

void playWav(const std::string& filePath) {
    // Build the command to play the audio file using aplay
    std::string command = "aplay " + filePath;

    // Execute the command using system
    int result = system(command.c_str());

    // Check the result
    if (result != 0) {
        // Error occurred while playing the audio
        std::cerr << "Error: Failed to play audio file: " << filePath << std::endl;
    }
}

int main() {
    // Example usage:
    for (int i = 0; i < 3; i++){ 
        playWav("piano2.wav");  // replace with our actual audio files
    }
    return 0;
}
