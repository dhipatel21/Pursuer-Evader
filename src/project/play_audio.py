import subprocess

def play_audio(file_path, device):
    # Construct the command
    command = ["aplay", "--device=" + device, file_path]

    # Execute the command
    subprocess.run(command)

# Example usage:
file_path = "piano2.wav"    # replace with our actual audio files
device = "hw:0,0"
play_audio(file_path, device)