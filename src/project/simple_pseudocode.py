while Polo drives around:
    every 8 seconds play audio
    keep running away --> evader algo
    if (end condition sound is heard):
        shutdown

while Marco drives around:
    continuously listen to mic and run apriltag detection
    follow path based on direction of audio --> pursuer algo
    if (distance from apriltag < threshold):
        emit end condition sound
        shutdown