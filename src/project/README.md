### Information flows: Inputs
#### Pursuer
(apriltag_video_cam[1, 2, 3].py) >>> CAMERA_[1, 2, 3]_CHANNEL >>> Angle relative to self, distance relative to self
(good_mic.py) >>> GOOD_MICROPHONE_CHANNEL >>> Angle relative to self, frequency of detected sound

#### Evader
(bad_mic.py) >>> BAD_MICROPHONE_CHANNEL >>> Frequency of detected sound

### Information flows: Processing
#### Pursuer
(pursuit_command.py) >>> PE_WAYPOINT >>> Point to go to

#### Evader
(evade_command.py) >>> PE_WAYPOINT >>> Point to go to

### Information flows: Outputs
#### Pursuer
(play_audio.py) >>> noting, sound >>> Shutdown sound

#### Evader
(play_audio.py) >>> noting, sound >>> Marco sound