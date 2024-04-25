#!/bin/bash
source setenv.sh
./bin/timesync &> /dev/null &
./bin/shim &> /dev/null &
./bin/rplidar_driver &> /dev/null &
./bin/slam &> /dev/null &
./bin/motion_controller &> /dev/null &
./bin/pursuit &> /dev/null &
python3 ./src/AprilTag/scripts/apriltag_video_cam_1.py &> /dev/null &
python3 ./src/AprilTag/scripts/apriltag_video_cam_2.py &> /dev/null &
python3 ./src/AprilTag/scripts/apriltag_video_cam_3.py &> /dev/null &
python3 ./src/project/pursuit_command.py &> /dev/null &
