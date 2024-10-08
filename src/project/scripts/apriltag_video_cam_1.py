#!/usr/bin/env python

from argparse import ArgumentParser
import os
import cv2
import apriltag
import numpy as np
import subprocess
import sys
import time

import lcm
from lcmtypes import pose_xyt_t
from lcm import LCM

################################################################################
threshold = 0.4  # TODO: Adjust based on testing
play_audio = 0

global last_msg
last_msg = None

import subprocess

def calculate_distance(i, poses):
    tx = poses[i][0][0][3]
    ty = poses[i][0][1][3]
    tz = poses[i][0][2][3]
    # print(tx, ' ', ty, ' ', tz)
    distance = np.sqrt(tx**2 + ty**2 + tz**2)
    return distance

def cam_angles(i, poses):
    tx = poses[i][0][0][3]
    # ty = poses[i][0][1][3]
    tz = poses[i][0][2][3]

    angle = np.arctan2(tx, tz)
    return angle

def apriltag_video(input_streams=[1], # For default cam use -> [0]
                   output_stream=False,
                   display_stream=False,
                   detection_window_name='AprilTag',
                   play_sound=False
                  ):

    '''
    Detect AprilTags from video stream.

    Args:   input_streams [list(int/str)]: Camera index or movie name to run detection algorithm on
            output_stream [bool]: Boolean flag to save/not stream annotated with detections
            display_stream [bool]: Boolean flag to display/not stream annotated with detections
            detection_window_name [str]: Title of displayed (output) tag detection window
    '''

    parser = ArgumentParser(description='Detect AprilTags from video stream.')
    apriltag.add_arguments(parser)
    options = parser.parse_args()

    global last_msg

    '''
    Set up a reasonable search path for the apriltag DLL.
    Either install the DLL in the appropriate system-wide
    location, or specify your own search paths as needed.
    '''
    lcm = LCM()

    detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())

    for stream in input_streams:

        video = cv2.VideoCapture(stream)

        output = None

        if output_stream:
            width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = int(video.get(cv2.CAP_PROP_FPS))
            codec = cv2.VideoWriter_fourcc(*'XVID')
            if type(stream) != int:
                output_path = '../media/output/'+str(os.path.split(stream)[1])
                output_path = output_path.replace(str(os.path.splitext(stream)[1]), '.avi')
            else:
                output_path = '../media/output/'+'camera_'+str(stream)+'.avi'
            output = cv2.VideoWriter(output_path, codec, fps, (width, height))

        while(video.isOpened()):

            success, frame = video.read()
            if not success:
                break

            result, overlay, detections, poses = apriltag.detect_tags(frame,
                                                   detector,
                                                   camera_params=(418.6614059, 419.0883449, 323.88311642, 242.64846068),
                                                   tag_size=0.0762,
                                                   vizualization=3,
                                                   verbose=3,
                                                   annotation=True
                                                  )
            
            if len(detections) > 0:
                for i, detection in enumerate(detections):
                    distance = calculate_distance(i, poses)
                    print("Distance to AprilTag ", detection.tag_id, ': ', distance)
                    angle = cam_angles(i, poses)
                    print("Angle to AprilTag ", detection.tag_id, ': ', angle)

                    # Message Handling
                    msg = pose_xyt_t()

                    msg.utime = int(time.time())
                    msg.x = distance
                    msg.theta = angle
                    msg.y = 1
                    
                    last_msg = msg
                    lcm.publish("CAMERA_1_CHANNEL", msg.encode())

                    if distance < threshold:
                        msg = pose_xyt_t()
                        msg.x = 1
                        msg.y = 1
                        msg.theta = 1
                        msg.utime = 1
                        play_audio = 1
                        lcm.publish("SHUTDOWN_CHANNEL", msg.encode())
                        print("Threshold Reached! Distance to AprilTag ", detection.tag_id, ': ', distance)

                        subprocess.run(["aplay", "4khz.wav"])
                        video.release()
                        break
            
            else:
                    msg = pose_xyt_t()

                    msg.utime = int(time.time())
                    msg.x = 0
                    msg.theta = 0
                    msg.y = -1

                    lcm.publish("CAMERA_1_CHANNEL", msg.encode())

            if output_stream:
                output.write(overlay)

            if display_stream:
                cv2.imshow(detection_window_name, overlay)
                if cv2.waitKey(1) & 0xFF == ord(' '): # Press space bar to terminate
                    break

################################################################################

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("-c", "--camera", type=int, default=1)
    parser.add_argument("-o", "--output", type=int, default=0)
    parser.add_argument("-d", "--display", type=int, default=1)
    parser.add_argument("-s", "--sound", type=int, default=0)

    args = parser.parse_args()
    cam = [args.camera]
    output = (args.output == 1)
    disp = (args.display == 1)
    sound = (args.sound == 1)

    apriltag_video(input_streams=cam, output_stream=output, display_stream=disp, play_sound=sound)
