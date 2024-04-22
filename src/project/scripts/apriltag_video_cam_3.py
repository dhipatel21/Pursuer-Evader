#!/usr/bin/env python

from argparse import ArgumentParser
import os
import cv2
import apriltag
import numpy as np
import subprocess

import time
from apriltag_msg import apriltag_msg
from lcm import LCM

################################################################################
threshold = 0.2  # TODO: Adjust based on testing

import wave
import pyaudio

def play_wav(file_path):
    # Open the WAV file
    wf = wave.open(file_path, 'rb')

    # Initialize PyAudio
    p = pyaudio.PyAudio()

    # Open a stream for playback
    stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)

    # Read data from the WAV file and play it
    data = wf.readframes(1024)
    while data:
        stream.write(data)
        data = wf.readframes(1024)

    # Stop and close the stream
    stream.stop_stream()
    stream.close()

    # Terminate PyAudio
    p.terminate()

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

def apriltag_video(input_streams=[3], # For default cam use -> [0]
                   output_stream=False,
                   display_stream=True,
                   detection_window_name='AprilTag',
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
                    # print("Distance to AprilTag ", detection.tag_id, ': ', distance)
                    angle = cam_angles(i, poses)
                    # Message Handling
                    msg = apriltag_msg()
                    msg.timestamp = int(time.time() * 1000000)  # Current timestamp in microseconds
                    msg.distance = distance

                    lcm.publish("april tag", msg.encode()) # TODO : Handle off command in algo after receiving distance?

                    # if distance < threshold:
                    #     # TODO: send turn off command over LCM
                    #     print("Threshold Reached! Distance to AprilTag ", detection.tag_id, ': ', distance)
                    #     play_wav('3khz.wav')   # replace with actual end condition sound

            if output_stream:
                output.write(overlay)

            if display_stream:
                cv2.imshow(detection_window_name, overlay)
                if cv2.waitKey(1) & 0xFF == ord(' '): # Press space bar to terminate
                    break

################################################################################

if __name__ == '__main__':
    apriltag_video()
