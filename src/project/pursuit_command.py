from pursuit import Pursuit
from evasion import Evasion
from Vector2D import Vector2D

import pygame
from pygame.locals import *
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import sys
from lcm import LCM
from lcmtypes import mbot_motor_command_t, pose_xyt_t

THRESHOLD = 0.001

lc = LCM("udpm://239.255.76.67:7667?ttl=2")

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

def camera_1_handler(channel, data):
    global continue_pursuit
    global evader_direction
    cam_msg = pose_xyt_t.decode(data)

    evader_direction = cam_msg.theta
    evader_distance = cam_msg.x

    if evader_distance < THRESHOLD:
        msg = pose_xyt_t()
        msg.x = 1
        msg.y = 1
        msg.theta = 1
        msg.utime = 1

        lc.publish("PE_SHUTDOWN", msg.encode())
        play_wav("3khz.wav")
        continue_pursuit = False
        lc.unsubscribe(subscription_cam_1)
    

def camera_2_handler(channel, data):
    global continue_pursuit
    global evader_direction
    cam_msg = pose_xyt_t.decode(data)

    evader_direction = cam_msg.theta
    evader_distance = cam_msg.x

    if evader_distance < THRESHOLD:
        msg = pose_xyt_t()
        msg.x = 1
        msg.y = 1
        msg.theta = 1
        msg.utime = 1

        lc.publish("PE_SHUTDOWN", msg.encode())
        continue_pursuit = False
        play_wav("3khz.wav")
        lc.unsubscribe(subscription_cam_2)
    

def camera_3_handler(channel, data):
    global continue_pursuit
    global evader_direction
    cam_msg = pose_xyt_t.decode(data)

    evader_direction = cam_msg.theta
    evader_distance = cam_msg.x

    if evader_distance < THRESHOLD:
        msg = pose_xyt_t()
        msg.x = 1
        msg.y = 1
        msg.theta = 1
        msg.utime = 1

        lc.publish("PE_SHUTDOWN", msg.encode())
        play_wav("3khz.wav")
        continue_pursuit = False
        lc.unsubscribe(subscription_cam_3)

def good_mic_handler(channel, data):
    global continue_pursuit
    global evader_direction
    mic_msg = pose_xyt_t.decode(data)

    evader_direction = mic_msg.theta
    

# Initialize the simulation environment
pursuer_initial_position = Vector2D(0, 0)
evader_initial_position = Vector2D(10, 10)
pursuer_speed = 1.0
evader_speed = 1.0

lower_bounds : Vector2D = Vector2D(0, 0)
upper_bounds : Vector2D = Vector2D(10, 10)

pursuit_agent = Pursuit(pursuer_initial_position, pursuer_speed, evader_initial_position, evader_speed, upper_bounds, lower_bounds)

# Begin
subscription_cam_1 = lc.subscribe("CAMERA_1_CHANNEL", camera_1_handler)
subscription_cam_2 = lc.subscribe("CAMERA_2_CHANNEL", camera_2_handler)
subscription_cam_3 = lc.subscribe("CAMERA_3_CHANNEL", camera_3_handler)
subscription_good_mic = lc.subscribe("GOOD_MICROPHONE_CHANNEL", good_mic_handler)

try:
    while continue_pursuit:
        lc.handle()
        next_waypoint_pursuer = pursuit_agent.update_pursuer_converging_chase(Vector2D(np.cos(evader_direction), np.sin(evader_direction)))
        msg = pose_xyt_t()
        msg.x = next_waypoint_pursuer.x
        msg.y = next_waypoint_pursuer.y
        msg.theta = 0
        msg.utime = 0

        lc.publish("PE_WAYPOINT", msg.encode())
except KeyboardInterrupt:
    pass
