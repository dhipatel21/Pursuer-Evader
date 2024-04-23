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
import lcm
from lcmtypes import mbot_motor_command_t, pose_xyt_t

THRESHOLD = 3000

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=2")

def evasion_handler(channel, data):
    global continue_evasion
    mic_msg = pose_xyt_t.decode(data)

    # TODO: do something with next_waypoint_pursuer
    # TODO: move this to a different location, tie it to time
    next_waypoint_evader = evasion_agent.update_evader_CW()

    if mic_msg.x > THRESHOLD:
        # TODO publish shutdown sequence
        continue_evasion = False
        lc.unsubscribe(subscription_mic)
    

# Initialize the simulation environment
pursuer_initial_position = Vector2D(0, 0)
evader_initial_position = Vector2D(10, 10)
pursuer_speed = 1.0
evader_speed = 1.0

lower_bounds : Vector2D = Vector2D(0, 0)
upper_bounds : Vector2D = Vector2D(10, 10)

evasion_agent = Evasion(evader_initial_position, evader_speed, upper_bounds, lower_bounds)

# Begin
subscription_mic = lc.subscribe("BAD_MICROPHONE_CHANNEL", evasion_handler)

try:
    while continue_evasion:
        lc.handle()
except KeyboardInterrupt:
    pass
