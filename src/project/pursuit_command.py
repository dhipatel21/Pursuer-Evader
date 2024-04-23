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

THRESHOLD = 0.001

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=2")

def pursuit_handler(channel, data):
    global continue_pursuit
    cam_msg = pose_xyt_t.decode(data)

    evader_direction = cam_msg.theta # TODO : 3 Camera logic
    evader_vector = Vector2D(cam_msg.x, cam_msg.y)
    evader_distance = evader_vector.magnitude()

    next_waypoint_pursuer = pursuit_agent.update_pursuer_converging_chase(evader_vector.normalize())

    if evader_distance < THRESHOLD:
        # publish shutdown sequence
        continue_pursuit = False
        lc.unsubscribe(subscription_1)
    
    # publish waypoint

# Initialize the simulation environment
pursuer_initial_position = Vector2D(0, 0)
evader_initial_position = Vector2D(10, 10)
pursuer_speed = 1.0
evader_speed = 1.0

lower_bounds : Vector2D = Vector2D(0, 0)
upper_bounds : Vector2D = Vector2D(10, 10)

pursuit_agent = Pursuit(pursuer_initial_position, pursuer_speed, evader_initial_position, evader_speed, upper_bounds, lower_bounds)

# Begin
subscription_1 = lc.subscribe("CAMERA_CHANNEL", pursuit_handler)

try:
    while continue_pursuit:
        lc.handle()
except KeyboardInterrupt:
    pass

# While (! shut down sequence):
#   receieve lcm evader direction
#   get pursuit waypoint (evader direction)
#   publish pursuit waypoint to c++