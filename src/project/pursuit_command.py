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
sys.path.append("lcmtypes")
import lcm
sys.path.append("../lcmtypes")
from mbot_motor_command_t import mbot_motor_command_t
from cam_t import cam_t

THRESHOLD = 0.001

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=2")

def pursuit_handler(channel, data):
    global continue_pursuit
    cam_msg = cam_t.decode(data)

    evader_direction = Vector2D(cam_msg.tx, cam_msg.tz) # TODO : 3 Camera logic

    next_waypoint_pursuer = pursuit_agent.update_pursuer_converging_chase(evader_direction)

    if cam_msg.distance < THRESHOLD:
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
subscription_1 = lc.subscribe("cam", pursuit_handler)

try:
    while continue_pursuit:
        lc.handle()
except KeyboardInterrupt:
    pass

# evasion_agent = Evasion(evader_initial_position, evader_speed, upper_bounds, lower_bounds)

# def update_direction(pursuer : Pursuit, evader : Evasion):
#     return (evader.evader_position - pursuer.pursuer_position).normalize()

# next_waypoint_evader = evasion_agent.update_evader_random()

# While (! shut down sequence):
#   receieve lcm evader direction
#   get pursuit waypoint (evader direction)
#   publish pursuit waypoint to c++