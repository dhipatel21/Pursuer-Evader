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
sys.path.append("../lcmtypes")
from mbot_motor_command_t import mbot_motor_command_t
from pose_xyt_t import pose_xyt_t

THRESHOLD = 3000

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=2")

def evasion_handler(channel, data):
    global continue_evasion
    mic_msg = pose_xyt_t.decode(data)

    next_waypoint_evader = evasion_agent.update_evader_CW()

    if mic_msg.frequency > THRESHOLD:
        # publish shutdown sequence to driver
        continue_evasion = False
        lc.unsubscribe(subscription_1)
    
    # publish waypoint to driver

# Initialize the simulation environment
pursuer_initial_position = Vector2D(0, 0)
evader_initial_position = Vector2D(10, 10)
pursuer_speed = 1.0
evader_speed = 1.0

lower_bounds : Vector2D = Vector2D(0, 0)
upper_bounds : Vector2D = Vector2D(10, 10)

evasion_agent = Evasion(evader_initial_position, evader_speed, upper_bounds, lower_bounds)

# pursuit_agent = Pursuit(pursuer_initial_position, pursuer_speed, evader_initial_position, evader_speed, upper_bounds, lower_bounds)

# Begin
subscription_1 = lc.subscribe("bad mic", evasion_handler)

try:
    while continue_evasion:
        lc.handle()
except KeyboardInterrupt:
    pass

# def update_direction(pursuer : Pursuit, evader : Evasion):
#     return (evader.evader_position - pursuer.pursuer_position).normalize()

# evader_direction = update_direction(pursuit_agent, evasion_agent)
# next_waypoint_pursuer = pursuit_agent.update_pursuer_converging_chase(evader_direction)
# evader_direction = update_direction(pursuit_agent, evasion_agent)