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

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=2")

# Initialize the simulation environment
pursuer_initial_position = Vector2D(0, 0)
evader_initial_position = Vector2D(10, 10)
pursuer_speed = 1.0
evader_speed = 1.0

lower_bounds : Vector2D = Vector2D(0, 0)
upper_bounds : Vector2D = Vector2D(10, 10)

evasion_agent = Evasion(evader_initial_position, evader_speed, upper_bounds, lower_bounds)

def update_direction(pursuer : Pursuit, evader : Evasion):
    return (evader.evader_position - pursuer.pursuer_position).normalize()

next_waypoint_evader = evasion_agent.update_evader_random()

