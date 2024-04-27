from pursuit import Pursuit
from evasion import Evasion
from Vector2D import Vector2D

from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
import sys
from lcm import LCM
from lcmtypes import mbot_motor_command_t, pose_xyt_t
import subprocess

THRESHOLD = 3500
MIC_MODE = 1

global continue_evasion
continue_evasion = True

global current_x
current_x = 0

global current_y
current_y = 0

global received_state
received_state = False

global high_freq_acc
high_freq_acc = 0

state_memory = []

lc = LCM("udpm://239.255.76.67:7667?ttl=2")

def evasion_handler(channel, data):
    global continue_evasion
    global high_freq_acc
    mic_msg = pose_xyt_t.decode(data)

    if mic_msg.x >= THRESHOLD:
        high_freq_acc += 1
        if (high_freq_acc > 10):
            msg = pose_xyt_t()
            msg.x = 1
            msg.y = 1
            msg.theta = 1
            msg.utime = 1

            lc.publish("PE_SHUTDOWN", msg.encode())
            continue_evasion = False
            lc.unsubscribe(subscription_mic)
            print("=======EXITING NOW=======")
            sys.exit()
        else:
            print("Current status of HF acc? ", high_freq_acc)

def pose_handler(channel, data):
    global current_x
    global current_y
    global received_state

    print("Got a pose")

    current_pose = pose_xyt_t.decode(data)
    current_x = current_pose.x
    current_y = current_pose.y
    received_state = True
    

# Initialize the simulation environment
pursuer_initial_position = Vector2D(0, 0)
evader_initial_position = Vector2D(10, 10)
pursuer_speed = 1
evader_speed = 1

lower_bounds : Vector2D = Vector2D(0, 0)
upper_bounds : Vector2D = Vector2D(10, 10)

evasion_agent = Evasion(evader_initial_position, evader_speed, upper_bounds, lower_bounds)

# Begin
subscription_mic = lc.subscribe("BAD_MICROPHONE_CHANNEL", evasion_handler)
subscription_pose = lc.subscribe("SLAM_POSE", pose_handler)

continue_evasion = True
while (received_state == False):
    lc.handle_timeout(100)
    print("ERROR: Awaiting SLAM pose")
    time.sleep(1)

while continue_evasion and (high_freq_acc < 10):
    lc.handle_timeout(1)

    evasion_agent.evader_position = Vector2D(current_x, current_y)
    state_memory.append(evasion_agent.evader_position)
    distance_traveled = 1
    if (len(state_memory) > 20):
        state_memory = state_memory[-20:]
        distance_traveled = (state_memory[-1] - state_memory[-20]).magnitude()

    leg_done = False
    if (distance_traveled < 1):
        leg_done = True

    next_waypoint_evader = evasion_agent.update_evader_CW(leg_done)
    print("Desired evader position: ", next_waypoint_evader)
    msg = pose_xyt_t()
    msg.x = next_waypoint_evader.x
    msg.y = next_waypoint_evader.y
    msg.theta = 0
    msg.utime = 0

    lc.publish("PE_WAYPOINT", msg.encode())


    if (MIC_MODE == 0):
        time.sleep(1)
    else:
        subprocess.run(["aplay", "hello.wav"])


print("=======EXITING NOW=======")
sys.exit()

