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

THRESHOLD = 0.4

lc = LCM("udpm://239.255.76.67:7667?ttl=2")

import wave
import pyaudio

global continue_pursuit
continue_pursuit = True

global cam_dt_threshold
cam_dt_threshold = 3

global cam_1_detect
global cam_1_last_detection
cam_1_detect = False
cam_1_last_detection = int(time.time())

CAM_1_OFFSET = 0

evader_direction = 0

global current_x
current_x = 0

global current_y
current_y = 0

global current_theta
current_theta = 0

global evader_distance
evader_distance = 0

def camera_1_handler(channel, data):
    global continue_pursuit
    global evader_distance
    global evader_direction
    global cam_1_detect
    global cam_dt_threshold
    global cam_1_last_detection

    cam_msg = pose_xyt_t.decode(data)

    current_time = int(time.time())

    if (cam_msg.y == -1):
        cam_1_detect = False
        print("No detection on cam 1")
    else:
        evader_direction = cam_msg.theta + CAM_1_OFFSET
        evader_distance = cam_msg.x

        print("INFO: Evader distance: ", evader_distance)
        cam_1_detect = True
        cam_1_last_detection = current_time
        if evader_distance < THRESHOLD:
            print("INFO: Distance within threshold")
            msg = pose_xyt_t()
            msg.x = 1
            msg.y = 1
            msg.theta = 1
            msg.utime = 1
            continue_pursuit = False

            lc.publish("PE_SHUTDOWN", msg.encode())
            lc.unsubscribe(subscription_cam_1)
            sys.exit()

def shutdown_handler(channel, data):
    quit()

def good_mic_handler(channel, data):
    global continue_pursuit
    global evader_direction
    mic_msg = pose_xyt_t.decode(data)

    evader_direction = mic_msg.theta
    
def pose_handler(channel, data):
    global current_x
    global current_y
    global current_theta
    current_pose = pose_xyt_t.decode(data)
    current_x = current_pose.x
    current_y = current_pose.y
    current_theta = current_pose.theta


# Initialize the simulation environment
pursuer_initial_position = Vector2D(0, 0)
evader_initial_position = Vector2D(10, 10)
pursuer_speed = 0.3
evader_speed = 0.3

lower_bounds : Vector2D = Vector2D(0, 0)
upper_bounds : Vector2D = Vector2D(10, 10)

pursuit_agent = Pursuit(pursuer_initial_position, pursuer_speed, evader_initial_position, evader_speed, upper_bounds, lower_bounds)

# Begin
# subscription_cam = lc.subscribe("CAMERA_CHANNEL", camera_handler)
subscription_cam_1 = lc.subscribe("CAMERA_1_CHANNEL", camera_1_handler)
subscription_good_mic = lc.subscribe("GOOD_MICROPHONE_CHANNEL", good_mic_handler)
subscription_pose = lc.subscribe("SLAM_POSE", pose_handler)
subscription_shutdown = lc.subscribe("PE_SHUTODWN", shutdown_handler)

# try:
continue_pursuit = True
while (len(pursuit_agent.pursuer_position_memory) < 2):
    lc.handle_timeout(1)

    current_time = int(time.time())
    cam_1_dt = current_time - cam_1_last_detection
    if not cam_1_detect and (cam_1_dt > cam_dt_threshold):
        print("INFO: STARTUP: No detection on any cameras - executing turn")
        msg = pose_xyt_t()
        msg.x = -1
        msg.y = -1
        msg.theta = -1
        msg.utime = 0
        lc.publish("TURN_TO_SOURCE", msg.encode())
    elif (cam_1_detect):
        print("INFO: Initial waypoints %i", len(pursuit_agent.pursuer_position_memory))
        print(Vector2D(evader_distance * np.cos(evader_direction), evader_distance * np.sin(evader_direction)))
        pursuit_agent.populate_initial_memory(Vector2D(current_x, current_y), Vector2D(evader_distance * np.cos(evader_direction), evader_distance * np.sin(evader_direction)))
    else:
        print("WARNING: INIT: Camera timeout in progress. Current camera dt is at ", cam_1_dt, " seconds")

    time.sleep(1)


while continue_pursuit:
    lc.handle_timeout(1)

    current_time = int(time.time())
    cam_1_dt = current_time - cam_1_last_detection
    if not cam_1_detect and (cam_1_dt > cam_dt_threshold):
        print("INFO: No detection on any cameras - executing turn")
        msg = pose_xyt_t()
        msg.x = -1
        msg.y = -1
        msg.theta = -1
        msg.utime = 0
        lc.publish("TURN_TO_SOURCE", msg.encode())
    else:
        next_waypoint_pursuer = pursuit_agent.update_pursuer_converging_chase(Vector2D(current_x, current_y), Vector2D(np.cos(evader_direction), np.sin(evader_direction)))
        msg = pose_xyt_t()
        msg.x = next_waypoint_pursuer.x
        msg.y = next_waypoint_pursuer.y
        msg.theta = 0
        msg.utime = 10

        print("Evader position: ", Vector2D(evader_distance * np.cos(evader_direction), evader_distance * np.sin(evader_direction)))
        print("Desired pursuer position: ", next_waypoint_pursuer)
        lc.publish("PE_WAYPOINT", msg.encode())

    if not cam_1_detect:
        print("WARNING: MAIN: Camera timeout in progress. Current camera dt is at ", cam_1_dt, " seconds")

    time.sleep(1)

print("Ending pursuit")        
# except KeyboardInterrupt:
#     pass
