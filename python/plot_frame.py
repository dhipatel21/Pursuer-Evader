import os
import sys

import lcm
import matplotlib.pyplot as plt
import numpy as np

from lcmtypes import odometry_t, robot_path_t, timestamp_t, pose_xyt_t


def is_between(a, b, c):
    return a <= c <= b or b <= c <= a


sys.path.append("lcmtypes")

WHEEL_BASE = 0.165
WHEEL_DIAMETER = 0.084
GEAR_RATIO = 78
ENCODER_RES = 20
enc2meters = WHEEL_DIAMETER * np.pi / (GEAR_RATIO * ENCODER_RES)

if len(sys.argv) < 2:
    sys.stderr.write("usage: plot_square.py <logfile>")
    sys.exit(1)

file = sys.argv[1]
log = lcm.EventLog(file, "r")

# path_data = []
# path_init = 0

command_init = 0

odometry_data = np.empty((0, 4), dtype=float)
odometry_init = 0

timesync_data = np.empty((0, 1), dtype=int)

for event in log:
    # if event.channel == "CONTROLLER_PATH":
    #     path_msg = robot_path_t.decode(event.data)

    #     if (path_init == 0):
    #         path_start_utime = path_msg.utime
    #         print("path_start_utime: {}".format(path_start_utime))
    #     path_data = path_msg.path


    if event.channel == "ODOMETRY":
        odometry_msg = odometry_t.decode(event.data)

        if command_init == 0:
            odometry_start_utime = odometry_msg.utime
            print("odometry_start_utime: {}".format(odometry_start_utime))
            command_init = 1
        odometry_data = np.append(odometry_data, np.array([[
            (odometry_msg.utime - odometry_start_utime)/1.0E6,
            odometry_msg.x,
            odometry_msg.y,
            odometry_msg.theta
        ]]), axis=0)

    if event.channel == "MBOT_TIMESYNC":
        timesync_msg = timestamp_t.decode(event.data)
        timesync_data = np.append(timesync_data, np.array([[
            (timesync_msg.utime)/1.0E6,
        ]]), axis=0)

# Path data
# path_x = []
# path_y = []
# for i in range(0, len(path_data)):
#     path_x.append(path_data[i].x)
#     path_y.append(path_data[i].y)

# Odometry data
odometry_time = odometry_data[:, 0]
enc_time_diff = np.diff(odometry_time)
odom_x = odometry_data[:, 1]
odom_y = odometry_data[:, 2]
odom_theta = odometry_data[:, 3]

fig, axs = plt.subplots(1, 2, sharey=False, figsize=(10, 10))

# Plot everything

# Left wheel
axs[0].plot(odometry_time, odom_x,
               c='b', label="Robot Frame X vs t")
axs[0].set_xlabel("Time (s)")
axs[0].set_ylabel("Robot X-Position (m)")
axs[0].set_title("Robot X-Position (m) vs. Time (s)")
axs[0].set(ylim=(-2, 2))

axs[1].plot(odometry_time, odom_theta,
               c='b', label="Robot Frame Θ vs t")
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Robot Heading (rad)")
axs[1].set_title("Robot Frame Θ (rad) vs Time (s) ")
axs[1].set(ylim=(-2*3.14159265, 2*3.14159265))

plt.savefig(f"{file}.png")

plt.show()
