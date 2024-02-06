import lcm
import numpy as np
from lcmtypes import pid_values_t, timestamp_t
import time

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

pid_msg = pid_values_t()


def current_utime(): return int(time.time() * 1e6)


# Motor a (LEFT MOTOR)
pid_msg.motor_a_kp = 2.0
pid_msg.motor_a_ki = 50
pid_msg.motor_a_kd = 0.000
pid_msg.motor_a_Tf = 20.0

# Motor c (RIGHT_MOTOR)
pid_msg.motor_c_kp = 2.0
pid_msg.motor_c_ki = 50
pid_msg.motor_c_kd = 0.000
pid_msg.motor_c_Tf = 20.0

# Translational velocity
pid_msg.bf_trans_kp = 0.9
pid_msg.bf_trans_ki = 0.0
pid_msg.bf_trans_kd = 0.0
pid_msg.bf_trans_Tf = 20.0

# Angular velocity
pid_msg.bf_rot_kp = 0.3
pid_msg.bf_rot_ki = 1.5
pid_msg.bf_rot_kd = 0.0
pid_msg.bf_rot_Tf = 20.0

# We don't use this
pid_msg.motor_b_kp = 0.0
pid_msg.motor_b_ki = 0.0
pid_msg.motor_b_kd = 0.0
pid_msg.motor_b_Tf = 0.0

pub_time = timestamp_t()
pub_time.utime = current_utime()

# lc.publish("MBOT_TIMESYNC", pub_time.encode())
lc.publish("MBOT_PIDS", pid_msg.encode())

print("Published PID values!")
