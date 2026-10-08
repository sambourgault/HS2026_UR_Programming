############################################################
##
## 01_align_robot_to_table.py
## author: Sam Bourgault
## date: 2026-10-08
## place: ETH Zurich
## source: inspired by example code from STDU ur_rtde library
## https://gitlab.com/sdurobotics/ur_rtde/-/tree/master/examples/py
##
############################################################

# first you import the necessary libraries
import math
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# you create variables for the rtde_control and rtde_receive interfaces
rtde_c = RTDEControl("169.254.10.10")
rtde_r = RTDEReceive("169.254.10.10")

# print the robot's current TCP pose
# Pose: [x, y, z, rx, ry, rz] in meters and axis-angle [rad]
tcp_pose = rtde_r.getActualTCPPose()
print("TCP pose at home:", tcp_pose[:3])

# keep the x, y, z coordinates but rewrite the orientation vectors
# for a tool pointing straight down relative to the robot base:
aligned_pose = [
    tcp_pose[0],      # x
    tcp_pose[1],      # y
    tcp_pose[2],      # z
    math.pi,          # rx (180 degrees in radians)
    0.0,              # ry
    0.0               # rz 
]

# move the robot linearly to the aligned pose
# arguments: target_pose, speed (m/s), acceleration (m/s^2)
rtde_c.moveL(aligned_pose, 0.05, 0.1)

# quit program
rtde_c.stopScript()
rtde_c.disconnect()
rtde_r.disconnect()
print("Program completed")