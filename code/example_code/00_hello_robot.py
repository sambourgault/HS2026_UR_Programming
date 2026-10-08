############################################################
##
## 00_hello_robot.py
## author: Sam Bourgault
## date: 2026-10-08
## place: ETH Zurich
## source: inspired by example code from STDU ur_rtde library
## https://gitlab.com/sdurobotics/ur_rtde/-/tree/master/examples/py
##
############################################################

#first you import the necessary libraries
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# you create variables for the rtde_control and rtde_receive interfaces
rtde_c = RTDEControl("169.254.10.10")
rtde_r = RTDEReceive("169.254.10.10")


# print the robot's current TCP pose
# Pose: [x, y, z, rx, ry, rz] in meters and axis-angle [rad]
tcp_pose = rtde_r.getActualTCPPose()
print("TCP pose at home:", tcp_pose[:3])

# quit program
rtde_c.stopScript()
rtde_c.disconnect()
rtde_r.disconnect()



