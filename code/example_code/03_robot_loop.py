############################################################
##
## 03_robot_loop.py
## author: Sam Bourgault
## date: 2026-10-08
## place: ETH Zurich
## source: inspired by example code from STDU ur_rtde library
## https://gitlab.com/sdurobotics/ur_rtde/-/tree/master/examples/py
##
############################################################

# first you import the necessary libraries
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# you create variables for the rtde_control and rtde_receive interfaces
rtde_c = RTDEControl("169.254.10.10")
rtde_r = RTDEReceive("169.254.10.10")

# get the robot's current TCP pose
tcp_pose = rtde_r.getActualTCPPose()
print("TCP pose at home:", tcp_pose[:3])

# initiate variables
robot_TCP_z_move = -0.05

# start main loop
try:
    while True:
        # change robot move direction every loop
        robot_TCP_z_move *= -1

        # move robot up by 5 cm in Z using moveL (Cartesian linear move)
        new_pose = tcp_pose[:]
        new_pose[2] += robot_TCP_z_move
        # arguments for moveL are: pose, speed [m/s], acceleration [m/s^2]
        rtde_c.moveL(new_pose, 0.25, 0.5)

# this will be triggered by a keyboard interrupt (Ctrl+C) to stop the loop
except KeyboardInterrupt:
    print("Program interrupted by user")

finally:
    # return to initial TCP pose
    rtde_c.moveL(tcp_pose, 0.25, 0.5)

    # quit program
    rtde_c.stopScript()
    rtde_c.disconnect()
    rtde_r.disconnect()
    print("Program completed")



