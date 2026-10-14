############################################################
##
## 04_load_robot_path.py
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
from compas.data import json_load
from compas.geometry import Frame, Transformation
from utils.frame_to_pose import frame_to_pose
from utils.pose_to_frame import pose_to_frame

# you create variables for the rtde_control and rtde_receive interfaces
rtde_c = RTDEControl("169.254.10.10")
rtde_r = RTDEReceive("169.254.10.10")

# get the robot's current TCP pose
tcp_pose = rtde_r.getActualTCPPose()
print("TCP pose at home:", tcp_pose[:3])
tcp_frame = pose_to_frame(tcp_pose)

# load the robot path from a JSON file
robot_path = json_load("data/robot_toolpath.json")
workobject = robot_path["workobject"]
frames = robot_path["frames"]
print("Loaded workobject:", workobject)

# COMPAS/Rhino path coordinates are in millimetres; UR RTDE expects metres.
mm_to_m = 0.001
workobject = Frame(
    [coordinate * mm_to_m for coordinate in workobject.point],
    workobject.xaxis,
    workobject.yaxis,
)
frames = [
    Frame(
        [coordinate * mm_to_m for coordinate in frame.point],
        frame.xaxis,
        frame.yaxis,
    )
    for frame in frames
]

# move the workobject origin to the current TCP point while keeping its axes
moved_workobject = Frame(tcp_frame.point, workobject.xaxis, workobject.yaxis)
t = Transformation.from_frame_to_frame(
    workobject,
    moved_workobject,
)

# convert the frames to a list of poses (x, y, z, rx, ry, rz)
speed = 0.005
acceleration = 0.05
blend_radius = 0.0005

path = []

for index, frame in enumerate(frames):
    robot_frame = frame.transformed(t)
    pose = frame_to_pose(robot_frame)

    # Stop precisely at the last waypoint.
    blend = 0.0 if index == len(frames) - 1 else blend_radius
    path.append(pose + [speed, acceleration, blend])

    print("Loaded pose:", pose)

rtde_c.moveL(path)
