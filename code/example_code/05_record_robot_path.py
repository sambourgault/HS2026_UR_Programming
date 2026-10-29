############################################################
##
## 05_record_robot_path.py
## author: Sam Bourgault
## date: 2026-10-28
## place: ETH Zurich
## source: inspired by example code from STDU ur_rtde library
## https://gitlab.com/sdurobotics/ur_rtde/-/tree/master/examples/py
##
############################################################

# first you import the necessary libraries
import csv
from pathlib import Path
import time, threading, queue
from pynput import keyboard 
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from compas.data import json_load, json_dump
from compas.geometry import Frame
from utils.pose_to_frame import pose_to_frame

# define the base and recording folders
BASE_DIR = Path(__file__).resolve().parent
current_recording_index = 0
recording_filename = (BASE_DIR / "recordings" / time.strftime("session_%Y_%m_%d_")+str(current_recording_index))
print("Recording folder:", recording_filename)

# thread variables
key_pressed = queue.Queue()
stop_event = threading.Event()

# robot mode
robot_mode = 0 # 0 for idle (free drive), 1 for recording (force mode), 2 for replay

def new_filename():
    global current_recording_index, recording_filename
    current_recording_index += 1
    recording_filename = (BASE_DIR / "recordings" / time.strftime("session_%Y_%m_%d_")+str(current_recording_index))
    print("Recording folder:", recording_filename)

def pose_to_json_frame(pose):
    frame = pose_to_frame(pose)

    return Frame(
        [coordinate * 1000.0 for coordinate in frame.point],
        frame.xaxis,
        frame.yaxis,
    )

# key press functions
# source: https://pynput.readthedocs.io/en/latest/keyboard.html
def on_press(key):
    try:
        key_pressed.put(key.char)
    except AttributeError:
        key_pressed.put(str(key))

def on_release(key):
    if key == keyboard.Key.esc:
        stop_event.set()
        return False

# robot function
def robot_loop():
    RTDE_FREQUENCY = 125.0 # loop frequency for RTDE communication

    # you create variables for the rtde_control and rtde_receive interfaces
    rtde_c = RTDEControl("169.254.10.10", RTDE_FREQUENCY)
    rtde_r = RTDEReceive("169.254.10.10", RTDE_FREQUENCY)
    work_object = [0,0,0,0,0,0]

    # get the current robot pose
    pose = rtde_r.getActualTCPPose()
    frame = pose_to_frame(pose)
    print("Current robot frame:", frame)

    # set damping for force mode
    rtde_c.forceModeSetDamping(0.0075)

    # mode and recording strcture
    robot_mode = 0

    recording_data = {
        "workobject": Frame.worldXY(),
        "frames": [],
    }

    # sampling control
    record_dt = 1.0 / RTDE_FREQUENCY  # same as RTDE period, e.g. 1/125 for 125Hz
    next_record_t = 0.0

    try:
        while not stop_event.is_set():
            t_start = rtde_c.initPeriod()
            # check for key presses to change robot mode
            while not key_pressed.empty():
                key = key_pressed.get()
                if key == "0":
                    # if switching from force mode to free drive, save the recorded data if any
                    if robot_mode == 1 and recording_data["frames"]:
                        json_dump(recording_data, recording_filename)
                        print("Saved " + str(len(recording_data['frames'])) + " frames to " + recording_filename)
                        new_filename()

                    robot_mode = 0
                    # initialization steps: stop force mode and refresh force-torque sensor
                    rtde_c.forceModeStop()
                    rtde_c.zeroFtSensor()
                    time.sleep(0.1)
                    print("Free drive mode activated")
                elif key == "1":
                    # reset recording data if switching to force mode
                    if robot_mode != 1:
                        recording_data["frames"] = []
                        rec_i = 0
                        next_record_t = time.monotonic()

                    robot_mode = 1
                    # initialization steps: stop freedrive and refresh force-torque sensor
                    rtde_c.endFreedriveMode()
                    rtde_c.zeroFtSensor()
                    time.sleep(0.1)
                    print("Force mode activated")

                elif key == "2":
                    robot_mode = 2
                    # initialization steps: stop freedrive/force mode and refresh force-torque sensor
                    rtde_c.forceModeStop()
                    rtde_c.endFreedriveMode()
                    rtde_c.zeroFtSensor()
                    time.sleep(0.1)
                    print("Replay mode activated")
                elif key == "w":
                    print("Saving work object")
                    pose = rtde_r.getActualTCPPose()
                    recording_data["workobject"] = pose_to_json_frame(pose)
                    #json_dump(work_object, "work_object.json")
                elif key == "esc":
                    print("Exiting program")
                    stop_event.set()
            
            # robot mode = 0 (idle in free drive)
            if robot_mode == 0:
                # set up the free drive mode
                free_axes = [1, 1, 1, 0,0,0]
                currentTCP = rtde_r.getActualTCPPose() 
                base = [0,0,0,0,0,0]
                rtde_c.freedriveMode(free_axes, currentTCP)
            
            # robot mode = 1 (recording in forcemode)
            elif robot_mode == 1:
                # start force mode
                free_axes = [1, 1, 1, 0, 0, 0]
                currentTCP = rtde_r.getActualTCPPose() 
                base = [0,0,0,0,0,0]
                speed = [0.15,0.15,0.15,.785,.785,0.785]
                rtde_c.forceMode(base,free_axes,[0,0,1,0,0,0], 2, speed)

                # record at fixed intervals
                now = time.monotonic()

                if now >= next_record_t:
                    pose = rtde_r.getActualTCPPose()
                    frame = pose_to_json_frame(pose)

                    recording_data["frames"].append(frame)

                    sample_count = len(recording_data["frames"])
                    if sample_count % 100 == 0:
                        print(str(sample_count) + " samples", end="")

                    next_record_t = now + record_dt
                    
            # robot mode = 2 (replay mode)
            elif robot_mode == 2:
                # replay mode (implementation depends on your replay logic)
                pass

            rtde_c.waitPeriod(t_start)
            
    except Exception as e:
        print("Error in robot loop:", e)
    finally:
        # quit program
        rtde_c.stopScript()
        rtde_c.disconnect()
        rtde_r.disconnect()
        print("Program completed")

if __name__ == "__main__":
    # start the keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # start the robot loop in a separate thread
    robot_thread = threading.Thread(target=robot_loop)
    robot_thread.start()
    robot_thread.join()



