# HS2026_UR_Programming

This repository serves as a starting point for programming a UR robot using the ur-rtde python library. This was specifically developed to guide student at ETH Zurich doing independant study semester under my supervision.

## Getting started

### With code

To install and use ur_rtde, you will need Python 3.12 or earlier. Check on your machine which version you have or create a new environment to run your code.

```python --version```

#### If you use an environment

- Download miniconda.
- Run ```conda create -n nameOfYourEnv python=3.9.11```
- Run ```conda activate nameOfYourEnv```
- Run ```python -m pip install ur_rtde``` 
- Check the library is properly installed, run ```python -c "import rtde_control; print('ur_rtde works')"``` 

#### If you are already running python < 3.12 on your machine

- Run ```pip install ur_rtde``` 
- Check the library is properly installed, run ```python -c "import rtde_control; print('ur_rtde works')"``` 


### With the UR robot

First, turn on the robot:

- Press at the bottom left of the teaching pendant on Power off.
- Set the payload to something that make sense with your end effector in place. I currently use .100 kg.
- Press on ON.
- Press on START and make sure you are not too close, the joints will crack if you are using an older UR robot!! 
- Press Exit.
- Check the IP address of the robot in the hamburger menu > System > Network. (This will be important in the code and for your computer to connect properly to the robot.) (The Settings option might be disabled is you are not in _Local control_ at the top right of the pendant.)
- Plug the E-thernet cable from the robot to your laptop. (If you are still in the Network tab of the pendant, you should see _Network is connected_ as a first sign)
- Turn the robot to _Remote control_ at the top right of the pendant.
- Check in your terminal if the computer sees the robot: ```ping theRobotIPadsress```. If it replies, you are good to go.


