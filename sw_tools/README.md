# README.md

This is a playground for various debug scripts to assist debug and deploy process. Please add a short description of how the script intends to work when you upload a new script.

Python-based scripts are encouraged. This repository will remain having no dependency settings and use as a place to serve this utilities. If your script depends on any ROS/pip, please also mention it explicitly.

## `any_sub.py`
- Testing out `rospy.AnyMsg` class, this is only useful when the message is simple enough that the (de)serialize buffer is trival to read.

## `dash.py`
- Proving a dashboard to visualize the status of the chassis/base (and more). Specific for Diego, it relays the signal of `RC_STOP` and shutdown the chassis.
- depends: `blessed`

## `teleop_raw.py`
- Proving a teleop(`cmd_vel`+`cmd_disinfection`) interface with full terminal keyboard capture
- originally for disinfection demo and later added auto stopping on lidar detection
- option: `_use_mux:=True` for using `cmd_vel_mux`
- depends: `blessed`
