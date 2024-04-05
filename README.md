# Fetch Manipulator Control

This repository contains code samples and examples for controlling various components of the Fetch mobile manipulator. Below is a breakdown of the functionalities covered:

- Controlling the base: 
```
rosrun applications keyboard_teleop.py
```
- Controlling the gripper: 
```
rosrun applications gripper_demo.py close
rosrun applications gripper_demo.py open
```
- Controlling the torso:
```
rosrun applications torso_demo.py 0.1
rosrun applications torso_demo.py 0.4
rosrun applications torso_demo.py 0.0
```
- Controlling the head:
```
rosrun applications head_demo.py look_at base_link 1 0 0.3
rosrun applications head_demo.py pan_tilt 0 0
```
- Controlling the arm:
```
rosrun applications arm_demo.py
```
