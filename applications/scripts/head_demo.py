#!/usr/bin/env python

import rospy
from robot_api import Head  # Make sure to import the Head class correctly

def print_usage():
    print('Usage:')
    print('    rosrun applications head_demo.py look_at FRAME_ID X Y Z')
    print('    rosrun applications head_demo.py pan_tilt PAN_ANG TILT_ANG')
    print('Examples:')
    print('    rosrun applications head_demo.py look_at base_link 1 0 0.3')
    print('    rosrun applications head_demo.py pan_tilt 0 0.707')

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('head_demo')
    wait_for_time()

    head = Head()  # Initialize the Head object

    argv = rospy.myargv()
    if len(argv) < 2:
        print_usage()
        return
    command = argv[1]

    if command == 'look_at':
        if len(argv) < 6:
            print_usage()
            return
        frame_id = argv[2]
        x = float(argv[3])
        y = float(argv[4])
        z = float(argv[5])
        head.look_at(frame_id, x, y, z)  # Use the look_at method from the Head class
    elif command == 'pan_tilt':
        if len(argv) < 4:
            print_usage()
            return
        pan = float(argv[2])
        tilt = float(argv[3])
        head.pan_tilt(pan, tilt)  # Use the pan_tilt method from the Head class
    else:
        print_usage()

if __name__ == '__main__':
    main()

