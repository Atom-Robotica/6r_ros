#!/usr/bin/env python
import rospy

if __name__ == "__main__":
    rospy.init_node("PY_Echo_node")
    print("Sample output through a Python ROS Node.")
    try:
        input("Press ENTER to exit")
    except SyntaxError:
        exit(0)
