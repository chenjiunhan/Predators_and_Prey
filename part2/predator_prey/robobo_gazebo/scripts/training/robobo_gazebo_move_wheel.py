#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import String, Int8, Int16, Int32
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-n", "--robot_namespace", help="Namespace of the robot", dest="ns", default="/robot")
args = parser.parse_args()
NS = args.ns

print("NS:", NS)

def gazebo_robobo_move_wheel(left_wheel_speed, right_wheel_speed):
    rospy.wait_for_service(NS + '/move_wheel')
    try:
        move_wheel_proxy = rospy.ServiceProxy(NS + '/move_wheel', MoveWheels)
        move_wheel_proxy(Int8(left_wheel_speed), Int8(right_wheel_speed), Int32(2000), Int16(0))
        return 
    except rospy.ServiceException:
        print("Service call failed: %s"%rospy.ServiceException)
        
if __name__ == "__main__":    
    gazebo_robobo_move_wheel(20, -20)
