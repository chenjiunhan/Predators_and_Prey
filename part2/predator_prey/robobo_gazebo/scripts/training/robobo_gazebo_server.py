#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import String, Int8, Int16, Int32
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from argparse import ArgumentParser

parser = ArgumentParser()
parser.add_argument("-n", "--robot_namespace", help="Namespace of the robot", dest="ns", default="/robot")
args = parser.parse_args()
NS = args.ns
ROBOT_NAME = NS[1:]
get_model_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
set_model_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
vel_pub = rospy.Publisher(NS + '/mobile_base_controller/cmd_vel', Twist, queue_size=5)

print("NS:", NS)

def handle_gazebo(req):
    left_wheel_speed = int(req.lspeed.data)
    right_wheel_speed = int(req.rspeed.data)    
    rospy.wait_for_service("/gazebo/get_model_state")
    left_wheel_link_prop = get_model_proxy(ROBOT_NAME, 'robobo_left_wheel_link')
    right_wheel_link_prop = get_model_proxy(ROBOT_NAME, 'robobo_right_wheel_link')
    
    # TODO publish left_wheel right_wheel msg, and subscribe them with plugin
    vel_cmd = Twist()
    vel_cmd.linear.x = 0.05
    vel_cmd.angular.z = -0.3
    vel_pub.publish(vel_cmd)
    print("pub")
    
    return Int8(0)

def move_wheel():
    rospy.init_node('move_wheel_server')
    s = rospy.Service(NS + '/move_wheel', MoveWheels, handle_gazebo)
    print("move_wheel")
    rospy.spin()

if __name__ == "__main__":
    move_wheel()
    #gazebo_robobo_move_wheel()
