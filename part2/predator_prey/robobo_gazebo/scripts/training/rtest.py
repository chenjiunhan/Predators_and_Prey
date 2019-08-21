#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int8, Int16, Int32
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk
import time
rospy.init_node("robobo__demo")

robobo_move_srv = rospy.ServiceProxy('/robot/prey/moveWheels', MoveWheels)

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    print(time.time())
    robobo_move_srv(Int8(30), Int8(-30), Int32(500), Int16(0))

    #rate.sleep()
