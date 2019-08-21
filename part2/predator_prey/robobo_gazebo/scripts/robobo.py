#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, Range
from std_msgs.msg import Int32MultiArray
from gazebo_msgs.msg import ModelStates
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk, MovePanTilt
import cv2
import numpy as np
import threading
import torch
import copy
from robobo_msgs.msg import IRs
from std_msgs.msg import String, Int8, Int16, Int32
import time

#seperate class

DEBUG_TIME = False

class Robobo:
    def __init__(self, model_name, real = False):
        self.model_name = model_name      
        self.wheel_vels = Int32MultiArray()
        self.wheel_vels.data = [0, 0]
        self.camera_img = None        
        self.sensor_state = [-1.0] * 8        
        self.position = None
        self.avg_num = 5
        self.avg_wheel_vels = np.array([0.0, 0.0])
        self.left_vels = []
        self.right_vels = []
        self.max_speed = 20
        self.position = None
        self.orientation = None
        self.prev_position = None
        self.time = None
        self.prev_time = None        
        self.img_update_time = None
        self.real = real
        
        if self.real:
            
            self.robobo_move_srv = rospy.ServiceProxy('/robot/' + self.model_name + '/moveWheels', MoveWheels)
            self.robobo_move_tilt = rospy.ServiceProxy('/robot/' + self.model_name + '/movePanTilt', MovePanTilt)            
            #if self.model_name == "prey":
            rospy.wait_for_service('/robot/' + self.model_name + '/movePanTilt')    
            self.robobo_move_tilt(Int16(0), Int8(0), Int16(0), Int16(90), Int8(100), Int16(1))
        
        self.t = threading.Thread(target = self.start)
        self.t.daemon = True
        self.t.start()

    def set_vels(self, wheel_vels_data):
        self.wheel_vels.data = wheel_vels_data
        
        #self.left_vels += [self.wheel_vels.data[0]]
        #if len(self.left_vels) > 5:
        #    self.left_vels = self.left_vels[1:]
            
        #self.right_vels += [self.wheel_vels.data[1]]
        #if len(self.right_vels) > 5:
        #    self.right_vels = self.right_vels[1:]
        if DEBUG_TIME:
            print("set_vels", time.time())    
            
        #print(wheel_vels_data)
        #print('L', self.left_vels)
        #self.avg_wheel_vels = np.array([np.array(self.left_vels).mean(), np.array(self.right_vels).mean()]) / self.max_speed
        
    def set_time(self, time):
    
        self.prev_time = copy.deepcopy(self.time)
        self.time = time
    
    def raw_image_callback(self, image_msg, name):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", image_msg)
        # Image to numpy array
        if hasattr(image_msg, "data"):        
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            image_np = np.reshape(np_arr, (image_msg.height, image_msg.width, 3))
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            self.camera_img = image_np
            #cv2.imshow(self.model_name + " image", image_np)
            #cv2.waitKey(1)
            #cv2.imshow(cv2_img)

    '''def callback(image_msg):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", image_msg)
        # Image to numpy array
        if hasattr(image_msg, "data"):
            #print(image_msg.height)
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #np_arr = np.reshape(np_arr, (image_msg.height, image_msg.width, 3))

            # Decode to cv2 image and store
            #print(image_np.shape)
            cv2.imshow(self.model_name + " image", image_np)
            cv2.waitKey(1)
            #cv2.imshow(cv2_img)'''
            
    def image_callback(self, image_msg, name):            
    
        if hasattr(image_msg, "data"):
            #print(image_msg.height)
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)            
            self.camera_img = image_np
            
    def real_image_callback(self, image_msg, name):            
        
        #if type(self.img_update_time) != type(None) and type(self.time) != type(None):
        #    
        #    if self.time - self.img_update_time < 1.0:
        #        print("RETURN ", self.model_name)
        #        return
    
        if hasattr(image_msg, "data"):
            #print(image_msg.height)
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)            
            self.img_update_time = self.time
            self.camera_img = np.flip(image_np, 1)
            #print(self.time, self.img_update_time, self.model_name)
            
    def get_image(self):
        return self.camera_img    
    
    def ir_sensor_callback(self, ir_msg, name):
    
        #print(ir_msg, self.model_name)
    
        if ir_msg.range < 0.19:
            ir_msg.range = 1.0
        else:
            ir_msg.range = -1.0
    
        #print(ir_msg.range, ir_msg, name)
        if name == "ir_front_1":
            self.sensor_state[0] = ir_msg.range
        elif name == "ir_front_2":
            self.sensor_state[1] = ir_msg.range
        elif name == "ir_front_3":
            self.sensor_state[2] = ir_msg.range
        elif name == "ir_front_4":
            self.sensor_state[3] = ir_msg.range
        elif name == "ir_front_5":
            self.sensor_state[4] = ir_msg.range
        elif name == "ir_back_1":
            self.sensor_state[5] = ir_msg.range
        elif name == "ir_back_2":
            self.sensor_state[6] = ir_msg.range
        elif name == "ir_back_3":
            self.sensor_state[7] = ir_msg.range
            
    def real_ir_sensor_callback(self, ir_msg, name):            
    
        self.sensor_state[0] = ir_msg.FrontLL.range
        self.sensor_state[1] = ir_msg.FrontL.range
        self.sensor_state[2] = ir_msg.FrontC.range
        self.sensor_state[3] = ir_msg.FrontR.range
        self.sensor_state[4] = ir_msg.FrontRR.range
        self.sensor_state[5] = ir_msg.BackR.range
        self.sensor_state[6] = ir_msg.BackC.range
        self.sensor_state[7] = ir_msg.BackL.range        

        if ir_msg.FrontC.range > 19:
            self.sensor_state[2] = -1.0
        else:
            self.sensor_state[2] = -1.0

    def pose_callback(self, msg, name):     
     
        self.prev_position = copy.deepcopy(self.position)
        self.position = msg.pose[msg.name.index(name)].position
        self.orientation = msg.pose[msg.name.index(name)].orientation

    def timer_callback(self, event):
        global pub
        pub.publish(last_data)                                      
        
    def quaternion_to_euler(self, x_agent, y_agent, z_agent, w_agent):
        x = x_agent
        y = y_agent
        z = z_agent
        w = w_agent
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw, pitch, roll                
        
    def start(self):
        
        if not self.real:
        #rospy.Subscriber("/prey/camera1/image_raw/compressed", CompressedImage, callback)
            rospy.Subscriber("/" + self.model_name + "/camera1/image_raw", Image, self.raw_image_callback, ('image_raw'), queue_size = 4)
            #rospy.Subscriber("/" + self.model_name + "/camera1/image_raw/compressed", CompressedImage, self.image_callback, ('image_raw'), queue_size = 2)
            #rospy.Subscriber("/" + self.model_name + "/sensor/ir_front_1", Range, self.ir_sensor_callback, ('ir_front_1'), queue_size = 4)
            #rospy.Subscriber("/" + self.model_name + "/sensor/ir_front_2", Range, self.ir_sensor_callback, ('ir_front_2'), queue_size = 4)
            rospy.Subscriber("/" + self.model_name + "/sensor/ir_front_3", Range, self.ir_sensor_callback, ('ir_front_3'), queue_size = 4)
            #rospy.Subscriber("/" + self.model_name + "/sensor/ir_front_4", Range, self.ir_sensor_callback, ('ir_front_4'), queue_size = 4)
            #rospy.Subscriber("/" + self.model_name + "/sensor/ir_front_5", Range, self.ir_sensor_callback, ('ir_front_5'), queue_size = 4)
            #rospy.Subscriber("/" + self.model_name + "/sensor/ir_back_1", Range, self.ir_sensor_callback, ('ir_back_1'), queue_size = 4)
            #rospy.Subscriber("/" + self.model_name + "/sensor/ir_back_2", Range, self.ir_sensor_callback, ('ir_back_2'), queue_size = 4)
            #rospy.Subscriber("/" + self.model_name + "/sensor/ir_back_3", Range, self.ir_sensor_callback, ('ir_back_3'), queue_size = 4)
            rospy.Subscriber("/gazebo/model_states", ModelStates, self.pose_callback, self.model_name, queue_size = 2)        
            pub = rospy.Publisher("/" + self.model_name + "/vel_cmd", Int32MultiArray, queue_size=10)          
            
        else:
            #pass
            rospy.Subscriber("/robot/" + self.model_name + "/camera/image/compressed", CompressedImage, self.real_image_callback, ('real_image_callback'), queue_size = 2)
            rospy.Subscriber("/robot/" + self.model_name + "/irs", IRs, self.real_ir_sensor_callback, ('irs'), queue_size = 5)
            #pub = rospy.Publisher("/" + self.model_name + "/vel_cmd", Int32MultiArray, queue_size=10)
                        
        rate = rospy.Rate(50) # 10hz
        
        while not rospy.is_shutdown():            
            #rospy.loginfo(self.wheel_vels)
            try:                    
                if self.real:
                    #print("NNNNNNNNNNNNNNNN", self.model_name)
                    if DEBUG_TIME:
                        print("robobo_move_srv TIME:", time.time())
                    rospy.wait_for_service('/robot/' + self.model_name + '/moveWheels')
                    if DEBUG_TIME:
                        print(self.model_name)
                    self.robobo_move_srv(Int8(self.wheel_vels.data[0]), Int8(self.wheel_vels.data[1]), Int32(500), Int16(1))
                    #print("??????????????")
                    #self.robobo_move_srv(Int8(0), Int8(0), Int32(200), Int16(0))
                else:                    
                    pub.publish(self.wheel_vels)
                
                #rospy.sleep(1)
                
                rate.sleep()                
                
            except Exception as e:
                rospy.logerr("Error: %s" % (e,))
                
                #exit(0)
                
            #print(self.model_name)
            
    
    
    
    
    
