import gym
from gym import error, spaces, utils
from gym.utils import seeding
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import Image, CompressedImage, Range
from gazebo_msgs.srv import GetWorldProperties

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from rosgraph_msgs.msg import Clock
from robobo_msgs.srv import MoveWheels, SetEmotion, Talk

import sys
from robobo import Robobo
import numpy as np
import time

class State(object):
    def __init__(self):
        self.prey = []
        self.predator = []
      
class RoboboPredatorPreyEnvReal(gym.Env):
    metadata = {'render.modes': ['human']}
    
    def __init__(self):
        
        rospy.init_node('RoboboPredatorPreyEnvReal', anonymous=True, log_level=rospy.FATAL)
        
        self.ARENA_SIDE_LENGTH = 4.0
        self.start_time = 0.0
        self.current_time = 0.0
        
        self.state = State()
        self.state.prey = []
        self.state.predator = []
        
        self.catch_threshold = 0.25
        
        self.reward = None
        self.done = False
        self.info = {}
        
        self.period = 90
        
        self.num_prey = 1
        self.num_predators = 3
        self.num_agents = self.num_prey + self.num_predators
        
        self.prey = [Robobo('prey', real=True)]
        #rospy.wait_for_message("/robot/" + self.prey[0].model_name + "/camera/image/compressed", CompressedImage)
        
        self.predators = []
        
        for i in range(self.num_predators):
            self.predators += [Robobo('predator' + str(i), real=True)]            
            rospy.wait_for_message("/robot/" + self.predators[i].model_name + "/camera/image/compressed", CompressedImage)
        
        
    def step(self, action):
        #print('step')
        #print('sim_time:', self.get_world_properties_proxy().sim_time - self.start_time)
        
        #print('sim_time:', self.current_time - self.start_time)
                
        self.state.prey = []
        self.state.predator = []
        self.reward = {"predators": 0.0, "prey": 0.0} 
        self.info = {}       
        
        self.time_diff = self.current_time - self.start_time
        
        for idx, predator in enumerate(self.predators):
            predator.set_vels(action[idx,:])
            self.state.predator += [predator.sensor_state + [predator.camera_img]]
            predator.set_time(self.time_diff)
        
        for idx, prey in enumerate(self.prey):
            prey.set_vels(action[self.num_predators + idx,:])
            self.state.prey += [prey.sensor_state + [prey.camera_img]]              
            prey.set_time(self.time_diff)  
        
        predators_fitness, done, ds = self.compute_predators_fitness()       
        prey_fitness = self.compute_prey_fitness(done)

        self.info["distances"] = ds
        self.current_time = time.time()                
        
        if not done:
            if (self.time_diff) >= self.period:                
                
                self.done = True    
                self.reward = {"predators": predators_fitness, "prey": prey_fitness}
        else:                        
            
            self.done = True
            self.reward = {"predators": predators_fitness, "prey": prey_fitness}                        
            
        self.info["arena_length"] = self.ARENA_SIDE_LENGTH        
        
        self.info['time'] = time.time() - self.start_time
                        
        return self.state, self.reward, self.done, self.info
    
    def compute_predators_fitness(self):
        
        return 3.0, False, [1.0, 1.0, 1.0]
    
        prey = self.prey[0]
        fitness = self.num_predators * 1.0 #cast
        done = False
        ds = []
        for idx, predator in enumerate(self.predators):
            d = self.compute_distance(predator, prey)
            #print(predator.model_name, "to prey distance:", self.compute_distance(predator, prey))
            if d <= self.catch_threshold:
                print("CAUGHT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                done = True
                
            ds += [d]
        
        if not done:
            for d in ds:
                fitness -= d / (self.ARENA_SIDE_LENGTH * np.sqrt(2))
            
        return fitness, done, ds
    
    def compute_prey_fitness(self, done):

        return 0.0

        fitness = self.num_predators * 1.0 # for comparison        

        if done:
        
            time_diff = min(self.period, (self.current_time - self.start_time))
            fitness = fitness * time_diff / self.period
    
        return fitness
    
    def compute_distance(self, agent1, agent2):
        return np.sqrt((agent1.position.x - agent2.position.x) ** 2 + (agent1.position.y - agent2.position.y) ** 2)
    
    def euler_to_quaternion(self, yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return qx, qy, qz, qw
    
    def set_agent_pose(self, model_name, position_x, position_y, position_z, yaw, pitch, roll):
    
        model_state = ModelState()
        model_state.model_name = model_name
        
        model_state.pose.position.x = position_x
        model_state.pose.position.y = position_y
        model_state.pose.position.z = position_z
        
        x,y,z,w = self.euler_to_quaternion(np.random.uniform(-np.pi, np.pi),0,0)            
        model_state.pose.orientation.x = x 
        model_state.pose.orientation.y = y
        model_state.pose.orientation.z = z
        model_state.pose.orientation.w = w
                                
    
    def reset(self):
        #print('reset')                          
        
        self.reward = None
        self.done = False
        
        self.state.prey = []
        self.state.predator = []

        for idx, predator in enumerate(self.predators):
            predator.set_vels([0, 0])
            self.state.predator += [predator.sensor_state + [predator.camera_img]]    
            #self.info[predator.model_name + "_position"] = predator.position
            #self.info[predator.model_name + "_orientation"] = predator.orientation                   
            
        for idx, prey in enumerate(self.prey):
            prey.set_vels([0, 0])
            self.state.prey += [prey.sensor_state + [prey.camera_img]]        
        
        self.info['time'] = 0.0
        try:
            while not rospy.is_shutdown() and input("Ready?") != "y":                                       
                pass
        except:
            pass
        
        print("Click pygame window...")    
        time.sleep(5)
        print("Start!") 
        
        self.start_time = time.time()
        
        return self.state, self.reward, self.done, self.info
        
    def render(self, mode='human'):
        print('render')            
        
    def close(self):
        print('close')
