import gym
from gym import error, spaces, utils
from gym.utils import seeding
import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import Image, CompressedImage, Range
from gazebo_msgs.srv import GetWorldProperties

from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState

from rosgraph_msgs.msg import Clock
import sys
from robobo import Robobo
import numpy as np

class State(object):
    def __init__(self):
        self.prey = []
        self.predator = []
      
class RoboboPredatorPreyEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    
    def __init__(self):        
        
        rospy.init_node('RoboboPredatorPreyEnv', anonymous=True, log_level=rospy.FATAL)
        
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.set_agent_pose_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        #self.get_world_properties_proxy = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        
        rospy.Subscriber("/clock", Clock, self.set_current_time_callback, queue_size = 2)
        
        self.ARENA_SIDE_LENGTH = 4.0
        self.start_time = 0.0
        self.current_time = 0.0
        self.time_diff = None
        
        self.state = State()
        self.state.prey = []
        self.state.predator = []
        
        self.catch_threshold = 0.25
        
        self.reward = None
        self.done = False
        self.info = {}
        
        self.period = 30
        
        self.num_prey = 1
        self.num_predators = 3
        self.num_agents = self.num_prey + self.num_predators                
        
        self.prey = [Robobo('prey')]
        rospy.wait_for_message("/" + self.prey[0].model_name + "/camera1/image_raw", Image)
        self.predators = []
        
        for i in range(self.num_predators):
            self.predators += [Robobo('predator' + str(i))]
            rospy.wait_for_message("/" + self.predators[i].model_name + "/camera1/image_raw", Image)
            
        rospy.wait_for_message("/gazebo/model_states", ModelStates)    
        
    def set_current_time_callback(self, msg):
        self.current_time = msg.clock.secs + msg.clock.nsecs * 10e-10
        
    def step(self, action):
        #print('step')
        #print('sim_time:', self.get_world_properties_proxy().sim_time - self.start_time)        
        #print('sim_time:', self.current_time - self.start_time)
        
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause()
        
        self.state.prey = []
        self.state.predator = []
        self.reward = {"predators": 0.0, "prey": 0.0} 
        self.info = {}
        
        self.time_diff = self.current_time - self.start_time        
        
        for idx, predator in enumerate(self.predators):
            #print('!!!!!!!!!!', predator.orientation, predator.model_name)
            predator.set_vels(action[idx,:])            
            self.state.predator += [predator.sensor_state + predator.avg_wheel_vels.tolist() + [predator.camera_img]]
            self.info[predator.model_name + "_position"] = predator.position
            self.info[predator.model_name + "_orientation"] = predator.orientation            
            predator.set_time(self.time_diff)
        
        for idx, prey in enumerate(self.prey):
            prey.set_vels(action[self.num_predators + idx,:])
            self.state.prey += [prey.sensor_state + prey.avg_wheel_vels.tolist() + [prey.camera_img]]                
            self.info[prey.model_name + "_position"] = prey.position
            self.info[prey.model_name + "_orientation"] = prey.orientation
            prey.set_time(self.time_diff)
        
        predators_fitness, done, ds = self.compute_predators_fitness()       
        prey_fitness = self.compute_prey_fitness(done)
        self.reward = {"predators": predators_fitness, "prey": prey_fitness}         
        
        self.info["distances"] = ds                        
        
        if not done:
            self.info["caught"] = False
            if self.time_diff >= self.period:
                rospy.wait_for_service('/gazebo/pause_physics')
                self.pause()
                
                self.done = True    
                #self.reward = {"predators": predators_fitness, "prey": prey_fitness}
        else:            
            rospy.wait_for_service('/gazebo/pause_physics')
            self.pause()
            
            self.done = True
            self.info["caught"] = True
            #self.reward = {"predators": predators_fitness, "prey": prey_fitness}                        
                        
        self.info["arena_length"] = self.ARENA_SIDE_LENGTH                
        self.info["time"] = self.time_diff                        
                        
        return self.state, self.reward, self.done, self.info
    
    def compute_predators_fitness(self):
        prey = self.prey[0]
        fitness = self.num_predators * 1.0 #cast
        done = False
        ds = []
        for idx, predator in enumerate(self.predators):
            d = self.compute_distance(predator, prey)
            #print(predator.model_name, "to prey distance:", self.compute_distance(predator, prey))
            if d <= self.catch_threshold:
                #print("CAUGHT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                done = True
                
            ds += [d]
        
        if not done:
            for d in ds:
                fitness -= d / (self.ARENA_SIDE_LENGTH * np.sqrt(2))
            
        return fitness, done, ds
    
    def compute_prey_fitness(self, done):

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
        
        #print(model_state)
        
        rospy.wait_for_service('/gazebo/set_model_state')     
        self.set_agent_pose_proxy(model_state)
    
    def reset(self):
        #print('reset')  
        
        try:                        
            rospy.wait_for_service('/gazebo/reset_world')
            self.reset_proxy()   
            
            rospy.wait_for_service('/gazebo/reset_world')
            self.reset_proxy()            
            
            '''position_x = 0.0
            position_y = 1.5
            position_z = 0.03
            self.set_agent_pose(self.prey[0].model_name, position_x, position_y, position_z, np.random.uniform(-np.pi, np.pi),0,0)
            
            position_x = -1.0
            position_y = -1.5
            position_z = 0.03
            self.set_agent_pose(self.predators[0].model_name, position_x, position_y, position_z, np.random.uniform(-np.pi, np.pi),0,0)
            
            position_x = 0.0
            position_y = -1.5
            position_z = 0.03
            self.set_agent_pose(self.predators[1].model_name, position_x, position_y, position_z, np.random.uniform(-np.pi, np.pi),0,0)
            
            position_x = 1.0
            position_y = -1.5
            position_z = 0.03
            self.set_agent_pose(self.predators[2].model_name, position_x, position_y, position_z, np.random.uniform(-np.pi, np.pi),0,0)'''           
    
            rospy.wait_for_service('/gazebo/unpause_physics')
            self.unpause()
            
        except Exception as e:
            print(e)        
        
        #rospy.wait_for_service('/gazebo/get_world_properties')
        #self.start_time = self.get_world_properties_proxy().sim_time
        self.start_time = self.current_time        
        self.reward = None
        self.done = False
        
        self.state.prey = []
        self.state.predator = []
        self.info = {}

        for idx, predator in enumerate(self.predators):         
            #print('!!!!!!!!!!', predator.orientation, predator.model_name)
            predator.set_vels([0, 0])
            self.state.predator += [predator.sensor_state + predator.avg_wheel_vels.tolist() + [predator.camera_img]]      
            self.info[predator.model_name + "_position"] = predator.position
            self.info[predator.model_name + "_orientation"] = predator.orientation
            
        for idx, prey in enumerate(self.prey):
            prey.set_vels([0, 0])
            self.state.prey += [prey.sensor_state + prey.avg_wheel_vels.tolist() + [prey.camera_img]]        
            self.info[prey.model_name + "_position"] = prey.position
            self.info[prey.model_name + "_orientation"] = prey.orientation            
            
        return self.state, self.reward, self.done, self.info
        
    def render(self, mode='human'):
        print('render')            
        
    def close(self):
        print('close')
