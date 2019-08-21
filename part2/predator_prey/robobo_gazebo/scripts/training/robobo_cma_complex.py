#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage, Range
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
import threading
from nn import Net
import torch
import torch.nn as nn
import cma
import gym
import gym_robobo_predator_prey
from nn import Net
    
class PredatorPreyEvolution:

    def __init__(self, predator_controllers, prey_controller, predator_same_controller = True, predator_evolution = True, prey_evolution = False):
            
        self.episode_end = threading.Event()
        
        self.count_episode_start = 0
        self.count_episode_end = 0
        
        self.num_predators = 3   
        self.prey_controller = Net(8, 2)
        self.fitnesses = [0.0] * num_predators
        self.predator_controllers = []
        
        for i in range(num_predators):
            self.predator_controllers += [Net(8, 2)]
 
        self.env = gym.make('gym_robobo_predator_prey-v0')
               
        self.t = threading.Thread(target = self.start)
        self.t.daemon = True
        self.t.start()

    def tensors_to_params_list(tensors):
        
        params_list = []
        
        #print(tensors)
        
        for tensor in tensors:
            print('-----------')
            print(tensor.shape)
            if len(tensor.shape) == 1:
                params_list += tensor.tolist()

            elif len(tensor.shape) == 2:
                l = tensor.tolist()
                flat_list = [item for sublist in l for item in sublist]
                params_list += flat_list
                #print(params_list)
            else:
                raise Exception("Wrong dim")
                
        return params_list
        
    def params_list_into_net(params_list, net):

        new_state_dict = net.state_dict()
        start_idx = 0
        end_idx = 0
        for state, params in net.state_dict().items():        
            end_idx = start_idx + params.numel()
            
            new_state_dict[state] = torch.nn.Parameter(torch.tensor(params_list[start_idx:end_idx]).view(params.shape))
            
            start_idx = end_idx

        net.load_state_dict(new_state_dict)
        #print(net.state_dict())

        return new_state_dict
   
    def gym_robobo_train(params_list):

        #print(params_list)
        self.params_list_into_net(params_list, net)
        
        #print('new!!!!!!!!!!!!!!', net.state_dict())
        
        #self.count_episode_end += 1
        #if self.count_episode_end == 3:
        #    self.count_episode_end = 0
        #    self.episode_end.set()        
                
        self.episode_end.wait()
        
        return fitness
        
    def start(net):
        
        params_list = self.tensors_to_params_list(list(net.parameters()))
        es = cma.CMAEvolutionStrategy(params_list, 0.5)                    
        es.optimize(gym_robobo_train)
        
    def play():
        
        while
        
            observations = env.reset()
            
            while not done and not rospy.is_shutdown():
                action = np.zeros((num_predators, 2), dtype=int)
                for predator_idx, obs in enumerate(observations):
                    action[predator_idx,:] = self.nets[predator_idx](torch.FloatTensor(obs)).numpy()
                observations, reward, done, _ = env.step(action)                
            
            self.episode_end.clear()
            self.episode_start.wait()

evolution = EvolvedPredator()
