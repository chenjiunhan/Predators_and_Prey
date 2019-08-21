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

net = Net(8, 2)

def tensors_to_params_list(tensors):
    
    params_list = []
    
    print(tensors)
    
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

    global net

    print(params_list)
    params_list_into_net(params_list, net)
    
    print('new!!!!!!!!!!!!!!', net.state_dict())
    
    num_predators = 3
    done = False
    observations = env.reset()
    
    fitness = 0.0
    
    while not done and not rospy.is_shutdown():
        action = np.zeros((num_predators, 2), dtype=int)
        for predator_idx, obs in enumerate(observations):
            action[predator_idx,:] = net(torch.FloatTensor(obs)).numpy()
        observations, reward, done, _ = env.step(action)        
        
    return 0.0


#print(net.state_dict())
#torch.nn.Parameter(torch.zeros(D_in,H))

#for parameters in net.parameters():
#    print(parameters)

params_list = tensors_to_params_list(list(net.parameters()))

es = cma.CMAEvolutionStrategy(params_list, 0.5)

env = gym.make('gym_robobo_predator_prey-v0')
            
es.optimize(gym_robobo_train)
