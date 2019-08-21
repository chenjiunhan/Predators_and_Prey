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
import time
import robobo_detection
import copy

NUM_PREDATORS = 3
NUM_PREY = 1

EVO_FLAG = 0     # 0 = predator, 1 = prey

fitnesses = [0.0] * NUM_PREDATORS
best_fitnesses = [0.0] * NUM_PREDATORS

fitnesses_prey = [0.0] * NUM_PREY
best_fitnesses_prey = [0.0] * NUM_PREY

INPUT_DIM = 3
PARAM_MIN = -1.0
PARAM_MAX = 1.0

nets = []
best_nets = []
best_fitnesses = [0.0]

nets_prey = []
best_nets_prey = []


for i in range(NUM_PREDATORS):
    nets += [Net(INPUT_DIM, 2)]
    best_nets = copy.deepcopy(nets[i])

net_prey = [Net(INPUT_DIM, 2)]
best_net_prey = copy.deepcopy(net_prey[0])

def tensors_to_params_list(tensors):
    
    params_list = []
    
    for tensor in tensors:
        print('-----------')
        print(tensor.shape)
        if len(tensor.shape) == 1:
            params_list += tensor.tolist()

        elif len(tensor.shape) == 2:
            l = tensor.tolist()
            flat_list = [item for sublist in l for item in sublist]
            params_list += flat_list
        else:
            raise Exception("Wrong dim")
            
    return params_list
    
def params_list_to_state_dict(params_list, net):

    new_state_dict = net.state_dict()
    start_idx = 0
    end_idx = 0
    for state, params in net.state_dict().items():        
        end_idx = start_idx + params.numel()
        
        new_state_dict[state] = torch.nn.Parameter(torch.tensor(params_list[start_idx:end_idx]).view(params.shape))
        
        start_idx = end_idx
    
    return new_state_dict  

event_episode_start = threading.Event()
event_episode_end = threading.Event()
count_episode_start = 0

event_episode_start.daemon = True
event_episode_end.daemon = True

def cma_train(net, obj_function):
    params_list = tensors_to_params_list(list(net.parameters()))

    es = cma.CMAEvolutionStrategy(params_list, 1.0)            
    es.optimize(obj_function)

def set_event_episode_start():
    
    global NUM_PREDATORS, count_episode_start, event_episode_start
    
    count_episode_start += 1
    if count_episode_start == (NUM_PREDATORS + 1):  # +1 for main thread
        print("Episode can start")
        event_episode_end.clear()
        count_episode_start = 0
        event_episode_start.set()
        
    print("episode_start.wait")  
    event_episode_start.wait()
    print("episode_start.wait_finish")    
    
def obj_function_helper(params_list, fitnesses, nets, idx):

    global count_episode_start, event_episode_start, event_episode_end
    
    #params_list = np.clip(params_list, , 8)
    
    new_state_dict = params_list_to_state_dict(params_list, nets[idx])
    
    #if idx == 2:
        #print(new_state_dict)
    #    print("XDDD")
    
    nets[idx].load_state_dict(new_state_dict)    
    
    set_event_episode_start()
    
    print("episode_end.wait")    
    event_episode_end.wait()
    print("episode_end.wait_finish")    
    print('weight:', params_list)
    
    print("set fitness")
    fitness = -fitnesses[idx]
    
    return fitness

def predator1_obj_function(params_list):    

    global nets, fitnesses
    
    fitness = obj_function_helper(params_list, fitnesses, nets, 0)

    print("return fitness1:", fitness)
    
    return fitness
    
def predator2_obj_function(params_list):

    global nets, fitnesses
    
    fitness = obj_function_helper(params_list, fitnesses, nets, 1)
    
    print("return fitness2:", fitness)
    
    return fitness
    
def predator3_obj_function(params_list):    

    global nets, fitnesses
    
    fitness = obj_function_helper(params_list, fitnesses, nets, 2)
    
    print("return fitness3:", fitness)
    
    return fitness
    
def prey1_obj_function(params_list):    

    global nets, fitnesses_prey
    
    fitness = obj_function_helper(params_list, fitnesses_prey, nets_prey, 0)
    
    print("prey fitness1:", fitness)
    
    return fitness    

t = threading.Thread(target = cma_train, args = (nets[0], predator1_obj_function))
t.daemon = True
t.start()

t = threading.Thread(target = cma_train, args = (nets[1], predator2_obj_function))
t.daemon = True
t.start()

t = threading.Thread(target = cma_train, args = (nets[2], predator3_obj_function))
t.daemon = True
t.start()

'''t = threading.Thread(target = cma_train, args = (nets_prey[0], prey1_obj_function))
t.daemon = True
t.start()'''

env = gym.make('gym_robobo_predator_prey-v0')

done = False
count_episode = 0

while not rospy.is_shutdown():
            
    count_episode += 1
    print('episode:', count_episode)    
    observations = env.reset()
    done = False            
    set_event_episode_start()
    
    fitnesses = [0.0] * 3
    step = 0    
    
    while not done and not rospy.is_shutdown():
        action = np.zeros((NUM_PREDATORS, 2), dtype=int)
        step += 1
        for predator_idx, obs in enumerate(observations):
            if obs == None:
                continue
            obs_img = obs[-1]
            #obs = obs[0:8]
            obs = np.array([0.0, 0.0, -1.0])
            #obs = np.concatenate((obs, np.array([0.0, 0.0, -1.0])), axis=None)
            
            if type(obs_img) != type(None):
                rects, detect_img = robobo_detection.detect_robobo(obs_img)
                        
                window_name = "predator" + str(predator_idx + 1) + "_image"
                cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
                cv2.moveWindow(window_name, 0, 30 + predator_idx * 350)                
                cv2.resizeWindow(window_name, 300,300)
                cv2.imshow(window_name, detect_img)
                cv2.waitKey(1)                                            
                
                detect_img_height = detect_img.shape[0]
                detect_img_width = detect_img.shape[1]
                detect_img_area = detect_img_height * detect_img_width
                
                #print(detect_img.shape)
                #cv2.imwrite('images/' + str(time.time()) + ".png", obs_img)
                
                for rect in rects:
                    target_name = rect[4]
                    area = rect[2] * rect[3]
                    if target_name == "Prey" and area > 300:
                        #print('area:', area, predator_idx)
                        fitnesses[predator_idx] += area / (abs(rect[0] - (detect_img_width / 2)) / detect_img_width + 1)
                        obs[-3] = rect[0] / (detect_img_width / 2) - 1.0 # -1.0 - 1.0 
                        obs[-2] = rect[1] / (detect_img_height / 2) - 1.0
                        obs[-1] = area / detect_img_area                                                                            
                
            action[predator_idx,:] = nets[predator_idx](torch.FloatTensor(obs)).numpy()
                
        observations, reward, done, _ = env.step(action)

    print('obs:', obs)

    for predator_idx, _ in enumerate(observations):
        fitnesses[predator_idx] = fitnesses[predator_idx] / step

    print("Clean start event")
    event_episode_start.clear()    

    print("Save fitness info")
    print("Episode end")
    event_episode_end.set()
        
    
    
