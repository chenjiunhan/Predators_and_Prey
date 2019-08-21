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

DEBUG_THREAD = False
DEBUG_WEIGHT = False
DEBUG_FITNESS = False

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

nets_prey = []
best_nets_prey = []

for i in range(NUM_PREDATORS):
    nets += [Net(INPUT_DIM, 2)]
    best_nets += [copy.deepcopy(nets[i])]

nets_prey = [Net(INPUT_DIM, 2)]
best_nets_prey = [copy.deepcopy(nets_prey[0])]

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

event_episode_start_prey = threading.Event()
event_episode_end_prey = threading.Event()
count_episode_start_prey = 0
event_episode_start_prey.daemon = True
event_episode_end_prey.daemon = True

def cma_train(net, obj_function):
    params_list = tensors_to_params_list(list(net.parameters()))

    es = cma.CMAEvolutionStrategy(params_list, 1.0)            
    es.optimize(obj_function)

def set_event_episode_start():
    
    global NUM_PREDATORS, count_episode_start, event_episode_start
    
    count_episode_start += 1
        
    count_target = NUM_PREDATORS + 1 # +1 for main thread
    if count_episode_start == count_target:  
    
        if DEBUG_THREAD:
            print("Episode can start")
        event_episode_end.clear()
        count_episode_start = 0
        event_episode_start.set()
    
    if DEBUG_THREAD:                     
        print("episode_start.wait")
    event_episode_start.wait()
    if DEBUG_THREAD:
        print("episode_start.wait_finish")
    
def set_event_episode_start_prey():
    global NUM_PREDATORS, count_episode_start_prey, event_episode_start_prey
    
    count_episode_start_prey += 1
    
    count_target = NUM_PREY + 1
    if count_episode_start_prey == count_target:          
    
        if DEBUG_THREAD:
            print("Episode_Prey can start")
        event_episode_end_prey.clear()
        count_episode_start_prey = 0
        event_episode_start_prey.set()       

    if DEBUG_THREAD:
        print("event_episode_start_prey.wait")
    event_episode_start_prey.wait()
    
    if DEBUG_THREAD:
        print("event_episode_start_prey.wait_finish")
    
def obj_function_helper(params_list, idx):

    global count_episode_start, event_episode_start, event_episode_end, fitnesses, best_fitnesses, nets, best_nets
    
    #params_list = np.clip(params_list, , 8)
    
    new_state_dict = params_list_to_state_dict(params_list, nets[idx])
    
    #if idx == 2:
        #print(new_state_dict)
    #    print("XDDD")
    
    print("LOAD NETS PREDATOR:", idx)    
    nets[idx].load_state_dict(new_state_dict)    
    
    set_event_episode_start()
    if DEBUG_THREAD:
        print("episode_end.wait")    
    event_episode_end.wait()
    if DEBUG_THREAD:
        print("episode_end.wait_finish")    
    
    if DEBUG_WEIGHT:
        print('weight, idx:', params_list, idx)            
    
    fitness = -fitnesses[idx]
    if fitness < best_fitnesses[idx]:
        print('best fitness, idx:', fitness, idx)
        best_fitnesses[idx] = fitness
        best_nets[idx] = copy.deepcopy(nets[idx])
    
    if DEBUG_FITNESS:
        print("fitnesses???", fitnesses)
    
    return fitness
    
def obj_function_helper_prey(params_list, idx):

    global count_episode_start_prey, event_episode_start_prey, event_episode_end_prey, fitnesses_prey, best_fitnesses_prey, nets_prey, best_nets_prey
    
    #params_list = np.clip(params_list, , 8)
    
    new_state_dict = params_list_to_state_dict(params_list, nets_prey[idx])
    
    #if idx == 2:
        #print(new_state_dict)
    #    print("XDDD")
    
    nets_prey[idx].load_state_dict(new_state_dict)    
    
    set_event_episode_start_prey()
    
    if DEBUG_THREAD:
        print("episode_end_prey.wait")    
    event_episode_end_prey.wait()
    
    if DEBUG_THREAD:
        print("episode_end_prey.wait_finish")    
        
    if DEBUG_WEIGHT:
        print('weight_prey, idx:', params_list, idx)     
    
    fitness = -fitnesses_prey[idx]
    
    if fitness < best_fitnesses_prey[idx]:
        print('best fitness_prey, idx:', fitness, idx)
        best_fitnesses_prey[idx] = fitness
        best_nets_prey[idx] = copy.deepcopy(nets_prey[idx])
    
    if DEBUG_FITNESS:
        print("fitnesses_prey!!!", fitnesses_prey)
    
    return fitness

def predator1_obj_function(params_list):    

    global nets, fitnesses
    
    fitness = obj_function_helper(params_list, 0)

    print("return fitness1:", fitness)
    
    return fitness
    
def predator2_obj_function(params_list):

    global nets, fitnesses
    
    fitness = obj_function_helper(params_list, 1)
    
    print("return fitness2:", fitness)
    
    return fitness
    
def predator3_obj_function(params_list):    

    global nets, fitnesses
    
    fitness = obj_function_helper(params_list, 2)
    
    print("return fitness3:", fitness)
    
    return fitness
    
def prey1_obj_function(params_list):    

    global nets, fitnesses_prey
    
    fitness = obj_function_helper_prey(params_list, 0)
    
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

t = threading.Thread(target = cma_train, args = (nets_prey[0], prey1_obj_function))
t.daemon = True
t.start()

env = gym.make('gym_robobo_predator_prey-v0')

done = False
count_episode = 0

while not rospy.is_shutdown():
            
            
    if EVO_FLAG == 0:  
        print("==============================================")   
        print("Evolve Predator")
        print("==============================================")
        count_episode += 1
        print('episode:', count_episode)
        observations = env.reset()
        done = False        
        
        #nets_prey = copy.deepcopy(best_nets_prey)
        step = 0             
        
        set_event_episode_start()

        fitnesses = [0.0] * NUM_PREDATORS
        
    else:
        print("==============================================")
        print("Evolve Prey")
        print("==============================================")        
        print('episode:', count_episode)
        observations = env.reset()
        done = False        
        
        #nets = copy.deepcopy(best_nets)
        step = 0

        set_event_episode_start_prey()
        
        fitnesses_prey = [0.0] * NUM_PREY
            
    while not done and not rospy.is_shutdown():
        action = np.zeros((NUM_PREDATORS + NUM_PREY, 2), dtype=int)
        step += 1
        
        for predator_idx, obs in enumerate(observations.predator):
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
            if EVO_FLAG == 0:    
                action[predator_idx,:] = nets[predator_idx](torch.FloatTensor(obs)).numpy()
            else:
                action[predator_idx,:] = best_nets[predator_idx](torch.FloatTensor(obs)).numpy()
            
        for prey_idx, obs in enumerate(observations.prey):
            if obs == None:
                continue
            obs_img = obs[-1]
            #obs = obs[0:8]
            obs = np.array([0.0, 0.0, -1.0])
            #obs = np.concatenate((obs, np.array([0.0, 0.0, -1.0])), axis=None)
            
            if type(obs_img) != type(None):
                rects, detect_img = robobo_detection.detect_robobo(obs_img)
                        
                window_name = "prey" + str(prey_idx + 1) + "_image"
                cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
                cv2.moveWindow(window_name, 390, 30 + prey_idx * 350)                
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
                    if target_name == "Predator":                                          
                        #fitnesses_prey[prey_idx] += -area / (abs(rect[0] - (detect_img_width / 2)) / detect_img_width + 1)
                        obs[-3] = rect[0] / (detect_img_width / 2) - 1.0 # -1.0 - 1.0 
                        obs[-2] = rect[1] / (detect_img_height / 2) - 1.0
                        obs[-1] = area / detect_img_area                                                                            
                
                if len(rects) == 0: 
                    fitnesses_prey[prey_idx] += -detect_img_width * detect_img_height * 3
                        
            if EVO_FLAG == 1:
                action[prey_idx + NUM_PREDATORS,:] = nets_prey[prey_idx](torch.FloatTensor(obs)).numpy()                               
            else:
                action[prey_idx + NUM_PREDATORS,:] = best_nets_prey[prey_idx](torch.FloatTensor(obs)).numpy()
                
        observations, reward, done, _ = env.step(action)

    print('obs:', obs)

    for predator_idx, _ in enumerate(observations.predator):
        fitnesses[predator_idx] = fitnesses[predator_idx] / step
        
    for prey_idx, _ in enumerate(observations.prey):
        fitnesses_prey[prey_idx] = fitnesses_prey[prey_idx] / step

    if DEBUG_FITNESS:
        print("fitnesses???", fitnesses)
        print("fitnesses_prey???", fitnesses_prey)

    if EVO_FLAG == 0:
        EVO_FLAG = 1
        if DEBUG_THREAD:
            print("Clean start event")
        event_episode_start.clear()

        if DEBUG_THREAD:
            print("Save fitness info")
            print("Episode end")
        event_episode_end.set()
    else:
        EVO_FLAG = 0
        
        if DEBUG_THREAD:
            print("Clean start event prey")
        event_episode_start_prey.clear()

        if DEBUG_THREAD:
            print("Save fitness info")
            print("Episode end prey")
        event_episode_end_prey.set()

    
        
        
    
    
