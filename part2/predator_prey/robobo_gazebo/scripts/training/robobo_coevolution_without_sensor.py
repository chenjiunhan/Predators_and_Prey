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
import os
import neat
import random
import pickle
import math

DEBUG_THREAD = False
DEBUG_WEIGHT = True
DEBUG_FITNESS = True
DEBUG_IMAGE = False
DEBUG_MANUAL = False
DEBUG_INOUT = False
REAL = False

LOAD_WEIGHT = False
LOADED_GENERATION = 0

NUM_PREDATORS = 3
NUM_PREY = 1
NUM_AGENTS = NUM_PREDATORS + NUM_PREY

IR_MAX_RANGE = 0.2
MAX_SPEED = 20

ARENA_LENGTH = 4

AGENT_IMAGE_MAX = 60000

class DummyPlayer():
    def activate(self, x):
        return np.array([0.0, 0.0])

class EvolvedAgent:
    
    def __init__(self, agent_type, name, ea_framework, idx):
        
        self.nets = []
        self.genomes = []
        self.genome_ids = []
        self.configs = []
        self.fitnesses = []
        
        self.prev_nets = []
        self.prev_genomes = []
        self.prev_genome_ids = []
        self.prev_configs = []
        self.prev_fitnesses = []
        
        self.ea_framework = ea_framework
        self.evolved = False
        
        
        
        self.idx = idx
        
        self.type = agent_type                        # type = 0, type = 1
        
        # 兩個evolution_event 是由ea_framework決定
        self.evolution_event = threading.Event()
        self.evolution_end_event = threading.Event()

        self.evolution_event.daemon = True
        self.evolution_end_event.daemon = True
        
        self.name = name
        self.neat = None
        
        local_dir = os.path.dirname(__file__)
        if agent_type == 0:
            config_path = os.path.join(local_dir, 'config_predator')
        else:
            config_path = os.path.join(local_dir, 'config_prey')
        
        config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)
        if not LOAD_WEIGHT:
            p = neat.Population(config)  
            self.neat = p
        else:
            p = neat.Checkpointer.restore_checkpoint('checkpoints/' + self.name + '/neat-checkpoint-' + str(LOADED_GENERATION))
            self.neat = p
            pkl_file = open('generations/' + self.name + "/" + str(self.neat.generation - 1) + '.pkl', 'rb')
            data_dict = pickle.load(pkl_file)
            
            self.nets = data_dict['nets']
            self.genomes = data_dict['genomes']
            self.genome_ids = data_dict['genome_ids']
            self.configs = data_dict['configs']
            self.fitnesses = data_dict['fitnesses']
            
    
        self.neat.add_reporter(neat.StdOutReporter(True))
        stats = neat.StatisticsReporter()
        self.neat.add_reporter(stats)
        save_dir = "checkpoints/" + self.name
        save_prefix = save_dir + "/neat-checkpoint-"
        
        self.fitness_dir = "fitness/" + self.name
        self.fitness_path = self.fitness_dir + "/fitness.txt"
        
        if not os.path.exists(save_dir):
            os.mkdir(save_dir)
            
        if not os.path.exists(self.fitness_dir):
            os.mkdir(self.fitness_dir)                    
        
        if os.path.exists(self.fitness_path):
            os.remove(self.fitness_path)
        
        self.neat.add_reporter(neat.Checkpointer(1, filename_prefix=save_prefix))
        
        
            
    def agent_thread(self):
        self.neat.run(self.eval_genomes, 300)
            
    def eval_genomes(self, genomes, config):
    
        self.ea_framework.generation_loaded.clear()
        self.ea_framework.count_generation_loaded = 0
    
        if DEBUG_THREAD:
            print("G start!")
                   
        self.ea_framework.count_generation_start += 1
        print("generation_start WAIT")
        
        if self.ea_framework.count_generation_start == self.ea_framework.num_agents:
            self.ea_framework.generation_start.set()
        
        self.ea_framework.generation_start.wait()
        print("generation_start")                
        
        print("DDDDDDDDDEPPP copy")
        self.prev_nets = copy.deepcopy(self.nets)
        self.prev_genomes = copy.deepcopy(self.genomes)
        self.prev_genome_ids = copy.deepcopy(self.genome_ids)
        self.prev_configs = copy.deepcopy(self.configs)
        self.prev_fitnesses = copy.deepcopy(self.fitnesses)
        
        if self.neat.generation > 0:
        
            data_dict = {'nets': self.prev_nets, 'genomes': self.prev_genomes, 'genome_ids': self.prev_genome_ids, 
                         'configs': self.prev_configs, 'fitnesses': self.prev_fitnesses, 'generation': self.neat.generation - 1}
            output = open('generations/' + self.name + "/" + str(self.neat.generation - 1) + '.pkl', 'wb')
            pickle.dump(data_dict, output)
        
        print("GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG", self.neat.generation)
        
        print("CLEAR ODDDDDDDDDDDDDDDDDDDL")
        self.nets = []
        self.genomes = []
        self.genome_ids = []
        self.configs = []
        self.fitnesses = []
              
        self.ea_framework.count_generation_loaded += 1
        
        if self.ea_framework.count_generation_loaded == self.ea_framework.num_agents:
            self.ea_framework.generation_loaded.set()
        
        self.ea_framework.generation_loaded.wait()
        
                
        self.ea_framework.generation_start.clear()
        self.ea_framework.count_generation_start = 0
        
        for genome_id, genome in genomes:
        
            print("genome_idgenome_idgenome_id", genome_id, self.name)
        
            #if len(genomes) != 3:
            print("len(genomes)", len(genomes))
        
            if DEBUG_THREAD:
                print("WAIT START", self.name)          
            
            self.evolution_event.wait() 
                                        
                        
            if DEBUG_THREAD:
                print("WAIT END", self.name)             
            
            new_net = neat.nn.FeedForwardNetwork.create(genome, config)
            
            self.nets += [new_net]
            self.genomes += [genomes]
            self.genome_ids += [genome_id]
            self.configs += [config]              
            
            if DEBUG_WEIGHT:
                print("genome start", genome)
                print("genome end")
                
            self.evolution_event.clear()                                                
                        
            if DEBUG_THREAD:
                print("CLEAR", self.name)
            
            # wait end
                       
            self.ea_framework.game_start.set()
                
            if DEBUG_THREAD:
                print("Set game start")
            
            print("Evolve:", self.name)
            
            if DEBUG_THREAD:
                print("evolution_end_event.wait()")
                
            self.evolution_end_event.wait()
            
            genome.fitness = copy.deepcopy(self.fitnesses[-1])
            
            if DEBUG_FITNESS:
                print("Fitness genome:", genome.fitness, self.name)
            
            if DEBUG_THREAD:
                print("evolution_end_event.clear()")
                            
            self.evolution_end_event.clear()
            
            '''net = neat.nn.FeedForwardNetwork.create(genome, config)
            for xi, xo in zip(xor_inputs, xor_outputs):
                output = net.activate(xi)
                genome.fitness -= (output[0] - xo[0]) ** 2'''
                
    def select_player(self, evolved = False):        
    
        print("Now GEMO!!!!!!!!!!!!!!d", self.genome_ids, self.name)
        print("Now fitness", self.fitnesses, self.name)
    
        if evolved:
            return self.nets[-1]
        else:
            if len(self.prev_nets) == 0:
                if len(self.nets) == 0:            
                    return DummyPlayer()
                else:
                    return random.choice(self.nets)
            else:                              
            
                print("gemoi!!!!!!!!!!!!!!d", self.prev_genome_ids, self.name)
                print("prev_fitnesses!!!!!!!!!!!!!!d", self.prev_fitnesses, self.name)
            
                prev_fitnesses = np.array(self.prev_fitnesses)
                idx = np.argmax(prev_fitnesses)                                
                                        
                #if DEBUG_FITNESS:
                #    print("best_prev", prev_fitnesses[idx], prev_fitnesses)
                
                return self.prev_nets[idx]

    def set_fitness(self, fitness):
        #if DEBUG_FITNESS:
        print("set fitness:", fitness, self.name)
        
        
        f = open(self.fitness_path, "a+")
        f.write(str(fitness) + " ")
        f.close()
        self.fitnesses += [fitness]
        print("self.fitnesses:", self.fitnesses, self.name)
        

    


class EAframework:

    def __init__(self, num_predators, num_prey):
        
        self.predators = []
        self.prey = []
        
        # game start是由進化的agent決定
        self.game_start = threading.Event()
        # generation start是由所有agents決定
        self.generation_start = threading.Event() 
        
        # generation_loaded是由所有agents決定
        self.generation_loaded = threading.Event() 
        
        self.count_generation_start = 0
        self.count_generation_loaded = 0
        
        self.target_count = 0
        self.target = None      
        
        self.num_predators = num_predators
        self.num_prey = num_prey
        self.num_agents = self.num_predators + self.num_prey
                        
        for i in range(num_predators):       
            self.predators += [EvolvedAgent(0, "predator" + str(i), self, len(self.predators))];
            
        for i in range(num_prey):
            self.prey += [EvolvedAgent(1, "prey", self, len(self.prey))];                            
        
        self.init_agents()
        
    def init_agents(self):
                        
        for predator in self.predators:            
            t = threading.Thread(target = predator.agent_thread, args = ())
            t.daemon = True
            t.start()
            
        for prey in self.prey:            
            t = threading.Thread(target = prey.agent_thread, args = ())
            t.daemon = True
            t.start()

    def start_evolve(self):
        
        
        
        if REAL:
            env = gym.make('gym_robobo_predator_prey_real-v0')
        else:
            env = gym.make('gym_robobo_predator_prey-v0')

        self.set_next_evolution_target()                        
        
        while not rospy.is_shutdown():            
        
            
            
            self.game_start.wait()
            if DEBUG_THREAD:
                print("start game")
            self.game_start.clear()                                                          
                 
            play_net = []                        
            
            for i in range(NUM_PREDATORS):
                if self.predators[i].name == self.target.name:
                    play_net += [self.predators[i].select_player(True)]
                else:
                    play_net += [self.predators[i].select_player()]
                    
            for i in range(NUM_PREY):            
                if self.prey[i].name == self.target.name:
                    play_net += [self.prey[i].select_player(True)]
                else:
                    play_net += [self.prey[i].select_player()]
                 
            done = False
            step = 0
            observations = env.reset()
            fitness = 0.0
            look_at_fitness = 0.0
            info = None
            init_distance = None
                 
            s0_avg = 0.0
            s1_avg = 0.0
            s2_avg = 0.0
            sp_avg = 0.0
                 
            while not done and not rospy.is_shutdown():
                action = np.zeros((NUM_PREDATORS + NUM_PREY, 2), dtype=int)
                step += 1
                
                predator0_input = []
                predator1_input = []
                predator2_input = []
                prey_input = []
                
                if type(info) != type(None):
                    #print(look_at_fitness/step)
                    if (info["time"] > 3 and info["time"] < 5 and info[self.target.name + "_position"].y < -1.4) or (
                        info["time"] > 3 and look_at_fitness < 0.2): 
                        pass
                        
                s0 = self.compute_speed(env.predators[0])      
                s1 = self.compute_speed(env.predators[1])
                s2 = self.compute_speed(env.predators[2])
                sp = self.compute_speed(env.prey[0])
                
                s0_avg += s0
                s1_avg += s1
                s2_avg += s2
                sp_avg += sp
                        
                d01 = self.compute_distance(env.predators[0], env.predators[1]) / (ARENA_LENGTH * np.sqrt(2))
                d02 = self.compute_distance(env.predators[0], env.predators[2]) / (ARENA_LENGTH * np.sqrt(2))
                d0p = self.compute_distance(env.predators[0], env.prey[0]) / (ARENA_LENGTH * np.sqrt(2))
                d12 = self.compute_distance(env.predators[1], env.predators[2]) / (ARENA_LENGTH * np.sqrt(2))
                d1p = self.compute_distance(env.predators[1], env.prey[0]) / (ARENA_LENGTH * np.sqrt(2))
                d2p = self.compute_distance(env.predators[2], env.prey[0]) / (ARENA_LENGTH * np.sqrt(2))
                        
                y01 = self.compute_face_yaw_diff(env.predators[0], env.predators[1]) / np.pi
                y02 = self.compute_face_yaw_diff(env.predators[0], env.predators[2]) / np.pi
                y0p = self.compute_face_yaw_diff(env.predators[0], env.prey[0]) / np.pi
                y10 = self.compute_face_yaw_diff(env.predators[1], env.predators[0]) / np.pi
                y12 = self.compute_face_yaw_diff(env.predators[1], env.predators[2]) / np.pi
                y1p = self.compute_face_yaw_diff(env.predators[1], env.prey[0]) / np.pi
                y20 = self.compute_face_yaw_diff(env.predators[2], env.predators[0]) / np.pi
                y21 = self.compute_face_yaw_diff(env.predators[2], env.predators[1]) / np.pi
                y2p = self.compute_face_yaw_diff(env.predators[2], env.prey[0]) / np.pi
                yp0 = self.compute_face_yaw_diff(env.prey[0], env.predators[0]) / np.pi
                yp1 = self.compute_face_yaw_diff(env.prey[0], env.predators[1]) / np.pi
                yp2 = self.compute_face_yaw_diff(env.prey[0], env.predators[2]) / np.pi
                
                env.predators[0], env.predators[1]
                
                '''
                print("d01", d01)
                print("d02", d02)
                print("d0p", d0p)
                print("d12", d12)
                print("d1p", d1p)
                print("d2p", d2p)
                
                print("y01", y01)
                print("y02", y02)
                print("y0p", y0p)
                print("y10", y10)
                print("y12", y12)
                print("y1p", y1p)
                
                print("y20", y20)
                print("y21", y21)
                print("y2p", y2p)
                print("yp0", yp0)
                print("yp1", yp1)
                print("yp2", yp2)
                '''                                
                
                obs0 = np.array([d0p, y0p])
                obs1 = np.array([d1p, y1p])
                obs2 = np.array([d2p, y2p])
                obsp = np.array([d0p, d1p, d2p, yp0, yp1, yp2])
                
                for predator_idx, obs in enumerate(observations.predator):                
                
                    if obs == None:
                        continue   
                        
                    if predator_idx == 0:
                        obs0 = np.concatenate((np.array([obs[2]]), obs0))
                    elif predator_idx == 1:
                        obs1 = np.concatenate((np.array([obs[2]]), obs1))
                    elif predator_idx == 2:
                        obs2 = np.concatenate((np.array([obs[2]]), obs2))                    
                  
                for prey_idx, obs in enumerate(observations.prey):
                    if obs == None:
                        continue  
                    
                    obsp = np.concatenate((np.array([obs[2]]), obsp))  
                    
                #print("obs0", obs0)
                #print("obs1", obs1)
                #print("obs2", obs2)
                #print("obsp", obsp)
                
                if DEBUG_MANUAL:
                    action[0,:] = np.array(play_net[0].activate(obs0), dtype=float) * 0
                    action[1,:] = np.array(play_net[1].activate(obs1), dtype=float) * 0
                    action[2,:] = np.array(play_net[2].activate(obs2), dtype=float) * 0
                    action[3,:] = np.array(play_net[3].activate(obsp), dtype=float) * 0
                else:
                    action[0,:] = np.array(play_net[0].activate(obs0), dtype=float) * MAX_SPEED
                    action[1,:] = np.array(play_net[1].activate(obs1), dtype=float) * MAX_SPEED
                    action[2,:] = np.array(play_net[2].activate(obs2), dtype=float) * MAX_SPEED
                    action[3,:] = np.array(play_net[3].activate(obsp), dtype=float) * MAX_SPEED
                
                '''
                for predator_idx, obs in enumerate(observations.predator):                
                
                    if obs == None:
                        continue                        

                    obs_img = None
                    #print("obs", obs)
                    #obs = np.array([obs[2], obs[5], obs[7], obs[8], obs[9]])
                    #obs_img_feature = np.array([0.0, -1.0, 0.0, -1.0, 0.0, -1.0])
                    obs = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    #obs = np.array([0.0, -1.0])
                    #obs = np.concatenate((obs_img_feature, obs))
                    #obs = np.concatenate((obs, np.array([0.0, 0.0, -1.0])), axis=None)
                    
                    if type(obs_img) != type(None):
                        rects, detect_img = robobo_detection.detect_robobo(obs_img, real=True)
                                
                        window_name = "predator" + str(predator_idx + 1) + "_image"
                        if DEBUG_IMAGE:
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
                        
                        count_predators = 0
                        feature1 = []
                        feature2 = []
                            
                        for rect in rects:
                            target_name = rect[4]
                            area = rect[2] * rect[3]
                            if target_name == "Prey" and area > 200:
                                #print('area:', area, predator_idx)
                                
                                #if self.predators[predator_idx].name == self.target.name:
                             
                                #    fitness += area / (abs(rect[0] - (detect_img_width / 2)) + detect_img_area)
                                    
                                #if DEBUG_FITNESS:
                                #    print('FFFF', area / (abs(rect[0] - (detect_img_width / 2)) + 1.0) ** 2)                                                                    
                                
                                obs[0] = rect[0] / (detect_img_width) - 0.5 # -1.0 - 1.0 
                                #obs[-2] = rect[1] / (detect_img_height / 2) - 1.0
                                obs[1] = area / AGENT_IMAGE_MAX
                            elif target_name == "Predator" and area > 200:
                            
                                feature1 += [rect[0] / (detect_img_width / 2) - 1.0]
                                feature2 += [area / AGENT_IMAGE_MAX]                                                                                          
                        
                        if self.predators[predator_idx].name == self.target.name:
                            if obs[1] > 0:
                                look_at_fitness += 1.0
                                
                        feature2_sorted_idx = sorted(range(len(feature2)), key=lambda k: feature2[k], reverse=True)
                                                  
                                                          
                    if DEBUG_MANUAL:
                        action[predator_idx,:] = np.array(play_net[predator_idx].activate(obs), dtype=float) * 0
                    else:
                        action[predator_idx,:] = np.array(play_net[predator_idx].activate(obs), dtype=float) * MAX_SPEED
                    #if predator_idx == 1: 
                    #    print("predator input:", obs)    
                    #print("predator output:", np.array(play_net.activate(obs), dtype=float) * MAX_SPEED)          
                                                  
                    
                    
                    if DEBUG_INOUT:
                        print("predator input:", predator_idx, obs)    
                        print("predator output:", predator_idx, np.array(play_net[predator_idx].activate(obs), dtype=float) * MAX_SPEED)
                    
                for prey_idx, obs in enumerate(observations.prey):
                    if obs == None:
                        continue
                    obs_img = obs[-1]
                    
                    #obs = np.array([obs[2], obs[5], obs[7], obs[8], obs[9]])
                    #obs_img_feature = np.array([0.0, -1.0, 0.0, -1.0, 0.0, -1.0])
                    #obs_img_feature = np.array([0.0, -1.0, 0.0, -1.0, 0.0, -1.0])
                    obs = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    #obs_img_feature = np.array([0.0, -1.0])
                    #obs = np.concatenate((obs_img_feature, obs))
                    #obs = np.array([0.0, -1.0])
                    
                    count_predators = 0
                    feature1 = []
                    feature2 = []
                    
                    if type(obs_img) != type(None):
                        rects, detect_img = robobo_detection.detect_robobo(obs_img, real=True)
                                
                        window_name = "prey" + str(prey_idx + 1) + "_image"
                        
                        if DEBUG_IMAGE:
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
    
                                #if self.prey[prey_idx].name == self.target.name:                                       
                                #    fitness -= area / detect_img_area
    
                                feature1 += [rect[0] / (detect_img_width / 2) - 1.0]
                                feature2 += [area / AGENT_IMAGE_MAX]
                                    
                                #obs[count_predators * 2] = rect[0] / (detect_img_width / 2) - 1.0                              
                                #obs[count_predators * 2 + 1] = area / MAX_SPEED000                                                                
                            
                        # big to small                                   
                        feature2_sorted_idx = sorted(range(len(feature2)), key=lambda k: feature2[k], reverse=True)
                        
                        
                        for idx in feature2_sorted_idx:
                            obs[count_predators * 2] = feature1[idx]
                            obs[count_predators * 2 + 1] = feature2[idx]
                            count_predators += 1
                            if count_predators == 1:
                                break                                                    
                        
                        
                            #fitness -= miss_num
                        
                        #if len(rects) == 0: 
                        #    fitnesses_prey[prey_idx] += -detect_img_width * detect_img_height * 3
                                
                    
                                          
                    if self.prey[prey_idx].name == self.target.name:
                        if obs[1] > 0:
                            look_at_fitness += 1.0
                                          
                    #action[prey_idx + NUM_PREDATORS,:] = np.array(play_net.activate(obs), dtype=float) * MAX_SPEED
                    if DEBUG_MANUAL:
                        action[prey_idx + NUM_PREDATORS,:] = np.array(play_net[prey_idx + NUM_PREDATORS].activate(obs), dtype=float) * 0
                    else:
                        action[prey_idx + NUM_PREDATORS,:] = np.array(play_net[prey_idx + NUM_PREDATORS].activate(obs), dtype=float) * MAX_SPEED
                        
                    if DEBUG_INOUT:
                        print("prey input:", obs)    
                        print("prey output:", np.array(play_net[prey_idx + NUM_PREDATORS].activate(obs), dtype=float) * MAX_SPEED)
                        
                '''
                    
                observations, reward, done, info = env.step(action)
                
            s0_avg = s0_avg / step
            s1_avg = s1_avg / step
            s2_avg = s2_avg / step
            sp_avg = sp_avg / step
                
            print('s0_avg', s0_avg)
            print('s1_avg', s1_avg)
            print('s2_avg', s2_avg)
            print('sp_avg', sp_avg)
                
            if self.target.name == self.prey[0].name:
                print("Prey fitness!!!")
                fitness = reward['prey']
                fitness = fitness * sp_avg
                #look_at_fitness = look_at_fitness / step
                #fitness = fitness * look_at_fitness
            else:                
                print("Predator fitness", reward['predators'], info["distances"][self.target.idx], info["arena_length"] - info["distances"][self.target.idx])
                fitness = reward['predators'] * (info["arena_length"] - info["distances"][self.target.idx])
                if self.target.idx == 0:
                    fitness = fitness * s0_avg * ((env.period - info['time'] + 1) / (env.period))
                elif self.target.idx == 1:
                    fitness = fitness * s1_avg * ((env.period - info['time'] + 1) / (env.period))
                elif self.target.idx == 2:
                    fitness = fitness * s2_avg * ((env.period - info['time'] + 1) / (env.period))
                #look_at_fitness = look_at_fitness / step
                #fitness = fitness * look_at_fitness
                
            if DEBUG_FITNESS:
                print("time fitness factor:", (env.period - info['time'] + 1) / (env.period))
                print("look_at_fitness:", look_at_fitness)
                print("Final fitness", fitness)           
                              
           
            self.target.set_fitness(fitness)
            
            print("Total step:", step)
            
            if DEBUG_THREAD:
                print("end game")
            self.set_next_evolution_target()
                
    def set_next_evolution_target(self):

        if self.target == None:
            self.target = self.predators[self.target_count]
        else:
            if DEBUG_THREAD:
                print("evolution_event_end.set()")
            self.target.evolution_end_event.set()
        
            self.target_count = (self.target_count + 1) % self.num_agents
            if self.target_count >= self.num_predators: # Prey
               self.target = self.prey[self.target_count - self.num_predators]
            else:
               self.target = self.predators[self.target_count]
        if DEBUG_THREAD:
            print("evolution_event.set()")

        for i in range(NUM_PREDATORS):
            self.predators[i].evolved = False
                
        for i in range(NUM_PREY):            
            self.prey[i].evolved = False
            
        self.target.evolved = True
        self.target.evolution_event.set()

    def compute_distance(self, agent1, agent2):
        return np.sqrt((agent1.position.x - agent2.position.x) ** 2 + (agent1.position.y - agent2.position.y) ** 2)
        
    def compute_speed(self, agent):
    
        if type(agent.prev_time) == type(None) or type(agent.prev_position) == type(None):
            return 0.0
            
        if abs(agent.time - agent.prev_time) < 1e-6:
            return 0.0
            
        #print("SPPPEEED", np.sqrt((agent.prev_position.x - agent.position.x) ** 2 + (agent.prev_position.y - agent.position.y) ** 2) / (agent.time - agent.prev_time))
            
        return np.sqrt((agent.prev_position.x - agent.position.x) ** 2 + (agent.prev_position.y - agent.position.y) ** 2) / (agent.time - agent.prev_time)
        
    def yaw2vector(self, yaw): # y==0
        y = np.cos(yaw)
        x = -np.sin(yaw)
        
        #print("x0y0", x, y)
        
        return x, y
        
    def compute_vector(self, agent1, agent2):
    
        x_diff = agent2.position.x - agent1.position.x
        y_diff = agent2.position.y - agent1.position.y
        norm = np.sqrt(x_diff * x_diff + y_diff * y_diff)
    
        return x_diff / norm, y_diff / norm
       
    def vector2yaw(self, x, y):
        
        if y < 0:            
            if x > 0:
                yaw = np.pi - math.atan(-x/y)
            else:
                yaw = -np.pi - math.atan(-x/y)
        elif y > 0:
            yaw = -math.atan(-x/y)
        else:
            if x > 0:
                yaw = -np.pi / 2
            else:
                yaw = np.pi / 2
        
        yaw = -yaw        
                
        return yaw        
        
    def compute_face_yaw_diff(self, agent1, agent2):
        yaw1, _, _ = self.quaternion_to_euler(agent1)
        
        #if agent1.model_name == "prey":
        #    print("Yaw1", yaw1, agent1.model_name)
        
        x0, y0 = self.yaw2vector(yaw1)
        
        v0 = np.array([x0, y0, 0.0])
        
        x1, y1 = self.compute_vector(agent1, agent2)
        
        v1 = np.array([x1, y1, 0.0])
        
        cross = np.cross(v1, v0)
        theta = np.arcsin(np.linalg.norm(cross))
        
        if cross[2] < 0.0:
            theta = -theta
        
        cos = v1.dot(v0)
        if cos < 0.0:
            theta = (-theta) + np.pi * theta / abs(theta)
        
        theta = -theta
        
        #print("theta", theta)
        
        return theta
        
    def quaternion_to_euler(self, agent):
        x = agent.orientation.x
        y = agent.orientation.y
        z = agent.orientation.z
        w = agent.orientation.w
        
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
            
                    

ea = EAframework(NUM_PREDATORS, NUM_PREY)
ea.start_evolve()
