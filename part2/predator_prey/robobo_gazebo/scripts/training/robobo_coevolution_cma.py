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

DEBUG_THREAD = False
DEBUG_WEIGHT = True
DEBUG_FITNESS = True
DEBUG_IMAGE = False
DEBUG_MANUAL = False
DEBUG_INOUT = False
REAL = False

LOAD_WEIGHT = False
LOADED_GENERATION = 1

NUM_PREDATORS = 3
NUM_PREY = 1
NUM_AGENTS = NUM_PREDATORS + NUM_PREY

IR_MAX_RANGE = 0.2
MAX_SPEED = 20


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
            
            while not done and not rospy.is_shutdown():
                action = np.zeros((NUM_PREDATORS + NUM_PREY, 2), dtype=int)
                step += 1
                
                if type(info) != type(None):
                    #print(look_at_fitness/step)
                    if (info["time"] > 3 and info["time"] < 5 and info[self.target.name + "_position"].y < -1.4) or (
                        info["time"] > 3 and look_at_fitness < 0.2): 
                        break
                
                for predator_idx, obs in enumerate(observations.predator):                
                
                    if obs == None:
                        continue                        

                    obs_img = obs[-1]
                    #print("obs", obs)
                    #obs = np.array([obs[2], obs[5], obs[7], obs[8], obs[9]])
                    #obs_img_feature = np.array([0.0, -1.0, 0.0, -1.0, 0.0, -1.0])
                    obs = np.array([obs[2], obs[5], obs[7]])
                    obs_img_feature = np.array([0.0, -1.0])
                    #obs = np.array([0.0, -1.0])
                    obs = np.concatenate((obs_img_feature, obs))
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
                        '''
                        for idx in feature2_sorted_idx:
                            count_predators += 1
                            obs[count_predators * 2] = feature1[idx]
                            obs[count_predators * 2 + 1] = feature2[idx]
                            
                            if count_predators == 2:
                                break
                        '''
                        
                    '''if self.predators[predator_idx].name == self.target.name:
                        play_net = self.predators[predator_idx].select_player(True)
                    else:
                        play_net = self.predators[predator_idx].select_player()'''                              
                                                          
                    if DEBUG_MANUAL:
                        action[predator_idx,:] = np.array(play_net[predator_idx].activate(obs), dtype=float) * 0
                    else:
                        action[predator_idx,:] = np.array(play_net[predator_idx].activate(obs), dtype=float) * MAX_SPEED
                    #if predator_idx == 1: 
                    #    print("predator input:", obs)    
                    #print("predator output:", np.array(play_net.activate(obs), dtype=float) * MAX_SPEED)          
                                                  
                    '''if EVO_FLAG == 0:    
                        action[predator_idx,:] = nets[predator_idx](torch.FloatTensor(obs)).numpy()
                    else:
                        action[predator_idx,:] = best_nets[predator_idx](torch.FloatTensor(obs)).numpy()'''
                    
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
                    obs = np.array([obs[2], obs[5], obs[7]])
                    obs_img_feature = np.array([0.0, -1.0])
                    obs = np.concatenate((obs_img_feature, obs))
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
                        
                        '''if self.prey[prey_idx].name == self.target.name:   
                            miss_num = (self.num_predators - len(rects))
                            if miss_num <= 0:
                                miss_num = 0
                        ''' 
                            #fitness -= miss_num
                        
                        #if len(rects) == 0: 
                        #    fitnesses_prey[prey_idx] += -detect_img_width * detect_img_height * 3
                                
                    '''if self.prey[prey_idx].name == self.target.name:                        
                        play_net = self.prey[prey_idx].select_player(True)
                    else:
                        play_net = self.prey[prey_idx].select_player()'''
                                          
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
                        
                
                    
                observations, reward, done, info = env.step(action)
                
            if self.target.name == self.prey[0].name:
                print("Prey fitness!!!")
                fitness = reward['prey']
                look_at_fitness = look_at_fitness / step
                fitness = fitness * look_at_fitness
            else:                
                print("Predator fitness", reward['predators'], info["distances"][self.target.idx], info["arena_length"] - info["distances"][self.target.idx])
                fitness = reward['predators'] * (info["arena_length"] - info["distances"][self.target.idx])           
                look_at_fitness = look_at_fitness / step
                fitness = fitness * look_at_fitness
                
            if DEBUG_FITNESS:
                print("look_at_fitness:", look_at_fitness)
                print("fitnessfitness", fitness)
           
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

        
            
                    

ea = EAframework(NUM_PREDATORS, NUM_PREY)
ea.start_evolve()
