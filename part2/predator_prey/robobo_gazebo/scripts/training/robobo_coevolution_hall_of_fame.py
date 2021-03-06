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
import pygame
import pygame.locals

DEBUG_THREAD = False
DEBUG_WEIGHT = True
DEBUG_FITNESS = True
DEBUG_IMAGE = False
DEBUG_MANUAL = False
DEBUG_INOUT = False
REAL = False
SAVE_IMAGE = False
EVALUATION = False
HUMAN = False
EVOLVE_PREY = True

LOAD_WEIGHT = False
LOADED_GENERATION = 37

NUM_EVALUATION = 5

PW_RATIO = 0.4

NUM_PREDATORS = 3
NUM_PREY = 1
NUM_AGENTS = NUM_PREDATORS + NUM_PREY

IR_MAX_RANGE = 0.2
if REAL:
    MAX_SPEED = 30
else:
    MAX_SPEED = 10
    
EVOVLED_PREY_SPEED_FACTOR = 1.5


PREDATOR_SIGMA = 0.5
WALL_SIGMA = 0.4

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
        
        self.best_nets = []
        
        self.ea_framework = ea_framework
        self.evolved = False
        
        
        self.human_left = 0.0
        self.human_right = 0.0
        
        self.idx = idx
        
        self.type = agent_type                        # type = 0, type = 1
        
        # 兩個evolution_event 是由ea_framework決定
        self.evolution_event = threading.Event()
        self.evolution_end_event = threading.Event()

        self.evolution_event.daemon = True
        self.evolution_end_event.daemon = True
        
        self.name = name
        self.neat = None
        
        self.orientation = None
        self.position = None
        
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
            self.best_nets = data_dict['best_nets']
        
        if not EVALUATION:
                
            self.neat.add_reporter(neat.StdOutReporter(True))
            stats = neat.StatisticsReporter()
            self.neat.add_reporter(stats)
            save_dir = "checkpoints/" + self.name
            save_prefix = save_dir + "/neat-checkpoint-"
            self.neat.add_reporter(neat.Checkpointer(1, filename_prefix=save_prefix))                
            
        
        
            if not os.path.exists(save_dir):
                os.mkdir(save_dir)
                
        self.fitness_dir = "fitness/" + self.name
        self.fitness_path = self.fitness_dir + "/fitness.txt"                
                
        if not os.path.exists(self.fitness_dir):
            os.mkdir(self.fitness_dir)                                
        
        if os.path.exists(self.fitness_path):
            os.remove(self.fitness_path)       
            
            
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
            idx = np.argmax(self.prev_fitnesses)       
            self.best_nets += [self.prev_nets[idx]]
        
            data_dict = {'nets': self.prev_nets, 'genomes': self.prev_genomes, 'genome_ids': self.prev_genome_ids, 
                         'configs': self.prev_configs, 'fitnesses': self.prev_fitnesses, 'generation': self.neat.generation - 1, 'best_nets': self.best_nets}
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
    
        if EVALUATION:
            prev_fitnesses = np.array(self.prev_fitnesses)
            idx = np.argmax(prev_fitnesses)                                               
            
            return self.prev_nets[idx]
    
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

    def fixed_strategy(self, x_list, y_list, sigma_x_list, sigma_y_list, w_x_list, w_y_list, w_sigma_x_list, w_sigma_y_list, target_x, target_y, prey_orientation, PW_RATIO = 0.1): 
        
        x = 0.0
        y = 0.0
        
        #print('yyyyyyyyyy', y_list, target_y)
        
        for idx, value in enumerate(x_list):
            x += 1 / (2 * np.pi * sigma_x_list[idx] * sigma_y_list[idx]) * \
                 np.exp(-(target_x - x_list[idx])**2 / (2*sigma_x_list[idx]**2)) * \
                 np.exp(-(target_y - y_list[idx])**2 / (2*sigma_y_list[idx]**2)) * \
                 -(target_x - x_list[idx])/(sigma_x_list[idx]**2) * PW_RATIO

            y += 1 / (2 * np.pi * sigma_x_list[idx] * sigma_y_list[idx]) * \
                 np.exp(-(target_x - x_list[idx])**2 / (2*sigma_x_list[idx]**2)) * \
                 np.exp(-(target_y - y_list[idx])**2 / (2*sigma_y_list[idx]**2)) * \
                 -(target_y - y_list[idx])/(sigma_y_list[idx]**2) * PW_RATIO


        for idx, value in enumerate(w_x_list):
            
            if abs(w_sigma_x_list[idx] - 0.0) > 0.00001:
                x += 1 / (2 * np.pi * w_sigma_x_list[idx]) * \
                     np.exp(-(target_x - w_x_list[idx])**2 / (2*w_sigma_x_list[idx]**2)) * \
                     -(target_x - w_x_list[idx])/(w_sigma_x_list[idx]**2)
            
            if abs(w_sigma_y_list[idx] - 0.0) > 0.00001:
                y += 1 / (2 * np.pi * w_sigma_y_list[idx]) * \
                     np.exp(-(target_y - w_y_list[idx])**2 / (2*w_sigma_y_list[idx]**2)) * \
                     -(target_y - w_y_list[idx])/(w_sigma_y_list[idx]**2)
        
        yaw, _, _ = self.quaternion_to_euler(prey_orientation.x, prey_orientation.y, prey_orientation.z, prey_orientation.w)
                        
        wheel_speed = [0, 0]
        
        grad_x = x
        grad_y = y
        
        #print("prey yaw:", yaw)        
        
        yaw_y = np.cos(yaw)
        yaw_x = -np.sin(yaw)
        p_grad_x = grad_x
        p_grad_y = grad_y 
        g_norm = -np.sqrt(p_grad_x**2 + p_grad_y**2)
        p_grad_x = p_grad_x / g_norm
        p_grad_y = p_grad_y / g_norm
        
        #print('p_grad_x:', p_grad_x, p_grad_y)
        
        yaw_vec = np.array([yaw_x, yaw_y, 0.0])
        grad_vec = np.array([p_grad_x, p_grad_y, 0.0])
        yg_cross = np.cross(yaw_vec, grad_vec)
        yg_dot = np.dot(yaw_vec, grad_vec)        
        
        TURN_SPEED = 1.0
        STRAIGHT_SPEED = 1.0
        
        THRESHOLD = 0.8 
        
        #print('yg_cross', yg_cross)
        #print('yg_dot', yg_dot)
        
        if yg_cross[2] > 0:
            if yg_dot < THRESHOLD:  
                #print("LEFT")  
                wheel_speed[0] = -TURN_SPEED
                wheel_speed[1] = TURN_SPEED
            else:
                #print("STRAIGHT_SPEED") 
                wheel_speed[0] = STRAIGHT_SPEED
                wheel_speed[1] = STRAIGHT_SPEED
        else:
            if yg_dot < THRESHOLD:
                #print("RIGHT")  
                wheel_speed[0] = TURN_SPEED
                wheel_speed[1] = -TURN_SPEED
            else:
                #print("STRAIGHT_SPEED") 
                wheel_speed[0] = STRAIGHT_SPEED
                wheel_speed[1] = STRAIGHT_SPEED
        
        if HUMAN:
            wheel_speed[0] = self.human_left
            wheel_speed[1] = self.human_right
        
        return wheel_speed[0], wheel_speed[1]

    def pk_key_listener(self, name):

        button_delay = 0.01

        while True:
            DELAY = False        
            keys = pygame.key.get_pressed()    
                
            if keys[pygame.K_a]:
                '''if self.human_right > 0.0:
                    self.human_right = 0.8
                    self.human_left = 0.5
                else:
                    self.human_right = -0.5
                    self.human_left = -0.3'''
                    
                    
                self.human_right = 1.0
                self.human_left = -1.0    
                    
                DELAY = True 

            if keys[pygame.K_d]:
            
                '''
                if self.human_right > 0.0:
                    self.human_left = 0.8             
                    self.human_right = 0.5
                else:
                    self.human_left = -0.5
                    self.human_right = -0.3
                '''
                
                self.human_right = -1.0
                self.human_left = 1.0    
                
                DELAY = True
       
            if keys[pygame.K_w]:
                self.human_right = 1.0
                self.human_left = 1.0
                DELAY = True
            
            if keys[pygame.K_s]:   
                self.human_right = -1.0
                self.human_left = -1.0
                DELAY = True                                
                    
            if DELAY:
                time.sleep(button_delay)
                   
            pygame.event.pump()           

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
            
            if HUMAN:    
            
                pygame.init()
                BLACK = (0,0,0)
                WIDTH = 100
                HEIGHT = 100
                windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
                windowSurface.fill(BLACK)
            
                t = threading.Thread(target = prey.pk_key_listener, args = ("pk",))
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
                 
            count_evalution = 0 
            average_fitness = 0.0
            while count_evalution < NUM_EVALUATION and count_evalution <= len(self.prey[0].best_nets): 
                
                prey_bool = False
                if self.prey[0].name == self.target.name:
                    prey_bool = True
            
                if count_evalution == 0 or len(self.prey[0].best_nets) == 0:
                
                    print("LOADNEAT")
                
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
              
                else:
              
                    print("LOADBEST")
              
                    play_net = []
                
                    if prey_bool:
                
                        for i in range(NUM_PREDATORS):                            
                            play_net += [self.predators[i].best_nets[-count_evalution]]
                                
                        for i in range(NUM_PREY):                                        
                            play_net += [self.prey[i].select_player(True)]                            
                            
                    else:
                        
                        for i in range(NUM_PREDATORS):                            
                            if self.predators[i].name == self.target.name:
                                play_net += [self.predators[i].select_player(True)]
                            else:
                                play_net += [self.predators[i].select_player()]
                                
                        for i in range(NUM_PREY):
                            play_net += [self.prey[i].best_nets[-count_evalution]]
              
                count_evalution += 1
                print('count_evalution:', count_evalution)
                done = False
                step = 0
                observations, reward, done, info = env.reset()
                fitness = 0.0
                look_at_fitness = 0.0            
                init_distance = None                
                
                previous_position = np.array([0.0] * (NUM_PREDATORS + NUM_PREY))
                     
                sigma_x_list = np.array([1.0] * NUM_PREDATORS) * PREDATOR_SIGMA
                sigma_y_list = np.array([1.0] * NUM_PREDATORS) * PREDATOR_SIGMA
                w_x_list = [2.0,-2.0,0.0,0.0]
                w_y_list = [0.0,0.0,2.0,-2.0]
                w_sigma_x_list = np.array([1.0,1.0,0.0,0.0]) * WALL_SIGMA
                w_sigma_y_list = np.array([0.0,0.0,1.0,1.0]) * WALL_SIGMA                
                                 
                while not done and not rospy.is_shutdown():    
                  
                    action = np.zeros((NUM_PREDATORS + NUM_PREY, 2), dtype=int)
                    step += 1
                    
                    if type(info) != type(None) and 'time' in info and not EVALUATION:
                        #print(look_at_fitness/step)
                        if self.target.name != 'prey':                    
                            if (info["time"] > 3 and info["time"] < 5 and info[self.target.name + "_position"].y < -1.4) or (
                                info["time"] > 3 and look_at_fitness / step < 0.2): 
                                #print("LLLL:", look_at_fitness/ step)
                                break
                    
                    
                    #print("????????????????", info, step)
                    
                    if not EVOLVE_PREY:
                        x_list = [info['predator0_position'].x, info['predator1_position'].x, info['predator2_position'].x]
                        y_list = [info['predator0_position'].y, info['predator1_position'].y, info['predator2_position'].y]
                        prey_x = info['prey_position'].x
                        prey_y = info['prey_position'].y
                        prey_orientation = info['prey_orientation']
                    
                    for predator_idx, obs in enumerate(observations.predator):                
                    
                        if obs == None:
                            continue                        

                        obs_img = obs[-1]
                        #print("obs", obs)
                        #obs = np.array([obs[2], obs[5], obs[7], obs[8], obs[9]])
                        #obs_img_feature = np.array([0.0, -1.0, 0.0, -1.0, 0.0, -1.0])
                        obs = np.array([obs[2]])
                        obs_img_feature = np.array([previous_position[predator_idx], 0.0])
                        #obs = np.array([0.0, -1.0])
                        obs = np.concatenate((obs_img_feature, obs))
                        #obs = np.concatenate((obs, np.array([0.0, 0.0, -1.0])), axis=None)
                        
                        if type(obs_img) != type(None):
                            rects, detect_img = robobo_detection.detect_robobo(obs_img, real=REAL)
                                    
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
                                
                            if SAVE_IMAGE:
                                cv2.imwrite('images/' + str(time.time()) + ".png", detect_img)
                                
                            largest_area = 0
                            for rect in rects:
                                #print("FFFFFFFFFFFFFFFFFFFFFFFFFFF", predator_idx)
                                target_name = rect[4]
                                area = rect[2] * rect[3]                                                        
                                
                                if target_name == "Prey" and area > 50 and area > largest_area:
                                    #print('area:', area, predator_idx)
                                    
                                    #if self.predators[predator_idx].name == self.target.name:
                                 
                                    #    fitness += area / (abs(rect[0] - (detect_img_width / 2)) + detect_img_area)
                                        
                                    #if DEBUG_FITNESS:
                                    #    print('FFFF', area / (abs(rect[0] - (detect_img_width / 2)) + 1.0) ** 2)                                                                    
                                    
                                    obs[0] = rect[0] / (detect_img_width) - 0.5 # -1.0 - 1.0 
                                    #obs[-2] = rect[1] / (detect_img_height / 2) - 1.0
                                    obs[1] = area / AGENT_IMAGE_MAX
                                    
                                    previous_position[predator_idx] = obs[0]
                                    
                                    largest_area = area
                                    
                                    
                                    
                                elif target_name == "Predator" and area > 50:
                                
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
                            #action[predator_idx,:] = np.array([1.0, 1.0]) * MAX_SPEED
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
                        obs_img_feature = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                        obs = np.array([obs[2]])
                        #obs_img_feature = np.array([0.0, -1.0])
                        obs = np.concatenate((obs_img_feature, obs))
                        #obs = np.array([0.0, -1.0])
                        
                        count_predators = 0
                        feature1 = []
                        feature2 = []
                        
                        if type(obs_img) != type(None) and EVOLVE_PREY:
                            rects, detect_img = robobo_detection.detect_robobo(obs_img, real=REAL)
                                    
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
                            if EVOLVE_PREY:                    
                                action[prey_idx + NUM_PREDATORS,:] = np.array(play_net[prey_idx + NUM_PREDATORS].activate(obs), dtype=float) * 0
                            else:                    
                                action[prey_idx + NUM_PREDATORS,:] = np.array(self.prey[prey_idx].fixed_strategy(x_list, y_list, sigma_x_list, sigma_y_list, w_x_list, w_y_list, 
                                                                   w_sigma_x_list, w_sigma_y_list, prey_x, prey_y, prey_orientation, PW_RATIO)) * MAX_SPEED
                        else:                    
                            if EVOLVE_PREY:                    
                                action[prey_idx + NUM_PREDATORS,:] = np.array(play_net[prey_idx + NUM_PREDATORS].activate(obs), dtype=float) * MAX_SPEED
                            else:                                                    
                                action[prey_idx + NUM_PREDATORS,:] = np.array(self.prey[prey_idx].fixed_strategy(x_list, y_list, sigma_x_list, sigma_y_list, w_x_list, w_y_list, 
                                                                   w_sigma_x_list, w_sigma_y_list, prey_x, prey_y, prey_orientation, PW_RATIO)) * MAX_SPEED
                                #print(action[prey_idx + NUM_PREDATORS,:])
                            
                        if DEBUG_INOUT:
                            print("prey input:", obs)    
                            print("prey output:", np.array(play_net[prey_idx + NUM_PREDATORS].activate(obs), dtype=float) * MAX_SPEED)
                            
                        
                        
                    observations, reward, done, info = env.step(action)
                    
                    if not EVOLVE_PREY and self.target.name == "prey":
                        break
                
                if self.target.name == self.prey[0].name:
                    print("Prey fitness!!!")
                    fitness = reward['prey']
                    look_at_fitness = look_at_fitness / step
                    fitness = fitness# * look_at_fitness
                else:                
                    print("Predator fitness", reward['predators'], info["distances"][self.target.idx], info["arena_length"] * np.sqrt(2) - info["distances"][self.target.idx])
                    fitness = reward['predators'] * (info["arena_length"] * np.sqrt(2) - info["distances"][self.target.idx])
                    look_at_fitness = look_at_fitness / step
                    fitness = fitness# * look_at_fitness
                    
                average_fitness += fitness
            print("FINAL count_evalution:", count_evalution)
            average_fitness /= count_evalution
                
            if DEBUG_FITNESS:                
                print("average_fitness", average_fitness)
            
            self.target.set_fitness(average_fitness)
            
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
