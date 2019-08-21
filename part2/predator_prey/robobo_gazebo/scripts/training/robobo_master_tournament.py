#!/usr/bin/env python3

import warnings
warnings.filterwarnings("ignore")

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

DATA = "f1"
PREDATOR_START = 99

DEBUG_THREAD = False
DEBUG_WEIGHT = False
DEBUG_FITNESS = False
DEBUG_IMAGE = True
DEBUG_MANUAL = True
DEBUG_INOUT = False
REAL = False
SAVE_IMAGE = False
EVALUATION = True
HUMAN = True
EVOLVE_PREY = True
TIME = False
SAVE = False

if HUMAN:
    EVOLVE_PREY = False

END_GAME = False

LOAD_WEIGHT = True
LOADED_GENERATION = 100

NUM_EVALUATION = 5

PW_RATIO = 0.4

NUM_PREDATORS = 3
NUM_PREY = 1
NUM_AGENTS = NUM_PREDATORS + NUM_PREY

IR_MAX_RANGE = 0.2

if REAL:
    MAX_SPEED = 20
else:
    MAX_SPEED = 20
    
EVOVLED_PREY_SPEED_FACTOR = 0.6

PREDATOR_SIGMA = 0.5
WALL_SIGMA = 0.4

ARENA_LENGTH = 4


if REAL:
    AGENT_IMAGE_MAX = 1500
else:
    AGENT_IMAGE_MAX = 600

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
            p = neat.Checkpointer.restore_checkpoint('/home/jaqq/catkin_ws/src/robobo/robobo_gazebo/scripts/data/' + DATA + '/checkpoints/' + self.name + '/neat-checkpoint-' + str(LOADED_GENERATION))
            #p = neat.Checkpointer.restore_checkpoint('/home/jaqq/Documents/robobo_bak/scripts_without_random/checkpoints/' + self.name + '/neat-checkpoint-' + str(LOADED_GENERATION))
            self.neat = p
            pkl_file = open('/home/jaqq/catkin_ws/src/robobo/robobo_gazebo/scripts/data/' + DATA + '/generations/' + self.name + "/" + str(self.neat.generation - 1) + '.pkl', 'rb')
            #pkl_file = open('/home/jaqq/Documents/robobo_bak/scripts_without_random/generations/' + self.name + "/" + str(self.neat.generation - 1) + '.pkl', 'rb')
            
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
        if DEBUG_THREAD:
            print("generation_start WAIT")
        
        if self.ea_framework.count_generation_start == self.ea_framework.num_agents:
            self.ea_framework.generation_start.set()
        
        self.ea_framework.generation_start.wait()
        if DEBUG_THREAD:
            print("generation_start")                
            
            print("DDDDDDDDDEPPP copy")
        self.prev_nets = copy.deepcopy(self.nets)
        self.prev_genomes = copy.deepcopy(self.genomes)
        self.prev_genome_ids = copy.deepcopy(self.genome_ids)
        self.prev_configs = copy.deepcopy(self.configs)
        self.prev_fitnesses = copy.deepcopy(self.fitnesses)
        
        if self.neat.generation > 0 and not EVALUATION:
            idx = np.argmax(self.prev_fitnesses)       
            self.best_nets += [self.prev_nets[idx]]
        
            data_dict = {'nets': self.prev_nets, 'genomes': self.prev_genomes, 'genome_ids': self.prev_genome_ids, 
                         'configs': self.prev_configs, 'fitnesses': self.prev_fitnesses, 'generation': self.neat.generation - 1, 'best_nets': self.best_nets}
            output = open('generations/' + self.name + "/" + str(self.neat.generation - 1) + '.pkl', 'wb')
            pickle.dump(data_dict, output)
        if DEBUG_THREAD:
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
            if DEBUG_THREAD:
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
        
        wheel_speed = [0, 0]
        
        if not HUMAN:
        
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
            
            TURN_SPEED = 0.7
            STRAIGHT_SPEED = 0.7
            
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
        
        else:
            wheel_speed[0] = self.human_left
            wheel_speed[1] = self.human_right
        
        return wheel_speed[0], wheel_speed[1]

    def pk_key_listener(self, name):

        global END_GAME

        button_delay = 0.01
        try:
            while True and not rospy.is_shutdown():
                DELAY = False        
                keys = pygame.key.get_pressed()    
                    
                if keys[pygame.K_a]:
                    #print("a")
                    '''if self.human_right > 0.0:
                        self.human_right = 0.8
                        self.human_left = 0.5
                    else:
                        self.human_right = -0.5
                        self.human_left = -0.3'''
                        
                        
                    self.human_right = 0.8
                    self.human_left = -0.8    
                        
                    DELAY = True 

                if keys[pygame.K_d]:
                    #print("d")
                    '''
                    if self.human_right > 0.0:
                        self.human_left = 0.8             
                        self.human_right = 0.5
                    else:
                        self.human_left = -0.5
                        self.human_right = -0.3
                    '''
                    
                    self.human_right = -0.8
                    self.human_left = 0.8    
                    
                    DELAY = True
           
                if keys[pygame.K_w]:
                    #print("w")
                    self.human_right = 1.0
                    self.human_left = 1.0
                    DELAY = True
                
                if keys[pygame.K_s]:   
                    #print("s")
                    self.human_right = -1.0
                    self.human_left = -1.0
                    DELAY = True    
                    
                if keys[pygame.K_g]:                                                
                    END_GAME = True
                    DELAY = True                      
                        
                if DELAY:
                    time.sleep(button_delay)
                       
                pygame.event.pump()           
        except:
            return
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
                WIDTH = 400
                HEIGHT = 400
                windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
                windowSurface.fill(BLACK)
            
                t = threading.Thread(target = prey.pk_key_listener, args = ("pk",))
                t.daemon = True
                t.start()   

    def start_evolve(self):        
        
        global END_GAME
        
        if REAL:
            env = gym.make('gym_robobo_predator_prey_real-v0')
        else:
            env = gym.make('gym_robobo_predator_prey-v0')

        #self.set_next_evolution_target()                        
        
        interval = 1
        prey_fitnesses = np.zeros((len(range(0, len(self.predators[0].best_nets), interval)), len(range(0, len(self.predators[0].best_nets), interval))))
        fitnesses = np.zeros((len(range(0, len(self.predators[0].best_nets), interval)), len(range(0, len(self.predators[0].best_nets), interval))))
        caught_matrix = np.zeros((len(range(0, len(self.prey[0].best_nets), interval)), len(range(0, len(self.prey[0].best_nets), interval))), dtype=int)
        end_time = np.zeros((len(range(0, len(self.predators[0].best_nets), interval)), len(range(0, len(self.predators[0].best_nets), interval))), dtype=float)
        
        #print(len(self.prey[0].best_nets), len(self.predators[0].best_nets), "???")
        try:
            if not rospy.is_shutdown():            
                
                #print(len(self.predators[0].best_nets), len(self.predators[1].best_nets), len(self.prey[0].best_nets))
                
                if not HUMAN:
                    predator_start = 0
                    predator_end = 100
                    num_prey_controller = 100
                else:
                    predator_start = PREDATOR_START
                    predator_end = PREDATOR_START + 1
                    num_prey_controller = 8
                
                for save_predator_idx, predator_generation_idx in enumerate(range(predator_start, predator_end, interval)):
                    
                    for save_prey_idx, prey_generation_idx in enumerate(range(0, num_prey_controller, interval)):
                        #print(predator_generation_idx, prey_generation_idx)
                        
                        print("Round " + str(prey_generation_idx) + ":")
                        
                        play_net = []
                    
                        for i in range(NUM_PREDATORS):
                            
                            play_net += [self.predators[i].best_nets[predator_generation_idx]]                        
                            
                            
                            
                        for i in range(NUM_PREY):
                            
                            play_net += [self.prey[i].best_nets[prey_generation_idx]]
                            
                            self.prey[i].human_left = 0.0
                            self.prey[i].human_right = 0.0
            
                        done = False
                        step = 0
                        observations, reward, done, info = env.reset()
                        fitness = 0.0
                        look_at_fitness = 0.0            
                        init_distance = None                
                        END_GAME = False
                        previous_position = np.array([0.0] * (NUM_PREDATORS + NUM_PREY))
                             
                        sigma_x_list = np.array([1.0] * NUM_PREDATORS) * PREDATOR_SIGMA
                        sigma_y_list = np.array([1.0] * NUM_PREDATORS) * PREDATOR_SIGMA
                        w_x_list = [2.0,-2.0,0.0,0.0]
                        w_y_list = [0.0,0.0,2.0,-2.0]
                        w_sigma_x_list = np.array([1.0,1.0,0.0,0.0]) * WALL_SIGMA
                        w_sigma_y_list = np.array([0.0,0.0,1.0,1.0]) * WALL_SIGMA   
                        
                        s0_avg = 0.0
                        s1_avg = 0.0
                        s2_avg = 0.0
                        sp_avg = 0.0                               
                             
                        prey_break = False 
                        predator0_break = False
                        predator1_break = False        
                        predator2_break = False 
                        
                        tracking = []
                        
                        while not done and not rospy.is_shutdown():       
                            if TIME:
                                print("start", time.time())    
                    
                            action = np.zeros((NUM_PREDATORS + NUM_PREY, 2), dtype=int)
                            step += 1
                            if not REAL:
                                prey_input = []
                                
                                s0 = self.compute_speed(env.predators[0])      
                                s1 = self.compute_speed(env.predators[1])
                                s2 = self.compute_speed(env.predators[2])
                                sp = self.compute_speed(env.prey[0])
                                
                                s0_avg += s0
                                s1_avg += s1
                                s2_avg += s2
                                sp_avg += sp
                                      
                                
                                x = env.prey[0].position.x / 2.0
                                y = env.prey[0].position.y / 2.0
                                
                                d0p = self.compute_distance(env.predators[0], env.prey[0]) / (ARENA_LENGTH * np.sqrt(2))                
                                d1p = self.compute_distance(env.predators[1], env.prey[0]) / (ARENA_LENGTH * np.sqrt(2))
                                d2p = self.compute_distance(env.predators[2], env.prey[0]) / (ARENA_LENGTH * np.sqrt(2))
                                
                                yp0 = self.compute_face_yaw_diff(env.prey[0], env.predators[0]) / np.pi
                                yp1 = self.compute_face_yaw_diff(env.prey[0], env.predators[1]) / np.pi
                                yp2 = self.compute_face_yaw_diff(env.prey[0], env.predators[2]) / np.pi
                                
                                obsp = np.array([x, y, d0p, d1p, d2p, yp0, yp1, yp2])   
                            
                            
                            #print("????????????????", info, step)
                            
                                if not EVOLVE_PREY:
                                    x_list = [info['predator0_position'].x, info['predator1_position'].x, info['predator2_position'].x]
                                    y_list = [info['predator0_position'].y, info['predator1_position'].y, info['predator2_position'].y]
                                    prey_x = info['prey_position'].x
                                    prey_y = info['prey_position'].y
                                    prey_orientation = info['prey_orientation']
                            
                            else:
                                x_list = [0.0, 0.0, 0.0]
                                y_list = [0.0, 0.0, 0.0]
                                prey_x = 0.0
                                prey_y = 0.0
                                prey_orientation = 0.0
                                
                            for predator_idx, obs in enumerate(observations.predator):                
                            
                                if obs == None:
                                    continue                        

                                obs_img = obs[-1]
                                #print("obs", obs)
                                #obs = np.array([obs[2], obs[], obs[7], obs[8], obs[9]])
                                #obs_img_feature = np.array([0.0, -1.0, 0.0, -1.0, 0.0, -1.0])
                                obs = np.array([obs[2]])
                                obs_img_feature = np.array([previous_position[predator_idx], -1.0])
                                #obs = np.array([0.0, -1.0])
                                obs = np.concatenate((obs_img_feature, obs))
                                #obs = np.concatenate((obs, np.array([0.0, 0.0, -1.0])), axis=None)
                                
                                if type(obs_img) != type(None):
                                    if TIME:
                                        print("robobo_detection start", time.time())    
                                
                                    rects, detect_img = robobo_detection.detect_robobo(obs_img, real=REAL)
                                    if TIME:
                                        print("robobo_detection end", time.time())    
                                    
                                    window_name = "predator" + str(predator_idx + 1) + "_image"
                                    
                                    if DEBUG_IMAGE:
                                        cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
                                        '''if not REAL:
                                            cv2.moveWindow(window_name, 100 + predator_idx * 320, 30)                
                                        else:'''
                                        cv2.moveWindow(window_name, 100, 30 + predator_idx * 350)                
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
                                        #print(area)
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
                                                                                                                                              
                                    '''if self.predators[predator_idx].name == self.target.name:
                                        if obs[1] > 0:
                                            look_at_fitness += 1.0'''
                                            
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
                                
                                if not REAL:    
                                    obs = obsp
                                
                                if DEBUG_MANUAL:
                                    if EVOLVE_PREY:                    
                                        action[prey_idx + NUM_PREDATORS,:] = np.array(play_net[prey_idx + NUM_PREDATORS].activate(obs), dtype=float) * 0
                                    else:                    
                                        action[prey_idx + NUM_PREDATORS,:] = np.array(self.prey[prey_idx].fixed_strategy(x_list, y_list, sigma_x_list, sigma_y_list, w_x_list, w_y_list, 
                                                                           w_sigma_x_list, w_sigma_y_list, prey_x, prey_y, prey_orientation, PW_RATIO)) * 0
                                else:                    
                                    if EVOLVE_PREY:                    
                                        action[prey_idx + NUM_PREDATORS,:] = np.array(play_net[prey_idx + NUM_PREDATORS].activate(obs), dtype=float) * MAX_SPEED
                                    else:                                                    
                                        action[prey_idx + NUM_PREDATORS,:] = np.array(self.prey[prey_idx].fixed_strategy(x_list, y_list, sigma_x_list, sigma_y_list, w_x_list, w_y_list, 
                                                                           w_sigma_x_list, w_sigma_y_list, prey_x, prey_y, prey_orientation, PW_RATIO)) * MAX_SPEED * EVOVLED_PREY_SPEED_FACTOR
                                        #print(action[prey_idx + NUM_PREDATORS,:])
                                    
                                #if DEBUG_INOUT:
                                #    print("prey input:", obs)    
                                #    #print("prey output:", np.array(play_net[prey_idx + NUM_PREDATORS].activate(obs), dtype=float) * MAX_SPEED)
                                    
                           
                            
                            if not REAL:
                            
                                xp = info['prey_position'].x
                                yp = info['prey_position'].y
                                yaw_p, _, _ = self.orientation_quaternion_to_euler(info['prey_orientation'])
                                x0 = info['predator0_position'].x
                                y0 = info['predator0_position'].y
                                yaw_0, _, _ = self.orientation_quaternion_to_euler(info['predator0_orientation'])
                                x1 = info['predator1_position'].x
                                y1 = info['predator1_position'].y
                                yaw_1, _, _ = self.orientation_quaternion_to_euler(info['predator1_orientation'])
                                x2 = info['predator2_position'].x
                                y2 = info['predator2_position'].y
                                yaw_2, _, _ = self.orientation_quaternion_to_euler(info['predator2_orientation'])
                                     
                                xp_info = (xp, yp, yaw_p)     
                                x0_info = (x0, y0, yaw_0)
                                x1_info = (x1, y1, yaw_1)
                                x2_info = (x2, y2, yaw_2)
                                predators_info = (x0_info, x1_info, x2_info)  
                                     
                                tracking += [(xp_info, predators_info)]
                            if TIME:   
                                print("step start", time.time()) 
                            observations, reward, done, info = env.step(action)
                            if TIME:
                                print("step end", time.time()) 
                                print("end", time.time()) 

                            if END_GAME:
                                break
                            
                            #if not EVOLVE_PREY and self.target.name == "prey":
                            #    break
                        
                        #if self.target.name == self.prey[0].name:
                        
                        if not REAL:
                        
                            fitness = reward['prey']
                            #look_at_fitness = look_at_fitness / step
                            prey_fitness = fitness# * look_at_fitness
                            #print("Prey fitness!!!", prey_fitness)
                            #else:                
                            #print("Predator fitness", reward['predators'], info["distances"][self.target.idx], info["arena_length"] * np.sqrt(2) - info["distances"][self.target.idx])
                            fitness = (1 / info["distances"][0] + 1 / info["distances"][1] + 1 / info["distances"][2]) / 3
                            #look_at_fitness = look_at_fitness / step
                            fitness = fitness# * look_at_fitness
                            #print("predators fitness!!!", fitness)
                            
                            if info["caught"]:
                                caught_matrix[save_predator_idx, save_prey_idx] = 1
                            else:
                                caught_matrix[save_predator_idx, save_prey_idx] = -1                      
                                
                            fitnesses[save_predator_idx, save_prey_idx] = fitness
                            prey_fitnesses[save_predator_idx, save_prey_idx] = prey_fitness
                            
                            if info["time"] > 30:
                                info["time"] = 30.0
                             
                            print("END TIME:", info["time"])
                            print("-----------------------------------------------------------------------------------")


                            end_time[save_predator_idx, save_prey_idx] = info["time"]
                            output = open("output/tracking" + str(save_predator_idx) + "_" + str(save_prey_idx), 'wb')
                            pickle.dump(tracking, output)
                            output.close()
                        else:
                            
                            if info["time"] > 30:
                                info["time"] = 30.0
                        
                            end_time[save_predator_idx, save_prey_idx] = info["time"]
                            #print("END TIME:", info["time"])
                            print("-----------------------------------------------------------------------------------")
                            #np.save("output/end_time", end_time)
                        
                        timestamp = time.time()
                        if SAVE:
                            if not REAL:                      
                                np.save("output/fitnesses", fitnesses)    
                                np.save("output/prey_fitnesses", prey_fitnesses)    
                                np.save("output/caught_matrix", caught_matrix)    
                                np.save("output/end_time", end_time)
                                
                                if HUMAN:
                                    np.save("output/fitnesses_" + str(int(timestamp)), fitnesses)    
                                    np.save("output/prey_fitnesses_" + str(int(timestamp)), prey_fitnesses)    
                                    np.save("output/caught_matrix_" + str(int(timestamp)), caught_matrix)    
                                    np.save("output/end_time_" + str(int(timestamp)), end_time)  
                            else:  
                            
                                np.save("output/end_time" + "_real_" + str(int(timestamp)), end_time)
                                            
                print("Avg: " + str(end_time.sum().sum() / 7))
                if SAVE:
                    if not REAL:
                        np.save("output/fitnesses", fitnesses)    
                        np.save("output/prey_fitnesses", prey_fitnesses)    
                        np.save("output/caught_matrix", caught_matrix)    
                        np.save("output/end_time", end_time)

                        if HUMAN:
                            np.save("human/sim/fitnesses_" + str(int(timestamp)), fitnesses)    
                            np.save("human/sim/prey_fitnesses_" + str(int(timestamp)), prey_fitnesses)    
                            np.save("human/sim/caught_matrix_" + str(int(timestamp)), caught_matrix)    
                            np.save("human/sim/end_time_" + str(int(timestamp)), end_time)  
                    else:  
                    
                        np.save("human/real/end_time" + "_real_" + str(int(timestamp)), end_time)
        except:
            pass
            
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
        
    def orientation_quaternion_to_euler(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        
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

