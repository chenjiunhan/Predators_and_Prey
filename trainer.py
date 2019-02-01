import socketserver
from random import randint
from time import sleep
import numpy as np
import matplotlib.pyplot as plt
import _thread
import math
import os
import sys
import time
import pygame, sys
import pygame.locals
import pickle
import cma
import sys
sys.path.append("./")
from bayes_opt import BayesianOptimization
from evostra import EvolutionStrategy

DATA_PATH = "data/"

# record fitness
ff = open('fitness.txt', 'w+')
ff.close()
fp = open('fitness_prey.txt', 'w+')
ff.close()

RANDOM_POSITION = False
SHOW_DRAW = False
LOAD_WEIGHT = True
LOAD_MUT = False
PK = False
HUMAN = False
CMA = False
BAYESIAN = False
SIMPLE_ES = False
MAX_PARAMETERS = False
SAVE_PK_FITNESS = False

if MAX_PARAMETERS and not LOAD_WEIGHT and not PK:
    print("Check flag 1")
    exit(0)

# init human controlling variable
human_left = 0.0
human_right = 0.0

f_NUM_PREDATOR = open("configuration/configuration.txt", "r")
f_line = f_NUM_PREDATOR.read()
NUM_PREDATOR = int(f_line)
NUM_PREY = 1

# parameters of EA
N_GENERATIONS = 999
MUT_STRENGTH = 0.5
PREY_MUT_STRENGTH = 1.0
MUT_MAX = 0.5
MUT_MIN = 0.01
W_MAX = 1
B_MAX = 0.5

# dim of NN
I_S = 3
H_S = 4
O_S = 2

NUM_PARAMETERS = I_S * H_S + H_S + H_S * O_S + O_S

# Play with best opponent if ALTERNATIVE_FLAG = 1, only for co-evolution
ALTERNATIVE_FLAG = 0
REEVALUATION_PROB = 0.0
PREY_REEVALUATION_PROB = 0.0
 
# average to reduce noise
NUM_EVALUATION = 1

# Time per episode
PLAY_PERIOD = 60

# count sim time 
sim_time = 0

# record the end of time of previous episode
previous_sim_time = 0

# record the end of time of previous episode
switch_times = [0] * (NUM_PREDATOR + NUM_PREY)

# init for storing fitness
total_parent_fitness = 0.0
total_kid_fitness = 0.0
prey_total_parent_fitness = 0.0
prey_total_kid_fitness = 0.0

# count generation
count_generation = 1

# store individual fitnesses
parent_fitnesses = np.zeros((NUM_PREDATOR, )) - 1000000
kid_fitnesses = np.zeros((NUM_PREDATOR, )) - 1000000
prey_parent_fitness = -1000000
prey_kid_fitness = -1000000

# prey position
prey_x = 0.0
prey_y = 0.0

# average values for computing fitness
average_cos = np.zeros((NUM_PREDATOR, ))
average_speed = np.zeros((NUM_PREDATOR, ))
average_cos_speed = np.zeros((NUM_PREDATOR, ))
average_distance = np.zeros((NUM_PREDATOR, ))
prey_average_cos_speed = 0.0
prey_average_speed = 0.0
prey_average_min_distance = 0.0

# record previous position of previous time interval of simulation
previous_position_x = np.zeros((NUM_PREDATOR, ))
previous_position_y = np.zeros((NUM_PREDATOR, ))

# initial positions
init_position_x = [-1000] * (NUM_PREDATOR + NUM_PREY)
init_position_y = [-1000] * (NUM_PREDATOR + NUM_PREY)

# for saving fitnesses as file
good_fitness = []
good_fitness_prey = []
good_generation = []

# for computing dangerous value, storing position of predators
x0_list = [0] * NUM_PREDATOR
y0_list = [0] * NUM_PREDATOR

# center of gaussian distribution of wall
wall_x_list = [1.0,-1.0,0.0,0.0]
wall_y_list = [0.0,0.0,1.0,-1.0]

# tuned parameter for the importance ratio between wall and predator
PW_RATIO = 0.1

PW_RATIO_LOW = 0.05
PW_RATIO_HIGH = 0.4
PW_RATIO_STEP = 0.05

PREDATOR_SIGMA_LOW = 0.05
PREDATOR_SIGMA_HIGH = 0.4
PREDATOR_SIGMA_STEP = 0.05

WALL_SIGMA_LOW = 0.05
WALL_SIGMA_HIGH = 0.4
WALL_SIGMA_STEP = 0.05

# sigma of predators
sigma_x_list = np.array([1.0] * NUM_PREDATOR) * 0.25
sigma_y_list = np.array([1.0] * NUM_PREDATOR) * 0.25

# sigma of wall
wall_sigma_x_list = np.array([1.0,1.0,0.0,0.0]) * 0.2
wall_sigma_y_list = np.array([0.0,0.0,1.0,1.0]) * 0.2

# storing gradient
grad_x = 1.0
grad_y = 0.0

# acerage dangerous value
average_danger_value = 0.0

# combine individual fitness values
def prod(iterable):
    p = 0
    f = []
    for n in iterable:
        p += n
        f += [n]        
    print("STD:", np.std(np.array(f)))
    return p * (np.std(np.array(f)) + 1)

def play_game(prey_info, predators_info):
    return 'test'  

def make_kid(w = None):              
                  
    global H_S, O_S, I_S, W_MAX, B_MAX, PK
    
    if PK:
        return w
    
    w_max = W_MAX
    w_min = -W_MAX
    
        if type(w) == type(None):
        w = MUT_STRENGTH * np.random.randn(I_S * H_S + H_S + H_S * O_S + O_S, )
        
    w = w + MUT_STRENGTH * np.random.randn(I_S * H_S + H_S + H_S * O_S + O_S, )
    w[w > w_max] = w_max
    w[w < w_min] = w_min
    
    return w

# process communication from C++ bridge, C++ bridge is between Gazebo and python trainer.py
class MyTCPHandler(socketserver.BaseRequestHandler):

    def gazebo_parser(self, data):
            
            gazebo_info_split = data.decode("utf-8").rstrip('\0').split(";")[-(6 + NUM_PREDATOR + NUM_PREY):-1];            
                
            prey_info = []
            predators_info = []          

            for gazebo_info_single in gazebo_info_split:
    
                if gazebo_info_single == "":
                    continue
                
                p_infos = gazebo_info_single.split(",")
                
                if p_infos[0] != "P" and p_infos[0] != "t" and p_infos[0] != "w" and p_infos[0] != "p":
                    continue

                if p_infos[0] == "w":
                    continue

                if p_infos[0] == "t":
                    sim_time = float(p_infos[1])
                    continue

                if (p_infos[0] == "p" or p_infos[0] == "P") and len(p_infos) < 5:
                    continue
                
                # p_id
                '''
                p_id = float(p_infos[1])
                '''
                p_x = float(p_infos[2])
                p_y = float(p_infos[3])                
                p_yaw = float(p_infos[4].rstrip('\0')[0:5])
                p_vector = np.array([p_x, p_y, p_yaw])
                
                if p_infos[0] == "p":                
                    prey_info.append(p_vector)
                if p_infos[0] == "P":
                    predators_info.append(p_vector)
                                
            prey_info = np.array(prey_info)
            predators_info = np.array(predators_info)
            
            return prey_info, predators_info, sim_time
                    
    def input_processor(self, prey_info, predators_info):
        
        nn_input = np.zeros((NUM_PREDATOR, I_S))        
        prey = prey_info[0]
        
        xy_diffs = np.sqrt(np.sum((predators_info[:,0:2] - prey[0:2])**2, axis=1)) 
        argsort = np.argsort(xy_diffs)
        ranks = np.empty_like(argsort)
        ranks[argsort] = np.arange(len(xy_diffs))
        copy_rank = ranks.copy()
        ranks_mean = ranks.mean()
        ranks_std = ranks.std()
        ranks = (ranks - ranks_mean) / ranks_std
        
        
        for predator_idx, predator in enumerate(predators_info):
            
            nn_input_end = 0
            # coordinates of predator itself
            
            # id
            
            # shortest distance            
            
            predator_diffs = predators_info - predator
            predator_distances = np.sqrt(predator_diffs[:,0] ** 2 + predator_diffs[:,1] ** 2)
            predator_distance_min = np.sort(predator_distances)[1]
            min_idx = np.argsort(predator_distances)[1]
            
            a_vector = np.array([np.cos(predator[2]), np.sin(predator[2]), 0.0])
            xy_diff = (predators_info[min_idx] - predator)[0:2]
            xy_diff = xy_diff / np.linalg.norm(xy_diff)
            b_vector = np.array([xy_diff[0], xy_diff[1], 0.0])
            cross = np.cross(b_vector, a_vector)
            #theta = np.arcsin(np.linalg.norm(cross))
            
            if cross[2] < 0.0:
                input_block = np.array([1/predator_distance_min]) / 8.0
            else: 
                input_block = np.array([-1/predator_distance_min]) / 8.0
                
            nn_input_step = 1
            nn_input_start = nn_input_end
            nn_input_end = nn_input_start + nn_input_step
            nn_input[predator_idx][nn_input_start:nn_input_end] = input_block
            
            # angle and distance between predator and prey
            
            a_vector = np.array([np.cos(predator[2]), np.sin(predator[2]), 0.0])
            xy_diff = (prey - predator)[0:2]
            xy_diff = xy_diff / np.linalg.norm(xy_diff)
            b_vector = np.array([xy_diff[0], xy_diff[1], 0.0])
            cross = np.cross(b_vector, a_vector)
            theta = np.arcsin(np.linalg.norm(cross))
            
            if cross[2] < 0.0:
                theta = -theta
            
            cos = b_vector.dot(a_vector)
            if cos < 0.0:
                theta = (-theta) + np.pi * theta / abs(theta)
          
            nn_input_step = 2
            nn_input_start = nn_input_end
            nn_input_end = nn_input_start + nn_input_step
            input_block = np.array([0.0, 0.0])
            input_block[0] = np.linalg.norm((prey - predator)[0:2])
            input_block[1] = theta
            
            nn_input[predator_idx][nn_input_start:nn_input_end] = input_block                       
        return nn_input
    
    def fitness(self, current_fitness, step, prey_info, predators_info, sim_time, dangerous_value):
        predator_prey_vectors = np.array(prey_info[0][0:2] - predators_info[:,0:2])
        avg_min_distances = np.zeros(predators_info.shape[0])
        
        for idx, predator in enumerate(predators_info):
            predator_diffs = predators_info - predator
            predator_distances = np.sqrt(predator_diffs[:,0] ** 2 + predator_diffs[:,1] ** 2)
            predator_distance_min = np.sort(predator_distances)[1]            
            avg_min_distances[idx] = predator_distance_min
        current_fitness['avg_min_distances'] = step * current_fitness['avg_min_distances'] / (step + 1) + avg_min_distances / (step + 1)
        current_fitness['avg_distances'] = step * current_fitness['avg_distances'] / (step + 1) + np.linalg.norm(predator_prey_vectors, axis = 1) / (step + 1)
        current_fitness['dangerous_value'] = step * current_fitness['dangerous_value'] / (step + 1) + dangerous_value / (step + 1)
        
        return current_fitness       
        
    def compute_fitness(self):
        inv_avg_distances = 1.0 / self.fitness_dict['avg_distances']
        return np.mean(inv_avg_distances) + np.mean(self.fitness_dict['avg_min_distances'])
        
    def reset_variables(self):
        self.fitness_dict = {'avg_distances': np.zeros(NUM_PREDATOR, dtype=float), 'avg_min_distances': np.zeros(NUM_PREDATOR, dtype=float), 'dangerous_value': 0.0}
        self.step = 0
    
    def weight_reshape(self, w):
        
        start = 0        
        w1 = w[start:start + I_S * H_S].reshape((I_S, H_S))
        start += I_S * H_S
        b1 = w[start:start + H_S].reshape((H_S, 1))
        start += H_S
        w2 = w[start:start + H_S * O_S].reshape((H_S, O_S))
        start += H_S * O_S
        b2 = w[start:start + O_S].reshape((O_S, 1))
        
        return w1, w2, b1, b2
    
    def forward(self, x, w): #w1, w2, b1, b2):
        global I_S, H_S, O_S
        #x = x.reshape(x.shape[0], 1);
        
        w1, w2, b1, b2 = self.weight_reshape(w)
        
        '''
        print(x.shape)
        print(w1.shape)
        print(w2.shape)
        print(b1.shape)
        print(b2.shape)
        '''
        
        x = x[:, np.newaxis]        
        
        a1 = w1.T.dot(x) + b1    
        z1 = np.tanh(a1)
        a2 = w2.T.dot(z1) + b2
        z2 = np.tanh(a2) * 5.6
        

        return z2.flatten()    
    
    def make_kid(self, w):
        return make_kid(w)
    
    def check_reset(self, prey_info, predators_info, prey_initial_info, predators_initial_info):            
    
        #print("!!!!!!!!", prey_info, predators_info, prey_initial_info, predators_initial_info)
    
        flag = True
                        
        for i in range(len(predators_initial_info)):
            flag = flag & np.allclose(predators_info[i][0:2], predators_initial_info[i][0:2], atol = 0.01)
        
        for i in range(len(prey_initial_info)):   
            flag = flag & np.allclose(prey_info[i][0:2], prey_initial_info[i][0:2], atol = 0.01)            
        
        return flag
    
    def check_out_of_boundary(self, prey_info, predators_info):    
    
        if (np.max(prey_info[0]) > 0.8 or np.min(prey_info[0]) < -0.8) and (np.max(predators_info[:]) > 0.8 or np.min(predators_info[:]) < -0.8):
            print("out of boundary")
            return True
        return False
    
    def prey_wheel_speed(self, prey_info, predators_info):
    
        global grad_x, grad_y, sigma_x_list, sigma_y_list, wall_sigma_x_list, wall_sigma_y_list, human_right, human_left                        
        
        prey_x = prey_info[0][0]
        prey_y = prey_info[0][1]
    
        a = []
        b = []
        
        wheel_speed = np.zeros(2, dtype=float)
        
        for predator in predators_info:
            a += [predator[0]]
            b += [predator[1]]
        
        c = sigma_x_list.copy()
        d = sigma_y_list.copy()
        
        grad_x, grad_y = create_2d_gradient(a, b, c, d, wall_x_list, wall_y_list, wall_sigma_x_list, wall_sigma_y_list, prey_x, prey_y)
        
        yaw_x = np.cos(prey_info[0][2]) 
        yaw_y = np.sin(prey_info[0][2]) 
        p_grad_x = grad_x
        p_grad_y = grad_y 
        g_norm = -np.sqrt(p_grad_x**2 + p_grad_y**2)
        p_grad_x = p_grad_x / g_norm
        p_grad_y = p_grad_y / g_norm
        yaw_vec = np.array([yaw_x, yaw_y, 0.0])
        grad_vec = np.array([p_grad_x, p_grad_y, 0.0])
        yg_cross = np.cross(yaw_vec, grad_vec)
        yg_dot = np.dot(yaw_vec, grad_vec)
        
        
        TURN_SPEED = 5.6 * 0.5
        STRAIGHT_SPEED = 5.6 * 0.8
        
        #TURN_SPEED = 0.0
        #STRAIGHT_SPEED = 0.0
        
        if yg_cross[2] > 0:
            if yg_dot < 0.9:    
                wheel_speed[0] = TURN_SPEED
                wheel_speed[1] = -TURN_SPEED
            else:
                wheel_speed[0] = STRAIGHT_SPEED
                wheel_speed[1] = STRAIGHT_SPEED
        else:
            if yg_dot < 0.9:
                wheel_speed[0] = -TURN_SPEED
                wheel_speed[1] = TURN_SPEED
            else:
                #print("Forward")
                wheel_speed[0] = STRAIGHT_SPEED
                wheel_speed[1] = STRAIGHT_SPEED
        
        if HUMAN:
            wheel_speed[0] = human_right
            wheel_speed[1] = human_left
                
        return wheel_speed        
                
    
    def handle(self):

        #global switch_times, current_players, parent_fitnesses, kid_fitnesses, total_parent_fitness, total_kid_fitness, prey_parent_fitness, prey_kid_fitness, prey_total_parent_fitness, prey_total_kid_fitness, sim_time, MUT_STRENGTH, MUT_MAX, pk_rate, DNA_SIZE, parent, kid, good_fitness, good_fitness_prey, good_generation, count_generation, prey_parent, prey_kid, PREY_MUT_STRENGTH, previous_position_x, previous_position_y, average_speed, average_cos, average_cos_speed, previous_sim_time, prey_average_cos_speed, ALTERNATIVE_FLAG, prey_average_min_distance, prey_average_speed, average_distance, x0_list, y0_list, sigma_x_list, sigma_y_list, prey_x, prey_y, grad_x, grad_y, average_danger_value, MUT_MIN, human_left, human_right
        
        global sigma_x_list, sigma_y_list, wall_x_list, wall_y_list, x0_list, y0_list, prey_x, prey_y, grad_x, grad_y

        # self.request is the TCP socket connected to the client
        
        # !!!!!!!receive less should be better
    
        random_position_x_prey = 0.0
        random_position_y_prey = 0.0
        random_position_x_predator = np.array([-0.85] * NUM_PREDATOR)
        random_position_y_predator = np.linspace(-0.5, 0.5, num=NUM_PREDATOR)     
                
        self.reset_variables()        
        
        requestForUpdate = self.request.recv(1024)                       
        
        start_sim_time = None
        
        reset_flag = False
        
        prey_initial_info = None
        predators_initial_info = None
        
        tracking = []
        count_generation = 0
        
        while requestForUpdate != '':            

            self.data = requestForUpdate.strip()                        
            try:
                prey_info, predators_info, sim_time = self.gazebo_parser(self.data)
            except:
                continue
            
            #print('wh', predators_info)
            
            if server.playing == False:
                
                send_string = 'RESET;'
                for i in range(NUM_PREDATOR):
                    send_string += str(i+1) + "," + str(random_position_x_predator[i]) + "," + str(random_position_y_predator[i]) + ";"
                
                send_string += "p" + "," + str(random_position_x_prey) + "," + str(random_position_y_prey) + ";"
                self.request.sendall(bytes("\n", "utf-8"))
                requestForUpdate = self.request.recv(1024)
                continue
            
            if len(prey_info) == 0 or len(predators_info) == 0 or abs(sim_time - 0.0) < 0.000001:
                send_string = ''    
                for i in range(NUM_PREDATOR):   
                    send_string += str(i+1) + "," + str(0.0) + "," + str(0.0) + ";"
                                    
                send_string += "p" + "," + str(0.0) + "," + str(0.0) + ";"   
                self.request.sendall(bytes("\n", "utf-8"))
                requestForUpdate = self.request.recv(1024)                
                continue                            
                
            if type(predators_initial_info) == type(None):
                prey_initial_info = prey_info
                predators_initial_info = predators_info
                
            if reset_flag:
            
                if not self.check_reset(prey_info, predators_info, prey_initial_info, predators_initial_info):                             
                    send_string = "RESET;"
                    print("RESET")
                    for i in range(NUM_PREDATOR):
                        send_string += str(i+1) + "," + str(random_position_x_predator[i]) + "," + str(random_position_y_predator[i]) + ";"
                
                    send_string += "p" + "," + str(random_position_x_prey) + "," + str(random_position_y_prey) + ";"
                    
                    self.request.sendall(bytes(send_string + "\n", "utf-8"))                 
                    requestForUpdate = self.request.recv(1024)                    
                    continue
                  
                else:  
                    self.request.sendall(bytes(send_string + "\n", "utf-8"))                 
                    requestForUpdate = self.request.recv(1024)
                    
                    reset_flag = False
                    #print(self.fitness_dict['avg_distances'])
                    #print(1.0 / self.fitness_dict['avg_distances'])
                    self.server.fitness = self.compute_fitness()
                    self.server.prey_fitness = -self.fitness_dict['dangerous_value']
                                       
                    output = open(DATA_PATH + 'trainer/tracking/tracking_' + sys.argv[1] + "_" + str(count_generation) + "_" + str(self.server.fitness) + ".pkl", 'wb')
                    pickle.dump(tracking, output)
                    output.close()
                    tracking = []
                    
                    # LAST!!!!!!!!
                    self.server.playing = False
                    
                    continue
                
            if not self.server.playing:
                self.request.sendall(bytes("\n", "utf-8"))
                requestForUpdate = self.request.recv(1024)                
                
                continue 
                
            if self.server.new_game:   
                count_generation += 1 
                self.reset_variables()
                start_sim_time = sim_time
                self.server.new_game = False
                
                
            if sim_time - start_sim_time >= PLAY_PERIOD:                                                                
                
                if RANDOM_POSITION:
                
                    resample = True
                    
                    while resample: 
                        print('REsma')
                        resample = False               
                        random_position_x_prey = np.random.uniform(-0.8, 0.8)
                        random_position_y_prey = np.random.uniform(-0.8, 0.8)
                        random_position_x_predator = np.random.uniform(-0.8, 0.8, (NUM_PREDATOR, ))
                        random_position_y_predator = np.random.uniform(-0.8, 0.8, (NUM_PREDATOR, ))
                        for i, _ in enumerate(random_position_x_predator):                                                
                            if i < len(random_position_x_predator) - 1:
                                for j in range(i+1, len(random_position_x_predator)):
                                    if np.sqrt((random_position_x_predator[i] - random_position_x_predator[j]) ** 2 + 
                                       (random_position_y_predator[i] - random_position_y_predator[j]) ** 2) < 0.2:
                                        resample = True
                
                        for i, _ in enumerate(random_position_x_predator):
                            if np.sqrt((random_position_x_predator[i] - random_position_x_prey) ** 2 + 
                                       (random_position_y_predator[i] - random_position_y_prey) ** 2) < 0.2:
                                resample = True
                
                    prey_initial_info = np.array([[random_position_x_prey, random_position_y_prey, prey_info[0][2]]])
                    predators_initial_info = np.vstack((random_position_x_predator, random_position_y_predator, predators_info[:,2])).T
                
                reset_flag = True
                
                send_string = "RESET;"                
                print("RESET2")
                for i in range(NUM_PREDATOR):
                    send_string += str(i+1) + "," + str(random_position_x_predator[i]) + "," + str(random_position_y_predator[i]) + ";"
                
                send_string += "p" + "," + str(random_position_x_prey) + "," + str(random_position_y_prey) + ";"
                
                self.request.sendall(bytes(send_string + "\n", "utf-8"))                    
                
                requestForUpdate = self.request.recv(1024)                                
                
                continue
             
            nn_inputs = self.input_processor(prey_info, predators_info)
            
            danger_value = 0.0
            danger_value += np.sum(gaussian_predator(prey_info[0][0], prey_info[0][1], predators_info[:,0], predators_info[:,1], sigma_x_list, sigma_y_list))
            danger_value += np.sum(gaussian_wall(prey_info[0][0], wall_x_list, wall_sigma_x_list))
            danger_value += np.sum(gaussian_wall(prey_info[0][1], wall_y_list, wall_sigma_y_list))                                       
            
            x0_list = predators_info[:,0]
            y0_list = predators_info[:,1]
            prey_x = prey_info[0][0]
            prey_y = prey_info[0][1]
            #grad_x, grad_y
            
            tracking += [(prey_info[0], predators_info)]
            
            self.fitness_dict = self.fitness(self.fitness_dict, self.step, prey_info, predators_info, sim_time, danger_value)
            
            distance_prey_predators = []
            
            for predator_idx, predator in enumerate(predators_info):
                distance_prey_predators += [np.linalg.norm((prey_info[0] - predator)[0:2])]
            distance_prey_predators = np.array(distance_prey_predators)
            if not hasattr(self.server, "avg_avg_distances"):
                #self.server.avg_avg_distances = [np.mean(self.fitness_dict['avg_distances'])]
                self.server.avg_avg_distances = [np.mean(distance_prey_predators)]
            else:
                #self.server.avg_avg_distances += [np.mean(self.fitness_dict['avg_distances'])] 
                self.server.avg_avg_distances += [np.mean(distance_prey_predators)]        
                      
            self.step += 1
            
            wheel_speeds = []
            
            for i in range(NUM_PREDATOR):
                wheel_speed = self.forward(nn_inputs[i], self.server.player_weight)
                wheel_speeds += [wheel_speed[0]]
                wheel_speeds += [wheel_speed[1]]
                
            send_string = ''                        
                        
            for i in range(NUM_PREDATOR):
                left_wheel_speed = wheel_speeds[i*2]
                right_wheel_speed = wheel_speeds[i*2 + 1]
                send_string += str(i+1) + "," + str(left_wheel_speed) + "," + str(right_wheel_speed) + ";"
                #send_string += str(i+1) + "," + str(0) + "," + str(0) + ";"
            
            wheel_speeds = self.prey_wheel_speed(prey_info, predators_info)
            #send_string += "p" + "," + str(5.6) + "," + str(5.6) + ";"
            
            send_string += "p" + "," + str(wheel_speeds[1]) + "," + str(wheel_speeds[0]) + ";"
            #send_string += "p" + "," + str(0) + "," + str(0) + ";"
            self.request.sendall(bytes(send_string + "\n", "utf-8"))
            
            requestForUpdate = self.request.recv(1024)



def draw_2d_heatmap_thread(name):
    global x0_list, y0_list, prey_x, prey_y, grad_x, grad_y, sigma_x_list, sigma_y_list, wall_x_list, wall_y_list, wall_sigma_x_list, wall_sigma_y_list, SHOW_DRAW, PW_RATIO
    
    x = None
    y = None
    z = None

    while True:

        a = x0_list.copy()
        b = y0_list.copy()
        c = sigma_x_list.copy()
        d = sigma_y_list.copy()

        for idx, value in enumerate(x0_list):
            if type(x) == type(None):
                x, y, z = create_2d_gaussian(a[idx], b[idx], c[idx], d[idx])
            else:
                x_a, y_a, z_a = create_2d_gaussian(a[idx], b[idx], c[idx], d[idx])
                z = z + z_a
        
        for idx, value in enumerate(wall_x_list):
            x_a, y_a, z_a = create_2d_wall_function(wall_x_list[idx],wall_y_list[idx],wall_sigma_x_list[idx],wall_sigma_y_list[idx])
            z = z + z_a       

        #print(grad_x, grad_y)
        
        if SHOW_DRAW:
                        
            plt.clf()
            fig = plt.gcf()
            fig.set_size_inches(6.5,5)            
            ax = plt.gca()
            
            plt.contourf(x, y, z, cmap='Reds')
            cbar = plt.colorbar()
            cbar.ax.tick_params(labelsize=16)
            for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
                             ax.get_xticklabels() + ax.get_yticklabels()):
                item.set_fontsize(16)
            c1 = plt.Circle((prey_x, prey_y), 0.05, color='g')
            
            g_nor = -np.sqrt(grad_x**2 + grad_y**2) * 5
            if abs(g_nor - 0.0) > 0.00001:
                plt_grad_x = grad_x / g_nor
                plt_grad_y = grad_y / g_nor
                l = plt.Line2D([prey_x, prey_x + plt_grad_x], [prey_y, prey_y + plt_grad_y])
                ax.add_line(l)

            ax.add_artist(c1)
            plt.axis('off')
            plt.draw()
            plt.pause(0.001)
            
        
        x = None
        y = None
        z = None
    

def gaussian_predator(x, y, x0, y0, sigma_x, sigma_y):    

    #print('x, y, x0, y0, sigma_x, sigma_y', x, y, x0, y0, sigma_x, sigma_y)

    return (1/(2*np.pi*sigma_x*sigma_y) * np.exp(-((x - x0)**2/(2*sigma_x**2)
	 + (y - y0)**2/(2*sigma_y**2)))) * PW_RATIO

def gaussian_wall(a, a0, sigma_a):
    z = 1/(2*np.pi*sigma_a) * np.exp(-((a - a0)**2/(2*sigma_a**2)))
    z = np.nan_to_num(z, False)
    return z

def create_2d_gaussian(x0, y0, sigma_x, sigma_y):

    global PW_RATIO

    size = 100
    x = np.linspace(-1, 1, size)
    y = np.linspace(-1, 1, size)

    x, y = np.meshgrid(x, y)
    z = gaussian_predator(x, y, x0, y0, sigma_x, sigma_y)

    return x, y, z

def create_2d_gradient(x_list, y_list, sigma_x_list, sigma_y_list, w_x_list, w_y_list, w_sigma_x_list, w_sigma_y_list, target_x, target_y): 
    global PW_RATIO    

    x = 0.0
    y = 0.0

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

    return x, y



def create_2d_wall_function(x0, y0, sigma_x, sigma_y):

    size = 100
    x = np.linspace(-1, 1, size)
    y = np.linspace(-1, 1, size)
    
    x, y = np.meshgrid(x, y)
    if abs(sigma_x - 0.0) <= 0.0001:        
        z = gaussian_wall(y, y0, sigma_y)
    else:
        z = gaussian_wall(x, x0, sigma_x)

    return x, y, z

def draw_thread(name):

    global good_fitness
    x = np.arange(1, len(good_fitness) + 1, 1, dtype=int)

    plt.clf()
    plt.title("Fitness of every child")
    plt.xlabel("Generation")
    plt.ylabel("Fitness")
    plt.plot(x, good_fitness, '-')
    plt.draw()
    plt.pause(0.001)

def G(x): return 1 / (1 + np.exp(-0.6 * x))

def key_listener(name):
    global SHOW_DRAW
    while True:
        if input("Show map:") == "h":
            SHOW_DRAW = not SHOW_DRAW
            
def pk_key_listener(name):
    global human_right, human_left
    button_delay = 0.01

    while True:
        DELAY = False        
        keys = pygame.key.get_pressed()    
            
        if keys[pygame.K_a]:
            if human_right > 0.0:
                human_right = 0.8
                human_left = 0.5
            else:
                human_right = -0.5
                human_left = -0.3
            DELAY = True 

        if keys[pygame.K_d]:
            if human_right > 0.0:
                human_left = 0.8             
                human_right = 0.5
            else:
                human_left = -0.5
                human_right = -0.3
            DELAY = True
   
        if keys[pygame.K_w]:
            human_right = 0.7
            human_left = 0.7 
            DELAY = True
        
        if keys[pygame.K_s]:   
            human_right = -0.5
            human_left = -0.5
            DELAY = True                                
                
        if DELAY:
            time.sleep(button_delay)
               
        pygame.event.pump()           

def gazebo_thread(name, server):
    server.allow_reuse_address = True
    server.server_bind()
    server.server_activate()
    server.serve_forever()
    
def initialize_generation(NUM_PARENT, NUM_KID):
    parents = []
    kids = []

    for i in range(NUM_PARENT):
        parents += [make_kid()]
        
    for i in range(NUM_KID):
        kids += [make_kid()]

    return parents, kids

def selection(parents, kids, parents_fitness, kids_fitness, NUM_KID):
    all_fitness = np.append(parents_fitness, kids_fitness)
    best_idx = np.argsort(all_fitness)[::-1]      
    
    new_parents = [0.0] * NUM_KID
    new_parents_fitness = np.zeros(len(parents))
    
    for i in range(NUM_KID):
        if best_idx[i] / NUM_KID >= 1:
            new_parents[i] = kids[best_idx[i] % NUM_KID]
            new_parents_fitness[i] = kids_fitness[best_idx[i] % NUM_KID]
        else:
            new_parents[i] = parents[best_idx[i]]
            new_parents_fitness[i] = parents_fitness[best_idx[i]]
                
    return new_parents, new_parents_fitness

def new_generation(parents):

    kids = []

    for i in range(len(parents)):
        kids += [make_kid(parents[i])]

    kids_fitness = np.zeros(len(kids))

    return kids, kids_fitness

def optimization_function(x): #CMA
    #print(x)
    global server, kids, parents, kids_fitness, parents_fitness, count_population, count_generation, NUM_KID, es, bo

    server.new_game = True
    server.player += 1
    server.player_type = 'kid'
    kids[server.player] = x
    server.player_weight = kids[server.player]        
    
    server.playing = True 
    
    while True:
        if not server.playing:
            break
    
    print(server.player_weight)
    
    while True:
        if not server.playing:
            break
    
    kids_fitness[server.player] = server.fitness
    
    data_dict = {'fitness': server.fitness, 'weights': server.player_weight, 'count_generation': count_generation, 'player_idx': server.player}
    output = open(DATA_PATH + 'weights/cma/cma_' + str(count_generation) + "_" + str(server.player) + "_" + str(server.fitness) + '.pkl', 'wb')
    pickle.dump(data_dict, output)
    output.close()
    
    avg_file = open(DATA_PATH + 'distance/cma/cma_' + str(count_generation) + "_" + str(server.player) + "_" + str(server.fitness) + '.pkl', 'wb')
    pickle.dump(server.avg_avg_distances, avg_file)
    avg_file.close()
    server.avg_avg_distances = []
    
    print('parents fitness', parents_fitness)
    print('kids fitness', kids_fitness)
    
    if server.player % NUM_KID == (NUM_KID - 1):
        parents, parents_fitness = selection(parents, kids, parents_fitness, kids_fitness, NUM_KID)
        print('new parents fitness', parents_fitness)
        server.player = -1
        fit_file = open(DATA_PATH + 'fitness/cma.txt', 'a+')
        fit_file.write(" " + np.array2string(kids_fitness)[1:-1])
        fit_file.close()
        kids_fitness = np.zeros(NUM_KID)
        
        output = open(DATA_PATH + 'es/' + sys.argv[1] + "_" + str(count_generation) + ".pkl", 'wb')
        pickle.dump([es, count_generation], output)
        output.close()
        
        if count_generation >= N_GENERATIONS:
            exit(0)
        
        count_generation += 1
        
    print('CMA:', es.sp.disp())
    
    return -server.fitness

def optimization_function_BAYESIAN(**weights):   
    
    global server, kids, parents, kids_fitness, parents_fitness, count_generation, NUM_KID, es, bo, BAYES_GENE_FACTOR

    x = [0] * NUM_PARAMETERS
    for key in weights.keys():
        x[int(key[1:])] = weights[key]

    x = np.array(x)

    server.new_game = True
    server.player = 0
    server.player_type = 'kid'
    kids[player] = x
    server.player_weight = kids[player]
    server.playing = True 
    
    while True:
        if not server.playing:
            break
    
    kids_fitness[player] = server.fitness
    
    data_dict = {'fitness': server.fitness, 'weights': server.player_weight, 'count_generation': count_generation, 'player_idx': server.player}
    output = open(DATA_PATH + 'weights/bo/bo_' + str(count_generation) + "_" + str(server.fitness) + '.pkl', 'wb')
    pickle.dump(data_dict, output)
    output.close()
    
    avg_file = open(DATA_PATH + 'distance/bo/bo_' + str(count_generation) + "_" + str(server.player) + "_" + str(server.fitness) + '.pkl', 'wb')
    pickle.dump(server.avg_avg_distances, avg_file)
    avg_file.close()
    server.avg_avg_distances = []
    
    print('parents fitness', parents_fitness)
    print('kids fitness', kids_fitness)
    
    parents, parents_fitness = selection(parents, kids, parents_fitness, kids_fitness, NUM_KID)

    print('new parents fitness', parents_fitness)
    fit_file = open(DATA_PATH + 'fitness/bo.txt', 'a+')
    fit_file.write(" " + np.array2string(kids_fitness)[1:-1])
    fit_file.close()
    print(count_generation)
    if count_generation >= N_GENERATIONS * BAYES_GENE_FACTOR:
        print('Finished!!!')
        exit(0)
        
    count_generation += 1

    return kids_fitness[player]
    
def optimization_function_simple_es(weights):   
    
    global server, kids, parents, kids_fitness, parents_fitness, count_generation, NUM_KID, simple_es, W_MAX
    
    server.new_game = True
    server.player += 1
    server.player_type = 'kid'
    kids[server.player] = np.clip(np.array(weights), -W_MAX, W_MAX)
    server.player_weight = kids[server.player]    
    server.playing = True 
   
    print(server.player_weight)
    
    while True:
        if not server.playing:
            break
    
    kids_fitness[server.player] = server.fitness
    
    data_dict = {'fitness': server.fitness, 'weights': server.player_weight, 'count_generation': count_generation, 'player_idx': server.player}
    output = open(DATA_PATH + 'weights/es/es_' + str(count_generation) + "_" + str(server.player) + "_" + str(server.fitness) + '.pkl', 'wb')
    pickle.dump(data_dict, output)
    output.close()
    
    avg_file = open(DATA_PATH + 'distance/es/es_' + str(count_generation) + "_" + str(server.player) + "_" + str(server.fitness) + '.pkl', 'wb')
    pickle.dump(server.avg_avg_distances, avg_file)
    avg_file.close()
    server.avg_avg_distances = []
    
    print('parents fitness', parents_fitness)
    print('kids fitness', kids_fitness)
    
    if server.player % NUM_KID == (NUM_KID - 1):
        parents, parents_fitness = selection(parents, kids, parents_fitness, kids_fitness, NUM_KID)
        print('new parents fitness', parents_fitness)
        server.player = -1
        fit_file = open(DATA_PATH + 'fitness/es.txt', 'a+')
        fit_file.write(" " + np.array2string(kids_fitness)[1:-1])
        fit_file.close()
        kids_fitness = np.zeros(NUM_KID)
        
        if count_generation >= N_GENERATIONS:
            exit(0)
        
        count_generation += 1
        
    return server.fitness

def update_parameters(predator_sigma, wall_sigma, alpha):

    global sigma_x_list, sigma_y_list, wall_sigma_x_list, wall_sigma_y_list

    # sigma of predators
    sigma_x_list = np.array([1.0,1.0,1.0,1.0]) * predator_sigma
    sigma_y_list = np.array([1.0,1.0,1.0,1.0]) * predator_sigma

    # sigma of wall
    wall_sigma_x_list = np.array([0.1,0.1,0.0,0.0]) * wall_sigma
    wall_sigma_y_list = np.array([0.0,0.0,0.1,0.1]) * wall_sigma

if __name__ == "__main__":

    global server
    global parents, kids, parents_fitness, kids_fitness, es, count_population, BAYES_GENE_FACTOR
    
    if sys.argv[1] == "CMA":
        CMA = True
        BAYESIAN = False
        SIMPLE_ES = False
    elif sys.argv[1] == "BO":
        CMA = False
        BAYESIAN = True
        SIMPLE_ES = False
    elif sys.argv[1] == "ES":
        CMA = False
        BAYESIAN = False
        SIMPLE_ES = True
    elif sys.argv[1] == "MAXP":
        CMA = False
        BAYESIAN = False
        SIMPLE_ES = False
        LOAD_WEIGHT = True
        MAX_PARAMETERS = True
        PK = True
    elif sys.argv[1] == "PK":
        CMA = False
        BAYESIAN = False
        SIMPLE_ES = False
        LOAD_WEIGHT = True
        MAX_PARAMETERS = False
        PK = True
    else:
        print('sys argv 1')
        exit(0)
    
    if HUMAN:
        pygame.init()
        BLACK = (0,0,0)
        WIDTH = 100
        HEIGHT = 100
        windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
        windowSurface.fill(BLACK)
        if PK and HUMAN:    
            _thread.start_new_thread(pk_key_listener, ("pk",))

    if SHOW_DRAW:
        _thread.start_new_thread(draw_2d_heatmap_thread, ("heatmap",))

        _thread.start_new_thread(key_listener, ("key",))   

    HOST, PORT = "127.0.0.1", 9527
            
    server = socketserver.TCPServer((HOST, PORT), MyTCPHandler, False)
    
    _thread.start_new_thread(gazebo_thread, ("gazebo", server))

    NUM_PARENT = NUM_KID = 13
    
    if CMA:
        count_population = 1
        opts = cma.CMAOptions()
        #opts.set('ftarget', np.inf)
        #print(opts)
        #opts.set('maxfevals', 1)
        #opts.set('popsize', 1)
        es = cma.CMAEvolutionStrategy(NUM_PARAMETERS * [0.0], 0.5,
            {'BoundaryHandler': cma.BoundPenalty,
                'bounds': [-W_MAX, W_MAX]
            })
        #NUM_PARENT = NUM_KID = 16
        
    elif BAYESIAN: 
        BAYES_GENE_FACTOR = NUM_PARENT
        NUM_PARENT = NUM_KID = 1                
    
    parents_fitness = np.ones(NUM_PARENT) * -10000
    kids_fitness = np.zeros(NUM_KID)
    
    parents, kids = initialize_generation(NUM_PARENT, NUM_KID)
    server.playing = False
    
    player_type = 'kid'
    player = -1
    count_generation = 1
    
    if LOAD_WEIGHT:
        NUM_PARENT = NUM_KID = 1
        parents_fitness = np.ones(NUM_PARENT) * -10000
        kids_fitness = np.zeros(NUM_KID)       
        parents, kids = initialize_generation(NUM_PARENT, NUM_KID)
        #pkl_file = open('/home/jaqq/pp/weights/cma/cma_109_10.pkl', 'rb')
        pkl_file = open('/home/jaqq/pp/weights/cma/cma_106_12.pkl', 'rb')
       
        #pkl_file = open('/home/jaqq/learning/gazebo/pp/trainer/data/1_25 VERY GOOD/weights/cma/cma_97_11_5.060749581628671.pkl', 'rb')
        data_dict = pickle.load(pkl_file)
        #count_generation = data_dict['count_generation']
        count_generation = 0
        #MUT_STRENGTH = data_dict['MUT_STRENGTH']
        parents_fitness[0] = data_dict['fitness']
        parents[0] = data_dict['weights']
        kids, kids_fitness = new_generation(parents)
        
        pkl_file.close()
    
    
    print('count_generation:', count_generation)        
    

    if CMA:
        server.player = -1
        es.optimize(optimization_function)        
    elif BAYESIAN: 
        weights = {}
        weights_explore = {}
        for i in range(NUM_PARAMETERS):
            weights['w' + str(i)] = (-W_MAX, W_MAX)  
            weights_explore['w' + str(i)] = [-W_MAX, W_MAX]
            
        gp_params = {
             'alpha': 1e-5,
             'xi': 0.01
            }
            
        kappa = 1.0
        bo = BayesianOptimization(optimization_function_BAYESIAN, weights)
        bo.maximize(init_points=NUM_PARAMETERS, n_iter=99999999999, acq='ucb', kappa=kappa, **gp_params)        
    elif SIMPLE_ES:
        server.player = -1
        es = EvolutionStrategy(np.zeros(NUM_PARAMETERS), optimization_function_simple_es, population_size=NUM_KID, sigma=0.4, learning_rate=0.2, decay=0.98, num_threads=1) # 0.4 0.2 0.99
        es.run(N_GENERATIONS, print_step=9999999999)
        
    else:
        
        if MAX_PARAMETERS:
            predator_sigma = PREDATOR_SIGMA_LOW
            wall_sigma = WALL_SIGMA_LOW
            PW_RATIO = PW_RATIO_LOW
            update_parameters(predator_sigma, wall_sigma, PW_RATIO)
            min_predator_fitness = 100000
            best_sigma = (predator_sigma, wall_sigma, PW_RATIO)        
        
        while True:
            if not server.playing:
                if player != -1:
                    kids_fitness[player] = server.fitness
                    
                    if MAX_PARAMETERS:
                
                        prey_negative_score = server.fitness      
                        print('sigma:', (predator_sigma, wall_sigma, PW_RATIO))
                        print('prey_negative_score:', prey_negative_score)                    

                        if prey_negative_score < min_predator_fitness:
                            best_sigma = (predator_sigma, wall_sigma, PW_RATIO)
                            min_predator_fitness = prey_negative_score
                        
                        print('best sigma:', best_sigma)

                        print()
                        
                        predator_sigma += PREDATOR_SIGMA_STEP
                        if abs(predator_sigma - (PREDATOR_SIGMA_HIGH + PREDATOR_SIGMA_STEP)) < 0.00001:
                            predator_sigma = PREDATOR_SIGMA_LOW
                            wall_sigma += WALL_SIGMA_STEP
                        
                        update_parameters(predator_sigma, wall_sigma, PW_RATIO)
                            
                        if abs(wall_sigma - (WALL_SIGMA_HIGH + WALL_SIGMA_STEP)) < 0.00001:
                            
                            PW_RATIO += PW_RATIO_STEP
                            predator_sigma = PREDATOR_SIGMA_LOW
                            wall_sigma = WALL_SIGMA_LOW
                            update_parameters(predator_sigma, wall_sigma, PW_RATIO)
                            
                            if abs(PW_RATIO - (PW_RATIO_HIGH + PW_RATIO_STEP)) < 0.00001:
                                print('END UPDATE PARAMETERS')
                                exit(0)                                             
                    
                player += 1
                            
                if player == NUM_KID:
                    # write python dict to a file
                    
                    if not PK:
                        data_dict = {'parents_fitness': parents_fitness, 'parents': parents, 'MUT_STRENGTH': MUT_STRENGTH, 'count_generation': count_generation}
                        output = open('trained_data/' + str(count_generation) + '.pkl', 'wb')
                        pickle.dump(data_dict, output)
                        output.close()
                    
                    if PK:
                        fit_file = open('fitness/pk.txt', 'a+')
                        fit_file.write(" " + str(kids_fitness[0]))
                        fit_file.close()
                        
                        avg_file = open('distance/pk/pk_' + str(count_generation) + "_" + str(server.player) + "_" + str(server.fitness) + '.pkl', 'wb')
                        pickle.dump(server.avg_avg_distances, avg_file)
                        avg_file.close()
                        server.avg_avg_distances = []
                    
                    print('parents fitness', parents_fitness)
                    print('kids fitness', kids_fitness)
                    parents, parents_fitness = selection(parents, kids, parents_fitness, kids_fitness, NUM_KID)
                    kids, kids_fitness = new_generation(parents)                                
                    player = 0
                    count_generation += 1
                    print('new parents fitness', parents_fitness)
                    print('count_generation:', count_generation)
                    MUT_STRENGTH *= 1 - 1 / DNA_SIZE
                    if MUT_STRENGTH < MUT_MIN:
                        MUT_STRENGTH = MUT_MAX
                    print('MUT_STRENGTH:', MUT_STRENGTH)
                
                if player != NUM_KID:
                    print('player:', player)
                    
                #if MAX_PARAMETERS:
                #    for predator_sigma in range(PREDATOR_SIGMA_LOW, PREDATOR_SIGMA_HIGH + PREDATOR_SIGMA_STEP, PREDATOR_SIGMA_STEP):
                #        for wall_sigma in range(WALL_SIGMA_LOW, WALL_SIGMA_HIGH + WALL_SIGMA_STEP, WALL_SIGMA_STEP):                
                
                if count_generation > N_GENERATIONS:
                
                    print('finish')
                    break
                                                               
                server.new_game = True
                server.player = player
                server.player_type = 'kid'            
                server.player_weight = kids[player]                
                server.playing = True            
            



