import socketserver
from random import randint

from time import sleep
import numpy as np
import matplotlib.pyplot as plt

import _thread

import math

parent_dir = "parent/"

ff = open('fitness.txt', 'w+')
ff.close()
fp = open('fitness_prey.txt', 'w+')
ff.close()

LOAD_WEIGHT = False
PK = False

left_s = 0.0
right_s = 0.0
update_prey = True

heatmap_lock = False

NUM_INPUTS_AGENT = 2
NUM_PREDATOR = 4
NUM_PREY = 1

N_GENERATIONS = 500000
MUT_STRENGTH = 3.0
PREY_MUT_STRENGTH = 2.0
MUT_MAX = 3.0
W_MAX = 0.5
B_MAX = 0.5


ALTERNATIVE_FLAG = 0
REEVALUATION_PROB = 0.5
PREY_REEVALUATION_PROB = 0.5

NUM_EVALUATION = 1
PLAY_PERIOD = 60

#I_S = int(NUM_INPUTS_AGENT * (math.factorial(NUM_PREDATOR + NUM_PREY) / (math.factorial((NUM_PREDATOR + NUM_PREY) - 2) * math.factorial(2))))
#I_S = int(2 * (math.factorial(NUM_PREDATOR + NUM_PREY) / (math.factorial((NUM_PREDATOR + NUM_PREY) - 2) * math.factorial(2)))) + NUM_PREDATOR + NUM_PREY

#I_S = NUM_INPUTS_AGENT * (NUM_PREDATOR + NUM_PREY)
I_S = 2 * 4 + 4 + 3
#I_S = NUM_INPUTS_AGENT * (1 + NUM_PREY)
H_S = 10
O_S = 2

DNA_SIZE = (H_S * I_S + H_S * O_S) * 10 #H_S*2 + 2             # DNA (real number)
pk_rate = 1/5
if not LOAD_WEIGHT:
    prey_parent = (np.random.rand(I_S, H_S) / 10, np.random.rand(H_S, O_S) / 10, np.random.rand(1, 1), np.random.rand(1, 1))
    prey_kid = (np.random.rand(I_S, H_S) / 10, np.random.rand(H_S, O_S) / 10, np.random.rand(1, 1), np.random.rand(1, 1))
else:
    prey_parent = (np.loadtxt(parent_dir + 'prey_w0.txt'), np.loadtxt(parent_dir + 'prey_w1.txt'), np.loadtxt(parent_dir + 'prey_b0.txt').reshape(1, 1), np.loadtxt(parent_dir + 'prey_b1.txt').reshape(1, 1))
    prey_kid = (np.loadtxt(parent_dir + 'prey_w0.txt'), np.loadtxt(parent_dir + 'prey_w1.txt'), np.loadtxt(parent_dir + 'prey_b0.txt').reshape(1, 1), np.loadtxt(parent_dir + 'prey_b1.txt').reshape(1, 1))



if not LOAD_WEIGHT:
    parent = (np.random.rand(I_S, H_S) / 10, np.random.rand(H_S, O_S) / 10, np.random.rand(1, 1), np.random.rand(1, 1))    
    kid = (np.random.rand(I_S, H_S) / 10, np.random.rand(H_S, O_S) / 10, np.random.rand(1, 1), np.random.rand(1, 1))
else:
    parent = (np.loadtxt(parent_dir + 'predator_w0.txt'), np.loadtxt(parent_dir + 'predator_w1.txt'), np.loadtxt(parent_dir + 'predator_b0.txt').reshape(1, 1), np.loadtxt(parent_dir + 'predator_b1.txt').reshape(1, 1))
    kid = (np.loadtxt(parent_dir + 'predator_w0.txt'), np.loadtxt(parent_dir + 'predator_w1.txt'), np.loadtxt(parent_dir + 'predator_b0.txt').reshape(1, 1), np.loadtxt(parent_dir + 'predator_b1.txt').reshape(1, 1))

parents = []
for i in range(NUM_PREDATOR):
    parents.append((np.random.rand(I_S, H_S) / 10, np.random.rand(H_S, O_S) / 10, np.random.rand(1, 1), np.random.rand(1, 1)))

kids = []
for i in range(NUM_PREDATOR):
    kids.append((np.random.rand(I_S, H_S) / 10, np.random.rand(H_S, O_S) / 10, np.random.rand(1, 1), np.random.rand(1, 1)))



sim_time = 0
previous_sim_time = 0
current_players = [0] * (NUM_PREDATOR + NUM_PREY)
switch_times = [0] * (NUM_PREDATOR + NUM_PREY)


parent_fitnesses = np.zeros((NUM_PREDATOR, )) - 1000000
kid_fitnesses = np.zeros((NUM_PREDATOR, )) - 1000000
prey_parent_fitness = -1000000
prey_kid_fitness = -1000000

total_parent_fitness = 0.0
total_kid_fitness = 0.0
prey_total_parent_fitness = 0.0
prey_total_kid_fitness = 0.0

average_cos = np.zeros((NUM_PREDATOR, ))
average_speed = np.zeros((NUM_PREDATOR, ))
average_cos_speed = np.zeros((NUM_PREDATOR, ))
average_distance = np.zeros((NUM_PREDATOR, ))


prey_average_cos_speed = 0.0
prey_average_speed = 0.0
prey_average_min_distance = 0.0

previous_position_x = np.zeros((NUM_PREDATOR, ))
previous_position_y = np.zeros((NUM_PREDATOR, ))

init_position_x = [-1000] * (NUM_PREDATOR + NUM_PREY)
init_position_y = [-1000] * (NUM_PREDATOR + NUM_PREY)

good_fitness = []
good_fitness_prey = []
good_generation = []
count_generation = 1

x0_list = [0,0,0,0]
y0_list = [0,0,0,0]

wall_x_list = [1.0,-1.0,0.0,0.0]
wall_y_list = [0.0,0.0,1.0,-1.0]

sigma_x_list = np.array([1.0,1.0,1.0,1.0]) * 0.2
sigma_y_list = np.array([1.0,1.0,1.0,1.0]) * 0.2

wall_sigma_x_list = [0.1,0.1,0.0,0.0] 
wall_sigma_y_list = [0.0,0.0,0.1,0.1]

def prod(iterable):
    p = 0
    f = []
    for n in iterable:
        p += n
        f += [n]        
    return p / (np.std(np.array(f)) + 2)

class MyTCPHandler(socketserver.BaseRequestHandler):
    """
    The RequestHandler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):

        global switch_times, current_players, parent_fitnesses, kid_fitnesses, total_parent_fitness, total_kid_fitness, prey_parent_fitness, prey_kid_fitness, prey_total_parent_fitness, prey_total_kid_fitness, sim_time, MUT_STRENGTH, MUT_MAX, pk_rate, DNA_SIZE, parent, kid, good_fitness, good_fitness_prey, good_generation, count_generation, prey_parent, prey_kid, PREY_MUT_STRENGTH, left_s, right_s, update_prey, previous_position_x, previous_position_y, average_speed, average_cos, average_cos_speed, previous_sim_time, prey_average_cos_speed, ALTERNATIVE_FLAG, prey_average_min_distance, prey_average_speed, average_distance, x0_list, y0_list, sigma_x_list, sigma_y_list, heatmap_lock

        # self.request is the TCP socket connected to the client
        requestForUpdate = self.request.recv(1024)                

        reset_flag = 0
        count_update = 0
        count_play = 1
        catch_flag = False
        while requestForUpdate != '':

            

            count_update = count_update + 1
            remove_data = 0

            p_info_list = [] # prey
            P_info_list = [] # predator            

            self.data = requestForUpdate.strip()
            
            #print('selfdata:', self.data)
            #print("{} wrote:".format(self.client_address[0]))
            gazebo_info_split = self.data.decode("utf-8").rstrip('\0').split(";")[-(6 + NUM_PREDATOR + NUM_PREY):-1];
            #print(gazebo_info_split)
            for gazebo_info_single in gazebo_info_split:
                #print(len(gazebo_info_single))
                if gazebo_info_single == "":
                    continue
                #print(gazebo_info_single)
                p_infos = gazebo_info_single.split(",")

                #print(p_infos, len(p_infos))

                if p_infos[0] == "w":
                    continue

                if p_infos[0] == "t":
                    sim_time = float(p_infos[1])
                    #print("sim: ", sim_time)
                    continue

                if (p_infos[0] == "p" or p_infos[0] == "P") and len(p_infos) < 5:
                    continue

                #p_id = float(p_infos[1])
                p_x = float(p_infos[2])
                p_y = float(p_infos[3])
                p_yaw = float(p_infos[4].rstrip('\0')[0:5])
                p_vector = np.array([p_x, p_y, p_yaw])
                
                if p_infos[0] == "p":                
                    p_info_list.append(p_vector)
                if p_infos[0] == "P":
                    P_info_list.append(p_vector)
            
            

            ##### CHECK RESET

            reset_failed = 0

            Pp_info_list = P_info_list + p_info_list

            if len(Pp_info_list) != NUM_PREDATOR + NUM_PREY:
                print("RESET1", gazebo_info_split)
                self.request.sendall(bytes("RESET;", "utf-8"))
                send_string = ""
                for i in range(NUM_PREDATOR):
                    send_string = send_string + str(i + 1) + "," + "0.0" + "," + "0.0" + ";"
                
                send_string = send_string + "p" + "," + "0.0" + "," + "0.0" + ";\n"

                self.request.sendall(bytes(send_string, "utf-8"))
                requestForUpdate=self.request.recv(1024)
                continue

            for idx_1, P_info_1 in enumerate(Pp_info_list):                
                if init_position_x[idx_1] == -1000:
                    init_position_x[idx_1] = P_info_1[0]
                    
                    if idx_1 < NUM_PREDATOR:
                        previous_position_x[idx_1] = P_info_1[0]
                if init_position_y[idx_1] == -1000:
                    init_position_y[idx_1] = P_info_1[1]
        
                    if idx_1 < NUM_PREDATOR:
                        previous_position_y[idx_1] = P_info_1[1]
                if reset_flag == 1:
                    if abs(P_info_1[0] - init_position_x[idx_1]) > 0.01 or abs(P_info_1[1] - init_position_y[idx_1]) > 0.01:                        
                        if idx_1 == NUM_PREDATOR:
                            pass
                        else:
                            reset_failed = 1
                    else:
                        pass

                    switch_times[idx_1] = sim_time

            

            if reset_flag == 1 and reset_failed == 1:
                print("RESET2")
                self.request.sendall(bytes("RESET;", "utf-8"))
                send_string = ""
                for i in range(NUM_PREDATOR):
                    send_string = send_string + str(i + 1) + "," + "0.0" + "," + "0.0" + ";"
                
                send_string = send_string + "p" + "," + "0.0" + "," + "0.0" + ";\n"
                self.request.sendall(bytes(send_string, "utf-8"))
                requestForUpdate=self.request.recv(1024)
                continue
            else:
                reset_flag = 0
            #####

            ##### CHECK CATCH
            for idx_1, P_info_1 in enumerate(P_info_list):                    
                distance = np.linalg.norm(p_info_list[0][0:2] - P_info_1[0:2])
                #print("DISTANCE: ", distance)
                if distance < 0.16 and catch_flag == False:
                    catch_flag = True
                    catch_time = sim_time - switch_times[idx_1]
                    #print("CATCH!!!")

            #num_prey = len(p_info_list)
            #num_predator = len(P_info_list)

            nn_input = np.zeros((NUM_PREDATOR + NUM_PREY, I_S))
            #nn_input_prey = np.zeros((NUM_PREY, I_S))
            prey_idx = NUM_PREDATOR

            for idx_1, P_info_1 in enumerate(Pp_info_list):
                if switch_times[idx_1] == 0:
                    switch_times[idx_1] = sim_time
                
                idx_list = []
                sin_list = []

                for idx_2, P_info_2 in enumerate(Pp_info_list):
                    
                    if idx_1 == idx_2:
                        continue
                    
                    if idx_2 == prey_idx:
                        continue
                    
                    a_vector = np.array([np.cos(P_info_1[2]), np.sin(P_info_1[2]), 0.0])
                    #print('a', a_vector)
                    xy_diff = (P_info_2 - P_info_1)[0:2]
                    #print('xy1', xy_diff)
                    xy_diff = xy_diff / np.linalg.norm(xy_diff)
                    #print('xy2', xy_diff)
                    b_vector = np.array([xy_diff[0], xy_diff[1], 0.0])
                    #print('b', b_vector)
                    cross = np.cross(b_vector, a_vector)
                    #print('o', cross)
                    theta = np.arcsin(np.linalg.norm(cross))
                    if cross[2] < 0.0:
                        theta = -theta
                    cos = b_vector.dot(a_vector)
                    if cos < 0.0:
                        theta = (-theta) + np.pi * theta / abs(theta)
                    
                    idx_list += [idx_2]
                    sin_list += [theta]

                # Ranking from left to right    
                arg_sort = np.argsort(np.array(sin_list))
                
                count_cross = 0
                
                for arg in arg_sort:
                    idx_2 = idx_list[arg]

                    nn_input_step = NUM_INPUTS_AGENT
                    nn_input_start = (count_cross) * NUM_INPUTS_AGENT
                    nn_input_end = (count_cross) * NUM_INPUTS_AGENT + nn_input_step

                    input_block = np.array([0.0, 0.0])
                    input_block[0] = np.linalg.norm((Pp_info_list[idx_2] - P_info_1)[0:2])
                    #input_block[2] = Pp_info_list[idx_2][2]
                    input_block[1] = sin_list[arg]
                    #print(nn_input_start, nn_input_end)
                    nn_input[idx_1][nn_input_start:nn_input_end] = input_block
                    count_cross += 1                                                    

                # angle between predator and prey
                if idx_1 + 1 != len(Pp_info_list):

                    a_vector = np.array([np.cos(P_info_1[2]), np.sin(P_info_1[2]), 0.0])
                    #print('a', a_vector)
                    xy_diff = (p_info_list[0] - P_info_1)[0:2]
                    #print('xy1', xy_diff)
                    xy_diff = xy_diff / np.linalg.norm(xy_diff)
                    #print('xy2', xy_diff)
                    b_vector = np.array([xy_diff[0], xy_diff[1], 0.0])
                    #print('b', b_vector)
                    cross = np.cross(b_vector, a_vector)
                    #print('o', cross)
                    theta = np.arcsin(np.linalg.norm(cross))
                    
                    if cross[2] < 0.0:
                        theta = -theta
                    
                    cos = b_vector.dot(a_vector)
                    if cos < 0.0:
                        theta = (-theta) + np.pi * theta / abs(theta)

                    nn_input_step = NUM_INPUTS_AGENT
                    nn_input_start = (count_cross) * (NUM_INPUTS_AGENT)
                    nn_input_end = (count_cross) * (NUM_INPUTS_AGENT) + nn_input_step
                    #print(nn_input_start, nn_input_end)
                    #input_block = np.abs((p_info_list[0] - P_info_1))
                    input_block = np.array([0.0, 0.0])
                    input_block[0] = np.linalg.norm((p_info_list[0] - P_info_1)[0:2])
                    input_block[1] = theta
                    nn_input[idx_1][nn_input_start:nn_input_end] = input_block
                    count_cross += 1
                    
                    
                    #print(nn_input[idx_1])
                else:
                    #nn_input[idx_1]
                    pass

                # wall
                nn_input_step = 4
                nn_input_start = (count_cross) * NUM_INPUTS_AGENT
                nn_input_end = (count_cross) * NUM_INPUTS_AGENT + nn_input_step
                #input_block = np.abs((p_info_list[0] - P_info_1))
                input_block = np.zeros(4)
                input_block[0] = 1.0 - P_info_1[0]
                input_block[1] = -1.0 - P_info_1[0]
                input_block[2] = 1.0 - P_info_1[1]  
                input_block[3] = -1.0 - P_info_1[1]
                input_block = np.abs(input_block)

                nn_input[idx_1][nn_input_start:nn_input_end] = input_block
                count_cross += 1
                
                nn_input[idx_1][-3:] = P_info_1
                #if idx_1 == prey_idx:
                #    print('input', nn_input[idx_1])

            for idx_1, P_info_1 in enumerate(Pp_info_list):

                if idx_1 != prey_idx:
                    predator_yaw_vector = np.array([np.cos(P_info_1[2]), np.sin(P_info_1[2])])
                    predator_prey_vector = np.array(p_info_list[0][0:2] - P_info_1[0:2])

                    cos_value = predator_yaw_vector.dot(predator_prey_vector) / (np.linalg.norm(predator_yaw_vector) * np.linalg.norm(predator_prey_vector))

                    
                    average_cos[idx_1] = count_update * average_cos[idx_1] / (count_update + 1) + cos_value / (count_update + 1)

                    average_distance[idx_1] = count_update * average_distance[idx_1] / (count_update + 1) + np.linalg.norm(predator_prey_vector) / (count_update + 1)


                    time_diff = sim_time - previous_sim_time
                    if time_diff != 0:
                        x_speed = abs(previous_position_x[idx_1] - P_info_1[0]) / time_diff
                        y_speed = abs(previous_position_y[idx_1] - P_info_1[1]) / time_diff
                        #print('x', x_speed, 'y', y_speed, previous_position_x[idx_1], P_info_1[0])
                        speed_value = (x_speed ** 2 + y_speed ** 2) ** (1/2) * 1000
                        average_speed[idx_1] = count_update * average_speed[idx_1] / (count_update + 1) + speed_value / (count_update + 1)
                        cos_speed_value = cos_value * speed_value / np.linalg.norm(predator_prey_vector)
                        average_cos_speed[idx_1] = count_update * average_cos_speed[idx_1] / (count_update + 1) + cos_speed_value / (count_update + 1)

                    previous_position_x[idx_1] = P_info_1[0]
                    previous_position_y[idx_1] = P_info_1[1]
    
                    if idx_1 + 1 == NUM_PREDATOR:
                        previous_sim_time = sim_time
                else:
                    distance_pP = []
                    for idx_distance, P_info_distance in enumerate(P_info_list):
                        distance_pP += [np.linalg.norm(p_info_list[0][0:2] - P_info_distance[0:2])]
                    prey_average_min_distance = count_update * prey_average_min_distance / (count_update + 1) + min(distance_pP) / (count_update + 1)


                if sim_time < switch_times[idx_1] + PLAY_PERIOD:# and catch_flag != 1:
                    if idx_1 + 1 != len(Pp_info_list):
                        current_player_parameters = kid
                    else:
                        current_player_parameters = prey_kid

                    
                else:
                    if idx_1 + 1 != len(Pp_info_list):
                        predator_yaw_vector = np.array([np.cos(P_info_1[2]), np.sin(P_info_1[2])])
                        predator_prey_vector = np.array(p_info_list[0][0:2] - P_info_1[0:2])
                        if average_speed[idx_1] > 1.0:
                            average_speed[idx_1] = 1.0
                        #kid_fitnesses[idx_1] = 1 / np.linalg.norm(p_info_list[0][0:2] - P_info_1[0:2])# * average_cos[idx_1] * average_speed[idx_1]
                        kid_fitnesses[idx_1] = 1 / average_distance[idx_1]
                        print(kid_fitnesses[idx_1])
                        #kid_fitnesses[idx_1] = average_cos_speed[idx_1]
                        diff_position = ((init_position_x[idx_1] - P_info_1[0]) ** 2 + (init_position_y[idx_1] - P_info_1[1]) ** 2) ** (1/2)
                        #if average_speed[idx_1] < 0.1 and diff_position < 0.1:
                        #    print('no move', idx_1)
                        #    kid_fitnesses[idx_1] = -1
                        #print('cos',idx_1 , ',', average_cos[idx_1])
                        #print('speed', idx_1 , ',', average_speed[idx_1])
                        #print('cos_speed', idx_1 , ',', average_cos_speed[idx_1])
                    else:
                        play_time = abs(sim_time - switch_times[idx_1])
                        if play_time < PLAY_PERIOD:
                            prey_kid_fitness = play_time
                        else:
                            distance_pP = []
                            for idx_distance, P_info_distance in enumerate(P_info_list):
                                distance_pP += [np.linalg.norm(p_info_list[0][0:2] - P_info_distance[0:2])]

                            #if not catch_flag:
                            #    prey_kid_fitness = PLAY_PERIOD + min(distance_pP) / NUM_PREDATOR * 50
                            #else:
                            #    prey_kid_fitness = catch_time
                            prey_kid_fitness = prey_average_min_distance
                            if catch_flag:
                                prey_kid_fitness *= 0.5

                    if abs(switch_times[idx_1] - sim_time) > PLAY_PERIOD + 1.0:
                        remove_data = 0

                    switch_times[idx_1] = sim_time

                    reset_flag = 1

                x0_list = previous_position_x
                y0_list = previous_position_y

                if reset_flag != 1:        
                    wheel_speed = forward(nn_input[idx_1], current_player_parameters[0], current_player_parameters[1], \
                                                    current_player_parameters[2], current_player_parameters[3])[0]
                    
                    if idx_1 + 1 != len(Pp_info_list):
                        
                        #self.request.sendall(bytes(str(idx_1 + 1) + "," + str(0.0) + "," + str(0.0) + ";", "utf-8"))

                        self.request.sendall(bytes(str(idx_1 + 1) + "," + str(wheel_speed[0]) + "," + str(wheel_speed[1]) + ";", "utf-8"))
                    else:
                        self.request.sendall(bytes("p" + "," + str(wheel_speed[0]) + "," + str(wheel_speed[1]) + ";", "utf-8"))                        
                        #self.request.sendall(bytes("p" + "," + str(0.0) + "," + str(0.0) + ";", "utf-8"))
                            
                        
                        '''if update_prey:
                            if int((int(abs(switch_times[idx_1] - sim_time)) / 2.5)) % 2 == 0:
                                left_s = (np.random.rand() - 0.5) * 6
                                if abs(left_s) < 0.5:
                                    left_s = left_s + left_s / abs(left_s) * 0.5
                                right_s = -left_s
                            else:
                                if (np.random.rand() - 0.5) > 0:
                                    left_s = 2.0
                                    right_s = 2.0
                                else:
                                    left_s = -2.0
                                    right_s = -2.0

                            update_prey = False
                        else:                        
                            if int(int(abs(switch_times[idx_1] - sim_time)) / 5) % 2 == 0:
                                update_prey = True
                        
                        self.request.sendall(bytes("p" + "," + str(left_s) + "," + str(right_s) + ";", "utf-8"))'''
                        
                else: 
                    if idx_1 + 1 != len(Pp_info_list):
                        self.request.sendall(bytes(str(idx_1 + 1) + "," + str(0.0) + "," + str(0.0) + ";", "utf-8"))
                    else:
                        self.request.sendall(bytes("p" + "," + str(0.0) + "," + str(0.0) + ";", "utf-8"))


            if reset_flag == 1:
                if remove_data == 0:
                    if count_play % NUM_EVALUATION == 0:
                        print("-----------------------------------------")
                        #print(kid)
                        total_kid_fitness += prod(kid_fitnesses)
                        print("KID Fitness: ", prod(kid_fitnesses))                
                        total_kid_fitness /= NUM_EVALUATION                        

                        if total_kid_fitness > total_parent_fitness:
                            #print("Kid is better")                        
                            #print("PARENT FITNESS:", prod(parent_fitnesses))
                            print("AVG KID FITNESS:", total_kid_fitness)
                            parent = kid
                            #for idx in range(NUM_PREDATOR):
                            #    parent_fitnesses[idx] = kid_fitnesses[idx]
                            total_parent_fitness = total_kid_fitness
                            ps = 1.0
                        else:
                            #print("Parent is better")
                            #print("PARENT FITNESS:", prod(parent_fitnesses))
                            print("AVG KID FITNESS:", total_kid_fitness)
                            ps = 0.0

                        
                        print("BEST FITNESS:", total_parent_fitness)
                        MUT_STRENGTH *= np.exp(1/np.sqrt(DNA_SIZE+1) * (ps - pk_rate)/(1 - pk_rate))

                        MUT_STRENGTH = 0.200

                        if MUT_STRENGTH >= MUT_MAX:
                            MUT_STRENGTH = MUT_MAX
                        print("MUT_STRENGTH:", MUT_STRENGTH)

                        good_fitness += [total_kid_fitness]
                        #print("F LEN: ", len(good_fitness))
                        if len(good_fitness) > 1:
                            ff = open('fitness.txt', 'a+')
                            for fitness in good_fitness:
                                ff.write(str(fitness) + ',')
                            ff.close()
                            good_fitness = []

                        #_thread.start_new_thread(draw_thread, ("draw", ))
                        

                        #if np.random.rand() > REEVALUATION_PROB:
                        if ALTERNATIVE_FLAG == 0:
                            kid = make_kid(parent, MUT_STRENGTH)
                            for idx in range(NUM_PREDATOR):
                                kid_fitnesses[idx] = 0.0
                            
                            total_kid_fitness = 0.0
                        else:
                            # Reevalution
                            print("Predators Reevaluate!")
                            kid = parent
                            for idx in range(NUM_PREDATOR):
                                kid_fitnesses[idx] = 0.0
                            
                            total_parent_fitness *= 0.9
                            total_kid_fitness = 0.0
                            MUT_STRENGTH /= np.exp(1/np.sqrt(DNA_SIZE+1) * (1.0 - pk_rate)/(1 - pk_rate))

                        # PREY

                        prey_total_kid_fitness += prey_kid_fitness
                        print("PREY KID Fitness: ", prey_kid_fitness)                
                        prey_total_kid_fitness /= NUM_EVALUATION

                        #print(prey_kid)
                        if prey_total_kid_fitness > prey_total_parent_fitness:
                            #print("Kid is better")                        
                            #print("PARENT FITNESS:", prod(parent_fitnesses))
                            print("PREY AVG KID FITNESS:", prey_total_kid_fitness)
                            prey_parent = prey_kid
                            #for idx in range(NUM_PREDATOR):
                            #    parent_fitnesses[idx] = kid_fitnesses[idx]
                            prey_total_parent_fitness = prey_total_kid_fitness
                            prey_ps = 1.0
                        else:
                            #print("Parent is better")
                            #print("PARENT FITNESS:", prod(parent_fitnesses))
                            print("PREY AVG KID FITNESS:", prey_total_kid_fitness)
                            prey_ps = 0.0

                        
                        print("PREY BEST FITNESS:", prey_total_parent_fitness)
                        PREY_MUT_STRENGTH *= np.exp(1/np.sqrt(DNA_SIZE+1) * (prey_ps - pk_rate)/(1 - pk_rate))
                        
                        PREY_MUT_STRENGTH = 0.200

                        good_fitness_prey += [prey_total_kid_fitness]
                        #print("F LEN: ", len(good_fitness))
                        if len(good_fitness_prey) > 1:
                            fp = open('fitness_prey.txt', 'a+')
                            for fitness in good_fitness_prey:
                                fp.write(str(fitness) + ',')
                            fp.close()
                            good_fitness_prey = []

                        if PREY_MUT_STRENGTH >= MUT_MAX:
                            PREY_MUT_STRENGTH = MUT_MAX
                        print("PREY_MUT_STRENGTH:", PREY_MUT_STRENGTH)
                        
                        #if np.random.rand() > PREY_REEVALUATION_PROB:
                        if ALTERNATIVE_FLAG == 1:
                            ALTERNATIVE_FLAG = 0
                            prey_kid = make_kid(prey_parent, PREY_MUT_STRENGTH)
                            
                            prey_kid_fitness = 0.0
                            
                            prey_total_kid_fitness = 0.0
                        else:
                            ALTERNATIVE_FLAG = 1
                            print("Prey Reevaluate!")
                            prey_kid = prey_parent

                            prey_kid_fitness = 0.0
                            
                            prey_total_parent_fitness *= 0.5
                            prey_total_kid_fitness = 0.0

                            PREY_MUT_STRENGTH /= np.exp(1/np.sqrt(DNA_SIZE+1) * (1.0 - pk_rate)/(1 - pk_rate))

                        print("Generation:", count_generation, "Finished")
                        print("-----------------------------------------")
                        if count_generation > 5:
                            _thread.start_new_thread(save_parent, ("predator", parent))
                            _thread.start_new_thread(save_parent, ("prey", parent))

                        catch_flag = False
                        count_generation = count_generation + 1
                        count_play = 1
                        count_update = 0
                        average_cos = np.zeros((NUM_PREDATOR, ))
                        average_speed = np.zeros((NUM_PREDATOR, ))
                        average_cos_speed = np.zeros((NUM_PREDATOR, ))
                        average_distance = np.zeros((NUM_PREDATOR, ))

                        previous_position_x = np.array(init_position_x)[0:NUM_PREDATOR]
                        previous_position_y = np.array(init_position_y)[0:NUM_PREDATOR]

                        previous_sim_time = 0
                        prey_average_min_distance = 0.0

                        catch_time = PLAY_PERIOD
                    else:

                        total_kid_fitness += prod(kid_fitnesses)
                        print("KID Fitness: ", prod(kid_fitnesses))

                        for idx in range(NUM_PREDATOR):
                            kid_fitnesses[idx] = 0.0
                        
                        prey_total_kid_fitness += prey_kid_fitness
                        print("PREY KID Fitness: ", prey_kid_fitness)

                        prey_kid_fitness = 0.0
                        
                        catch_flag = False
                        count_play += 1
                        count_update = 0
                        average_cos = np.zeros((NUM_PREDATOR, ))
                        average_speed = np.zeros((NUM_PREDATOR, ))
                        average_cos_speed = np.zeros((NUM_PREDATOR, ))
                        average_distance = np.zeros((NUM_PREDATOR, ))

                        previous_position_x = np.array(init_position_x)[0:NUM_PREDATOR]
                        previous_position_y = np.array(init_position_y)[0:NUM_PREDATOR]
                        previous_sim_time = 0
                        prey_average_min_distance = 0.0

                        catch_time = PLAY_PERIOD
                else: 
                    pass

                print("RESET3!!!!!!!!!!")
                self.request.sendall(bytes("RESET;", "utf-8"))
                
                send_string = ""
                for i in range(NUM_PREDATOR):
                    send_string = send_string + str(i + 1) + "," + "0.0" + "," + "0.0" + ";"                
                send_string = send_string + "p" + "," + "0.0" + "," + "0.0" + ";\n"

                self.request.sendall(bytes(send_string, "utf-8"))
                                
                requestForUpdate=self.request.recv(1024)
                continue
                
            self.request.sendall(bytes("\n", "utf-8"))

            # just send back the same data, but upper-cased
            requestForUpdate=self.request.recv(1024)

#0##########################################

# 1+1 ES Trainer

###########################################
def save_parent(name, parent):
    global parent_dir
    np.savetxt(parent_dir + name + '_w0.txt', parent[0])
    np.savetxt(parent_dir + name + '_w1.txt', parent[1])
    np.savetxt(parent_dir + name + '_b0.txt', parent[2])
    np.savetxt(parent_dir + name + '_b1.txt', parent[3])

def draw_2d_heatmap_thread(name):
    global x0_list, y0_list, sigma_x_list, sigma_y_list, wall_x_list, wall_y_list, wall_sigma_x_list, wall_sigma_y_list
    
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
                #x = np.concatenate((x,x_a))
                #y = np.concatenate((y,y_a))
                #z = np.concatenate((z,z_a))
                z = z + z_a
        
        for idx, value in enumerate(wall_x_list):
            x_a, y_a, z_a = create_2d_wall_function(wall_x_list[idx],wall_y_list[idx],wall_sigma_x_list[idx],wall_sigma_y_list[idx])
            z = z + z_a

        plt.clf()
        plt.contourf(x, y, z, cmap='Reds')
        plt.colorbar()
        plt.draw()
        plt.pause(0.01)

        x = None
        y = None
        z = None
    

def create_2d_gaussian(x0, y0, sigma_x, sigma_y):

    size = 100
    x = np.linspace(-1, 1, size)
    y = np.linspace(-1, 1, size)

    x, y = np.meshgrid(x, y)
    z = (1/(2*np.pi*sigma_x*sigma_y) * np.exp(-((x - x0)**2/(2*sigma_x**2)
	 + (y - y0)**2/(2*sigma_y**2))))

    return x, y, z

def create_2d_wall_function(x0, y0, sigma_x, sigma_y):
    

    size = 100
    x = np.linspace(-1, 1, size)
    y = np.linspace(-1, 1, size)
    
    x, y = np.meshgrid(x, y)

    if abs(sigma_x - 0.0) <= 0.0001:
        z = 1/(2*np.pi*sigma_y) * np.exp(-((y - y0)**2/(2*sigma_y**2)))

    else:
        z = 1/(2*np.pi*sigma_x) * np.exp(-((x - x0)**2/(2*sigma_x**2)))

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

def forward(x, w1, w2, b1, b2):
    #x = np.reshape(x, (1, 1));
    '''print("SHAPE")
    print(x.shape)
    print(w1.shape)
    print(w2.shape)
    print(b1.shape)
    print(b2.shape)'''
    a1 = x.dot(w1) + b1
    z1 = G(a1)
    #z1 = np.tanh(0.6 * a1)
    #print(z1.shape)
    y = z1.dot(w2) + b2
    z2 = np.tanh(0.6 * y) * 2
    #print(z2.shape)

    return z2

def get_fitness(x, target):
    global I_S
    w1 = target[0]
    w2 = target[1]
    b1 = target[2]
    b2 = target[3]
    
    #print(b1)
    target = forward(x, w1, w2, b1, b2)
    #print("Two values: ", target[0], F(x)[0])
    fitness = -(target[0] - F(x)) ** 2
    #print("Fitness", fitness)
    return fitness

def make_kid(parent, MUT_STRENGTH):
    global H_S, W_MAX, B_MAX, PK
    
    if PK:
        return parent

    w_max = W_MAX
    w_min = -W_MAX
    b_max = B_MAX
    b_min = -B_MAX
    k0 = parent[0] + MUT_STRENGTH * np.random.randn(I_S, H_S)
    k0[k0>w_max] = w_max
    k0[k0<w_min] = w_min
    k1 = parent[1] + MUT_STRENGTH * np.random.randn(H_S, O_S)
    k1[k1>w_max] = w_max
    k1[k1<w_min] = w_min
    b0 = parent[2] + MUT_STRENGTH * np.random.randn(1, 1)
    b0[b0>b_max] = b_max
    b0[b0<b_min] = b_min
    b1 = parent[3] + MUT_STRENGTH * np.random.randn(1, 1)
    b1[b1>b_max] = b_max
    b1[b1<b_min] = b_min
    '''k0 = parent[0] + MUT_STRENGTH * (np.random.normal(0.0, 1.0, (I_S, H_S)))
    k0[k0>w_max] = w_max
    k0[k0<w_min] = w_min
    k1 = parent[1] + MUT_STRENGTH * (np.random.normal(0.0, 1.0, (H_S, O_S)))
    k1[k1>w_max] = w_max
    k1[k1<w_min] = w_min
    b0 = parent[2] + MUT_STRENGTH * (np.random.normal(0.0, 1.0, (1, 1)))
    b0[b0>w_max] = w_max
    b0[b0<w_min] = w_min
    b1 = parent[3] + MUT_STRENGTH * (np.random.normal(0.0, 1.0, (1, 1)))
    b1[b1>w_max] = w_max
    b1[b1<w_min] = w_min'''
    
    k = (k0, k1, b0, b1)
    return k

def kill_bad(parent, kid):
    global MUT_STRENGTH, pk_rate

    x = (np.random.rand(1, I_S) - 0.5) * 100
    fp = get_fitness(x, parent)[0]
    fk = get_fitness(x, kid)[0]
    
    p_target = pk_rate
    if fp < fk:     # kid better than parent
        parent = kid
        ps = 1.     # kid win -> ps = 1 (successful offspring)
    else:
        ps = 0.
    
    # adjust global mutation strength
    MUT_STRENGTH *= np.exp(1/np.sqrt(DNA_SIZE+1) * (ps - p_target)/(1 - p_target))
    if MUT_STRENGTH > MUT_MAX:
        MUT_STRENGTH = MUT_MAX
    #print("MUT_S: ", MUT_STRENGTH)
    return parent

# random init

#parent = (np.random.rand(I_S, H_S) / 10, np.random.rand(H_S, O_S) / 10, np.random.rand(1, 1), np.random.rand(1, 1))

'''for i in range(N_GENERATIONS):    
    kid = make_kid(parent)
    parent = kill_bad(parent, kid)'''


########################################

#socket server

########################################

_thread.start_new_thread(draw_2d_heatmap_thread, ("heatmap",))

HOST, PORT = "127.0.0.1", 9527
# Create the server, binding to localhost on port 9999

server = socketserver.TCPServer((HOST, PORT), MyTCPHandler, False)

server.allow_reuse_address = True
# Activate the server; this will keep running until you
# interrupt the program with Ctrl-C
server.server_bind()
server.server_activate()
server.serve_forever()
