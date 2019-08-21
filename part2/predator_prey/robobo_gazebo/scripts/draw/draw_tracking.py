import matplotlib.pyplot as plt
import numpy as np
import pickle
import os

from matplotlib.lines import Line2D
PATH = '/home/jaqq/catkin_ws/src/robobo/robobo_gazebo/scripts/output'

eps = 0.1

fig, ax = plt.subplots(figsize=(7,7))

for filename in os.listdir(PATH):

    if "tracking" not in filename:
        continue

    filepath = PATH + "/" + filename

#tracking_f = open(PATH + "/tracking_CMA_739_2.041772848793996.pkl", "rb")
#tracking_f = open(PATH + "/tracking_CMA_656_3.0321868088378596.pkl", "rb")
#tracking_f = open(PATH + "/tracking_CMA_885_4.023143398076614.pkl", "rb")
#tracking_f = open(PATH + "/tracking_CMA_972_5.190037496283524.pkl", "rb")
    tracking_f = open(filepath, "rb")


    tracking = pickle.load(tracking_f)
    tracking_f.close()

    prey_x = []
    prey_y = []
    prey_yaw = []
    prey_triangle = []

    NUM_PREDATOR = len(tracking[0][1])
    #print(NUM_PREDATOR)
    predators_x = np.zeros((NUM_PREDATOR, len(tracking)))
    predators_y = np.zeros((NUM_PREDATOR, len(tracking)))
    predators_yaw = np.zeros((NUM_PREDATOR, len(tracking)))
    predators_triangle_x = np.zeros((NUM_PREDATOR, 3 * len(tracking)))
    predators_triangle_y = np.zeros((NUM_PREDATOR, 3 * len(tracking)))

    triangle_size = 1 / 20
    draw_step = 10
    
    start_idx = 0
    find_start = False
    for idx, point in enumerate(tracking):            
            
        if not find_start:
            if abs(point[1][0][0] - (-1.0)) <= eps and abs(point[1][0][1] - (-1.5)) <= eps and abs(point[1][1][0] - (0.0)) <= eps and abs(point[1][1][1] - (-1.5)) <= eps and abs(point[1][2][0] - (1.0)) <= eps and abs(point[1][2][1] - (-1.5)) <= eps:
                find_start = True
                start_idx = idx
            else:
                continue
                           
            
        prey_x += [point[0][0]]
        prey_x_value = point[0][0]
        prey_y += [point[0][1]]
        prey_y_value = point[0][1]
        prey_yaw += [point[0][2]]
        prey_yaw_value = point[0][2] + np.pi / 2
        prey_triangle += [[prey_x_value + np.cos(prey_yaw_value) * triangle_size, prey_y_value + np.sin(prey_yaw_value) * triangle_size], 
                          [prey_x_value + np.cos(prey_yaw_value + 2/3 * np.pi) * triangle_size, prey_y_value + np.sin(prey_yaw_value + 2/3 * np.pi) * triangle_size], 
                          [prey_x_value + np.cos(prey_yaw_value + 4/3 * np.pi) * triangle_size, prey_y_value + np.sin(prey_yaw_value + 4/3 * np.pi) * triangle_size]]        
        
        for i in range(NUM_PREDATOR):                                        
                          
            predators_x[i,(idx - start_idx)] = point[1][i][0]
            predators_y[i,(idx - start_idx)] = point[1][i][1]
            predators_yaw[i,(idx - start_idx)] = point[1][i][2] + np.pi / 2
            
            predators_triangle_x[i, 3 * (idx - start_idx)] = predators_x[i,(idx - start_idx)] + np.cos(predators_yaw[i,(idx - start_idx)]) * triangle_size 
            predators_triangle_x[i, 3 * (idx - start_idx) + 1] = predators_x[i,(idx - start_idx)] + np.cos(predators_yaw[i,(idx - start_idx)] + 2/3 * np.pi) * triangle_size 
            predators_triangle_x[i, 3 * (idx - start_idx) + 2] = predators_x[i,(idx - start_idx)] + np.cos(predators_yaw[i,(idx - start_idx)] + 4/3 * np.pi) * triangle_size
            
            predators_triangle_y[i, 3 * (idx - start_idx)] = predators_y[i,(idx - start_idx)] + np.sin(predators_yaw[i,(idx - start_idx)]) * triangle_size
            predators_triangle_y[i, 3 * (idx - start_idx) + 1] = predators_y[i,(idx - start_idx)] + np.sin(predators_yaw[i,(idx - start_idx)] + 2/3 * np.pi) * triangle_size 
            predators_triangle_y[i, 3 * (idx - start_idx) + 2] = predators_y[i,(idx - start_idx)] + np.sin(predators_yaw[i,(idx - start_idx)] + 4/3 * np.pi) * triangle_size
        
    prey_triangle = np.array(prey_triangle)
    
    if start_idx > 0:
        predators_x = predators_x[:,:-start_idx]
        predators_y = predators_y[:,:-start_idx]
        predators_yaw = predators_yaw[:,:-start_idx]
        predators_triangle_x = predators_triangle_x[:,:-start_idx]
        predators_triangle_y = predators_triangle_y[:,:-start_idx]
    
    
    fig.subplots_adjust(bottom=0.04, top=0.96, left=0.04, right=0.96)

    ax.axis([-2, 2, -2, 2])

    line = Line2D(prey_x, prey_y, c='black', label="prey")
    ax.add_line(line)

    colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]

    for i in range(NUM_PREDATOR):    
        if i == 0:
            line = Line2D(predators_x[i], predators_y[i], c=colors[i], label="predators")
        else:
            line = Line2D(predators_x[i], predators_y[i], c=colors[i])
        ax.add_line(line)
        predators_triangle = np.array(list(zip(predators_triangle_x[i,:],predators_triangle_y[i,:])))
        for j in range(int(len(predators_triangle)/3)):     
            if j % draw_step == 0:           
                t = plt.Polygon(predators_triangle[j*3:j*3 + 3], color=colors[i])
                ax.add_patch(t)

    for i in range(int(len(prey_triangle)/3)):
        if i % draw_step == 0:   
            t = plt.Polygon(prey_triangle[i*3:i*3 + 3], color='black')
            ax.add_patch(t)
    #plt.xlabel("x (m)")
    #plt.ylabel("y (m)")
    for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
                         ax.get_xticklabels() + ax.get_yticklabels()):
        item.set_fontsize(16)
    #plt.legend(prop={'size': 22})
    ax.axes.get_xaxis().set_ticklabels([])
    ax.axes.get_yaxis().set_ticklabels([])
    ax.tick_params(axis=u'both', which=u'both',length=0)
    #plt.title("Trajectory of best predators evolved by BO")
    #plt.title("Trajectory of worst random position")
    plt.savefig("draw_tracking/" + filename + ".png")
    plt.cla()
    #plt.show()
plt.close(fig)
